// 6522 VIA — used in two roles on the 1541:
//
//   * VIA1 ($1800) — IEC bus interface (ATN/CLK/DATA, device address)
//   * VIA2 ($1C00) — disk subsystem (motor, head step, byte under head)
//
// What this module currently implements:
//   * Full 16-register read/write with semantically-correct side-effects on
//     IFR/IER and the timer-write transfer.
//   * T1 and T2 16-bit countdown timers. T1 supports one-shot mode and free-
//     running ACR-bit-6 mode. T2 only does one-shot (TODO: pulse-counting).
//   * VIA1 port B routed to the shared IEC bus (DATA/CLK in/out, ATN in,
//     ATNA bit, device-8 jumpers).
//   * VIA1 ATN-edge -> CA1 IFR bit (so the 1541 ROM's CA1 interrupt fires).
//   * VIA2 port A reads the byte under the head from the Disk model.
//   * VIA2 port B step-motor/motor/density/WP/sync wiring to Disk.
//   * VIA2 CA1 IFR pulses on Disk's byte_ready.
//
// Not yet:
//   * T1/T2 PB7 output and pulse-counting modes
//   * Shift register
//   * CA2/CB2 output modes (only the 1541's PCR bit 5 "head mode" is read
//     when needed by VIA2 PA writes — write support is TODO)

use super::disk::DiskShared;
use super::iec::IecBusShared;

// --- VIA register offsets ---
const R_ORB:  usize = 0x0;
const R_ORA:  usize = 0x1;
const R_DDRB: usize = 0x2;
const R_DDRA: usize = 0x3;
const R_T1CL: usize = 0x4;
const R_T1CH: usize = 0x5;
const R_T1LL: usize = 0x6;
const R_T1LH: usize = 0x7;
const R_T2CL: usize = 0x8;
const R_T2CH: usize = 0x9;
const R_SR:   usize = 0xA;
const R_ACR:  usize = 0xB;
const R_PCR:  usize = 0xC;
const R_IFR:  usize = 0xD;
const R_IER:  usize = 0xE;
const R_ORA_NH: usize = 0xF;

// --- IFR bit positions ---
const IFR_CA2: u8 = 1 << 0;
const IFR_CA1: u8 = 1 << 1;
#[allow(dead_code)]
const IFR_SR:  u8 = 1 << 2;
#[allow(dead_code)]
const IFR_CB2: u8 = 1 << 3;
#[allow(dead_code)]
const IFR_CB1: u8 = 1 << 4;
const IFR_T2:  u8 = 1 << 5;
const IFR_T1:  u8 = 1 << 6;

// --- VIA1 (IEC) PB bit positions ---
const IEC_PB_DATA_IN:  u8 = 0;
const IEC_PB_DATA_OUT: u8 = 1;
const IEC_PB_CLK_IN:   u8 = 2;
const IEC_PB_CLK_OUT:  u8 = 3;
const IEC_PB_ATNA:     u8 = 4;
const IEC_PB_ATN_IN:   u8 = 7;

// --- VIA2 (Disk) PB bit positions ---
const DSK_PB_STEP:      u8 = 0x03;   // bits 0,1 = head step phase (00->01->10->11->00 = step out)
const DSK_PB_MOTOR:     u8 = 1 << 2;
#[allow(dead_code)]
const DSK_PB_LED:       u8 = 1 << 3;
#[allow(dead_code)]
const DSK_PB_WP:        u8 = 1 << 4;  // input: 1 = write enabled
const DSK_PB_DENSITY:   u8 = 0x60;   // bits 5,6
const DSK_PB_SYNC:      u8 = 1 << 7;  // input: 0 = sync mark under head

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViaKind {
    Iec,   // VIA1 at $1800
    Disk,  // VIA2 at $1C00
}

pub struct Via {
    pub kind: ViaKind,
    regs: [u8; 16],

    // Timer 1
    t1_counter: u16,
    t1_running: bool,
    // Timer 2
    t2_counter: u16,
    t2_running: bool,

    // VIA1 (IEC) only
    iec: Option<IecBusShared>,
    prev_atn_low: bool,

    // VIA2 (Disk) only
    disk: Option<DiskShared>,
    // Last seen step-phase bits — used to detect head-step transitions.
    step_phase: u8,

    pub irq_pending: bool,

    // Diagnostic: if true, print every ORB write with PC stashed via
    // `set_trace_pc()` so we can correlate bus changes with code addresses.
    pub trace: bool,
    pub trace_pc: u16,
}

impl Via {
    pub fn new(kind: ViaKind) -> Self {
        Via {
            kind,
            regs: [0; 16],
            t1_counter: 0xFFFF,
            t1_running: false,
            t2_counter: 0xFFFF,
            t2_running: false,
            iec: None,
            prev_atn_low: false,
            disk: None,
            step_phase: 0,
            irq_pending: false,
            trace: false,
            trace_pc: 0,
        }
    }

    pub fn set_iec(&mut self, iec: IecBusShared) { self.iec = Some(iec); }
    pub fn set_disk(&mut self, disk: DiskShared) { self.disk = Some(disk); }
    pub fn set_trace(&mut self, on: bool) { self.trace = on; }
    pub fn set_trace_pc(&mut self, pc: u16) { self.trace_pc = pc; }

    pub fn reset(&mut self) {
        for r in self.regs.iter_mut() { *r = 0; }
        self.t1_counter = 0xFFFF; self.t1_running = false;
        self.t2_counter = 0xFFFF; self.t2_running = false;
        self.prev_atn_low = false;
        self.step_phase = 0;
        self.irq_pending = false;
        if let Some(b) = &self.iec {
            let mut b = b.borrow_mut();
            b.drive_clk = false;
            b.drive_data = false;
            b.drive_atna = false;
        }
    }

    pub fn read(&mut self, reg: u8) -> u8 {
        let r = (reg & 0x0F) as usize;
        match r {
            R_ORB => {
                let ddr = self.regs[R_DDRB];
                let orb = self.regs[R_ORB];
                let pin = self.pb_pin();
                (orb & ddr) | (pin & !ddr)
            }
            R_ORA => {
                // Reading ORA with handshake — clears CA1/CA2 IFR bits.
                self.regs[R_IFR] &= !(IFR_CA1 | IFR_CA2);
                self.recompute_irq();
                self.pa_pin_value()
            }
            R_ORA_NH => self.pa_pin_value(),
            R_T1CL => {
                // Reading T1C-L clears the T1 IFR bit.
                self.regs[R_IFR] &= !IFR_T1;
                self.recompute_irq();
                (self.t1_counter & 0xFF) as u8
            }
            R_T1CH => (self.t1_counter >> 8) as u8,
            R_T1LL => self.regs[R_T1LL],
            R_T1LH => self.regs[R_T1LH],
            R_T2CL => {
                self.regs[R_IFR] &= !IFR_T2;
                self.recompute_irq();
                (self.t2_counter & 0xFF) as u8
            }
            R_T2CH => (self.t2_counter >> 8) as u8,
            R_IFR => {
                let any = (self.regs[R_IFR] & self.regs[R_IER] & 0x7F) != 0;
                (self.regs[R_IFR] & 0x7F) | if any { 0x80 } else { 0 }
            }
            R_IER => self.regs[R_IER] | 0x80,
            _ => self.regs[r],
        }
    }

    pub fn write(&mut self, reg: u8, val: u8) {
        let r = (reg & 0x0F) as usize;
        match r {
            R_ORB => {
                self.regs[R_ORB] = val;
                self.on_port_b_change();
                if self.trace && matches!(self.kind, ViaKind::Iec) {
                    let (dat, clk, atna) = if let Some(b) = &self.iec {
                        let b = b.borrow();
                        (b.drive_data as u8, b.drive_clk as u8, b.drive_atna as u8)
                    } else { (0, 0, 0) };
                    println!(
                        "[via1] STA $1800,#${:02X}  pc=${:04X}  -> dat={} clk={} atna={}",
                        val, self.trace_pc, dat, clk, atna,
                    );
                }
            }
            R_ORA => {
                self.regs[R_IFR] &= !(IFR_CA1 | IFR_CA2);
                self.regs[R_ORA] = val;
                // VIA2 ORA write with head in WRITE mode would push the byte
                // back to the track. We don't support write yet, so just
                // store the value for read-back.
            }
            R_ORA_NH => {
                self.regs[R_ORA] = val;
            }
            R_DDRB => {
                self.regs[R_DDRB] = val;
                self.on_port_b_change();
            }
            R_DDRA => {
                self.regs[R_DDRA] = val;
            }
            R_T1CL | R_T1LL => self.regs[R_T1LL] = val,
            R_T1CH => {
                // Latch high, transfer latches to counter, start, clear T1 IFR.
                self.regs[R_T1LH] = val;
                self.t1_counter = ((val as u16) << 8) | (self.regs[R_T1LL] as u16);
                self.t1_running = true;
                self.regs[R_IFR] &= !IFR_T1;
            }
            R_T1LH => {
                // Latch high only — does NOT transfer or clear IFR.
                self.regs[R_T1LH] = val;
            }
            R_T2CL => self.regs[R_T2CL] = val,
            R_T2CH => {
                // Load high, transfer T2CL into counter low, start, clear T2 IFR.
                self.t2_counter = ((val as u16) << 8) | (self.regs[R_T2CL] as u16);
                self.t2_running = true;
                self.regs[R_IFR] &= !IFR_T2;
            }
            R_ACR => self.regs[R_ACR] = val,
            R_PCR => self.regs[R_PCR] = val,
            R_IFR => {
                // Writing a 1 to a bit CLEARS that bit (active-low write).
                self.regs[R_IFR] &= !(val & 0x7F);
            }
            R_IER => {
                if val & 0x80 != 0 {
                    self.regs[R_IER] |= val & 0x7F;
                } else {
                    self.regs[R_IER] &= !(val & 0x7F);
                }
            }
            R_SR => self.regs[R_SR] = val,   // TODO: shift register
            _ => self.regs[r] = val,
        }
        self.recompute_irq();
    }

    /// Per-cycle bookkeeping: timers, ATN-edge detection on VIA1, byte-ready
    /// pulse on VIA2.
    pub fn tick(&mut self, cycles: u32) {
        // ----- Timers -----
        // T1: decrement; on underflow, set IFR_T1; in free-running mode
        // (ACR bit 6 set), reload from latch and continue; otherwise stop.
        for _ in 0..cycles {
            if self.t1_running {
                self.t1_counter = self.t1_counter.wrapping_sub(1);
                if self.t1_counter == 0xFFFF {
                    self.regs[R_IFR] |= IFR_T1;
                    if self.regs[R_ACR] & 0x40 != 0 {
                        // Free-running: reload from latch.
                        self.t1_counter = ((self.regs[R_T1LH] as u16) << 8)
                                        | (self.regs[R_T1LL] as u16);
                    } else {
                        self.t1_running = false;
                    }
                }
            }
            if self.t2_running {
                self.t2_counter = self.t2_counter.wrapping_sub(1);
                if self.t2_counter == 0xFFFF {
                    self.regs[R_IFR] |= IFR_T2;
                    self.t2_running = false;
                }
            }
        }

        // ----- VIA1: ATN-edge detection -----
        if matches!(self.kind, ViaKind::Iec) {
            if let Some(b) = &self.iec {
                let atn_low = b.borrow().atn_low();
                if atn_low != self.prev_atn_low {
                    self.regs[R_IFR] |= IFR_CA1;
                }
                self.prev_atn_low = atn_low;
            }
        }

        // ----- VIA2: byte-ready CA1 pulse -----
        if matches!(self.kind, ViaKind::Disk) {
            if let Some(d) = &self.disk {
                let mut d = d.borrow_mut();
                if d.byte_ready {
                    self.regs[R_IFR] |= IFR_CA1;
                    d.clear_byte_ready();
                }
            }
        }

        self.recompute_irq();
    }

    // ---- ports ----

    fn pa_pin_value(&self) -> u8 {
        let ddr = self.regs[R_DDRA];
        let ora = self.regs[R_ORA];
        let pin = match self.kind {
            ViaKind::Iec => 0xFF,
            ViaKind::Disk => match &self.disk {
                Some(d) => d.borrow().current_byte(),
                None => 0xFF,
            },
        };
        (ora & ddr) | (pin & !ddr)
    }

    fn pb_pin(&self) -> u8 {
        match self.kind {
            ViaKind::Iec => self.iec_pb_pin(),
            ViaKind::Disk => self.disk_pb_pin(),
        }
    }

    fn iec_pb_pin(&self) -> u8 {
        let Some(b) = self.iec.as_ref() else { return 0xFF; };
        let b = b.borrow();
        let mut v = 0xFFu8;
        if b.data_low() { v &= !(1 << IEC_PB_DATA_IN); }
        if b.clk_low()  { v &= !(1 << IEC_PB_CLK_IN); }
        if b.atn_low()  { v &= !(1 << IEC_PB_ATN_IN); }
        // device 8 — both jumper bits read 0
        v &= !(1 << 5);
        v &= !(1 << 6);
        v
    }

    fn disk_pb_pin(&self) -> u8 {
        // Latched outputs come from ORB; inputs come from disk state.
        // Inputs we model: WP (bit 4) — always 1 (writable). SYNC (bit 7) —
        // 0 while head is over a sync mark.
        let mut v = 0xFFu8;
        // WP=1 (writable)
        if let Some(d) = &self.disk {
            if d.borrow().sync_under_head() {
                v &= !DSK_PB_SYNC;
            }
        }
        v
    }

    fn on_port_b_change(&mut self) {
        match self.kind {
            ViaKind::Iec => self.push_iec_pb(),
            ViaKind::Disk => self.push_disk_pb(),
        }
    }

    fn push_iec_pb(&mut self) {
        let Some(b) = self.iec.as_ref() else { return; };
        let ddr = self.regs[R_DDRB];
        let orb = self.regs[R_ORB];
        let mut b = b.borrow_mut();
        b.drive_data = (ddr & (1 << IEC_PB_DATA_OUT) != 0) && (orb & (1 << IEC_PB_DATA_OUT) != 0);
        b.drive_clk  = (ddr & (1 << IEC_PB_CLK_OUT)  != 0) && (orb & (1 << IEC_PB_CLK_OUT)  != 0);
        b.drive_atna = (ddr & (1 << IEC_PB_ATNA)     != 0) && (orb & (1 << IEC_PB_ATNA)     != 0);
    }

    fn push_disk_pb(&mut self) {
        let Some(d) = self.disk.as_ref() else { return; };
        let ddr = self.regs[R_DDRB];
        let orb = self.regs[R_ORB];
        // Only driven bits count.
        let driven = orb & ddr;

        // Motor
        let motor = driven & DSK_PB_MOTOR != 0;
        // Density
        let density = (driven & DSK_PB_DENSITY) >> 5;
        // Head step phase (bits 0,1)
        let new_phase = driven & DSK_PB_STEP;

        let mut d = d.borrow_mut();
        d.set_motor(motor);
        d.set_density(density);

        // Step direction is determined by which way the phase rotates.
        // Gray-code-like: 00 -> 01 -> 10 -> 11 -> 00 steps the head one
        // half-track in one direction; the reverse sequence steps the other.
        let prev = self.step_phase;
        let diff = (new_phase as i8 - prev as i8) & 3;
        if diff != 0 {
            let mut new_track = d.track as i16;
            // We model integer tracks only: every two phase advances = one
            // physical-track step. Pragmatic shortcut; real hardware steps
            // by half-tracks.
            match diff {
                1 => new_track += 1,    // step in (toward center, higher track #)
                3 => new_track -= 1,    // step out (toward edge, lower track #)
                _ => {}                 // 2 = double-skip, rare; ignore
            }
            let nt = new_track.max(1).min(35) as u8;
            d.set_track(nt);
        }
        drop(d);
        self.step_phase = new_phase;
    }

    fn recompute_irq(&mut self) {
        self.irq_pending = (self.regs[R_IFR] & self.regs[R_IER] & 0x7F) != 0;
    }
}
