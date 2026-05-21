// 1541 Disk Drive — Level-3 ("TrueDrive") emulation scaffolding.
//
// What's wired up:
//   * 6502 CPU instance (via the `emulator_6502` crate)
//   * 2 KB RAM, 16 KB DOS ROM (load from rom/1541.rom)
//   * Two 6522 VIA chips at $1800 (IEC) and $1C00 (disk head)
//   * Per-cycle step() so the C64 main loop can advance us in lockstep
//
// What's NOT wired up yet (TODO across future sessions):
//   * VIA timers, shift register, handshake lines
//   * IEC serial bus: at the bit level, the drive's VIA1-PB carries
//       PB0: DATA-in   PB2: CLK-in    PB7: ATN-in
//       PB1: DATA-out  PB3: CLK-out   ATN-ack from PCR/CA1
//     and connects to the C64's CIA2-PA bits 4-7. Need a shared IecBus type
//     that both sides poke/observe.
//   * GCR encoder/decoder + sync-mark detection
//   * Disk-image rotation simulation (sector under head, byte-ready, etc.)
//   * Step rate, head positioning, write-protect sense
//   * 1541 ROM presence check at boot
//
// Today: drive ticks its CPU off the ROM and idles. Nothing is loaded over
// the bus yet (the existing $F4A5 KERNAL trap stays on unless --truedrive
// is specified, so loading still works the Level-2 way).

mod bus;
mod disk;
mod gcr;
pub mod iec;
mod via;

use std::path::Path;

use c64::d64::D64;
use emulator_6502::MOS6502;

pub use self::bus::DriveBus;
pub use self::iec::{IecBus, IecBusShared};

pub struct Drive1541 {
    cpu: MOS6502,
    pub bus: DriveBus,
    pub enabled: bool,    // false if no ROM was found / mounted
    pub cycle_count: u64,
    talk_trace_hits: u32, // limit IEC talk trace output
    clk_rel_hits: u32,   // limit CLK-release entry trace output
    irq_ack_hits: u32,   // limit IRQ-in-ack-wait trace output
}

impl Drive1541 {
    /// Construct a drive in the "off" state — no ROM, no ticking.
    /// Used when --truedrive wasn't requested; saves a noisy ROM-lookup warning.
    pub fn disabled() -> Self {
        Drive1541 {
            cpu: MOS6502::new(),
            bus: DriveBus::new(),
            enabled: false,
            cycle_count: 0,
            talk_trace_hits: 0,
            clk_rel_hits: 0,
            irq_ack_hits: 0,
        }
    }

    pub fn set_iec(&mut self, iec: IecBusShared) {
        self.bus.via1.set_iec(iec);
    }

    /// Enable per-write tracing on VIA1 and VIA2. Combined with the IEC-bus
    /// edge log in C64::trace_step this gives a full PC-by-PC view of the
    /// IEC handshake (VIA1) and the disk subsystem (VIA2: motor/step/density).
    pub fn set_trace(&mut self, on: bool) {
        self.bus.via1.set_trace(on);
        self.bus.via2.set_trace(on);
    }

    /// Mount a D64 image so the drive's read head sees real GCR-encoded bytes
    /// as the disk rotates. Without this the drive is "no disk inserted" —
    /// motor can spin but no data ever shows up on the head.
    pub fn mount_d64(&mut self, image: D64) {
        self.bus.disk.borrow_mut().mount(image);
    }

    /// Try to construct a drive by loading the ROM file at `rom_path`.
    /// Returns a disabled drive if the file is missing; returns Err only
    /// on real I/O problems (so a missing ROM is a soft-fail).
    pub fn new<P: AsRef<Path>>(rom_path: P) -> Self {
        let path = rom_path.as_ref();
        let mut bus = DriveBus::new();

        match std::fs::read(path) {
            Ok(bytes) => {
                if let Err(e) = bus.load_rom(&bytes) {
                    println!("1541: {} — drive disabled", e);
                    return Drive1541 {
                        cpu: MOS6502::new(),
                        bus,
                        enabled: false,
                        cycle_count: 0,
                        talk_trace_hits: 0,
                        clk_rel_hits: 0,
                        irq_ack_hits: 0,
                    };
                }
                println!("1541: loaded {} ({} bytes)", path.display(), bytes.len());
            }
            Err(e) => {
                println!(
                    "1541: ROM not found at {} ({}). Drive emulation disabled. \
                     Drop a 16 KB 1541 DOS ROM there (often named 1541.rom or dos1541) \
                     to enable.",
                    path.display(), e
                );
                return Drive1541 {
                    cpu: MOS6502::new(),
                    bus,
                    enabled: false,
                    cycle_count: 0,
                    talk_trace_hits: 0,
                    clk_rel_hits: 0,
                    irq_ack_hits: 0,
                };
            }
        }

        let mut drive = Drive1541 {
            cpu: MOS6502::new(),
            bus,
            enabled: true,
            cycle_count: 0,
            talk_trace_hits: 0,
            clk_rel_hits: 0,
            irq_ack_hits: 0,
        };
        drive.reset();
        drive
    }

    pub fn reset(&mut self) {
        self.bus.reset_chips();
        // emulator_6502's reset reads the reset vector at $FFFC and points PC there.
        self.cpu.reset(&mut self.bus);
        self.cycle_count = 0;
    }

    /// Advance the drive by one master cycle. Called once per C64 cycle.
    /// The 1541 actually runs at 1 MHz and the C64 at ~985 kHz, so they're
    /// within 1.5%; close enough for the timing the IEC handshake needs.
    pub fn step(&mut self) {
        if !self.enabled { return; }
        // Disk first — its byte-ready pulse is observed by VIA2 in the tick
        // immediately after, so the ROM can pick it up via CA1 IFR.
        self.bus.disk.borrow_mut().rotate(1);

        // SO pin on the 6502: in the 1541 hardware, the disk-controller's
        // BYTE-READY signal is wired to the CPU's SO (Set Overflow) input.
        // When BYTE READY pulses, the V flag of the processor status register
        // is set; the ROM polls it with `BVC` (e.g. $F3BE: BVC $F3BE) and
        // clears it with `CLV` after consuming the byte.
        if self.bus.disk.borrow().byte_ready {
            let p = self.cpu.get_status_register();
            self.cpu.set_status_register(p | 0x40);
        }

        self.bus.via1.tick(1);
        self.bus.via2.tick(1);

        // Keep the PC the VIA prints in trace mode current with the upcoming
        // cycle. CPU.pc here is the PC of the *next* instruction to fetch.
        let pc = self.cpu.get_program_counter();
        self.bus.via1.set_trace_pc(pc);
        self.bus.via2.set_trace_pc(pc);

        if self.bus.irq_asserted() {
            self.cpu.interrupt_request();
        }

        // Targeted trace for IEC TALK byte-send debugging.
        // Fires at the key decision points in the 1541 ROM's send loop;
        // limited to 200 total hits so the log stays readable.
        if self.talk_trace_hits < 200 {
            let pc = self.cpu.get_program_counter();
            let hit = match pc {
                // $E913: BMI check — is bit7 of F2[ch] set? (active channel)
                0xE913 => {
                    let ch = self.bus.ram[0x82] as usize;
                    let f2 = if ch < 16 { self.bus.ram[0xF2 + ch] } else { 0xFF };
                    eprintln!("[E913 bit7-chk] ch={} F2={:02X} cyc={}", ch, f2, self.cycle_count);
                    true
                }
                // $E931: AND #$08 — is bit3 set? (EOI vs direct-CLK path)
                0xE931 => {
                    let ch = self.bus.ram[0x82] as usize;
                    let f2   = if ch < 16          { self.bus.ram[0xF2 + ch] } else { 0xFF };
                    let ec   = if ch < 16          { self.bus.ram[0xEC + ch] } else { 0xFF };
                    let bsnd = if 0x23E + ch < 0x800 { self.bus.ram[0x23E + ch] } else { 0xFF };
                    eprintln!("[E931 bit3-chk] ch={} F2={:02X} EC={:02X} byte={:02X} cyc={}",
                        ch, f2, ec, bsnd, self.cycle_count);
                    true
                }
                // $EA4E: CLK release + jump to scheduler (wrong path — bit7 was 0)
                0xEA4E => {
                    let ch = self.bus.ram[0x82] as usize;
                    let f2 = if ch < 16 { self.bus.ram[0xF2 + ch] } else { 0xFF };
                    eprintln!("[EA4E clk-rel]  ch={} F2={:02X} cyc={}", ch, f2, self.cycle_count);
                    true
                }
                // $DCFA: STA $F2,X with #$01 (file-open sets F2=01, bit7=0)
                0xDCFA => {
                    let ch = self.bus.ram[0x82] as usize;
                    eprintln!("[DCFA F2<-01]   ch={} cyc={}", ch, self.cycle_count);
                    true
                }
                // $E142: STA $F2,Y with #$89 (non-last block, bit3=1)
                0xE142 => {
                    let ch = self.bus.ram[0x82] as usize;
                    eprintln!("[E142 F2<-89]   ch={} cyc={}", ch, self.cycle_count);
                    true
                }
                // $E14F: STA $F2,Y with #$81 (last block/EOF, bit3=0)
                0xE14F => {
                    let ch = self.bus.ram[0x82] as usize;
                    eprintln!("[E14F F2<-81]   ch={} cyc={}", ch, self.cycle_count);
                    true
                }
                _ => false,
            };
            if hit { self.talk_trace_hits += 1; }
        }

        // Trace A: log every entry into the CLK-release routine.
        // Normal calls come from the bit-send loop ($E958-$E985).
        // A call from any other origin (e.g. an IRQ handler) appears here
        // with a non-zero IFR, exposing the unexpected CLK-release at $E9BF.
        if self.clk_rel_hits < 200 && pc == 0xE9B7 {
            let ifr1 = self.bus.via1.ifr();
            let ifr2 = self.bus.via2.ifr();
            let irq  = self.bus.irq_asserted();
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[E9B7-CLK-REL] cyc={} ifr1={:02X} ifr2={:02X} irq={} \
                     c64={{atn:{} clk:{} dat:{}}} drv={{clk:{} dat:{} atna:{}}}",
                    self.cycle_count, ifr1, ifr2, irq as u8,
                    b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                    b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                );
            }
            self.clk_rel_hits += 1;
        }

        // Trace B: log whenever an IRQ is pending while the CPU is in the
        // ack-wait loop.  If this fires, a VIA interrupt is firing mid-ack-wait
        // and the IRQ handler is responsible for the unexpected CLK-release.
        if self.irq_ack_hits < 200 && self.bus.irq_asserted()
            && matches!(pc, 0xE987..=0xE990)
        {
            let ifr1 = self.bus.via1.ifr();
            let ifr2 = self.bus.via2.ifr();
            eprintln!(
                "[IRQ@ACK-WAIT] pc=${:04X} cyc={} via1_ifr={:02X} via2_ifr={:02X}",
                pc, self.cycle_count, ifr1, ifr2,
            );
            self.irq_ack_hits += 1;
        }

        self.cpu.cycle(&mut self.bus);
        self.cycle_count = self.cycle_count.wrapping_add(1);
    }

    pub fn pc(&self) -> u16 { self.cpu.get_program_counter() }
}
