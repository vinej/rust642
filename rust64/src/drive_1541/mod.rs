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
    post_irq_trace: u32,     // countdown: trace drive PC for N cycles after IRQ-in-ack-wait
    post_irq_prev_pc: u16,   // last PC printed in post-IRQ trace (print only on change)
    // ATN command trace: log the VIA1 PB byte each time drive enters the command
    // receive loop at $EC00, up to 200 entries.  Reveals what commands the C64
    // sends during the fast-loader handshake.
    atn_cmd_hits: u32,
    atn_cmd_prev_pc: u16,   // suppress consecutive hits at same PC
    drive_in_cmd_loop: bool, // was drive in $EC00-$EC80 on previous step
    drive_cmd_exit_hits: u32, // how many times drive exited command loop post-gate
    // Targeted command-processor traces (post-gate only, small hit limits).
    ec00_hits: u32,          // $EC00 entry: dump $022B cmd buffer + $7C/$026C
    df93_hits: u32,          // $DF93 entry: dump $82/$026C/IEC state
    post_df93_trace: u32,    // countdown: print unique PCs after each $DF93 call
    df93_exit_prev_pc: u16,  // suppress consecutive same-PC in post-DF93 trace
    ec98_hits: u32,          // $EC98 entry: dump $026C/$7C to confirm idle reason
    // $E85B fast-loader ATN handler trace (fires when $7C=1 at EC00).
    e85b_hits: u32,
    post_e85b_trace: u32,    // countdown: print unique PCs after $E85B entry
    e85b_exit_prev_pc: u16,
    // E8D7-ENTRY: trace the ATN exit path (fires when drive hits $E8D7 after an E85B).
    e8d7_hits: u32,
    // E909-ENTRY: trace every call to the data-ready check ($E909).
    // $E909: SEI; JSR $D0EB (find ch); BCS $E915; LDX $82; LDA $F2,X; BMI $E916; RTS
    // BMI → data ready (bit7=1); RTS → no data (bit7=0) → $EA4E releases bus.
    // Watching this reveals if/when disk data becomes available for the second file.
    e909_hits: u32,
    // E87B-STUCK: detect when drive is spinning in the $E87B CLK-wait loop for too long.
    // $E87B: LDA $1800; BPL $E8D7; AND #$04; BNE $E87B — exits when CLK released (bit2=0).
    // If c64_clk=1 indefinitely, this loop never exits. Track consecutive cycles at $E87B.
    e87b_stuck_cycles: u32,
    e87b_stuck_logged: bool,
    // CMD-CHAN: detect transitions in $022B[15] (command channel SA=15).
    // Reveals whether M-W/M-E commands are sent from the C64 during fast-load.
    cmd_chan_hits: u32,
    prev_cmd_chan: u8,
    // CH1-SLOT: track $022C ($022B[1]) — drive buffer slot for channel 1 (SA=1).
    // FF = channel 1 closed. Changes = SA=1 file opened/closed.
    // No gate — the core question is whether drive EVER opens channel 1 for file 2.
    ch1_slot_hits: u32,
    prev_ch1_slot: u8,
    // DRV-CMDBUF: watch first byte of drive command buffer ($0200) for changes.
    // M-W commands begin with 'M' ($4D) at $0200; detecting this confirms M-W arrived.
    drv_cmdbuf_hits: u32,
    prev_drv_cmdbuf0: u8,
    // DRV-ZONE: detect when drive PC exits/enters the $EC00-$EC80 IEC-command zone.
    // $EC14 idle loop lives inside this zone. An exit reveals the drive reacting to ATN.
    drv_zone_in_cmd: bool,
    drv_zone_hits: u32,
    // F2[1]-WATCH: monitor drive RAM $F3 (= F2+channel1) for any change, no gate.
    // $00 = never opened, $01 = opened/no data, $81/$89 = data ready.
    prev_f2_ch1: u8,
    f2_ch1_hits: u32,
    // ATN-RISE: detect every c64_atn false→true edge after 88M (post-first-file).
    // Shows when C64 asserts ATN for the second file open/talk sequence.
    prev_atn_low: bool,
    atn_rise_hits: u32,
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
            post_irq_trace: 0,
            post_irq_prev_pc: 0,
            atn_cmd_hits: 0,
            atn_cmd_prev_pc: 0xFFFF,
            drive_in_cmd_loop: false,
            drive_cmd_exit_hits: 0,
            ec00_hits: 0,
            df93_hits: 0,
            post_df93_trace: 0,
            df93_exit_prev_pc: 0xFFFF,
            ec98_hits: 0,
            e85b_hits: 0,
            post_e85b_trace: 0,
            e85b_exit_prev_pc: 0xFFFF,
            e8d7_hits: 0,
            e909_hits: 0,
            e87b_stuck_cycles: 0,
            e87b_stuck_logged: false,
            cmd_chan_hits: 0,
            prev_cmd_chan: 0xFF,
            ch1_slot_hits: 0,
            prev_ch1_slot: 0xFF,
            drv_cmdbuf_hits: 0,
            prev_drv_cmdbuf0: 0,
            drv_zone_in_cmd: false,
            drv_zone_hits: 0,
            prev_f2_ch1: 0,
            f2_ch1_hits: 0,
            prev_atn_low: false,
            atn_rise_hits: 0,
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
                        post_irq_trace: 0,
                        post_irq_prev_pc: 0,
                        atn_cmd_hits: 0,
                        atn_cmd_prev_pc: 0xFFFF,
                        drive_in_cmd_loop: false,
                        drive_cmd_exit_hits: 0,
                        ec00_hits: 0,
                        df93_hits: 0,
                        post_df93_trace: 0,
                        df93_exit_prev_pc: 0xFFFF,
                        ec98_hits: 0,
                        e85b_hits: 0,
                        post_e85b_trace: 0,
                        e85b_exit_prev_pc: 0xFFFF,
            e8d7_hits: 0,
            e909_hits: 0,
            e87b_stuck_cycles: 0,
            e87b_stuck_logged: false,
            cmd_chan_hits: 0,
            prev_cmd_chan: 0xFF,
            ch1_slot_hits: 0,
            prev_ch1_slot: 0xFF,
            drv_cmdbuf_hits: 0,
            prev_drv_cmdbuf0: 0,
            drv_zone_in_cmd: false,
            drv_zone_hits: 0,
            prev_f2_ch1: 0,
            f2_ch1_hits: 0,
            prev_atn_low: false,
            atn_rise_hits: 0,
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
                    post_irq_trace: 0,
                    post_irq_prev_pc: 0,
                    atn_cmd_hits: 0,
                    atn_cmd_prev_pc: 0xFFFF,
                    drive_in_cmd_loop: false,
                    drive_cmd_exit_hits: 0,
                    ec00_hits: 0,
                    df93_hits: 0,
                    post_df93_trace: 0,
                    df93_exit_prev_pc: 0xFFFF,
                    ec98_hits: 0,
                    e85b_hits: 0,
                    post_e85b_trace: 0,
                    e85b_exit_prev_pc: 0xFFFF,
            e8d7_hits: 0,
            e909_hits: 0,
            e87b_stuck_cycles: 0,
            e87b_stuck_logged: false,
            cmd_chan_hits: 0,
            prev_cmd_chan: 0xFF,
            ch1_slot_hits: 0,
            prev_ch1_slot: 0xFF,
            drv_cmdbuf_hits: 0,
            prev_drv_cmdbuf0: 0,
            drv_zone_in_cmd: false,
            drv_zone_hits: 0,
            prev_f2_ch1: 0,
            f2_ch1_hits: 0,
            prev_atn_low: false,
            atn_rise_hits: 0,
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
            post_irq_trace: 0,
            post_irq_prev_pc: 0,
            atn_cmd_hits: 0,
            atn_cmd_prev_pc: 0xFFFF,
            drive_in_cmd_loop: false,
            drive_cmd_exit_hits: 0,
            ec00_hits: 0,
            df93_hits: 0,
            post_df93_trace: 0,
            df93_exit_prev_pc: 0xFFFF,
            ec98_hits: 0,
            e85b_hits: 0,
            post_e85b_trace: 0,
            e85b_exit_prev_pc: 0xFFFF,
            e8d7_hits: 0,
            e909_hits: 0,
            e87b_stuck_cycles: 0,
            e87b_stuck_logged: false,
            cmd_chan_hits: 0,
            prev_cmd_chan: 0xFF,
            ch1_slot_hits: 0,
            prev_ch1_slot: 0xFF,
            drv_cmdbuf_hits: 0,
            prev_drv_cmdbuf0: 0,
            drv_zone_in_cmd: false,
            drv_zone_hits: 0,
            prev_f2_ch1: 0,
            f2_ch1_hits: 0,
            prev_atn_low: false,
            atn_rise_hits: 0,
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

        // Don't deliver IRQ while the CPU is inside the two-read debounce subroutine
        // ($E9C0-$E9C8: LDA $1800 / CMP $1800 / BNE loop / RTS).
        // If T1 fires between the two reads, the handler runs while the C64's ACK
        // pulse (DATA briefly low) may have ended. The second read then sees DATA=H,
        // the mismatch loops the debounce, and when stable it reads DATA=H → BEQ
        // at $E98F never clears → ack-wait hangs.
        if self.bus.irq_asserted() && !matches!(pc, 0xE9C0..=0xE9C8) {
            self.cpu.interrupt_request();
        }

        // ATN command trace: log IEC state each time the drive is in $EC00-$EC80 AND
        // ATN is asserted. The ATN filter prevents flooding from the idle loop which
        // also lives in this range ($EC14) but only fires during actual ATN sequences.
        // Gate lowered to 77M so the ~79M ATN events are visible.
        let atn_now = self.bus.via1.iec_opt().map_or(false, |b| b.borrow().atn_low());
        if self.atn_cmd_hits < 200 && self.cycle_count > 77_000_000
            && matches!(pc, 0xEC00..=0xEC80) && pc != self.atn_cmd_prev_pc
            && atn_now
        {
            let pb = self.bus.ram[0x1800 & 0x7FF]; // VIA1 PB register (direct RAM peek)
            // Read VIA1 PB properly via the via struct
            let via1_pb = self.bus.via1.ifr(); // we use ifr as a proxy; what we really want is PB
            // Actually read the VIA1 PB output register directly from bus RAM
            // VIA1 is at $1800; PB input (what drive reads) comes from IEC bus wiring.
            // bus.ram is 2KB; $1800 is above RAM — use via1 accessor instead.
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[ATN-CMD] pc=${:04X} cyc={} iec={{atn:{} clk:{} dat:{} c64_atn:{} c64_clk:{} c64_dat:{} drv_clk:{} drv_dat:{} atna:{}}}",
                    pc, self.cycle_count,
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                    b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                    b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                );
            } else {
                eprintln!("[ATN-CMD] pc=${:04X} cyc={}", pc, self.cycle_count);
            }
            let _ = pb; let _ = via1_pb; // silence unused warnings
            self.atn_cmd_prev_pc = pc;
            self.atn_cmd_hits += 1;
        }

        // Detect drive exiting the $EC00-$EC80 command loop after fast-loader phase.
        // Logs the destination PC so we can confirm whether the drive enters TALK mode ($E9xx)
        // or loops back; also updates drive_in_cmd_loop each cycle unconditionally.
        {
            let in_cmd = matches!(pc, 0xEC00..=0xEC80);
            if self.cycle_count > 77_000_000 && self.drive_cmd_exit_hits < 50
                && self.drive_in_cmd_loop && !in_cmd
            {
                if let Some(b) = self.bus.via1.iec_opt() {
                    let b = b.borrow();
                    eprintln!(
                        "[DRV-EXIT-CMD] cyc={} -> ${:04X} iec={{atn:{} clk:{} dat:{}}}",
                        self.cycle_count, pc,
                        b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                    );
                } else {
                    eprintln!("[DRV-EXIT-CMD] cyc={} -> ${:04X}", self.cycle_count, pc);
                }
                self.drive_cmd_exit_hits += 1;
            }
            self.drive_in_cmd_loop = in_cmd;
        }

        // EC00-ENTRY: at the very start of the ATN command-receive loop, dump the
        // pre-filled $022B command buffer and the $7C/$026C flags so we can see
        // what bytes the C64 sent (LISTEN/OPEN/TALK/SA) during each ATN sequence.
        // Gate lowered to 77M; ATN filter prevents idle-loop flooding (limit raised to 15).
        if self.ec00_hits < 15 && self.cycle_count > 77_000_000 && pc == 0xEC00 && atn_now {
            let r = &self.bus.ram;
            let cmd: Vec<String> = (0..12usize)
                .map(|i| format!("{:02X}", r[0x22B + i]))
                .collect();
            eprintln!(
                "[EC00-ENTRY] cyc={} $7C={:02X} $026C={:02X} $6F={:02X} $70={:02X} \
                 $022B..: {}",
                self.cycle_count, r[0x7C], r[0x26C], r[0x6F], r[0x70],
                cmd.join(" ")
            );
            self.ec00_hits += 1;
        }

        // DF93-CMD: command processor entry — dump $82 (SA), $026C (TALK device),
        // and IEC state so we can confirm what command was received and whether
        // TALK was addressed to this drive.
        if self.df93_hits < 15 && self.cycle_count > 77_000_000 && pc == 0xDF93 {
            let r = &self.bus.ram;
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[DF93-CMD] cyc={} $82={:02X} $84={:02X} $85={:02X} \
                     $7C={:02X} $026C={:02X} \
                     iec={{atn:{} clk:{} dat:{}}} drv={{clk:{} dat:{}}}",
                    self.cycle_count,
                    r[0x82], r[0x84], r[0x85], r[0x7C], r[0x26C],
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                    b.drive_clk as u8, b.drive_data as u8,
                );
            } else {
                eprintln!(
                    "[DF93-CMD] cyc={} $82={:02X} $84={:02X} $85={:02X} \
                     $7C={:02X} $026C={:02X}",
                    self.cycle_count, r[0x82], r[0x84], r[0x85], r[0x7C], r[0x26C],
                );
            }
            self.post_df93_trace = 60;
            self.df93_exit_prev_pc = 0xFFFF;
            self.df93_hits += 1;
        }

        // POST-DF93: follow command-processor code path for 60 unique PCs after
        // each DF93 invocation.  Shows whether the drive enters TALK send mode
        // ($E9xx) or falls through back to the idle loop.
        if self.post_df93_trace > 0 && pc != self.df93_exit_prev_pc {
            eprintln!("[POST-DF93] pc=${:04X} cyc={}", pc, self.cycle_count);
            self.df93_exit_prev_pc = pc;
            self.post_df93_trace -= 1;
        }

        // EC98-ENTRY: confirm idle reason the first few times the drive hits $EC98.
        // $026C=0 means no TALK cmd was received; $7C≠0 would mean fast-loader hook active.
        if self.ec98_hits < 3 && self.cycle_count > 77_000_000 && pc == 0xEC98 {
            let r = &self.bus.ram;
            eprintln!(
                "[EC98-ENTRY] cyc={} $026C={:02X} $7C={:02X} $6F={:02X} $70={:02X}",
                self.cycle_count, r[0x26C], r[0x7C], r[0x6F], r[0x70]
            );
            self.ec98_hits += 1;
        }

        // E85B-ENTRY: fast-loader ATN handler entry — dump $77/$78 (TALK/LISTEN
        // address registers for this drive) plus $79/$7A (TALK/LISTEN flags set
        // during the byte-receive loop) and current IEC state.
        // $77 should be $48 (TALK device 8) and $78 should be $28 (LISTEN device 8).
        // If these are wrong the drive ignores all TALK/LISTEN commands from the C64.
        // Gate lowered to 77M: 79M ATN events must be visible.
        if self.e85b_hits < 6 && self.cycle_count > 77_000_000 && pc == 0xE85B {
            let r = &self.bus.ram;
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[E85B-ENTRY] cyc={} $77={:02X} $78={:02X} $79={:02X} $7A={:02X} \
                     $82={:02X} $84={:02X} $85={:02X} $F8={:02X} \
                     iec={{atn:{} clk:{} dat:{}}} drv={{clk:{} dat:{} atna:{}}}",
                    self.cycle_count,
                    r[0x77], r[0x78], r[0x79], r[0x7A],
                    r[0x82], r[0x84], r[0x85], r[0xF8],
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                    b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                );
            }
            self.post_e85b_trace = 80;
            self.e85b_exit_prev_pc = 0xFFFF;
            self.e85b_hits += 1;
        }

        // POST-E85B: follow E85B's code path for 80 unique PCs.
        // Shows whether TALK mode ($EA2E) or LISTEN mode ($E99C) is entered,
        // and whether the drive ultimately releases or holds the IEC bus.
        if self.post_e85b_trace > 0 && pc != self.e85b_exit_prev_pc {
            eprintln!("[POST-E85B] pc=${:04X} cyc={}", pc, self.cycle_count);
            self.e85b_exit_prev_pc = pc;
            self.post_e85b_trace -= 1;
        }

        // E8D7-ENTRY: drive enters the ATN-exit cleanup path (fired after E85B spin loop
        // exits via BPL $E8D7 = ATN released). Dumps $7A (TALK flag), $79, $82 (SA), and
        // $F2[$82] (channel data-ready flag). When $7A=1 and $F2 bit7=0, the drive skips
        // the bit-send routine and returns to idle — that is the root cause of the deadlock.
        if self.e8d7_hits < 6 && self.e85b_hits > 0 && pc == 0xE8D7 {
            let r = &self.bus.ram;
            let ch = r[0x82] as usize;
            let f2 = if ch < 16 { r[0xF2 + ch] } else { 0xFF };
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[E8D7-ENTRY] cyc={} $7A={:02X} $79={:02X} $82={:02X} F2[ch]={:02X} \
                     iec={{atn:{} clk:{} dat:{}}}",
                    self.cycle_count,
                    r[0x7A], r[0x79], r[0x82], f2,
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                );
            }
            self.e8d7_hits += 1;
        }

        // E909-ENTRY: trace every call to the data-ready check routine.
        // Shows whether disk data ($F2[$82] bit7) becomes ready for the second file.
        // After the second TALK+SA at ~84M cycles the drive calls E909 and finds no data
        // (F2 bit7=0), then releases the bus via $EA4E. If F2 bit7 later gets set (after
        // disk read completes), a *new* TALK+SA would be needed to trigger sending.
        // Gate lowered to 77M to capture 79M ATN events.
        if self.e909_hits < 40 && self.cycle_count > 77_000_000 && pc == 0xE909 {
            let r = &self.bus.ram;
            let ch = r[0x82] as usize;
            let f2 = if ch < 16 { r[0xF2 + ch] } else { 0xFF };
            let ready = (f2 & 0x80) != 0;
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[E909-ENTRY] cyc={} ch=${:02X} F2=${:02X} ready={} $7A={:02X} $79={:02X} \
                     iec={{atn:{} clk:{} dat:{}}}",
                    self.cycle_count, ch, f2, ready as u8,
                    r[0x7A], r[0x79],
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                );
            }
            self.e909_hits += 1;
        }

        // E87B-STUCK: detect when drive is stuck in the CLK-wait spin loop.
        // $E87B: LDA $1800; BPL $E8D7; AND #$04; BNE $E87B
        //   bit2 of VIA1 PB = CLK IN (inverted, 1=CLK low on bus). Loops while CLK low.
        // If the C64 fast-loader holds CLK asserted indefinitely, this loop never exits.
        // Log a one-shot message when the drive has been at $E87B for 10000+ cycles.
        if pc == 0xE87B {
            self.e87b_stuck_cycles += 1;
            if !self.e87b_stuck_logged && self.e87b_stuck_cycles >= 10_000 {
                if let Some(b) = self.bus.via1.iec_opt() {
                    let b = b.borrow();
                    let r = &self.bus.ram;
                    eprintln!(
                        "[E87B-STUCK] drive looping at $E87B for {} cycles \
                         cyc={} $7A={:02X} $79={:02X} \
                         iec={{atn:{} clk:{} dat:{}}} \
                         drv={{clk:{} dat:{} atna:{}}}",
                        self.e87b_stuck_cycles, self.cycle_count,
                        r[0x7A], r[0x79],
                        b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                        b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                    );
                }
                self.e87b_stuck_logged = true;
            }
        } else {
            // Reset counter whenever drive leaves $E87B (spin ended).
            if self.e87b_stuck_cycles > 0 {
                self.e87b_stuck_cycles = 0;
                self.e87b_stuck_logged = false;
            }
        }

        // CMD-CHAN: detect transitions in command channel (SA=15, $022B[15]).
        // No gate — capture from cycle 0 so early M-W/M-E commands are visible.
        {
            let chan15 = self.bus.ram[0x22B + 15];
            if chan15 != self.prev_cmd_chan && self.cmd_chan_hits < 100 {
                let r = &self.bus.ram;
                eprintln!(
                    "[CMD-CHAN] cyc={} $022B[15]: {:02X} -> {:02X} \
                     (drive_pc=${:04X} $7C={:02X} $82={:02X})",
                    self.cycle_count, self.prev_cmd_chan, chan15,
                    pc, r[0x7C], r[0x82],
                );
                self.prev_cmd_chan = chan15;
                self.cmd_chan_hits += 1;
            }
        }

        // CH1-SLOT: track $022C ($022B[1]) — drive buffer slot for channel 1 (SA=1).
        // FF = channel 1 closed. Changes away from FF = SA=1 file opened on drive.
        // No gate, limit 30 — directly shows whether drive ever opens channel 1.
        {
            let ch1 = self.bus.ram[0x22C];
            if ch1 != self.prev_ch1_slot && self.ch1_slot_hits < 30 {
                let r = &self.bus.ram;
                eprintln!(
                    "[CH1-SLOT] cyc={} $022C: {:02X}->{:02X} drive_pc=${:04X} \
                     $7C={:02X} $82={:02X} F2[1]={:02X} $022B[15]={:02X}",
                    self.cycle_count, self.prev_ch1_slot, ch1,
                    pc, r[0x7C], r[0x82], r[0xF3], r[0x23A],
                );
                self.prev_ch1_slot = ch1;
                self.ch1_slot_hits += 1;
            }
        }

        // DRV-CMDBUF: watch $0200 (drive command buffer byte 0) for changes.
        // M-W commands start with 'M' ($4D) at $0200; detecting this confirms
        // an M-W command arrived. No gate, limit 20.
        {
            let buf0 = self.bus.ram[0x200];
            if buf0 != self.prev_drv_cmdbuf0 && self.drv_cmdbuf_hits < 20 {
                let cmd: Vec<String> = (0..8usize)
                    .map(|i| format!("{:02X}", self.bus.ram[0x200 + i]))
                    .collect();
                eprintln!(
                    "[DRV-CMDBUF] cyc={} $0200[0]: {:02X}->{:02X} buf=[{}] drive_pc=${:04X}",
                    self.cycle_count, self.prev_drv_cmdbuf0, buf0,
                    cmd.join(" "), pc
                );
                self.prev_drv_cmdbuf0 = buf0;
                self.drv_cmdbuf_hits += 1;
            }
        }

        // Targeted trace for IEC TALK byte-send debugging.
        // Fires at the key decision points in the 1541 ROM's send loop;
        // limit raised to 5000 so second-file activity is captured.
        if self.talk_trace_hits < 5000 {
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
            let ier1 = self.bus.via1.ier();
            let ifr2 = self.bus.via2.ifr();
            let ier2 = self.bus.via2.ier();
            if let Some(b) = self.bus.via1.iec_opt() {
                let b = b.borrow();
                eprintln!(
                    "[IRQ@ACK-WAIT] pc=${:04X} cyc={} \
                     via1_ifr={:02X} ier={:02X} via2_ifr={:02X} ier={:02X} \
                     iec={{atn:{} clk:{} dat:{}}}",
                    pc, self.cycle_count,
                    ifr1, ier1, ifr2, ier2,
                    b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                );
            } else {
                eprintln!(
                    "[IRQ@ACK-WAIT] pc=${:04X} cyc={} via1_ifr={:02X} ier={:02X} via2_ifr={:02X} ier={:02X}",
                    pc, self.cycle_count, ifr1, ier1, ifr2, ier2,
                );
            }
            self.irq_ack_hits += 1;
            // Enable post-IRQ PC trace for the next 500 drive cycles.
            if self.post_irq_trace == 0 {
                self.post_irq_trace = 500;
                self.post_irq_prev_pc = 0xFFFF; // force first print
            }
        }

        // Post-IRQ trace: show drive PC evolution for N cycles after IRQ-in-ack-wait.
        if self.post_irq_trace > 0 {
            if pc != self.post_irq_prev_pc {
                eprintln!(
                    "[POST-IRQ] pc=${:04X} cyc={} irq={} ifr1={:02X} ifr2={:02X}",
                    pc, self.cycle_count,
                    self.bus.irq_asserted() as u8,
                    self.bus.via1.ifr(), self.bus.via2.ifr(),
                );
                self.post_irq_prev_pc = pc;
            }
            self.post_irq_trace -= 1;
        }

        // F2[1]-WATCH: monitor drive RAM $F3 (F2 for channel 1) for any change.
        // No gate — capture from cycle 0 so the OPEN-processing write ($DCFA sets $01)
        // and disk-read completion ($E142/$E14F set $89/$81) are both visible.
        {
            let f2_1 = self.bus.ram[0xF3];
            if f2_1 != self.prev_f2_ch1 && self.f2_ch1_hits < 20 {
                eprintln!(
                    "[F2[1]-WATCH] cyc={} $F3: {:02X}->{:02X} drive_pc=${:04X} \
                     $82={:02X} $022C={:02X}",
                    self.cycle_count, self.prev_f2_ch1, f2_1,
                    pc, self.bus.ram[0x82], self.bus.ram[0x22C],
                );
                self.prev_f2_ch1 = f2_1;
                self.f2_ch1_hits += 1;
            }
        }

        // ATN-RISE: log each c64_atn false→true edge after 88M (just after first file done).
        // Shows every new ATN command session the C64 initiates for the second file.
        if self.atn_rise_hits < 10 && self.cycle_count > 88_000_000 {
            if let Some(b) = self.bus.via1.iec_opt() {
                let atn = b.borrow().atn_low();
                if atn && !self.prev_atn_low {
                    eprintln!(
                        "[ATN-RISE] cyc={} drive_pc=${:04X} $022C={:02X} $F3={:02X} \
                         $82={:02X} $7C={:02X}",
                        self.cycle_count, pc,
                        self.bus.ram[0x22C], self.bus.ram[0xF3],
                        self.bus.ram[0x82], self.bus.ram[0x7C],
                    );
                    self.atn_rise_hits += 1;
                }
                self.prev_atn_low = atn;
            }
        }

        // DRV-ZONE: detect when drive PC exits/enters the $EC00-$EC80 IEC-command zone.
        // $EC14 (idle poll loop) lives inside this zone. An exit after ATN assertion means
        // the drive jumped to its ATN handler or byte-receive code. An entry = returned idle.
        if self.drv_zone_hits < 500 && self.cycle_count > 77_000_000 {
            let in_cmd_zone = matches!(pc, 0xEC00..=0xEC80);
            if in_cmd_zone != self.drv_zone_in_cmd {
                if let Some(b) = self.bus.via1.iec_opt() {
                    let b = b.borrow();
                    eprintln!(
                        "[DRV-ZONE] cyc={} drv=${:04X} {} iec={{atn:{} clk:{} dat:{} atna:{}}}",
                        self.cycle_count, pc,
                        if in_cmd_zone { "ENTERED $EC00-EC80" } else { "LEFT $EC00-EC80" },
                        b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                        b.drive_atna as u8,
                    );
                } else {
                    eprintln!(
                        "[DRV-ZONE] cyc={} drv=${:04X} {}",
                        self.cycle_count, pc,
                        if in_cmd_zone { "ENTERED" } else { "LEFT" },
                    );
                }
                self.drv_zone_in_cmd = in_cmd_zone;
                self.drv_zone_hits += 1;
            }
        }

        self.cpu.cycle(&mut self.bus);
        self.cycle_count = self.cycle_count.wrapping_add(1);

        // Accelerate disk state machine: run up to 512 extra drive cycles per
        // C64 cycle, but ONLY when the drive PC is in the disk-SM / scheduler
        // region ($F000-$FFFF): T1 ISR $FE67, byte-ready loop $F3BE, GCR $F78B.
        // Everything below $F000 ($C000-$EFFF) contains IEC send/receive code
        // ($E000-$EAFF), command-receive loop ($EC0A), and file/buffer management
        // — all of which must run 1:1 with the C64 to preserve bus timing.
        // Without this restriction the idle command loop ($EC0A, outside the
        // old $E000-$EAFF break range) would spin 512× per C64 cycle, making
        // the emulator 513× slower for the host while the drive does nothing useful.
        let mut extra = 512u32;
        while extra > 0 {
            extra -= 1;
            let next_pc = self.cpu.get_program_counter();
            if !matches!(next_pc, 0xF000..=0xFFFF) { break; }
            self.bus.disk.borrow_mut().rotate(1);
            if self.bus.disk.borrow().byte_ready {
                let p = self.cpu.get_status_register();
                self.cpu.set_status_register(p | 0x40);
            }
            self.bus.via1.tick(1);
            self.bus.via2.tick(1);
            if self.bus.irq_asserted() && !matches!(next_pc, 0xE9C0..=0xE9C8) {
                self.cpu.interrupt_request();
            }
            self.cpu.cycle(&mut self.bus);
            self.cycle_count = self.cycle_count.wrapping_add(1);
        }
    }

    pub fn pc(&self) -> u16 { self.cpu.get_program_counter() }
}
