// main module for C64 updates
extern crate minifb;

pub mod cpu;
pub mod d64;
pub mod memory;
pub mod opcodes;
pub mod vic;
pub mod crt;

mod cia;
mod clock;
mod io;
mod sid;
mod sid_tables;
mod vic_tables;

use debugger;
use drive_1541;
use minifb::*;
use utils;


pub const SCREEN_WIDTH:  usize = 384; // extend 20 pixels left and right for the borders
pub const SCREEN_HEIGHT: usize = 272; // extend 36 pixels top and down for the borders

// PAL clock frequency in Hz
const CLOCK_FREQ: f64 = 1.5 * 985248.0;


pub struct C64 {
    pub main_window: minifb::Window,
    pub file_to_load: String,
    pub crt_to_load: String,
    memory: memory::MemShared,
    io:     io::IO,
    clock:  clock::Clock,
    cpu:  cpu::CPUShared,
    cia1: cia::CIAShared,
    cia2: cia::CIAShared,
    vic:  vic::VICShared,
    sid:  sid::SIDShared,

    debugger: Option<debugger::Debugger>,
    powered_on: bool,
    boot_complete: bool,
    cycle_count: u32,

    // mounted D64 + arm-flag so the LOAD trap fires once per call to $F4A5
    disk_image: Option<d64::D64>,
    load_trap_armed: bool,

    // if set, after the CLI-supplied PRG is poked into RAM we update BASIC
    // text pointers ($2D/$2E/$2F/$30/$31/$32) and stuff "RUN<CR>" into the
    // C64 keyboard buffer so BASIC executes the program without user input.
    auto_run: bool,

    // if set, stuff LOAD"*",8 + RETURN into the keyboard buffer once BASIC
    // warm-start is reached, so the drive LOAD is triggered automatically
    // (used for headless testing of truedrive).
    autoload: bool,
    autoload_done: bool,

    // Level-3 1541 emulation. When `truedrive` is true and the ROM is found,
    // the drive's 6502 runs alongside the C64's CPU and the KERNAL LOAD trap
    // stays out of the way (so the C64 talks to the drive "for real").
    truedrive: bool,
    drive: drive_1541::Drive1541,

    // tracedrive: log IEC bus transitions + a periodic drive PC.
    tracedrive: bool,
    iec_for_trace: Option<drive_1541::IecBusShared>,
    prev_trace_state: u8,
    trace_pc_counter: u32,
    // one-shot: dump C64 RAM at $6160-$616F the first time C64 PC hits that range
    fastload_dump_done: bool,
    // per-instruction trace of unique PCs while in $6000-$6300 (fast-loader body)
    fastload_loop_hits: u32,
    fastload_loop_prev_pc: u16,
    // per-instruction trace of unique PCs while in $C000-$D000 (game init / pre-loader code)
    game_init_hits: u32,
    game_init_prev_pc: u16,
    // Period-duration tracker for C64 holding ATN+CLK simultaneously.
    dd00_atn_clk_hits: u32,
    dd00_atn_clk_active: bool,
    dd00_atn_clk_start: u32,
    dd00_atn_clk_long_logged: bool,
    // detect drive PC landing in $0000-$07FF (drive-side fast-loader RAM code)
    drv_ram_hits: u32,
    // Watch $033E: log the first time it goes from 0 to non-zero. The C64 fast-loader
    // at $6160 (LDA $033E; BRK loop) polls this byte waiting for a "drive ready" signal.
    // On real hardware something (interrupt handler? drive CLK signal?) sets this.
    mem_033e_prev: u8,
    mem_033e_hits: u32,
    // Previous IEC DATA state — used to detect falling edge for CIA1 FLAG interrupt.
    prev_iec_data_low: bool,
    cia1_flag_hits: u32,
    // CPU port $01 banking tracker (active after cyc=80M)
    cpu_port_prev: u8,
    cpu_port_hits: u32,
    // Pre-fastload IEC state change tracker (cyc 80M–fastload_dump_done)
    pre_fastload_iec_hits: u32,
    pre_fastload_iec_prev: u8,
    // Pre-fastload PC trace for $0100-$03FF (cyc 105M–fastload_dump_done)
    pre_fastload_low_pc_hits: u32,
    pre_fastload_low_pc_prev: u16,
    // CIA2 PRA change tracker (active after cyc=80M) — direct confirmation
    // that the game writes $DD00 to assert IEC CLK/DATA/ATN.
    cia2_pra_prev: u8,
    cia2_pra_hits: u32,
    // Late CIA1 FLAG tracker — log FLAG events after cyc=80M (the 20-hit
    // early logger is exhausted by the KERNAL load; we need to see the
    // self-test event at ~97M and any events during the fast-loader wait).
    late_cia1_flag_hits: u32,
    // PC trace for the CIA1 FLAG self-test area ($0800-$08FF)
    pre_page8_pc_prev: u16,
    pre_page8_pc_hits: u32,
    // ATN sequence tracker: fires on every c64_atn 0→1 edge, no cycle gate.
    // Reveals every LISTEN/TALK sequence from boot — first-file + any second-file open.
    atn_seq_hits: u32,
    prev_c64_atn: bool,
    // CIA1 IER change tracker: log every write to CIA1's interrupt enable register.
    // Bit4=FLAG. Needed to diagnose why cia1_ier=$00 at fast-loader entry (cyc=128M).
    cia1_ier_prev: u8,
    cia1_ier_hits: u32,
    // HW IRQ/BRK vector $FFFE/$FFFF tracker: detect when the decompressor overwrites
    // these addresses. With KERNAL banked out, $FFFE/$FFFF read from RAM; if the
    // decompressor stores $6161 there instead of a proper handler address, BRK loops.
    mem_fffe_prev: [u8; 2],
    mem_fffe_hits: u32,
    // $6160-$616F write tracker: detect if the game's custom loader installs an IRQ
    // handler at $6161 (the BRK vector destination). On real hardware the handler
    // should replace the sprite data bytes at $6160-$616F before the BRK loop runs.
    mem_6160_snap: [u8; 16],
    mem_6160_hits: u32,
    // Source buffer tracker for decompressor: watch $CB60-$CB70 (the area around
    // $CB67/$CB68 — the source bytes the decompressor copies to $FFFE/$FFFF).
    // Should contain $66/$FE for BRK vec $FE66; if $FF/$61, the custom loader bug
    // delivers wrong bytes for the IRQ vector.
    mem_cb60_snap: [u8; 16],
    mem_cb60_hits: u32,
    // Trace unique C64 PCs in $0000-$03FF (low-page custom receive routines) to
    // understand the IEC receive protocol that fills the source buffer.
    recv_pc_prev: u16,
    recv_pc_hits: u32,
    // Milestone snapshots of $CA60-$CA6F at specific cycle counts.
    // Bits: 0=50M done, 1=80M done, 2=100M done, 3=120M done.
    ca60_milestone_done: u8,
    // CA67/CA68 watchpoint: fire whenever $CA67 or $CA68 changes; log IEC state + code dump.
    // These are the decompressor source bytes that determine the BRK vector in $FFFE/$FFFF.
    ca67_prev: u8,
    ca68_prev: u8,
    ca_watch_hits: u32,
    // Page-4 PC trace ($0400-$04FF) after cyc=90M: catch the decompressor/receive code
    // at $045E that writes wrong $FF/$61 into $CA67/$CA68.
    page4_pc_prev: u16,
    page4_pc_hits: u32,
    // SLOOP-WRITE: watch every STA ($FE),Y at $0472 (second inner loop of pass 4).
    // This loop writes the source buffer at $CA5A+Y from ($F8/$F9)+Y.
    // The wrong $FF/$61 at $CA5F/$CA60 originates here.
    sloop_watch_hits: u32,
    // ADDR4293-WATCH: fire when $4293 changes value (gate >80M, limit 10).
    // $4293 is pass-4's source byte that feeds $CA5F (Y=5 in the $CA5A iter).
    // Known $5F at 80M (raw disk data). Passes 1-3 overwrite it before pass-4 runs.
    // The wrong value ($FF or $43) is what produces the bad IRQ vector at $FFFE.
    mem_4293_prev: u8,
    mem_4293_hits: u32,
    // BRK-PATCH: one-shot flag. After the decompressor finishes (~115M cycles),
    // force $FFFE/$FFFF=$FE66 in RAM to test whether wrong BRK vector is the
    // sole cause of the fast-loader hang (fires once at cycle 115_500_000).
    brk_patch_done: bool,
    // IECIN-WATCH: log first N entries into KERNAL IECIN ($EE56) after 89M cycles.
    // Gate at 89M is past the first-file transfer (19.78M-87.5M) so only post-load
    // IECIN calls are captured.  0 hits = C64 never reaches IECIN for file 2.
    iecin_watch_hits: u32,
    // IECIN-HANG: one-shot detection when C64 spins at KERNAL bit-poll ($E5C0–$E5E0)
    // for more than 2M consecutive cycles.  Fires once and dumps:
    //   • spin cycle count, C64 PC, drive PC, IEC bus state
    //   • whether KERNAL ROM is on/off ($0001 bit1)
    //   • the 16 bytes at $E5C0 as seen by the CPU (ROM if kernal_on=1, RAM if 0)
    iecin_hang_cycles: u32,
    iecin_hang_logged: bool,
}

impl C64 {
    pub fn new(window_scale: Scale, debugger_on: bool, prg_to_load: &str, crt_to_load: &str, d64_to_mount: &str, auto_run: bool, truedrive: bool, tracedrive: bool, autoload: bool) -> C64 {
        let memory = memory::Memory::new_shared();
        let vic    = vic::VIC::new_shared();
        let cia1   = cia::CIA::new_shared(true);
        let cia2   = cia::CIA::new_shared(false);
        let cpu    = cpu::CPU::new_shared();
        let sid    = sid::SID::new_shared();

        let disk_image = if !d64_to_mount.is_empty() {
            match d64::D64::open(d64_to_mount) {
                Ok(img) => {
                    println!("Mounted D64: {}", d64_to_mount);
                    for entry in img.directory().iter().take(32) {
                        let type_char = match entry.file_type & 0x0F {
                            1 => "SEQ", 2 => "PRG", 3 => "USR", 4 => "REL", _ => "???",
                        };
                        println!("  {:>4} \"{}\" {}", entry.size_sectors, entry.name_ascii(), type_char);
                    }
                    Some(img)
                }
                Err(e) => {
                    println!("Failed to mount {}: {}", d64_to_mount, e);
                    None
                }
            }
        } else {
            None
        };

        let mut c64 = C64 {
            main_window: Window::new("Rust64", SCREEN_WIDTH, SCREEN_HEIGHT, WindowOptions { scale: window_scale, ..Default::default() }).unwrap(),
            file_to_load: String::from(prg_to_load),
            crt_to_load: String::from(crt_to_load),
            memory: memory.clone(), // shared system memory (RAM, ROM, IO registers)
            io:     io::IO::new(),
            clock:  clock::Clock::new(CLOCK_FREQ),
            cpu:  cpu.clone(),
            cia1: cia1.clone(),
            cia2: cia2.clone(),
            vic:  vic.clone(),
            sid:  sid.clone(),
            debugger: if debugger_on { Some(debugger::Debugger::new()) } else { None },
            powered_on: false,
            boot_complete: false,
            cycle_count: 0,
            disk_image,
            load_trap_armed: true,
            auto_run,
            autoload,
            autoload_done: false,
            truedrive,
            // Only try to load the 1541 ROM when the user opted into truedrive;
            // otherwise it would spam a "ROM not found" warning every boot.
            drive: if truedrive {
                drive_1541::Drive1541::new("rom/1541.rom")
            } else {
                drive_1541::Drive1541::disabled()
            },
            tracedrive,
            iec_for_trace: None,
            prev_trace_state: 0,
            trace_pc_counter: 0,
            fastload_dump_done: false,
            fastload_loop_hits: 0,
            fastload_loop_prev_pc: 0xFFFF,
            game_init_hits: 0,
            game_init_prev_pc: 0xFFFF,
            dd00_atn_clk_hits: 0,
            dd00_atn_clk_active: false,
            dd00_atn_clk_start: 0,
            dd00_atn_clk_long_logged: false,
            drv_ram_hits: 0,
            mem_033e_prev: 0,
            mem_033e_hits: 0,
            prev_iec_data_low: false,
            cia1_flag_hits: 0,
            cpu_port_prev: 0xFF,
            cpu_port_hits: 0,
            pre_fastload_iec_hits: 0,
            pre_fastload_iec_prev: 0xFF,
            pre_fastload_low_pc_hits: 0,
            pre_fastload_low_pc_prev: 0xFFFF,
            cia2_pra_prev: 0xFF,
            cia2_pra_hits: 0,
            late_cia1_flag_hits: 0,
            pre_page8_pc_prev: 0xFFFF,
            pre_page8_pc_hits: 0,
            atn_seq_hits: 0,
            prev_c64_atn: false,
            cia1_ier_prev: 0,
            cia1_ier_hits: 0,
            mem_fffe_prev: [0x48, 0xFF], // sentinel: KERNAL ROM value ($FF48)
            mem_fffe_hits: 0,
            mem_6160_snap: [0u8; 16],
            mem_6160_hits: 0,
            mem_cb60_snap: [0u8; 16],
            mem_cb60_hits: 0,
            recv_pc_prev: 0xFFFF,
            recv_pc_hits: 0,
            ca60_milestone_done: 0,
            ca67_prev: 0,
            ca68_prev: 0,
            ca_watch_hits: 0,
            page4_pc_prev: 0xFFFF,
            page4_pc_hits: 0,
            sloop_watch_hits: 0,
            mem_4293_prev: 0xFF,  // sentinel: force a log on the first cycle >80M to confirm tracker runs
            mem_4293_hits: 0,
            brk_patch_done: false,
            iecin_watch_hits: 0,
            iecin_hang_cycles: 0,
            iecin_hang_logged: false,
        };

        // If the drive is actually running, hand both ends a shared IEC bus
        // and (if a D64 is mounted) give the drive its own copy of the disk
        // image so the read head sees real bytes.
        if truedrive && c64.drive.enabled {
            let iec = drive_1541::IecBus::new_shared();
            c64.cia2.borrow_mut().set_iec(iec.clone());
            c64.drive.set_iec(iec.clone());
            // Always store IEC ref so the watchdog can print bus state without tracedrive.
            c64.iec_for_trace = Some(iec.clone());
            if tracedrive {
                c64.drive.set_trace(true);
            }
            if let Some(img) = &c64.disk_image {
                println!("truedrive: mounting d64.");
                c64.drive.mount_d64(img.clone());
            } else {
                println!("truedrive: no D64 mounted; drive head will see no data.");
            }
        } else if truedrive {
            println!("truedrive: ROM missing or invalid; falling back to KERNAL LOAD trap.");
        }

        c64.main_window.set_position(75, 20);

        // cyclic dependencies are not possible in Rust (yet?), so we have
        // to resort to setting references manually
        c64.cia1.borrow_mut().set_references(memory.clone(), cpu.clone(), vic.clone());
        c64.cia2.borrow_mut().set_references(memory.clone(), cpu.clone(), vic.clone());
        c64.vic.borrow_mut().set_references(memory.clone(), cpu.clone());
        c64.sid.borrow_mut().set_references(memory.clone());
        c64.cpu.borrow_mut().set_references(memory.clone(), vic.clone(), cia1.clone(), cia2.clone(), sid.clone());

        drop(memory);
        drop(cia1);
        drop(cia2);
        drop(vic);
        drop(cpu);
        drop(sid);

        c64
    }


    pub fn reset(&mut self) {
        self.memory.borrow_mut().reset();
        self.cpu.borrow_mut().reset();
        self.cia1.borrow_mut().reset();
        self.cia2.borrow_mut().reset();
        self.sid.borrow_mut().reset();
        if self.truedrive { self.drive.reset(); }
    }


    pub fn run(&mut self) {
        // attempt to load a program supplied with command line
        if !self.powered_on {
            // $FCE2 is the power-on reset routine, which searches for and starts
            // a cartridge amongst other things. The cartridge must be loaded here
            self.powered_on = self.cpu.borrow_mut().pc == 0xFCE2;
            if self.powered_on {
                let crt_file = &self.crt_to_load.to_owned()[..];
                if crt_file.len() > 0 {
                    let crt = crt::Crt::from_filename(crt_file).unwrap();
                    println!("{:?}", crt);
                    crt.load_into_memory(self.memory.borrow_mut());
                }
            }
        }

        if !self.boot_complete {
            // $A480 is the BASIC warm start sequence - safe to assume we can load a cmdline program now
            self.boot_complete = self.cpu.borrow_mut().pc == 0xA480;

            if self.boot_complete {
                let prg_file = &self.file_to_load.to_owned()[..];

                if prg_file.len() > 0 {
                    self.boot_complete = true; self.load_prg(prg_file);
                }

                // Inject LOAD"*",8<CR> into the keyboard buffer so the drive
                // LOAD is triggered without any human input. 10 bytes exactly.
                if self.autoload && !self.autoload_done {
                    self.autoload_done = true;
                    let cmd: &[u8] = b"LOAD\"*\",8\r";
                    let mut mem = self.memory.borrow_mut();
                    for (i, &b) in cmd.iter().enumerate() {
                        mem.write_byte(0x0277 + i as u16, b);
                    }
                    mem.write_byte(0x00C6, cmd.len() as u8);
                    println!("autoload: stuffed LOAD\"*\",8<CR> into keyboard buffer");
                }
            }
        }

        // KERNAL LOAD trap: when CPU is about to execute the routine at $F4A5 with
        // device 8 and we have a D64 mounted, satisfy the load ourselves and RTS.
        // We re-arm the trap as soon as PC moves away to avoid firing twice during
        // the multi-cycle instruction at $F4A5.
        //
        // Suppressed when --truedrive is on AND the drive ROM actually loaded;
        // in that mode the C64 talks to the real 1541 emulation instead.
        let use_load_trap = !(self.truedrive && self.drive.enabled);
        if use_load_trap {
            let (pc, at_fetch) = {
                let c = self.cpu.borrow();
                (c.pc, matches!(c.state, cpu::CPUState::FetchOp))
            };
            if pc == 0xF4A5 && at_fetch && self.load_trap_armed && self.disk_image.is_some() {
                if self.handle_load_trap() {
                    self.load_trap_armed = false;
                }
            }
            if pc != 0xF4A5 { self.load_trap_armed = true; }
        }

        // main C64 update - use the clock to time all the operations
        if self.clock.tick() {
            let mut should_trigger_vblank = false;

            if self.vic.borrow_mut().update(self.cycle_count, &mut should_trigger_vblank) {
                self.sid.borrow_mut().update();
            }

            self.cia1.borrow_mut().process_irq();
            self.cia2.borrow_mut().process_irq();
            self.cia1.borrow_mut().update();
            self.cia2.borrow_mut().update();

            // snapshot C64 PC for CIA1 timer-B trace (can't borrow cpu inside cpu.update)
            self.cia1.borrow_mut().trace_pc = self.cpu.borrow().pc;
            self.cpu.borrow_mut().update(self.cycle_count);

            // Tick the 1541 drive once per C64 cycle.
            //
            // Special case: when the VIC is stealing cycles from the C64 CPU
            // (ba_low=true) AND the CPU is inside the KERNAL serial bit-receive
            // loop ($EE56–$EE76), we also pause the drive.  Without this, the
            // drive advances through its ~30-cycle CLK=H window while the CPU
            // is halted, so the CPU misses the pulse and the IEC transfer
            // deadlocks.  Limiting the pause to the KERNAL range avoids
            // disturbing fast-loaders, which run their own code outside that
            // range and manage their own VIC-steal tolerance.
            if self.truedrive {
                let (ba_low, pc) = {
                    let cpu = self.cpu.borrow();
                    (cpu.ba_low, cpu.pc)
                };
                let in_kernal_iec_recv = pc >= 0xEE56 && pc <= 0xEE76;
                if !ba_low || !in_kernal_iec_recv {
                    self.drive.step();
                    // CIA1 FLAG pin is wired to IEC DATA IN on real C64 hardware.
                    // A falling edge (DATA going low) sets ICR bit 4 and, if IER
                    // bit 4 is enabled, fires a CIA1 IRQ to the CPU.
                    if let Some(ref iec) = self.iec_for_trace {
                        let data_low_now = iec.borrow().data_low();
                        if data_low_now && !self.prev_iec_data_low {
                            // Falling edge: DATA went low → CIA1 FLAG (ICR bit 4).
                            if self.cia1_flag_hits < 20 {
                                let ier = self.cia1.borrow().ier();
                                let icr = self.cia1.borrow().icr_peek();
                                let drv_pc = self.drive.pc();
                                eprintln!(
                                    "[CIA1-FLAG] cyc={} DATA↓ c64=${:04X} drv=${:04X} \
                                     cia1_ier=${:02X} cia1_icr=${:02X}",
                                    self.cycle_count, pc, drv_pc, ier, icr
                                );
                                self.cia1_flag_hits += 1;
                            }
                            // Separate late-phase logger: catch FLAG events after 80M
                            // (the early 20-hit logger is exhausted by the KERNAL load).
                            if self.cycle_count > 80_000_000 && self.late_cia1_flag_hits < 30 {
                                let ier = self.cia1.borrow().ier();
                                let icr = self.cia1.borrow().icr_peek();
                                let c2_pra = self.cia2.borrow().pra();
                                let c2_ddra = self.cia2.borrow().ddra();
                                if let Some(ref iec) = self.iec_for_trace {
                                    let b = iec.borrow();
                                    eprintln!(
                                        "[CIA1-FLAG-LATE] cyc={} DATA↓ c64=${:04X} drv=${:04X} \
                                         cia1_ier=${:02X} cia1_icr=${:02X} \
                                         c64_atn:{} c64_clk:{} c64_dat:{} auto_dat:{} \
                                         c2_pra=${:02X} c2_ddra=${:02X}",
                                        self.cycle_count, pc, self.drive.pc(),
                                        ier, icr,
                                        b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                                        (b.atn_low() && !b.drive_atna) as u8,
                                        c2_pra, c2_ddra
                                    );
                                }
                                self.late_cia1_flag_hits += 1;
                            }
                            let triggered = self.cia1.borrow_mut().trigger_irq(0x10);
                            if triggered {
                                self.cpu.borrow_mut().set_cia_irq(true);
                            }
                        }
                        self.prev_iec_data_low = data_low_now;
                    }
                    if self.tracedrive { self.trace_step(); }
                }
                // One-shot: first time C64 lands in the fast-loader range ($6000-$6300),
                // dump 128 bytes from $6100 and 32 bytes from $C840 (game-init area),
                // plus $033E value (polled by LDA $033E), DD00, and IEC bus state.
                if !self.fastload_dump_done && (0x6000..=0x6300).contains(&pc) {
                    self.fastload_dump_done = true;
                    let drv_pc = self.drive.pc();
                    let mut mem = self.memory.borrow_mut();
                    let dd00 = mem.read_byte(0xDD00);
                    let mem_033e = mem.read_byte(0x033E);
                    let brk_lo = mem.read_byte(0x0316);
                    let brk_hi = mem.read_byte(0x0317);
                    let brk_vec = (brk_hi as u16) << 8 | brk_lo as u16;
                    // 128-byte dump from $6100
                    let mut lo_bytes = [0u8; 128];
                    for (i, b) in lo_bytes.iter_mut().enumerate() {
                        *b = mem.read_byte(0x6100 + i as u16);
                    }
                    // 32-byte dump from $C840 (game init code seen at $C858)
                    let mut hi_bytes = [0u8; 32];
                    for (i, b) in hi_bytes.iter_mut().enumerate() {
                        *b = mem.read_byte(0xC840 + i as u16);
                    }
                    drop(mem);
                    eprintln!("[FASTLOAD-ENTRY] cyc={} c64=${:04X} drv=${:04X} DD00=${:02X} $033E=${:02X} BRK_VEC=${:04X}",
                              self.cycle_count, pc, drv_pc, dd00, mem_033e, brk_vec);
                    eprint!("[FASTLOAD-ENTRY] ram@6100:");
                    for (i, b) in lo_bytes.iter().enumerate() {
                        if i % 16 == 0 { eprint!("\n  {:04X}:", 0x6100 + i); }
                        eprint!(" {:02X}", b);
                    }
                    eprintln!();
                    eprint!("[FASTLOAD-ENTRY] ram@C840:");
                    for b in &hi_bytes { eprint!(" {:02X}", b); }
                    eprintln!();
                    let cia1_ier = self.cia1.borrow().ier();
                    let cia1_icr = self.cia1.borrow().icr_peek();
                    let irq_vec = {
                        let mut m = self.memory.borrow_mut();
                        let lo = m.read_byte(0x0314);
                        let hi = m.read_byte(0x0315);
                        (hi as u16) << 8 | lo as u16
                    };
                    eprintln!("[FASTLOAD-ENTRY] cia1_ier=${:02X} cia1_icr=${:02X} irq_vec=${:04X}",
                              cia1_ier, cia1_icr, irq_vec);
                    if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        eprintln!("[FASTLOAD-ENTRY] iec atn:{} clk:{} dat:{} c64_atn:{} c64_clk:{} c64_dat:{} drv_clk:{} drv_dat:{}",
                            b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                            b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                            b.drive_clk as u8, b.drive_data as u8);
                    }
                    // Dump banking state, HW IRQ vector, and BRK handler code.
                    // If KERNAL is active ($01 bit1=1), $FFFE→$FF48 and $FE66 is
                    // the standard PLA/TAY/PLA/TAX/PLA/RTI handler. Custom code at
                    // those addresses means KERNAL is banked out.
                    let (cpu_port, hw_irq_lo, hw_irq_hi,
                         brk_handler_bytes, ff48_bytes) = {
                        let mut m = self.memory.borrow_mut();
                        let cpu_port = m.read_byte(0x0001);
                        let hw_irq_lo = m.read_byte(0xFFFE);
                        let hw_irq_hi = m.read_byte(0xFFFF);
                        let mut brk_h = [0u8; 16];
                        for (i, b) in brk_h.iter_mut().enumerate() {
                            *b = m.read_byte(brk_vec.wrapping_add(i as u16));
                        }
                        let mut ff48 = [0u8; 16];
                        for (i, b) in ff48.iter_mut().enumerate() {
                            *b = m.read_byte(0xFF48u16.wrapping_add(i as u16));
                        }
                        (cpu_port, hw_irq_lo, hw_irq_hi, brk_h, ff48)
                    };
                    let hw_irq_vec = (hw_irq_hi as u16) << 8 | hw_irq_lo as u16;
                    eprintln!("[FASTLOAD-ENTRY] $01={:02X} kernal_hiram={} hw_irq_vec=${:04X}",
                              cpu_port, (cpu_port >> 1) & 1, hw_irq_vec);
                    eprint!("[FASTLOAD-ENTRY] brk_handler@{:04X}:", brk_vec);
                    for b in &brk_handler_bytes { eprint!(" {:02X}", b); }
                    eprintln!();
                    eprint!("[FASTLOAD-ENTRY] ff48@FF48:");
                    for b in &ff48_bytes { eprint!(" {:02X}", b); }
                    eprintln!();
                    // Dump $FE60-$FE7F: with KERNAL out, this is RAM.
                    // The game should have installed its IRQ handler here (via the
                    // decompressor writing to $FE00 page). If it's $00-filled, the
                    // decompressor wrote wrong data and the fast-loader will hang.
                    let fe60_bytes: [u8; 32] = {
                        let mut arr = [0u8; 32];
                        let mut m = self.memory.borrow_mut();
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0xFE60u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    eprint!("[FASTLOAD-ENTRY] ram@FE60:");
                    for b in &fe60_bytes { eprint!(" {:02X}", b); }
                    eprintln!();
                    // Dump $0410-$042F (page-4 fast-loader setup; $0424 clears CIA1 IER).
                    let page4_bytes: [u8; 32] = {
                        let mut arr = [0u8; 32];
                        let mut m = self.memory.borrow_mut();
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0x0410u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    eprint!("[FASTLOAD-ENTRY] ram@0410:");
                    for b in &page4_bytes { eprint!(" {:02X}", b); }
                    eprintln!();
                    // Dump $0100-$017F (stack page / custom receive stub area).
                    // Custom loaders often install M-W'd IEC receive routines here.
                    let page1_bytes: [u8; 128] = {
                        let mut arr = [0u8; 128];
                        let mut m = self.memory.borrow_mut();
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0x0100u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    eprint!("[FASTLOAD-ENTRY] ram@0100:");
                    for (i, b) in page1_bytes.iter().enumerate() {
                        if i % 16 == 0 { eprint!("\n  {:04X}:", 0x0100 + i); }
                        eprint!(" {:02X}", b);
                    }
                    eprintln!();
                    // Dump $CA60-$CA6F directly at fastload entry so we know final state.
                    let ca60_bytes: [u8; 16] = {
                        let mut arr = [0u8; 16];
                        let mut m = self.memory.borrow_mut();
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0xCA60u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    eprint!("[FASTLOAD-ENTRY] ram@CA60:");
                    for b in &ca60_bytes { eprint!(" {:02X}", b); }
                    eprintln!("  (CA60-snap-hits={})", self.mem_cb60_hits);
                    eprint!("[FASTLOAD-ENTRY] CA60-snap:");
                    for b in &self.mem_cb60_snap { eprint!(" {:02X}", b); }
                    eprintln!();

                    // BRK-PATCH (embedded here): fires exactly once, at the cycle the
                    // fast-loader opcode fetch at $6160 first advances pc to $6161.
                    // At this point LDA $033E has only fetched its opcode; BRK at $6163
                    // has NOT yet executed.  Patching now gives BRK the correct target.
                    if !self.brk_patch_done {
                        let (fffe_before, ffff_before) = {
                            let mut m = self.memory.borrow_mut();
                            (m.read_byte(0xFFFE), m.read_byte(0xFFFF))
                        };
                        {
                            let mut m = self.memory.borrow_mut();
                            m.write_byte(0xFFFE, 0x66);
                            m.write_byte(0xFFFF, 0xFE);
                        }
                        let (fffe_after, ffff_after) = {
                            let mut m = self.memory.borrow_mut();
                            (m.read_byte(0xFFFE), m.read_byte(0xFFFF))
                        };
                        let cpu_port = self.memory.borrow_mut().read_byte(0x0001);
                        eprintln!(
                            "[BRK-PATCH] cyc={} $FFFE: {:02X}->{:02X}  $FFFF: {:02X}->{:02X} \
                             $01=${:02X} c64=${:04X} (BRK not yet executed)",
                            self.cycle_count,
                            fffe_before, fffe_after, ffff_before, ffff_after,
                            cpu_port, pc
                        );
                        self.brk_patch_done = true;
                    }
                }

                // Per-instruction trace while C64 is in the fast-loader code range ($6000-$6300).
                // Logs unique PCs with the 3 bytes at that address and current DD00.
                // Limited to 100 entries to keep the log manageable.
                if self.fastload_dump_done && self.fastload_loop_hits < 100
                    && (0x6000..=0x6300).contains(&pc) && pc != self.fastload_loop_prev_pc
                {
                    let mut mem = self.memory.borrow_mut();
                    let dd00 = mem.read_byte(0xDD00);
                    let mem_033e = mem.read_byte(0x033E);
                    let b0 = mem.read_byte(pc);
                    let b1 = mem.read_byte(pc.wrapping_add(1));
                    let b2 = mem.read_byte(pc.wrapping_add(2));
                    drop(mem);
                    let (dat_low, drv_dat) = if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        (b.data_low() as u8, b.drive_data as u8)
                    } else { (0, 0) };
                    let cia1_ier = self.cia1.borrow().ier();
                    let cia1_icr = self.cia1.borrow().icr_peek();
                    eprintln!("[FASTLOAD-LOOP] pc=${:04X} bytes={:02X} {:02X} {:02X} DD00=${:02X} \
                               $033E=${:02X} dat:{} drv_dat:{} cia1_ier=${:02X} cia1_icr=${:02X}",
                              pc, b0, b1, b2, dd00, mem_033e, dat_low, drv_dat, cia1_ier, cia1_icr);
                    self.fastload_loop_prev_pc = pc;
                    self.fastload_loop_hits += 1;
                }

                // Per-instruction trace for game-init code in $C000-$D000.
                // Fires before the fast-loader entry to show what the game does to open
                // the second file.  Limited to 100 unique PCs.
                if !self.fastload_dump_done && self.game_init_hits < 100
                    && (0xC000..=0xD000).contains(&pc) && pc != self.game_init_prev_pc
                {
                    let mut mem = self.memory.borrow_mut();
                    let b0 = mem.read_byte(pc);
                    let b1 = mem.read_byte(pc.wrapping_add(1));
                    let b2 = mem.read_byte(pc.wrapping_add(2));
                    drop(mem);
                    eprintln!("[GAME-INIT] pc=${:04X} bytes={:02X} {:02X} {:02X}",
                              pc, b0, b1, b2);
                    self.game_init_prev_pc = pc;
                    self.game_init_hits += 1;
                }

                // $033E WATCHER: log every change to this byte.
                // The fast-loader at $6160 polls it via LDA $033E. Watching transitions
                // reveals WHO sets it and when, so we can understand what signal breaks
                // the fast-loader's wait loop on real hardware.
                if self.mem_033e_hits < 20 {
                    let v = self.memory.borrow_mut().read_byte(0x033E);
                    if v != self.mem_033e_prev {
                        let drv_pc = self.drive.pc();
                        if let Some(ref iec) = self.iec_for_trace {
                            let b = iec.borrow();
                            eprintln!(
                                "[MEM-033E] cyc={} $033E: {:02X} -> {:02X} c64=${:04X} drv=${:04X} \
                                 iec={{atn:{} clk:{} dat:{}}}",
                                self.cycle_count, self.mem_033e_prev, v, pc, drv_pc,
                                b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                            );
                        }
                        self.mem_033e_prev = v;
                        self.mem_033e_hits += 1;
                    }
                }

                // DD00-ATN+CLK PERIOD TRACKER
                // Tracks periods where C64 holds ATN+CLK simultaneously.
                // Standard IEC: C64 briefly holds both at ATN-start (< a few hundred cycles)
                // then releases CLK so the drive can receive bytes via $E87B.
                // Fast-loader deadlock: C64 holds CLK indefinitely while ATN is asserted,
                // blocking $E87B's "BNE $E87B" spin forever.
                // Per-cycle counting exhausted 200 hits on normal KERNAL ATN sequences and
                // never reached the fast-loader phase. Period tracking logs one entry per
                // period: KERNAL periods end quickly (silent), fast-loader appears as STUCK.
                if let Some(ref iec) = self.iec_for_trace {
                    let b = iec.borrow();
                    let both = b.c64_atn && b.c64_clk;
                    if both && !self.dd00_atn_clk_active {
                        // Rising edge: record start of new ATN+CLK period.
                        self.dd00_atn_clk_active = true;
                        self.dd00_atn_clk_start = self.cycle_count;
                        self.dd00_atn_clk_long_logged = false;
                    } else if self.dd00_atn_clk_active {
                        let duration = self.cycle_count.wrapping_sub(self.dd00_atn_clk_start);
                        if !both {
                            // Falling edge: period ended — log all periods.
                            if self.dd00_atn_clk_hits < 40 {
                                eprintln!(
                                    "[DD00-ATN+CLK END] start={} end={} dur={} c64=${:04X}{}",
                                    self.dd00_atn_clk_start, self.cycle_count, duration, pc,
                                    if duration > 10_000 { " *** LONG ***" } else { "" }
                                );
                                self.dd00_atn_clk_hits += 1;
                            }
                            self.dd00_atn_clk_active = false;
                        } else if !self.dd00_atn_clk_long_logged && duration >= 50_000 {
                            // Still held and extremely long: deadlock pattern.
                            let drv_pc = self.drive.pc();
                            eprintln!(
                                "[DD00-ATN+CLK STUCK] start={} dur_so_far={} c64=${:04X} \
                                 drv=${:04X} iec={{clk:{} dat:{} drv_clk:{} drv_dat:{}}}",
                                self.dd00_atn_clk_start, duration, pc, drv_pc,
                                b.clk_low() as u8, b.data_low() as u8,
                                b.drive_clk as u8, b.drive_data as u8,
                            );
                            self.dd00_atn_clk_long_logged = true;
                            self.dd00_atn_clk_hits += 1;
                        }
                    }
                }

                // DRV-RAM: fire when drive PC is in $0000-$07FF (full 1541 RAM).
                // Covers zero-page ($0000-$00FF) where M-E'd stubs sometimes land,
                // plus pages 1-7 ($0100-$07FF) where M-W'd routines typically live.
                if self.drv_ram_hits < 50 {
                    let drv_pc = self.drive.pc();
                    if drv_pc <= 0x07FF {
                        if let Some(ref iec) = self.iec_for_trace {
                            let b = iec.borrow();
                            eprintln!(
                                "[DRV-RAM] cyc={} drv=${:04X} c64=${:04X} \
                                 iec={{atn:{} clk:{} dat:{}}}",
                                self.cycle_count, drv_pc, pc,
                                b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                            );
                        } else {
                            eprintln!(
                                "[DRV-RAM] cyc={} drv=${:04X} c64=${:04X}",
                                self.cycle_count, drv_pc, pc,
                            );
                        }
                        self.drv_ram_hits += 1;
                    }
                }

                // CPU-PORT: track CPU port $01 writes after cyc=75M.
                // Bit 1 (HIRAM) controls KERNAL ROM — when clear, $E000-$FFFF is RAM.
                // Gate lowered to 75M so we catch banking changes right at load-end (~78M).
                if self.cycle_count > 75_000_000 && self.cpu_port_hits < 60 {
                    let v = self.memory.borrow_mut().read_byte(0x0001);
                    if v != self.cpu_port_prev {
                        eprintln!(
                            "[CPU-PORT] cyc={} $01: {:02X}->{:02X} c64=${:04X} \
                             loram={} hiram={} charen={}",
                            self.cycle_count, self.cpu_port_prev, v, pc,
                            v & 1, (v >> 1) & 1, (v >> 2) & 1
                        );
                        self.cpu_port_prev = v;
                        self.cpu_port_hits += 1;
                    }
                }

                // PRE-IEC: log every IEC bus state change in the 77M–fastload window.
                // Gate lowered to 77M so the ~79M ATN events (second file open) are captured.
                // Shows whether C64 asserts CLK/DATA to trigger the drive before the
                // BRK poll loop, and whether the drive asserts DATA in response.
                if !self.fastload_dump_done && self.cycle_count >= 77_000_000
                    && self.pre_fastload_iec_hits < 50
                {
                    if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        let state = (b.c64_atn as u8)
                            | ((b.c64_clk  as u8) << 1)
                            | ((b.c64_data as u8) << 2)
                            | ((b.drive_clk  as u8) << 3)
                            | ((b.drive_data as u8) << 4);
                        if state != self.pre_fastload_iec_prev {
                            eprintln!(
                                "[PRE-IEC] cyc={} c64=${:04X} drv=${:04X} \
                                 c64_atn:{} c64_clk:{} c64_dat:{} drv_clk:{} drv_dat:{}",
                                self.cycle_count, pc, self.drive.pc(),
                                b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                                b.drive_clk as u8, b.drive_data as u8
                            );
                            self.pre_fastload_iec_prev = state;
                            self.pre_fastload_iec_hits += 1;
                        }
                    }
                }

                // PRE-LOW-PC: trace unique PCs in $0000-$03FF after cyc=105M.
                // Late gate (105M) so we focus on the final setup window just
                // before the BRK loop at $6161. Covers zero page ($0000-$00FF)
                // and pages 1-3 ($0100-$03FF).
                if !self.fastload_dump_done && self.cycle_count > 105_000_000
                    && self.pre_fastload_low_pc_hits < 150
                    && (pc <= 0x03FF)
                    && pc != self.pre_fastload_low_pc_prev
                {
                    let (b0, b1, b2) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(pc), m.read_byte(pc.wrapping_add(1)),
                         m.read_byte(pc.wrapping_add(2)))
                    };
                    eprintln!("[PRE-LOW-PC] cyc={} pc=${:04X} bytes={:02X} {:02X} {:02X}",
                              self.cycle_count, pc, b0, b1, b2);
                    self.pre_fastload_low_pc_prev = pc;
                    self.pre_fastload_low_pc_hits += 1;
                }

                // CIA2-PRA: log every change to CIA2's port-A register after cyc=80M.
                // This is the DEFINITIVE check for whether the game writes $DD00 to
                // assert IEC CLK/DATA/ATN. PRE-IEC tracks the IEC *bus* result;
                // this tracks CIA2 PRA directly and cannot miss a short pulse.
                if self.cycle_count > 80_000_000 && self.cia2_pra_hits < 50 {
                    let pra = self.cia2.borrow().pra();
                    if pra != self.cia2_pra_prev {
                        let ddra = self.cia2.borrow().ddra();
                        let driven = pra & ddra;
                        eprintln!(
                            "[CIA2-PRA] cyc={} PRA:{:02X}->{:02X} DDRA={:02X} \
                             driven={{atn:{} clk:{} dat:{}}} c64=${:04X}",
                            self.cycle_count, self.cia2_pra_prev, pra, ddra,
                            (driven & 0x08 != 0) as u8,
                            (driven & 0x10 != 0) as u8,
                            (driven & 0x20 != 0) as u8,
                            pc
                        );
                        self.cia2_pra_prev = pra;
                        self.cia2_pra_hits += 1;
                    }
                }

                // CIA1-IER: log every change to CIA1 interrupt enable register.
                // Bit4=FLAG (DATA↓ triggers IRQ). cia1_ier=$00 at fast-loader entry
                // means FLAG IRQ is disabled; the game's $033E poll loop will never
                // be broken by a DATA↓ from the drive. No cycle gate: catch all writes.
                if self.cia1_ier_hits < 50 {
                    let ier = self.cia1.borrow().ier();
                    if ier != self.cia1_ier_prev {
                        eprintln!(
                            "[CIA1-IER] cyc={} IER:{:02X}->{:02X} c64=${:04X} drv=${:04X}",
                            self.cycle_count, self.cia1_ier_prev, ier, pc, self.drive.pc()
                        );
                        // Dump 32 bytes around PC to identify the instruction that changed IER.
                        let win = pc.saturating_sub(4);
                        eprint!("[CIA1-IER] code@{:04X}:", win);
                        {
                            let mut m = self.memory.borrow_mut();
                            for i in 0u16..32 {
                                eprint!(" {:02X}", m.read_byte(win.wrapping_add(i)));
                            }
                        }
                        eprintln!();
                        self.cia1_ier_prev = ier;
                        self.cia1_ier_hits += 1;
                    }
                }

                // MEM-FFFE: track changes to $FFFE/$FFFF as seen by the CPU.
                // When KERNAL is banked in, read_byte returns ROM ($48/$FF = $FF48).
                // When KERNAL is banked out, returns RAM — revealing decompressor writes.
                if self.mem_fffe_hits < 30 && self.cycle_count > 75_000_000 {
                    let (lo, hi) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(0xFFFE), m.read_byte(0xFFFF))
                    };
                    if lo != self.mem_fffe_prev[0] || hi != self.mem_fffe_prev[1] {
                        let vec = (hi as u16) << 8 | lo as u16;
                        let (cpu_port, zp_ae, zp_af) = {
                            let mut m = self.memory.borrow_mut();
                            (m.read_byte(0x0001), m.read_byte(0x00AE), m.read_byte(0x00AF))
                        };
                        // Also dump 16 bytes at the source pointer ($AE/$AF) context
                        // so we can see what data the decompressor is reading.
                        let src_ptr = (zp_af as u16) << 8 | zp_ae as u16;
                        let (src_fe, src_ff) = {
                            let mut m = self.memory.borrow_mut();
                            (m.read_byte(src_ptr.wrapping_add(0xFE)),
                             m.read_byte(src_ptr.wrapping_add(0xFF)))
                        };
                        eprintln!(
                            "[MEM-FFFE] cyc={} $FFFE/$FFFF: {:02X}{:02X}->{:02X}{:02X} \
                             (vec ${:04X}) $01=${:02X} hiram={} c64=${:04X} drv=${:04X} \
                             $AE/$AF=${:04X} src[$FE]=${:02X} src[$FF]=${:02X}",
                            self.cycle_count,
                            self.mem_fffe_prev[0], self.mem_fffe_prev[1],
                            lo, hi, vec, cpu_port, (cpu_port >> 1) & 1,
                            pc, self.drive.pc(),
                            src_ptr, src_fe, src_ff
                        );
                        // Cross-check: dump mem_cb60_snap to see if MEM-CA60 already
                        // caught the writes to $CA67/$CA68, or if they were never seen.
                        eprint!("[MEM-FFFE] CA60-snap:");
                        for b in &self.mem_cb60_snap { eprint!(" {:02X}", b); }
                        eprintln!(" (hits={})", self.mem_cb60_hits);
                        self.mem_fffe_prev = [lo, hi];
                        self.mem_fffe_hits += 1;
                    }
                }

                // MEM-6160: poll $6160-$616F for changes every cycle.
                // On real hardware the game's custom loader should overwrite the sprite
                // data at $6160-$616F with a proper IRQ handler BEFORE the BRK loop
                // starts. If $6160-$616F never change from sprite bytes, the loader
                // has a bug (or the IEC transfer delivers wrong bytes to that area).
                // Takes an initial snapshot the first time $6160 contains game code
                // ($AD = LDA opcode, which appears once the first file is loaded).
                if self.mem_6160_hits < 40 && self.cycle_count > 75_000_000 {
                    let cur = {
                        let mut m = self.memory.borrow_mut();
                        let mut arr = [0u8; 16];
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0x6160u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    // Take initial snapshot when $6160 first has the LDA opcode ($AD).
                    if self.mem_6160_snap[0] == 0 && cur[0] == 0xAD {
                        self.mem_6160_snap = cur;
                        eprint!("[MEM-6160-INIT] cyc={} c64=${:04X}:", self.cycle_count, pc);
                        for b in &cur { eprint!(" {:02X}", b); }
                        eprintln!();
                    } else if self.mem_6160_snap[0] != 0 {
                        // Detect any byte changing from snapshot.
                        for i in 0..16usize {
                            if cur[i] != self.mem_6160_snap[i] {
                                let cpu_port = self.memory.borrow_mut().read_byte(0x0001);
                                eprintln!(
                                    "[MEM-6160-CHG] cyc={} $61{:02X}: {:02X}->{:02X} \
                                     c64=${:04X} drv=${:04X} $01=${:02X}",
                                    self.cycle_count, 0x60 + i as u8,
                                    self.mem_6160_snap[i], cur[i],
                                    pc, self.drive.pc(), cpu_port
                                );
                                self.mem_6160_snap[i] = cur[i];
                                self.mem_6160_hits += 1;
                            }
                        }
                    }
                }

                // MEM-CA60: watch $CA60-$CA6F for changes (source buffer for $FFFE/$FFFF).
                // With decompressor base $AE/$AF=$C969 and Y=$FE/$FF:
                //   effective = $C969 + $FE = $CA67 → copied to $FFFE
                //   effective = $C969 + $FF = $CA68 → copied to $FFFF
                // Should contain $66/$FE for BRK vec $FE66; if $FF/$61, wrong data.
                // [Note: struct field is still named mem_cb60_snap for compatibility]
                if self.mem_cb60_hits < 60 && self.cycle_count > 75_000_000 {
                    let cur: [u8; 16] = {
                        let mut m = self.memory.borrow_mut();
                        let mut arr = [0u8; 16];
                        for (i, b) in arr.iter_mut().enumerate() {
                            *b = m.read_byte(0xCA60u16.wrapping_add(i as u16));
                        }
                        arr
                    };
                    for i in 0..16usize {
                        if cur[i] != self.mem_cb60_snap[i] {
                            eprintln!(
                                "[MEM-CA60-CHG] cyc={} $CA{:02X}: {:02X}->{:02X} \
                                 c64=${:04X} drv=${:04X}",
                                self.cycle_count, 0x60 + i as u8,
                                self.mem_cb60_snap[i], cur[i],
                                pc, self.drive.pc()
                            );
                            self.mem_cb60_snap[i] = cur[i];
                            self.mem_cb60_hits += 1;
                        }
                    }
                }

                // CA60-MILESTONE: dump $CA60-$CA6F at 50M, 80M, 100M, 120M cycles.
                // Answers: when does data appear at $CA67/$CA68 (the decompressor source
                // for $FFFE/$FFFF)? Resolves why MEM-CA60-CHG produces no output.
                {
                    let milestones: [(u32, u8); 4] = [
                        (50_000_000,  0x01),
                        (80_000_000,  0x02),
                        (100_000_000, 0x04),
                        (120_000_000, 0x08),
                    ];
                    for &(threshold, bit) in &milestones {
                        if self.cycle_count >= threshold && (self.ca60_milestone_done & bit) == 0 {
                            self.ca60_milestone_done |= bit;
                            let cur: [u8; 16] = {
                                let mut m = self.memory.borrow_mut();
                                let mut arr = [0u8; 16];
                                for (i, b) in arr.iter_mut().enumerate() {
                                    *b = m.read_byte(0xCA60u16.wrapping_add(i as u16));
                                }
                                arr
                            };
                            eprint!("[CA60-MILESTONE] cyc={} $CA60-$CA6F:", self.cycle_count);
                            for b in &cur { eprint!(" {:02X}", b); }
                            eprintln!("  (snap: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} \
                                       {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X})",
                                self.mem_cb60_snap[0], self.mem_cb60_snap[1],
                                self.mem_cb60_snap[2], self.mem_cb60_snap[3],
                                self.mem_cb60_snap[4], self.mem_cb60_snap[5],
                                self.mem_cb60_snap[6], self.mem_cb60_snap[7],
                                self.mem_cb60_snap[8], self.mem_cb60_snap[9],
                                self.mem_cb60_snap[10], self.mem_cb60_snap[11],
                                self.mem_cb60_snap[12], self.mem_cb60_snap[13],
                                self.mem_cb60_snap[14], self.mem_cb60_snap[15]);
                            // Also dump $4271-$4295: the pass-4 src_base region that the
                            // second inner loop reads from. $4293/$4294 feed $CA5F/$CA60
                            // which then determine $CA67/$CA68 → $FFFE/$FFFF (IRQ vector).
                            // In bad runs $4293 gets $FF or $43 instead of correct $66.
                            let src_snap: [u8; 37] = {
                                let mut m = self.memory.borrow_mut();
                                let mut arr = [0u8; 37];
                                for (i, b) in arr.iter_mut().enumerate() {
                                    *b = m.read_byte(0x4271u16.wrapping_add(i as u16));
                                }
                                arr
                            };
                            eprint!("[CA60-MILESTONE] cyc={} $4271-$4295:", self.cycle_count);
                            for b in &src_snap { eprint!(" {:02X}", b); }
                            eprintln!();
                        }
                    }
                }

                // PAGE4-PC: trace unique PCs in $0400-$04FF after cyc=90M.
                // The decompressor/IEC-receive stub at $045E runs here (pass 4, ~96M)
                // and writes the wrong $FF/$61 into $CA67/$CA68.
                if self.page4_pc_hits < 2000
                    && self.cycle_count > 103_000_000
                    && (0x0411..=0x04FF).contains(&pc)
                    && pc != self.page4_pc_prev
                {
                    let (b0, b1, b2) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(pc), m.read_byte(pc.wrapping_add(1)),
                         m.read_byte(pc.wrapping_add(2)))
                    };
                    let (dat_low, drv_dat, drv_clk) = if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        (b.data_low() as u8, b.drive_data as u8, b.drive_clk as u8)
                    } else { (0, 0, 0) };
                    eprintln!(
                        "[PAGE4-PC] cyc={} pc=${:04X} bytes={:02X} {:02X} {:02X} \
                         dat:{} drv_dat:{} drv_clk:{} $01=${:02X}",
                        self.cycle_count, pc, b0, b1, b2,
                        dat_low, drv_dat, drv_clk,
                        self.memory.borrow_mut().read_byte(0x0001)
                    );
                    self.page4_pc_prev = pc;
                    self.page4_pc_hits += 1;
                }

                // CA-WATCH: fire whenever $CA67 or $CA68 changes (after 75M).
                // Gate at 75M: avoids overhead during the first-file KERNAL load
                // (which must complete correctly for IEC timing to work).
                // These are the source bytes for the BRK/IRQ vector.
                if self.ca_watch_hits < 40 && self.cycle_count > 75_000_000 {
                    let (ca67, ca68) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(0xCA67), m.read_byte(0xCA68))
                    };
                    if ca67 != self.ca67_prev || ca68 != self.ca68_prev {
                        let full: [u8; 16] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 16];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0xCA60u16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let code: [u8; 16] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 16];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(pc.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let (dat_low, drv_dat, drv_clk, c64_clk, c64_dat) =
                            if let Some(ref iec) = self.iec_for_trace {
                                let b = iec.borrow();
                                (b.data_low() as u8, b.drive_data as u8, b.drive_clk as u8,
                                 b.c64_clk as u8, b.c64_data as u8)
                            } else { (0, 0, 0, 0, 0) };
                        eprintln!(
                            "[CA-WATCH] cyc={} $CA67:{:02X}->{:02X} $CA68:{:02X}->{:02X} \
                             pc=${:04X} drv=${:04X} $01=${:02X} \
                             iec:dat={} drv_dat={} drv_clk={} c64_clk={} c64_dat={}",
                            self.cycle_count,
                            self.ca67_prev, ca67, self.ca68_prev, ca68,
                            pc, self.drive.pc(),
                            self.memory.borrow_mut().read_byte(0x0001),
                            dat_low, drv_dat, drv_clk, c64_clk, c64_dat
                        );
                        eprint!("[CA-WATCH] CA60-CA6F:");
                        for b in &full { eprint!(" {:02X}", b); }
                        eprintln!();
                        eprint!("[CA-WATCH] code@{:04X}:", pc);
                        for b in &code { eprint!(" {:02X}", b); }
                        eprintln!();
                        // Dump ZP $28-$35 and $F8-$FF: decompressor uses these as
                        // source/dest pointers ($2D/$2E = src, $2F/$30 or $FA/$FB = dst).
                        let zp_lo: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0x0028u16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let zp_hi: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0x00F8u16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        // Also dump ZP $A8-$AF: contains $AE/$AF (decompressor source ptr)
                        // and $B0/$B1 (counted-copy dest ptr used by page-1 routine at $0164).
                        let zp_ae: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0x00A8u16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        eprint!("[CA-WATCH] zp$28:");
                        for b in &zp_lo { eprint!(" {:02X}", b); }
                        eprint!("  zp$A8:");
                        for b in &zp_ae { eprint!(" {:02X}", b); }
                        eprint!("  zp$F8:");
                        for b in &zp_hi { eprint!(" {:02X}", b); }
                        // Dump 16 bytes at ($AE/$AF) to see what the backward-copy source looks like.
                        let ae_ptr = (zp_ae[6] as u16) | ((zp_ae[7] as u16) << 8); // $AE=zp_ae[6], $AF=zp_ae[7]
                        let src_at_ae: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(ae_ptr.wrapping_add(i as u16));
                            }
                            arr
                        };
                        eprint!("  src@($AE/$AF)={:04X}:", ae_ptr);
                        for b in &src_at_ae { eprint!(" {:02X}", b); }
                        eprintln!();
                        self.ca67_prev = ca67;
                        self.ca68_prev = ca68;
                        self.ca_watch_hits += 1;
                    }
                }

                // SLOOP-WRITE: fire when PC=$0472 (STA ($FE),Y in the second inner loop
                // of the pass-4 decompressor) AND dest_base is in $CA00-$CAFF.
                // Filtered to $CA range to avoid exhausting hits on other passes
                // that share the same inner loop code but use different $FE/$FF values.
                if self.sloop_watch_hits < 200 && self.cycle_count > 90_000_000 && pc == 0x0472 {
                    let (cpu_a, cpu_y) = {
                        let cpu = self.cpu.borrow();
                        (cpu.a, cpu.y)
                    };
                    let (zp_fe, zp_ff, zp_f8, zp_f9, zp_fc) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(0x00FE), m.read_byte(0x00FF),
                         m.read_byte(0x00F8), m.read_byte(0x00F9),
                         m.read_byte(0x00FC))
                    };
                    let dest_base = (zp_ff as u16) << 8 | (zp_fe as u16);
                    if dest_base >= 0xCA00 && dest_base <= 0xCAFF {
                        let eff_addr = dest_base.wrapping_add(cpu_y as u16);
                        eprintln!(
                            "[SLOOP-WRITE] cyc={} A=${:02X} Y={} dest_base=${:04X} eff=${:04X} \
                             src_base=${:04X} fc={}",
                            self.cycle_count, cpu_a, cpu_y,
                            dest_base, eff_addr,
                            (zp_f9 as u16) << 8 | (zp_f8 as u16),
                            zp_fc
                        );
                        self.sloop_watch_hits += 1;
                    }
                }

                // ADDR4293-DIAG: unconditional heartbeat — confirms this code path is reached.
                if self.cycle_count > 79_000_000 && self.cycle_count < 79_000_010 {
                    let diag_val = self.memory.borrow_mut().read_byte(0x4293);
                    eprintln!("[ADDR4293-DIAG] cyc={} hits={} val={:02X} prev={:02X}",
                              self.cycle_count, self.mem_4293_hits, diag_val, self.mem_4293_prev);
                }

                // ADDR4293-WATCH: fires when $4293 changes (gate >79M, limit 10).
                // mem_4293_prev starts at sentinel $FF — fires on the FIRST cycle after 79M
                // to confirm the tracker runs and log the actual initial value.
                // Subsequent fires catch the real change from $5F → ($43/$FF/etc).
                // Gate lowered to 79M (from 80M) so we see the state at 80M+1 too.
                if self.mem_4293_hits < 10 && self.cycle_count > 79_000_000 {
                    let val = self.memory.borrow_mut().read_byte(0x4293);
                    // Force a log on the very first call to confirm the tracker runs
                    // and reveal what read_byte(0x4293) actually returns.
                    if self.mem_4293_hits == 0 || val != self.mem_4293_prev {
                        let code_at_pc: [u8; 4] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 4];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(pc.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let region: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0x428Eu16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let zp_ptrs: [u8; 8] = {
                            let mut m = self.memory.borrow_mut();
                            let mut arr = [0u8; 8];
                            for (i, b) in arr.iter_mut().enumerate() {
                                *b = m.read_byte(0x00F8u16.wrapping_add(i as u16));
                            }
                            arr
                        };
                        let tag = if self.mem_4293_hits == 0 { "INIT" } else { "CHG" };
                        eprintln!(
                            "[ADDR4293-{}] cyc={} $4293:{:02X}->{:02X} pc=${:04X} \
                             code@pc:{:02X} {:02X} {:02X} {:02X}",
                            tag, self.cycle_count, self.mem_4293_prev, val, pc,
                            code_at_pc[0], code_at_pc[1], code_at_pc[2], code_at_pc[3]
                        );
                        eprint!("[ADDR4293-WATCH] $428E-$4295:");
                        for b in &region { eprint!(" {:02X}", b); }
                        eprint!("  zp$F8:");
                        for b in &zp_ptrs { eprint!(" {:02X}", b); }
                        eprintln!();
                        self.mem_4293_prev = val;
                        self.mem_4293_hits += 1;
                    }
                }

                // RECV-PC: trace unique PCs in $0000-$03FF after cyc=93M.
                // Gate at 93M: avoids the massive $0181-$019B decompressor loop that
                // runs at 79-93M and would exhaust all 300 hits before $045E code runs.
                // Covers zero-page stubs, stack-page ($0100-$01FF), and pages 2-3.
                if self.recv_pc_hits < 300
                    && self.cycle_count > 93_000_000
                    && (pc <= 0x03FF)
                    && pc != self.recv_pc_prev
                {
                    let (b0, b1, b2) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(pc), m.read_byte(pc.wrapping_add(1)),
                         m.read_byte(pc.wrapping_add(2)))
                    };
                    let (dat_low, drv_dat) = if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        (b.data_low() as u8, b.drive_data as u8)
                    } else { (0, 0) };
                    let cpu_port = self.memory.borrow_mut().read_byte(0x0001);
                    eprintln!(
                        "[RECV-PC] cyc={} pc=${:04X} bytes={:02X} {:02X} {:02X} \
                         dat:{} drv_dat:{} $01=${:02X}",
                        self.cycle_count, pc, b0, b1, b2, dat_low, drv_dat, cpu_port
                    );
                    self.recv_pc_prev = pc;
                    self.recv_pc_hits += 1;
                }

                // ATN-SEQ: fire on every c64_atn 0→1 edge (ATN going LOW on bus).
                // No cycle gate — captures all LISTEN/TALK sequences from boot.
                // Limit 80: first-file KERNAL load uses ~5-15 ATN sequences; 80 is
                // enough headroom to capture a post-load second-file OPEN sequence too.
                if self.atn_seq_hits < 80 {
                    if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        let c64_atn_now = b.c64_atn;
                        if c64_atn_now && !self.prev_c64_atn {
                            let drv_pc = self.drive.pc();
                            eprintln!(
                                "[ATN-SEQ] cyc={} ATN↓ c64=${:04X} drv=${:04X} \
                                 iec={{clk:{} dat:{}}} c64={{clk:{} dat:{}}} drv={{clk:{} dat:{}}}",
                                self.cycle_count, pc, drv_pc,
                                b.clk_low() as u8, b.data_low() as u8,
                                b.c64_clk as u8, b.c64_data as u8,
                                b.drive_clk as u8, b.drive_data as u8,
                            );
                            self.atn_seq_hits += 1;
                        }
                        self.prev_c64_atn = c64_atn_now;
                    }
                }

                // IECIN-WATCH: log when C64 PC hits KERNAL IECIN entry ($EE56) after 89M.
                // Gate at 89M skips ALL first-file transfer bytes (transfer runs 19.78M-87.5M).
                // Limit 10 shows whether the game ever calls IECIN for the second file.
                // kernal_on=0 means the CPU is executing custom RAM code at the IECIN address.
                if self.cycle_count > 89_000_000 && self.iecin_watch_hits < 10
                    && pc == 0xEE56
                {
                    let cpu_port = self.memory.borrow_mut().read_byte(0x0001);
                    let kernal_on = (cpu_port >> 1) & 1;
                    let drv_pc = self.drive.pc();
                    if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        eprintln!(
                            "[IECIN-WATCH] cyc={} c64=${:04X} drv=${:04X} $01=${:02X} \
                             kernal_on={} iec={{atn:{} clk:{} dat:{}}} \
                             c64={{clk:{} dat:{}}} drv={{clk:{} dat:{}}}",
                            self.cycle_count, pc, drv_pc, cpu_port, kernal_on,
                            b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                            b.c64_clk as u8, b.c64_data as u8,
                            b.drive_clk as u8, b.drive_data as u8,
                        );
                    } else {
                        eprintln!(
                            "[IECIN-WATCH] cyc={} c64=${:04X} drv=${:04X} $01=${:02X} kernal_on={}",
                            self.cycle_count, pc, drv_pc, cpu_port, kernal_on
                        );
                    }
                    self.iecin_watch_hits += 1;
                }

                // IECIN-HANG: fire once when C64 PC stays in KERNAL bit-poll ($E5C0-$E5E0)
                // for more than 2M consecutive cycles — the IEC deadlock signature.
                // Dumps kernal_on, IEC bus state, and the bytes the CPU is actually executing
                // (KERNAL ROM if kernal_on=1, or custom RAM code if kernal_on=0).
                if matches!(pc, 0xE5C0..=0xE5E0) {
                    self.iecin_hang_cycles += 1;
                    if !self.iecin_hang_logged && self.iecin_hang_cycles > 2_000_000 {
                        let cpu_port = self.memory.borrow_mut().read_byte(0x0001);
                        let kernal_on = (cpu_port >> 1) & 1;
                        let drv_pc = self.drive.pc();
                        // read_byte respects banking: KERNAL off → returns RAM at this addr.
                        let bytes: Vec<u8> = (0u16..32).map(|i|
                            self.memory.borrow_mut().read_byte(0xE5C0u16.wrapping_add(i))
                        ).collect();
                        if let Some(ref iec) = self.iec_for_trace {
                            let b = iec.borrow();
                            eprintln!(
                                "[IECIN-HANG] cyc={} c64=${:04X} drv=${:04X} $01=${:02X} \
                                 kernal_on={} spin_cyc={} \
                                 iec={{atn:{} clk:{} dat:{}}} drv={{clk:{} dat:{}}}",
                                self.cycle_count, pc, drv_pc, cpu_port, kernal_on,
                                self.iecin_hang_cycles,
                                b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                                b.drive_clk as u8, b.drive_data as u8,
                            );
                        } else {
                            eprintln!(
                                "[IECIN-HANG] cyc={} c64=${:04X} drv=${:04X} spin_cyc={}",
                                self.cycle_count, pc, drv_pc, self.iecin_hang_cycles
                            );
                        }
                        eprint!("[IECIN-HANG] bytes@$E5C0 ({}):",
                                if kernal_on == 1 { "KERNAL ROM" } else { "RAM—custom code!" });
                        for b in &bytes { eprint!(" {:02X}", b); }
                        eprintln!();
                        self.iecin_hang_logged = true;
                    }
                } else if self.iecin_hang_cycles > 0 && !self.iecin_hang_logged {
                    self.iecin_hang_cycles = 0;
                }

                // PRE-PAGE8-PC: trace unique PCs in $0800-$08FF after cyc=90M.
                // The CIA1 FLAG self-test at ~$0820 runs in this range; tracing
                // it shows the actual opcode sequence that passes the test.
                if !self.fastload_dump_done && self.cycle_count > 90_000_000
                    && self.pre_page8_pc_hits < 80
                    && (0x0800..=0x08FF).contains(&pc)
                    && pc != self.pre_page8_pc_prev
                {
                    let (b0, b1, b2) = {
                        let mut m = self.memory.borrow_mut();
                        (m.read_byte(pc), m.read_byte(pc.wrapping_add(1)),
                         m.read_byte(pc.wrapping_add(2)))
                    };
                    eprintln!("[PRE-PAGE8-PC] cyc={} pc=${:04X} bytes={:02X} {:02X} {:02X}",
                              self.cycle_count, pc, b0, b1, b2);
                    self.pre_page8_pc_prev = pc;
                    self.pre_page8_pc_hits += 1;
                }

                // Periodic watchdog: every ~1M C64 cycles print C64 PC, drive PC,
                // and IEC bus state to detect hangs and protocol mismatches.
                if self.cycle_count % 1_000_000 == 0 {
                    let drv_pc = self.drive.pc();
                    if let Some(ref iec) = self.iec_for_trace {
                        let b = iec.borrow();
                        eprintln!("[WATCHDOG] cyc={} c64=${:04X} drv=${:04X} iec={{atn:{} clk:{} dat:{} c64_atn:{} c64_clk:{} c64_dat:{} drv_clk:{} drv_dat:{}}}",
                            self.cycle_count, pc, drv_pc,
                            b.atn_low() as u8, b.clk_low() as u8, b.data_low() as u8,
                            b.c64_atn as u8, b.c64_clk as u8, b.c64_data as u8,
                            b.drive_clk as u8, b.drive_data as u8);
                    } else {
                        eprintln!("[WATCHDOG] cyc={} c64_pc=${:04X} drv_pc=${:04X}",
                                  self.cycle_count, pc, drv_pc);
                    }
                }
            }

            // update the debugger window if it exists
            match self.debugger {
                Some(ref mut dbg) => {
                    dbg.update_vic_window(&mut self.vic);
                    if should_trigger_vblank {
                        dbg.render(&mut self.cpu, &mut self.memory);
                    }
                },
                None => (),
            }

            // redraw the screen and process input on VBlank
            if should_trigger_vblank {
                let _ = self.main_window.update_with_buffer(&self.vic.borrow_mut().window_buffer, SCREEN_WIDTH, SCREEN_HEIGHT);
                self.io.update(&self.main_window, &mut self.cia1);
                self.cia1.borrow_mut().count_tod();
                self.cia2.borrow_mut().count_tod();

                if self.io.check_restore_key(&self.main_window) {
                    self.cpu.borrow_mut().set_nmi(true);
                }
            }

            // process special keys: console ASM output and reset switch
            if self.main_window.is_key_pressed(Key::F11, KeyRepeat::No) {
                let di = self.cpu.borrow_mut().debug_instr;
                self.cpu.borrow_mut().debug_instr = !di;
            }

            if self.main_window.is_key_pressed(Key::F12, KeyRepeat::No) {
                self.reset();
            }

            self.cycle_count = self.cycle_count.wrapping_add(1);
        }

        // update SDL2 audio buffers
        self.sid.borrow_mut().update_audio();
    }


    // *** private functions *** //

    // load a *.prg file
    fn load_prg(&mut self, filename: &str) {
        let prg_data = utils::open_file(filename, 0);
        let start_address: u16 = ((prg_data[1] as u16) << 8) | (prg_data[0] as u16);
        println!("Loading {} to start location at ${:04x} ({})", filename, start_address, start_address);

        for i in 2..(prg_data.len()) {
            self.memory.borrow_mut().write_byte(start_address + (i as u16) - 2, prg_data[i]);
        }

        if self.auto_run {
            let end_plus_1: u16 = start_address.wrapping_add((prg_data.len() as u16).wrapping_sub(2));
            self.fix_basic_pointers_and_inject_run(end_plus_1);
        }
    }

    // After poking a BASIC program into RAM, BASIC's "end of program" pointers
    // at $2D-$32 still say the program is empty (= $0801). Bump them to the
    // real end so RUN finds the program, and stuff "RUN" + CR into the
    // keyboard buffer so BASIC's input loop runs it as if the user typed it.
    fn fix_basic_pointers_and_inject_run(&mut self, end_plus_1: u16) {
        let lo = (end_plus_1 & 0xFF) as u8;
        let hi = ((end_plus_1 >> 8) & 0xFF) as u8;
        let mut mem = self.memory.borrow_mut();
        // $2D/$2E = start of variables (= end of BASIC text + 1)
        // $2F-$32 mirror it so arrays/strings get the right starting point.
        mem.write_byte(0x002D, lo); mem.write_byte(0x002E, hi);
        mem.write_byte(0x002F, lo); mem.write_byte(0x0030, hi);
        mem.write_byte(0x0031, lo); mem.write_byte(0x0032, hi);

        // C64 keyboard buffer is 10 bytes at $0277-$0280, count at $00C6.
        // "RUN" + 0x0D (RETURN) -> 4 bytes, well within the buffer.
        mem.write_byte(0x0277, b'R');
        mem.write_byte(0x0278, b'U');
        mem.write_byte(0x0279, b'N');
        mem.write_byte(0x027A, 0x0D);
        mem.write_byte(0x00C6, 4);
        println!("auto-run: stuffed RUN<CR> into keyboard buffer");
    }

    // KERNAL LOAD trap handler. Triggered when the CPU is about to enter $F4A5
    // (the real-disk LOAD routine, reached via the ILOAD vector at $0330).
    //
    // Zero-page state set up by the caller's SETLFS / SETNAM / LOAD sequence:
    //   $93        = LOAD/VERIFY flag (0 = LOAD, 1 = VERIFY). Set by ROM at $F4A5
    //                from A, but at trap-entry we read A directly.
    //   $B7        = filename length
    //   $B9        = secondary address (0 = use caller X/Y, 1 = use file's load addr)
    //   $BA        = device number (we only handle 8)
    //   $BB/$BC    = filename pointer (low/high)
    //   $C3/$C4    = caller's X/Y (load address) — already saved by $F49E
    //
    // On return we set:
    //   X/Y        = end address + 1
    //   $AE/$AF    = end address + 1 (KERNAL convention)
    //   carry      = 0 (success) or 1 (error); A = error code on error
    // Then we simulate RTS to return to whoever called $FFD5.
    //
    // Returns true if the trap was handled (PC was redirected), false if it
    // should fall through to the real ROM (e.g. wrong device, VERIFY, no file).
    fn handle_load_trap(&mut self) -> bool {
        // Sanity: KERNAL must be banked in, else $F4A5 isn't actually our routine.
        if !self.memory.borrow().kernal_on {
            return false;
        }

        let (a_reg, device, secondary, fname_len, fname_ptr, cx, cy) = {
            let mut mem = self.memory.borrow_mut();
            let c = self.cpu.borrow();
            let a = c.a;
            let device = mem.read_byte(0x00BA);
            let secondary = mem.read_byte(0x00B9);
            let fname_len = mem.read_byte(0x00B7);
            let fname_lo = mem.read_byte(0x00BB);
            let fname_hi = mem.read_byte(0x00BC);
            let fname_ptr = ((fname_hi as u16) << 8) | (fname_lo as u16);
            let cx = mem.read_byte(0x00C3);
            let cy = mem.read_byte(0x00C4);
            (a, device, secondary, fname_len, fname_ptr, cx, cy)
        };

        // Only intercept LOAD (A=0) from device 8. Leave VERIFY and other devices
        // to the ROM so existing test programs aren't broken.
        if a_reg != 0 || device != 8 {
            return false;
        }

        // Pull filename bytes from RAM at the SETNAM pointer.
        let mut fname: Vec<u8> = Vec::with_capacity(fname_len as usize);
        {
            let mut mem = self.memory.borrow_mut();
            for i in 0..fname_len as u16 {
                fname.push(mem.read_byte(fname_ptr.wrapping_add(i)));
            }
        }

        let disk = self.disk_image.as_ref().unwrap();
        let entry = match disk.find_file(&fname) {
            Some(e) => e,
            None => {
                let name_disp: String = fname.iter().map(|&b| d64::petscii_to_ascii(b)).collect();
                let hex: String = fname.iter().map(|b| format!("{:02X} ", b)).collect();
                println!("LOAD trap: \"{}\" (len={}, bytes: {}) not found on mounted D64",
                    name_disp, fname.len(), hex.trim());
                println!("  directory has {} entries:", disk.directory().len());
                for e in disk.directory() {
                    let entry_hex: String = e.filename.iter()
                        .take_while(|&&b| b != 0xA0)
                        .map(|b| format!("{:02X} ", b))
                        .collect();
                    println!("    type=0x{:02X} \"{}\"  (bytes: {})",
                        e.file_type, e.name_ascii(), entry_hex.trim());
                }
                self.return_load_error(4); // 4 = FILE NOT FOUND
                return true;
            }
        };

        let file_bytes = match disk.read_file(&entry) {
            Ok(b) => b,
            Err(e) => {
                println!("LOAD trap: read error: {}", e);
                self.return_load_error(5); // 5 = DEVICE NOT PRESENT (loose mapping)
                return true;
            }
        };

        if file_bytes.len() < 2 {
            self.return_load_error(4);
            return true;
        }

        let file_load_addr: u16 = (file_bytes[0] as u16) | ((file_bytes[1] as u16) << 8);
        let load_addr: u16 = if secondary == 0 {
            ((cy as u16) << 8) | (cx as u16)
        } else {
            file_load_addr
        };

        let payload = &file_bytes[2..];
        {
            let mut mem = self.memory.borrow_mut();
            for (i, b) in payload.iter().enumerate() {
                mem.write_byte(load_addr.wrapping_add(i as u16), *b);
            }
        }

        let end_plus_1: u16 = load_addr.wrapping_add(payload.len() as u16);
        println!(
            "LOAD trap: \"{}\" -> ${:04X}-${:04X} ({} bytes), secondary={}",
            entry.name_ascii(), load_addr, end_plus_1.wrapping_sub(1), payload.len(), secondary
        );

        // KERNAL stores end+1 at $AE/$AF for BASIC's benefit.
        {
            let mut mem = self.memory.borrow_mut();
            mem.write_byte(0x00AE, (end_plus_1 & 0xFF) as u8);
            mem.write_byte(0x00AF, (end_plus_1 >> 8) as u8);
        }

        {
            let mut c = self.cpu.borrow_mut();
            c.x = (end_plus_1 & 0xFF) as u8;
            c.y = (end_plus_1 >> 8) as u8;
            c.a = 0;
            c.set_status_flag(cpu::StatusFlag::Carry, false);
        }
        self.do_rts();
        true
    }

    // Return from the LOAD trap with carry set and error code in A.
    // KERNAL error codes: 4 = FILE NOT FOUND, 5 = DEVICE NOT PRESENT, etc.
    fn return_load_error(&mut self, code: u8) {
        {
            let mut c = self.cpu.borrow_mut();
            c.a = code;
            c.set_status_flag(cpu::StatusFlag::Carry, true);
        }
        self.do_rts();
    }

    // Simulate an RTS: pull (return_addr - 1) off the 6502 stack and jump to it + 1.
    fn do_rts(&mut self) {
        let mut c = self.cpu.borrow_mut();
        let lo = c.pop_byte();
        let hi = c.pop_byte();
        let ret = (((hi as u16) << 8) | (lo as u16)).wrapping_add(1);
        c.pc = ret;
    }

    // tracedrive: log IEC bus edges (ATN/CLK/DATA + ATNA) and print the
    // drive CPU's PC every ~50000 cycles. Helps diagnose handshake hangs.
    fn trace_step(&mut self) {
        let Some(b) = self.iec_for_trace.as_ref() else { return; };
        let state: u8 = {
            let b = b.borrow();
            let mut s = 0u8;
            if b.atn_low()  { s |= 0x01; }
            if b.clk_low()  { s |= 0x02; }
            if b.data_low() { s |= 0x04; }
            if b.drive_atna { s |= 0x08; }
            if b.c64_atn    { s |= 0x10; }
            if b.c64_clk    { s |= 0x20; }
            if b.c64_data   { s |= 0x40; }
            if b.drive_clk  { s |= 0x80; }
            s
        };
        if state != self.prev_trace_state {
            let b = b.borrow();
            let c64_pc = self.cpu.borrow().pc;
            println!(
                "[iec] ATN={} CLK={} DATA={}  c64={{atn:{} clk:{} dat:{}}}  drv={{clk:{} dat:{} atna:{}}}  drv=${:04X}  c64=${:04X}",
                if b.atn_low()  { "L" } else { "H" },
                if b.clk_low()  { "L" } else { "H" },
                if b.data_low() { "L" } else { "H" },
                b.c64_atn  as u8, b.c64_clk  as u8, b.c64_data as u8,
                b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                self.drive.pc(),
                c64_pc,
            );
            self.prev_trace_state = state;
        }
        self.trace_pc_counter = self.trace_pc_counter.wrapping_add(1);
        if self.trace_pc_counter >= 5_000_000 {
            self.trace_pc_counter = 0;
            let c64_pc = self.cpu.borrow().pc;
            eprintln!("[drv] drv=${:04X}  c64=${:04X}", self.drive.pc(), c64_pc);
        }
    }
}
