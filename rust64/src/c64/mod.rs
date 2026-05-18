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
}

impl C64 {
    pub fn new(window_scale: Scale, debugger_on: bool, prg_to_load: &str, crt_to_load: &str, d64_to_mount: &str, auto_run: bool, truedrive: bool, tracedrive: bool) -> C64 {
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
        };

        // If the drive is actually running, hand both ends a shared IEC bus
        // and (if a D64 is mounted) give the drive its own copy of the disk
        // image so the read head sees real bytes.
        if truedrive && c64.drive.enabled {
            let iec = drive_1541::IecBus::new_shared();
            c64.cia2.borrow_mut().set_iec(iec.clone());
            c64.drive.set_iec(iec.clone());
            if tracedrive { c64.iec_for_trace = Some(iec); }
            if let Some(img) = &c64.disk_image {
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

            self.cpu.borrow_mut().update(self.cycle_count);

            // Tick the 1541 drive once per C64 cycle. Within 1.5% of real
            // timing (1.000 MHz drive vs ~985 kHz C64 PAL), close enough for
            // the IEC handshake. No-op if the drive ROM didn't load.
            if self.truedrive {
                self.drive.step();
                if self.tracedrive { self.trace_step(); }
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

            self.cycle_count += 1;
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
            println!(
                "[iec] ATN={} CLK={} DATA={}  c64={{atn:{} clk:{} dat:{}}}  drv={{clk:{} dat:{} atna:{}}}  pc=${:04X}",
                if b.atn_low()  { "L" } else { "H" },
                if b.clk_low()  { "L" } else { "H" },
                if b.data_low() { "L" } else { "H" },
                b.c64_atn  as u8, b.c64_clk  as u8, b.c64_data as u8,
                b.drive_clk as u8, b.drive_data as u8, b.drive_atna as u8,
                self.drive.pc(),
            );
            self.prev_trace_state = state;
        }
        self.trace_pc_counter = self.trace_pc_counter.wrapping_add(1);
        if self.trace_pc_counter >= 50_000 {
            self.trace_pc_counter = 0;
            println!("[drv] pc=${:04X}", self.drive.pc());
        }
    }
}
