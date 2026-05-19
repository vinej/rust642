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
                };
            }
        }

        let mut drive = Drive1541 {
            cpu: MOS6502::new(),
            bus,
            enabled: true,
            cycle_count: 0,
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
        self.cpu.cycle(&mut self.bus);
        self.cycle_count = self.cycle_count.wrapping_add(1);
    }

    pub fn pc(&self) -> u16 { self.cpu.get_program_counter() }
}
