// Address-space view that the 1541's 6502 sees. Implements emulator_6502's
// Interface6502 so we can hand `&mut DriveBus` to `MOS6502::cycle()`.
//
// 1541 memory map:
//   $0000-$07FF   2 KB RAM
//   $0800-$17FF   open bus / RAM mirrors (we ignore mirroring)
//   $1800-$1BFF   VIA1 (IEC bus)        — regs at $1800-$180F, mirrored
//   $1C00-$1FFF   VIA2 (disk head)      — regs at $1C00-$1C0F, mirrored
//   $2000-$BFFF   open bus
//   $C000-$FFFF   16 KB DOS ROM

use emulator_6502::Interface6502;

use super::disk::{Disk, DiskShared};
use super::via::{Via, ViaKind};

pub const RAM_SIZE: usize = 0x0800;
pub const ROM_SIZE: usize = 0x4000;

pub struct DriveBus {
    pub ram: [u8; RAM_SIZE],
    pub rom: [u8; ROM_SIZE],
    pub rom_loaded: bool,
    pub via1: Via,        // $1800 — IEC bus
    pub via2: Via,        // $1C00 — disk head
    pub disk: DiskShared, // shared with via2
}

impl DriveBus {
    pub fn new() -> Self {
        let disk = Disk::new_shared();
        let mut via2 = Via::new(ViaKind::Disk);
        via2.set_disk(disk.clone());
        DriveBus {
            ram: [0; RAM_SIZE],
            rom: [0; ROM_SIZE],
            rom_loaded: false,
            via1: Via::new(ViaKind::Iec),
            via2,
            disk,
        }
    }

    pub fn load_rom(&mut self, bytes: &[u8]) -> Result<(), String> {
        if bytes.len() != ROM_SIZE {
            return Err(format!("1541 ROM must be {} bytes, got {}", ROM_SIZE, bytes.len()));
        }
        self.rom.copy_from_slice(bytes);
        self.rom_loaded = true;
        Ok(())
    }

    pub fn reset_chips(&mut self) {
        self.via1.reset();
        self.via2.reset();
    }

    /// Either VIA asserting IRQ pulls the drive CPU's IRQ line low.
    pub fn irq_asserted(&self) -> bool {
        self.via1.irq_pending || self.via2.irq_pending
    }
}

impl Interface6502 for DriveBus {
    fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x07FF => self.ram[addr as usize],
            0x1800..=0x1BFF => self.via1.read((addr & 0x0F) as u8),
            0x1C00..=0x1FFF => self.via2.read((addr & 0x0F) as u8),
            0xC000..=0xFFFF => self.rom[(addr - 0xC000) as usize],
            _ => 0xFF,
        }
    }

    fn write(&mut self, addr: u16, val: u8) {
        match addr {
            0x0000..=0x07FF => self.ram[addr as usize] = val,
            0x1800..=0x1BFF => self.via1.write((addr & 0x0F) as u8, val),
            0x1C00..=0x1FFF => self.via2.write((addr & 0x0F) as u8, val),
            _ => {}
        }
    }
}
