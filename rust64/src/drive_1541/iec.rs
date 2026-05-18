// IEC serial bus — three open-collector lines between C64 and drive.
//
// All three lines are pulled HIGH (released) by a single pull-up resistor on
// the bus. Any device on the bus can pull a line LOW. So the actual bus state
// for each line is the logical OR of every device's "I am pulling this line".
//
// Polarity reminder (from both CPUs' perspective when accessing their port
// registers):
//   * WRITES — 1 = "pull line low", 0 = "release"  (output drives an inverter
//     to an open-collector transistor)
//   * READS  — 0 = "line is low (asserted)", 1 = "line is high (released)"
//
// Pin map:
//   C64 CIA2 PA bit 3 = ATN OUT
//   C64 CIA2 PA bit 4 = CLK OUT
//   C64 CIA2 PA bit 5 = DATA OUT
//   C64 CIA2 PA bit 6 = CLK IN
//   C64 CIA2 PA bit 7 = DATA IN
//   1541 VIA1 PB bit 0 = DATA IN
//   1541 VIA1 PB bit 1 = DATA OUT
//   1541 VIA1 PB bit 2 = CLK IN
//   1541 VIA1 PB bit 3 = CLK OUT
//   1541 VIA1 PB bit 4 = ATNA (ATN Acknowledge — when set, drive RELEASES
//                              DATA even while ATN is low. Cleared by default
//                              so unaddressed devices auto-hold DATA low.)
//   1541 VIA1 PB bit 5 = device address bit 0 (jumper input)
//   1541 VIA1 PB bit 6 = device address bit 1
//   1541 VIA1 PB bit 7 = ATN IN

use std::cell::RefCell;
use std::rc::Rc;

pub type IecBusShared = Rc<RefCell<IecBus>>;

#[derive(Default)]
pub struct IecBus {
    // Per-end "pulling" state. true = this end is pulling the line LOW.
    pub c64_atn:  bool,
    pub c64_clk:  bool,
    pub c64_data: bool,
    pub drive_clk:  bool,
    pub drive_data: bool,

    // Drive ATNA latch. When the drive sees ATN low and ATNA is 0, it auto-
    // pulls DATA low — that's the "I'm alive on the bus" handshake the C64
    // checks before talking to a device. Setting ATNA=1 releases DATA.
    pub drive_atna: bool,
}

impl IecBus {
    pub fn new_shared() -> IecBusShared { Rc::new(RefCell::new(IecBus::default())) }

    pub fn atn_low(&self) -> bool { self.c64_atn }
    pub fn clk_low(&self) -> bool { self.c64_clk || self.drive_clk }
    pub fn data_low(&self) -> bool {
        // Auto-DATA: per the 1541 hardware (NAND of ATN_LOW and inverted
        // ATNA), DATA is automatically pulled low whenever ATN is asserted
        // AND the drive has NOT yet set its ATNA latch. That gives the C64
        // an immediate "device present" signal *before* the drive's CPU has
        // had a chance to service the ATN interrupt. Once the drive sets
        // ATNA=1 to acknowledge, auto-DATA releases and the drive must
        // explicitly hold DATA via PB1 if it wants to keep the line low.
        let auto_data = self.atn_low() && !self.drive_atna;
        self.c64_data || self.drive_data || auto_data
    }
}
