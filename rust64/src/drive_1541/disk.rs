// Disk model — connects a mounted D64 image to the 1541's read head.
//
// Each track on a 1541 disk holds 17-21 sectors depending on which of the
// four density zones it's in. We synthesize a "track buffer" of GCR-encoded
// bytes representing one full revolution and stream them past the head as
// the drive rotates. The drive ROM:
//   * sees one byte at a time on VIA2 PA
//   * gets a "byte ready" pulse from VIA2 CA1 every ~26-32 cycles
//   * sees SYNC (VIA2 PB7=0) while the head is over a run of $FF bytes
//
// What's modelled here:
//   * Mounted D64 image (read-only)
//   * Integer-track head position (1-35)
//   * Rotation: cycles -> byte advances using the current density's period
//   * GCR-encoded sector format: SYNC, header block, gap, SYNC, data block, gap
//   * Sync-mark detection (head over $FF stream)
//   * Motor on/off
//
// Not modelled yet (TODO):
//   * Half-track positions (used by copy protection)
//   * Write (head writing back to track buffer + D64 image)
//   * Write-protect tab — for now WP is always reads "not protected"
//   * Track 36-40 extended tracks
//   * Per-zone GCR-byte timing precision (we use the density-bit period straight)

use std::cell::RefCell;
use std::rc::Rc;

use c64::d64::D64;
use super::gcr;

pub type DiskShared = Rc<RefCell<Disk>>;

// Cycles per GCR byte for each of the four density zones. The drive sets
// the density via VIA2 PB5/PB6, normally matched to track number:
//   3 = 26 cycles (tracks 1-17, outer)
//   2 = 28 cycles (tracks 18-24)
//   1 = 30 cycles (tracks 25-30)
//   0 = 32 cycles (tracks 31-35, inner)
const CYCLES_PER_BYTE: [u32; 4] = [32, 30, 28, 26];

pub struct Disk {
    pub image: Option<D64>,
    pub track: u8,             // 1-35; head is over this physical track
    pub motor_on: bool,
    pub density: u8,           // 0-3 from VIA2 PB5,PB6

    track_buffer: Vec<u8>,     // GCR-encoded bytes for current track
    sync_mask:    Vec<bool>,   // true at positions of a sync-mark byte
    byte_pos: usize,

    rotation_counter: u32,
    pub byte_ready: bool,      // pulsed true on the cycle a new byte slides
                               // under the head; consumer is expected to read+clear
}

impl Disk {
    pub fn new_shared() -> DiskShared {
        Rc::new(RefCell::new(Disk {
            image: None,
            track: 18,           // boot lands the head on track 18 (directory)
            motor_on: false,
            density: 3,
            track_buffer: Vec::new(),
            sync_mask: Vec::new(),
            byte_pos: 0,
            rotation_counter: 0,
            byte_ready: false,
        }))
    }

    pub fn mount(&mut self, image: D64) {
        self.image = Some(image);
        self.rebuild_track();
    }

    /// Called when the head steps. Clamps to track 1-35 and rebuilds the buffer.
    pub fn set_track(&mut self, t: u8) {
        let t = t.max(1).min(35);
        if t == self.track && !self.track_buffer.is_empty() { return; }
        self.track = t;
        self.byte_pos = 0;
        self.rotation_counter = 0;
        self.rebuild_track();
    }

    pub fn set_motor(&mut self, on: bool) {
        self.motor_on = on;
        if !on { self.byte_ready = false; }
    }

    pub fn set_density(&mut self, density: u8) {
        self.density = density & 0x03;
    }

    /// Advance the disk by `cycles` master clocks. When enough cycles have
    /// passed for one GCR byte to slide under the head, we advance byte_pos
    /// and set `byte_ready` so VIA2 can pulse CA1 once.
    ///
    /// byte_ready is suppressed while the new position is a sync-mark byte
    /// ($FF), mirroring the real 1541 hardware which gates byte_ready off
    /// during the sync field. Without suppression the ROM's BVC-after-sync
    /// loop would catch a byte_ready for an intermediate sync byte and read
    /// $FF instead of the first GCR data byte ($52), causing every header
    /// comparison to fail.
    pub fn rotate(&mut self, cycles: u32) {
        if !self.motor_on || self.track_buffer.is_empty() { return; }
        let period = CYCLES_PER_BYTE[(self.density & 0x03) as usize];
        self.rotation_counter += cycles;
        while self.rotation_counter >= period {
            self.rotation_counter -= period;
            self.byte_pos = (self.byte_pos + 1) % self.track_buffer.len();
            if !self.sync_mask[self.byte_pos] {
                self.byte_ready = true;
            }
        }
    }

    /// Read the byte currently under the head. Note: returning the current
    /// byte does NOT clear byte_ready — the 1541 ROM reads via2 CA1 to
    /// learn a new byte arrived, then reads via2 PA to get the value.
    pub fn current_byte(&self) -> u8 {
        if self.track_buffer.is_empty() { return 0x55; }
        self.track_buffer[self.byte_pos]
    }

    /// True when the head is currently over a SYNC mark byte ($FF in the GCR
    /// bitstream). The drive's VIA2 PB7 is the sync-detect input — when low,
    /// the ROM polls until sync clears, then starts reading the next block.
    pub fn sync_under_head(&self) -> bool {
        if self.sync_mask.is_empty() { return false; }
        self.sync_mask[self.byte_pos]
    }

    /// Called by VIA2 after it consumes a byte-ready event.
    pub fn clear_byte_ready(&mut self) { self.byte_ready = false; }

    // --- internals ---

    fn rebuild_track(&mut self) {
        self.track_buffer.clear();
        self.sync_mask.clear();

        let Some(img) = self.image.as_ref() else { return; };
        let sectors_per_track: u8 = match self.track {
            1..=17  => 21,
            18..=24 => 19,
            25..=30 => 18,
            31..=35 => 17,
            _ => return,
        };
        // Disk ID from BAM (track 18, sector 0, bytes $A2–$A3).
        // D64 BAM layout: $90–$9F = disk name, $A0–$A1 = filler ($A0),
        // $A2 = id1, $A3 = id2, $A4 = filler, $A5–$A6 = DOS type "2A".
        // The 1541 ROM reads the ID from BAM[$A2/$A3] after the first
        // successful BAM sector read, then verifies every subsequent
        // sector header's ID against it — a mismatch causes a retry loop.
        let (id1, id2) = img.sector(18, 0)
            .map(|bam| (bam[0xA2], bam[0xA3]))
            .unwrap_or((b'0', b'1'));

        // Lay out: for each sector,
        //   [5 x sync $FF] [10-byte GCR header] [9-byte gap $55]
        //   [5 x sync $FF] [325-byte GCR data]  [8-byte gap $55]
        for sector in 0..sectors_per_track {
            // --- header block (8 raw bytes -> 10 GCR bytes) ---
            // $08 = header ID marker, then checksum, sector, track, id2, id1, $0F, $0F
            let hdr_chksum: u8 = sector ^ self.track ^ id2 ^ id1;
            let header: [u8; 8] = [0x08, hdr_chksum, sector, self.track, id2, id1, 0x0F, 0x0F];
            for _ in 0..5 { self.track_buffer.push(gcr::SYNC_BYTE); self.sync_mask.push(true); }
            let g = gcr::encode(&header);
            for b in g.iter() { self.track_buffer.push(*b); self.sync_mask.push(false); }
            for _ in 0..9 { self.track_buffer.push(0x55); self.sync_mask.push(false); }

            // --- data block (260 raw bytes -> 325 GCR bytes) ---
            let mut data_block = vec![0u8; 260];
            data_block[0] = 0x07; // data block marker
            if let Ok(s) = img.sector(self.track, sector) {
                data_block[1..1 + 256].copy_from_slice(&s[..256]);
            }
            // simple XOR checksum of the 256 data bytes
            let mut cs = 0u8;
            for b in &data_block[1..=256] { cs ^= *b; }
            data_block[257] = cs;
            // bytes 258, 259 are $00 padding

            for _ in 0..5 { self.track_buffer.push(gcr::SYNC_BYTE); self.sync_mask.push(true); }
            let g = gcr::encode(&data_block);
            for b in g.iter() { self.track_buffer.push(*b); self.sync_mask.push(false); }
            for _ in 0..8 { self.track_buffer.push(0x55); self.sync_mask.push(false); }
        }

        debug_assert_eq!(self.track_buffer.len(), self.sync_mask.len());
        self.byte_pos = 0;
        self.rotation_counter = 0;
    }
}
