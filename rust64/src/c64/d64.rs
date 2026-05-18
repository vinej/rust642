// D64 disk image parser for level-2 KERNAL LOAD trap support.
// Standard 1541 .d64 layout: 35 tracks, 683 sectors total, 174848 bytes.
// Tracks are 1-indexed; track 18 holds BAM (sector 0) and directory (sector 1+).

use std::fs::File;
use std::io::Read;
use std::path::Path;

pub const SECTOR_SIZE: usize = 256;
const DIR_TRACK: u8 = 18;

// Cumulative sector count BEFORE track N starts (track 1 = entry [1] = 0).
const SECTORS_BEFORE: [u16; 41] = [
    0,
    0,                                                 // track 1
    21,                                                // 2
    42, 63, 84, 105, 126, 147, 168,                    // 3-9
    189, 210, 231, 252, 273, 294, 315, 336,            // 10-17
    357,                                               // 18
    376, 395, 414, 433, 452, 471,                      // 19-24
    490, 508, 526, 544, 562, 580,                      // 25-30
    598, 615, 632, 649, 666,                           // 31-35
    683, 700, 717, 734, 751,                           // 36-40 (extended; rarely used)
];

const SECTORS_PER_TRACK: [u8; 41] = [
    0,
    21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
    19,19,19,19,19,19,19,
    18,18,18,18,18,18,
    17,17,17,17,17,
    17,17,17,17,17,
];

#[derive(Debug, Clone)]
pub struct DirEntry {
    pub file_type: u8,    // bit 7 = closed, bit 6 = locked, bits 0-3 = type (2 = PRG)
    pub track: u8,        // first track of file data
    pub sector: u8,       // first sector of file data
    pub filename: [u8; 16], // PETSCII, padded with $A0
    pub size_sectors: u16,
}

impl DirEntry {
    pub fn is_prg(&self) -> bool { (self.file_type & 0x0F) == 2 }
    pub fn is_closed(&self) -> bool { (self.file_type & 0x80) != 0 }

    pub fn name_ascii(&self) -> String {
        let mut s = String::new();
        for &b in self.filename.iter() {
            if b == 0xA0 { break; }
            s.push(petscii_to_ascii(b));
        }
        s
    }
}

#[derive(Clone)]
pub struct D64 {
    data: Vec<u8>,
}

impl D64 {
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let mut file = File::open(&path).map_err(|e| format!("D64 open failed: {}", e))?;
        let mut data = Vec::new();
        file.read_to_end(&mut data).map_err(|e| format!("D64 read failed: {}", e))?;
        if data.len() < 174848 {
            return Err(format!("D64 too small: {} bytes (need >= 174848)", data.len()));
        }
        Ok(D64 { data })
    }

    fn sector_offset(&self, track: u8, sector: u8) -> Result<usize, String> {
        if track == 0 || (track as usize) >= SECTORS_BEFORE.len() {
            return Err(format!("invalid track {}", track));
        }
        if sector >= SECTORS_PER_TRACK[track as usize] {
            return Err(format!("invalid sector {} for track {}", sector, track));
        }
        Ok((SECTORS_BEFORE[track as usize] as usize + sector as usize) * SECTOR_SIZE)
    }

    pub fn sector(&self, track: u8, sector: u8) -> Result<&[u8], String> {
        let off = self.sector_offset(track, sector)?;
        if off + SECTOR_SIZE > self.data.len() {
            return Err(format!("sector out of bounds: t{} s{}", track, sector));
        }
        Ok(&self.data[off..off + SECTOR_SIZE])
    }

    pub fn directory(&self) -> Vec<DirEntry> {
        let mut entries = Vec::new();
        let mut track = DIR_TRACK;
        let mut sector: u8 = 1;
        // cycle guard: 1541 dir is tiny so a small fixed cap is plenty.
        for _ in 0..40 {
            let s = match self.sector(track, sector) {
                Ok(s) => s,
                Err(_) => break,
            };
            for i in 0..8 {
                let off = i * 32;
                let ft = s[off + 2];
                if (ft & 0x0F) == 0 { continue; } // DEL / scratched entry
                let mut fname = [0u8; 16];
                fname.copy_from_slice(&s[off + 5..off + 5 + 16]);
                entries.push(DirEntry {
                    file_type: ft,
                    track: s[off + 3],
                    sector: s[off + 4],
                    filename: fname,
                    size_sectors: ((s[off + 0x1F] as u16) << 8) | (s[off + 0x1E] as u16),
                });
            }
            let next_t = s[0];
            let next_s = s[1];
            if next_t == 0 { break; }
            track = next_t;
            sector = next_s;
        }
        entries
    }

    // Follow the file's sector chain and return the raw bytes including
    // the 2-byte load-address header.
    pub fn read_file(&self, entry: &DirEntry) -> Result<Vec<u8>, String> {
        let mut bytes = Vec::new();
        let mut track = entry.track;
        let mut sector = entry.sector;
        for _ in 0..683 {
            let s = self.sector(track, sector)?;
            let next_t = s[0];
            let next_s = s[1];
            if next_t == 0 {
                // last sector: byte 1 is the index of the last valid byte in the sector
                let last = next_s as usize;
                if last >= 2 {
                    bytes.extend_from_slice(&s[2..=last.min(255)]);
                }
                return Ok(bytes);
            }
            bytes.extend_from_slice(&s[2..256]);
            track = next_t;
            sector = next_s;
        }
        Err("file sector chain too long (possible cycle)".to_string())
    }

    // Match a PETSCII filename (as found in zero-page filename buffer) against
    // the directory. Supports trailing '*' wildcard and case-insensitive match.
    // An empty filename returns the first closed PRG (legacy "load first program").
    pub fn find_file(&self, name: &[u8]) -> Option<DirEntry> {
        let dir = self.directory();

        if name.is_empty() {
            return dir.into_iter().find(|e| e.is_prg() && e.is_closed());
        }

        // wildcard position (first '*')
        let wildcard_pos = name.iter().position(|&b| b == b'*');
        let match_len = wildcard_pos.unwrap_or(name.len());

        for entry in dir {
            if !entry.is_prg() || !entry.is_closed() { continue; }

            let mut entry_name: Vec<u8> = Vec::new();
            for &b in entry.filename.iter() {
                if b == 0xA0 { break; }
                entry_name.push(b);
            }

            if match_len == 0 {
                return Some(entry); // bare "*"
            }
            if match_len > entry_name.len() { continue; }

            let prefix_match = name[..match_len].iter()
                .zip(entry_name.iter())
                .all(|(a, b)| petscii_eq_ci(*a, *b));

            if !prefix_match { continue; }
            if wildcard_pos.is_some() { return Some(entry); }
            if name.len() == entry_name.len() { return Some(entry); }
        }
        None
    }
}

// Case-insensitive PETSCII byte comparison good enough for filename matching.
fn petscii_eq_ci(a: u8, b: u8) -> bool {
    petscii_upper(a) == petscii_upper(b)
}

fn petscii_upper(b: u8) -> u8 {
    match b {
        // Unshifted/shifted PETSCII both map A-Z to $41-$5A and $C1-$DA respectively
        0x61..=0x7A => b - 0x20,         // ASCII lowercase -> uppercase
        0xC1..=0xDA => b - 0x80,         // shifted PETSCII letters -> $41-$5A
        _ => b,
    }
}

pub fn petscii_to_ascii(b: u8) -> char {
    match b {
        0x20..=0x40 => b as char,
        0x41..=0x5A => b as char,
        0x5B..=0x5F => b as char,
        0xC1..=0xDA => (b - 0x80) as char,
        _ => '?',
    }
}
