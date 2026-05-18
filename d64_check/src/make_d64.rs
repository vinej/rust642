// Build a minimal 174848-byte D64 image containing one PRG file in the
// directory. Just enough to drive rust64's KERNAL LOAD trap.
//
// Usage: make_d64 <out.d64> <name-on-disk> <input.prg>

mod d64;

use std::env;
use std::fs;

const SECTOR: usize = 256;
const DIR_TRACK: u8 = 18;

// Cumulative sector offsets (same table as d64.rs, mirrored here so this tool stays self-contained).
const SECTORS_BEFORE: [u16; 36] = [
    0,
    0, 21, 42, 63, 84, 105, 126, 147, 168,
    189, 210, 231, 252, 273, 294, 315, 336,
    357,
    376, 395, 414, 433, 452, 471,
    490, 508, 526, 544, 562, 580,
    598, 615, 632, 649, 666,
];
const SECTORS_PER_TRACK: [u8; 36] = [
    0,
    21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
    19,19,19,19,19,19,19,
    18,18,18,18,18,18,
    17,17,17,17,17,
];

fn sector_off(track: u8, sector: u8) -> usize {
    (SECTORS_BEFORE[track as usize] as usize + sector as usize) * SECTOR
}

fn put_sector(image: &mut [u8], track: u8, sector: u8, data: &[u8]) {
    let off = sector_off(track, sector);
    image[off..off + SECTOR].fill(0);
    image[off..off + data.len().min(SECTOR)].copy_from_slice(&data[..data.len().min(SECTOR)]);
}

// Convert an ASCII filename to PETSCII (uppercase letters only).
fn ascii_to_petscii(s: &str) -> Vec<u8> {
    s.bytes()
        .map(|b| match b {
            b'a'..=b'z' => b - 0x20,
            _ => b,
        })
        .collect()
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        eprintln!("usage: make_d64 <out.d64> <disk-filename> <input.prg>");
        std::process::exit(2);
    }
    let out = &args[1];
    let name = &args[2];
    let prg = match fs::read(&args[3]) {
        Ok(b) => b,
        Err(e) => { eprintln!("read prg: {}", e); std::process::exit(1); }
    };

    let mut image = vec![0u8; 174848];

    // Lay out PRG starting at track 17, sector 0, walking forward through sectors,
    // skipping the directory track (18). Each non-final sector: bytes 0,1 point to
    // next sector and bytes 2..256 carry 254 bytes of payload. Final sector: byte 0=0,
    // byte 1 = index of last valid byte (>=2), bytes 2..=byte1 carry the tail.
    let payload_per_sector = 254;
    let num_sectors = (prg.len() + payload_per_sector - 1) / payload_per_sector;

    let mut sector_chain: Vec<(u8, u8)> = Vec::with_capacity(num_sectors);
    let mut t: u8 = 17;
    let mut s: u8 = 0;
    for _ in 0..num_sectors {
        sector_chain.push((t, s));
        // advance
        loop {
            s += 1;
            if s >= SECTORS_PER_TRACK[t as usize] {
                s = 0;
                t += 1;
                if t == DIR_TRACK { t += 1; }
                if (t as usize) >= SECTORS_PER_TRACK.len() {
                    panic!("ran off the disk while allocating sectors");
                }
            }
            // skip directory track entirely
            if t != DIR_TRACK { break; }
        }
    }

    for (i, &(track, sector)) in sector_chain.iter().enumerate() {
        let mut buf = [0u8; SECTOR];
        let chunk_start = i * payload_per_sector;
        let chunk_end = (chunk_start + payload_per_sector).min(prg.len());
        let chunk = &prg[chunk_start..chunk_end];
        if i + 1 < sector_chain.len() {
            let (nt, ns) = sector_chain[i + 1];
            buf[0] = nt;
            buf[1] = ns;
        } else {
            buf[0] = 0;
            // last-byte index: 2 + chunk.len() - 1 = 1 + chunk.len()
            buf[1] = (1 + chunk.len()) as u8;
        }
        buf[2..2 + chunk.len()].copy_from_slice(chunk);
        put_sector(&mut image, track, sector, &buf);
    }

    // BAM at t18 s0 — minimal: marks all sectors as free except those we used.
    // (We don't need accurate BAM for the trap, but make it plausible.)
    let mut bam = [0u8; SECTOR];
    bam[0] = DIR_TRACK; // next: dir starts at t18 s1
    bam[1] = 1;
    bam[2] = b'A';      // DOS version
    bam[3] = 0;
    // Per-track BAM entries: bytes 4..4+4*35. For each track: free count + 3-byte bitmap.
    for tr in 1u8..=35u8 {
        let off = 4 + (tr as usize - 1) * 4;
        bam[off] = SECTORS_PER_TRACK[tr as usize];
        bam[off + 1] = 0xFF;
        bam[off + 2] = 0xFF;
        bam[off + 3] = 0xFF;
    }
    // Disk name (16 chars, $A0 padded), id "01", DOS "2A"
    for i in 0..16 { bam[0x90 + i] = 0xA0; }
    let disk_name = ascii_to_petscii("RUST64 TEST");
    for (i, b) in disk_name.iter().take(16).enumerate() { bam[0x90 + i] = *b; }
    bam[0xA0] = 0xA0; bam[0xA1] = 0xA0;
    bam[0xA2] = b'0'; bam[0xA3] = b'1';   // disk id
    bam[0xA4] = 0xA0;
    bam[0xA5] = b'2'; bam[0xA6] = b'A';   // DOS type
    bam[0xA7] = 0xA0; bam[0xA8] = 0xA0;
    put_sector(&mut image, 18, 0, &bam);

    // Directory at t18 s1: one PRG entry.
    let mut dir = [0u8; SECTOR];
    dir[0] = 0;     // no next dir sector
    dir[1] = 0xFF;
    // Entry 0 starts at offset 2 (sector-link bytes only matter for sub-sector entries' link;
    // each entry occupies offsets 2..34, 34..66, ...).
    let e = 2;
    dir[e]     = 0x82;                            // closed PRG
    dir[e + 1] = sector_chain[0].0;               // first track
    dir[e + 2] = sector_chain[0].1;               // first sector
    let pet = ascii_to_petscii(name);
    for i in 0..16 {
        dir[e + 3 + i] = if i < pet.len() { pet[i] } else { 0xA0 };
    }
    let size = sector_chain.len() as u16;
    dir[e + 28] = (size & 0xFF) as u8;            // size low
    dir[e + 29] = ((size >> 8) & 0xFF) as u8;     // size high
    put_sector(&mut image, 18, 1, &dir);

    fs::write(out, &image).unwrap();
    println!("wrote {} bytes -> {}", image.len(), out);
    println!("  disk: \"RUST64 TEST\"");
    println!("  file: \"{}\" PRG, {} bytes, {} sectors", name, prg.len(), num_sectors);
    println!("  load addr ${:02X}{:02X}", prg[1], prg[0]);
}
