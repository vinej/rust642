// d64_check — small CLI used by the rust64 launcher GUI.
//
// Subcommands:
//   d64_check list <image.d64>
//       Print one line per closed PRG entry, machine-readable:
//           SIZE\tNAME
//       (size in sectors, NAME is the ASCII rendering of the PETSCII filename
//       with the 0xA0 padding stripped). Used by the GUI to populate its list.
//
//   d64_check extract <image.d64> <name> <output.prg>
//       Find <name> on the image (case-insensitive, supports trailing '*'
//       wildcard) and write the raw PRG bytes (including the 2-byte load
//       address header) to <output.prg>.
//
//   d64_check info <image.d64> [name]
//       Human-readable directory listing, optionally followed by details
//       for a matching file.

mod d64;

use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        usage();
        return ExitCode::from(2);
    }

    match args[1].as_str() {
        "list" if args.len() == 3 => list(&args[2]),
        "info" if args.len() >= 3 && args.len() <= 4 => {
            info(&args[2], args.get(3).map(String::as_str))
        }
        "extract" if args.len() == 5 => extract(&args[2], &args[3], &args[4]),
        _ => {
            usage();
            ExitCode::from(2)
        }
    }
}

fn usage() {
    eprintln!("usage:");
    eprintln!("  d64_check list <image.d64>");
    eprintln!("  d64_check info <image.d64> [name]");
    eprintln!("  d64_check extract <image.d64> <name> <output.prg>");
}

// Case-insensitive ASCII uppercase of a normal filename so we can normalize
// the disk filename into the form a user would type at the C64 prompt.
fn name_to_ascii_upper(b: u8) -> u8 {
    match b {
        b'a'..=b'z' => b - 0x20,
        _ => b,
    }
}

fn list(image: &str) -> ExitCode {
    let disk = match d64::D64::open(image) {
        Ok(d) => d,
        Err(e) => { eprintln!("open failed: {}", e); return ExitCode::from(1); }
    };
    for e in disk.directory() {
        if !e.is_prg() || !e.is_closed() { continue; }
        // strip trailing whitespace and uppercase for easier consumption
        let name: String = e.name_ascii()
            .bytes()
            .map(|b| name_to_ascii_upper(b) as char)
            .collect();
        println!("{}\t{}", e.size_sectors, name);
    }
    ExitCode::SUCCESS
}

fn info(image: &str, name: Option<&str>) -> ExitCode {
    let disk = match d64::D64::open(image) {
        Ok(d) => d,
        Err(e) => { eprintln!("open failed: {}", e); return ExitCode::from(1); }
    };
    println!("== directory ==");
    for e in disk.directory() {
        let kind = match e.file_type & 0x0F {
            1 => "SEQ", 2 => "PRG", 3 => "USR", 4 => "REL", _ => "???",
        };
        println!("  {:>4} \"{}\" {}{}", e.size_sectors, e.name_ascii(), kind,
                 if e.is_closed() { "" } else { "*" });
    }

    if let Some(n) = name {
        let pet: Vec<u8> = n.bytes().map(name_to_ascii_upper).collect();
        match disk.find_file(&pet) {
            Some(entry) => {
                println!("matched: \"{}\"  t{} s{}  {} sectors",
                    entry.name_ascii(), entry.track, entry.sector, entry.size_sectors);
                match disk.read_file(&entry) {
                    Ok(bytes) => {
                        if bytes.len() < 2 { println!("file too short"); return ExitCode::from(1); }
                        let load = (bytes[0] as u16) | ((bytes[1] as u16) << 8);
                        println!("loads to ${:04X}, payload {} bytes", load, bytes.len() - 2);
                    }
                    Err(e) => { println!("read error: {}", e); return ExitCode::from(1); }
                }
            }
            None => { println!("not found"); return ExitCode::from(1); }
        }
    }
    ExitCode::SUCCESS
}

fn extract(image: &str, name: &str, output: &str) -> ExitCode {
    let disk = match d64::D64::open(image) {
        Ok(d) => d,
        Err(e) => { eprintln!("open failed: {}", e); return ExitCode::from(1); }
    };
    let pet: Vec<u8> = name.bytes().map(name_to_ascii_upper).collect();
    let entry = match disk.find_file(&pet) {
        Some(e) => e,
        None => { eprintln!("\"{}\" not found on image", name); return ExitCode::from(1); }
    };
    let bytes = match disk.read_file(&entry) {
        Ok(b) => b,
        Err(e) => { eprintln!("read failed: {}", e); return ExitCode::from(1); }
    };
    if bytes.len() < 2 {
        eprintln!("file too short ({} bytes)", bytes.len());
        return ExitCode::from(1);
    }
    if let Err(e) = fs::write(output, &bytes) {
        eprintln!("write {}: {}", output, e);
        return ExitCode::from(1);
    }
    let load = (bytes[0] as u16) | ((bytes[1] as u16) << 8);
    println!("extracted \"{}\" -> {} ({} bytes, load ${:04X})",
        entry.name_ascii(), output, bytes.len(), load);
    ExitCode::SUCCESS
}
