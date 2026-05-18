// rust64 build script — only used to help the linker find SDL3.lib on Windows.
//
// `sdl3-sys` emits `cargo::rustc-link-lib=SDL3` but (without the use-pkg-config /
// use-vcpkg / build-from-source feature) it does not emit a link-search path,
// so the MSVC linker has no idea where to look. We give it one.
//
// Source order:
//   1. $SDL3_LIB_DIR if set (directory containing SDL3.lib)
//   2. $SDL3_PATH    if set (treated as the directory itself)
//   3. The crate parent directory (c:\jyv\rust\) as a fallback so a checkout
//      with SDL3.lib next to the rust64/ folder Just Works.

use std::env;
use std::path::PathBuf;

fn main() {
    let mut candidates: Vec<PathBuf> = Vec::new();

    if let Ok(p) = env::var("SDL3_LIB_DIR") { candidates.push(PathBuf::from(p)); }
    if let Ok(p) = env::var("SDL3_PATH")    { candidates.push(PathBuf::from(p)); }

    if let Ok(manifest) = env::var("CARGO_MANIFEST_DIR") {
        let parent = PathBuf::from(manifest).parent().map(|p| p.to_path_buf());
        if let Some(p) = parent { candidates.push(p); }
    }

    for dir in candidates {
        if dir.join("SDL3.lib").exists() || dir.join("libSDL3.so").exists() || dir.join("libSDL3.dylib").exists() {
            println!("cargo:rustc-link-search=native={}", dir.display());
            println!("cargo:warning=rust64: linking SDL3 from {}", dir.display());
            return;
        }
    }

    println!(
        "cargo:warning=rust64: SDL3 library not found via SDL3_LIB_DIR, SDL3_PATH, \
         or alongside the crate. Set one of these or the link step will fail."
    );
}
