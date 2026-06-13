// Build script: runs on the host at compile time — panicking IS the
// failure mode, so the firmware panic-policy lints don't apply here.
#![allow(clippy::unwrap_used, clippy::panic, clippy::string_slice)]

fn main() {
    // Single flash layout: the full 128K belongs to the program. This board
    // has no flash-backed config storage (see docs/flash-size.md) —
    // configuration is baked at build time (src/baked_config.rs).
    println!("cargo:rerun-if-changed=memory.x");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-link-search={out_dir}");
    std::fs::copy("memory.x", std::path::Path::new(&out_dir).join("memory.x")).unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
