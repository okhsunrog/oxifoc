use std::{env, fs, path::PathBuf};

fn main() -> std::io::Result<()> {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo sets OUT_DIR for build scripts"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x"))?;
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");
    println!("cargo:rustc-link-arg-tests=-Tembedded-test.x");
    let memory_x = fs::read_to_string("memory.x")?;
    println!(
        "cargo:rustc-env=FIRMWARE_END={}",
        flash_end(&memory_x).expect("memory.x defines FLASH ORIGIN and LENGTH")
    );
    Ok(())
}

/// End address of the FLASH region in memory.x, for the storage overlap check.
fn flash_end(memory_x: &str) -> Option<u32> {
    let line = memory_x
        .lines()
        .find(|l| l.trim_start().starts_with("FLASH"))?;
    let number = |key: &str| -> Option<u32> {
        let rest = line.split_once(key)?.1;
        let value = rest
            .trim_start_matches([' ', '='])
            .split([',', ' '])
            .next()?;
        u32::from_str_radix(value.trim_start_matches("0x").trim_start_matches("0X"), 16).ok()
    };
    Some(number("ORIGIN")? + number("LENGTH")?)
}
