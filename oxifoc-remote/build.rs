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
    Ok(())
}
