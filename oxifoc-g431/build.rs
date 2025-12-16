fn main() {
    // Rerun if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");

    // Tell linker where to find memory.x
    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-link-search={}", out_dir);

    // Copy memory.x to OUT_DIR so cortex-m-rt can find it
    std::fs::copy("memory.x", std::path::Path::new(&out_dir).join("memory.x")).unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
