fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-link-search={}", out_dir);
    std::fs::copy("memory.x", std::path::Path::new(&out_dir).join("memory.x")).unwrap();
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");
    println!("cargo:rustc-link-arg=-Tembedded-test.x");
}
