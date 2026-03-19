fn main() {
    println!("cargo:rerun-if-changed=memory.x");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-link-search={}", out_dir);
    std::fs::copy("memory.x", std::path::Path::new(&out_dir).join("memory.x")).unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Parse FLASH region from memory.x and export firmware end offset for compile-time checks.
    // storage.rs uses this to verify STORAGE_START doesn't overlap firmware.
    let memory_x = std::fs::read_to_string("memory.x").unwrap();
    let (origin, length) = parse_flash_region(&memory_x);
    let firmware_end_offset = origin - 0x0800_0000 + length;
    println!("cargo:rustc-env=FIRMWARE_END_OFFSET={firmware_end_offset}");
}

/// Extract FLASH ORIGIN and LENGTH from memory.x
fn parse_flash_region(content: &str) -> (u64, u64) {
    for line in content.lines() {
        let line = line.trim();
        if line.starts_with("FLASH") && line.contains("ORIGIN") {
            let origin = extract_value(line, "ORIGIN");
            let length = extract_value(line, "LENGTH");
            return (origin, length);
        }
    }
    panic!("Could not find FLASH region in memory.x");
}

fn extract_value(line: &str, key: &str) -> u64 {
    let pos = line.find(key).unwrap();
    let after_eq = &line[pos + key.len()..];
    let after_eq = after_eq.trim_start_matches([' ', '=']);
    // Take until comma, whitespace, or end
    let token: String = after_eq
        .chars()
        .take_while(|c| !c.is_whitespace() && *c != ',')
        .collect();
    parse_size(&token)
}

fn parse_size(s: &str) -> u64 {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).unwrap()
    } else if let Some(k) = s.strip_suffix('K') {
        k.trim().parse::<u64>().unwrap() * 1024
    } else if let Some(m) = s.strip_suffix('M') {
        m.trim().parse::<u64>().unwrap() * 1024 * 1024
    } else {
        s.parse().unwrap()
    }
}
