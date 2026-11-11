// Build script: runs on the host at compile time — panicking IS the
// failure mode, so the firmware panic-policy lints don't apply here.
#![allow(clippy::unwrap_used, clippy::panic, clippy::string_slice)]

#[path = "src/sensors/temperature_config.rs"]
mod temperature_config;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=ccm.x");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rerun-if-env-changed=OXIFOC_BENCH_CPU_MHZ");
    let requested = std::env::var("OXIFOC_BENCH_CPU_MHZ").ok();
    let overclock = std::env::var_os("CARGO_FEATURE_BENCH_OVERCLOCK").is_some();
    assert!(
        overclock || requested.is_none(),
        "CPU override requires bench-overclock"
    );
    let mhz: u32 = requested.as_deref().unwrap_or("168").parse().unwrap();
    assert!(
        [168, 180, 192, 204, 216, 228, 240, 252, 260].contains(&mhz),
        "unsupported bench clock step"
    );
    // Round down. Above 240 MHz the private bench HAL explicitly clamps
    // FLASH latency at WS7; those steps also overclock flash access itself.
    let multiplier = mhz * 1_000_000 * 8 / 7_944_893;
    std::fs::write(
        std::path::Path::new(&out_dir).join("bench_pll_n.rs"),
        multiplier.to_string(),
    )
    .unwrap();
    generate_ntc_tables(&out_dir);
    println!("cargo:rustc-link-search={out_dir}");
    std::fs::copy("memory.x", std::path::Path::new(&out_dir).join("memory.x")).unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    if std::env::var_os("CARGO_FEATURE_CCM").is_some()
        || std::env::var_os("CARGO_FEATURE_CCM_TELEMETRY").is_some()
    {
        std::fs::copy("ccm.x", std::path::Path::new(&out_dir).join("ccm.x")).unwrap();
        println!("cargo:rustc-link-arg-bins=-Tccm.x");
    }
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Parse FLASH region from memory.x and export firmware end offset for compile-time checks.
    // storage.rs uses this to verify STORAGE_START doesn't overlap firmware.
    let memory_x = std::fs::read_to_string("memory.x").unwrap();
    let (origin, length) = parse_flash_region(&memory_x);
    let firmware_end_offset = origin - 0x0800_0000 + length;
    println!("cargo:rustc-env=FIRMWARE_END_OFFSET={firmware_end_offset}");
}

fn generate_ntc_tables(out_dir: &str) {
    use std::fmt::Write;
    use temperature_config::{ADC_MAX_COUNTS, NTC_BOARD, NTC_MOTOR};
    println!("cargo:rerun-if-changed=src/sensors/temperature_config.rs");
    let mut tables = String::new();
    writeln!(tables, "const NTC_ADC_MAX_COUNTS: u16 = {ADC_MAX_COUNTS};").unwrap();
    for (name, config) in [("BOARD_NTC", NTC_BOARD), ("MOTOR_NTC", NTC_MOTOR)] {
        writeln!(
            tables,
            "static {name}: [i16; {}] = [",
            usize::from(ADC_MAX_COUNTS) + 1
        )
        .unwrap();
        for raw in 0..=ADC_MAX_COUNTS {
            write!(
                tables,
                "{},",
                config.temp_c_x10_from_adc(raw, ADC_MAX_COUNTS)
            )
            .unwrap();
        }
        writeln!(tables, "]; ").unwrap();
    }
    std::fs::write(std::path::Path::new(out_dir).join("ntc_tables.rs"), tables).unwrap();
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
