//! Host-only microbenchmark. Compile with rustc --edition 2024 -C opt-level=3/s/z.
//! Both paths include a non-inlined function call and black_box overhead.
use std::{hint::black_box, time::Instant};

#[inline(never)]
fn count_utf16(s: &str) -> usize {
    s.encode_utf16().count()
}
#[inline(never)]
fn fold_utf16(s: &str) -> usize {
    s.encode_utf16().fold(0usize, |n, _| n + 1)
}

fn run(s: &str, n: u64, fold: bool) -> f64 {
    let start = Instant::now();
    if fold {
        for _ in 0..n {
            black_box(fold_utf16(black_box(s)));
        }
    } else {
        for _ in 0..n {
            black_box(count_utf16(black_box(s)));
        }
    }
    start.elapsed().as_nanos() as f64 / n as f64
}

fn main() {
    let cases = [
        ("empty", String::new()),
        ("property_name", String::from("DeviceInterfaceGUIDs")),
        (
            "guid",
            String::from("{12345678-1234-5678-9ABC-DEF012345678}"),
        ),
        ("ascii128", "A".repeat(128)),
        ("ascii4096", "A".repeat(4096)),
        ("cyrillic", "Привет, мир! ".repeat(4)),
        ("cjk", "温度電流制御".repeat(16)),
        ("emoji", "😀😃🚀🦀".repeat(16)),
        ("mixed", "OxiFOC: ток 20А 🔧".repeat(8)),
    ];
    for (name, s) in cases {
        assert_eq!(count_utf16(&s), fold_utf16(&s));
        run(&s, 5000, false);
        run(&s, 5000, true);
        let slowest = run(&s, 10_000, false).max(run(&s, 10_000, true)).max(0.1);
        let n = (20_000_000.0 / slowest).clamp(1000.0, 20_000_000.0) as u64;
        let mut count = Vec::new();
        let mut fold = Vec::new();
        for sample in 0..15 {
            if sample % 2 == 0 {
                count.push(run(&s, n, false));
                fold.push(run(&s, n, true));
            } else {
                fold.push(run(&s, n, true));
                count.push(run(&s, n, false));
            }
        }
        // Preserve chronological samples as well as medians for auditability.
        println!(
            "{{\"case\":\"{name}\",\"utf8_bytes\":{},\"utf16_units\":{},\"iterations\":{n},\"count_ns\":{count:?},\"fold_ns\":{fold:?}}}",
            s.len(),
            count_utf16(&s)
        );
    }
}
