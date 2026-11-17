//! Read PCSR while running. Attach and detach with phase outputs disabled.
//! Records are little-endian u32 PC and flags. Bit 0 means phase channels
//! were enabled at both ends of the sampling block. Session setup/cleanup
//! can halt the core, so only the sampling loop is a non-halting operation.

#[cfg(not(feature = "desktop"))]
fn main() {
    eprintln!("pc_sampler requires the desktop feature");
}

#[cfg(feature = "desktop")]
fn main() -> anyhow::Result<()> {
    use anyhow::{Context, bail, ensure};
    use probe_rs::{MemoryInterface, Permissions, probe::list::Lister};
    use std::{
        io::{Read, Write},
        sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        },
        time::{Duration, Instant},
    };

    const DWT_PCSR: u64 = 0xe000_101c;
    const TIM1_CCER: u64 = 0x4001_0020;
    const DHCSR: u64 = 0xe000_edf0;
    let args: Vec<String> = std::env::args().collect();
    if !(5..=7).contains(&args.len()) {
        bail!("usage: pc_sampler SERIAL SECONDS BLOCK OUT [SPEED_KHZ [controlled]]");
    }
    let seconds: f64 = args[2].parse()?;
    ensure!(
        seconds.is_finite() && seconds > 0.0 && seconds <= 120.0,
        "use 0..120 seconds"
    );
    let block: usize = args[3].parse()?;
    ensure!((1..=4096).contains(&block), "block must be 1..4096");
    let speed: u32 = args.get(5).map_or(Ok(24_000), |value| value.parse())?;
    let controlled = args.get(6).is_some_and(|value| value == "controlled");
    ensure!(args.len() < 7 || controlled, "unknown sampling mode");
    let stop = Arc::new(AtomicBool::new(false));
    if controlled {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            let mut byte = [0];
            let _ = std::io::stdin().read(&mut byte);
            stop.store(true, Ordering::Release);
        });
    }
    let probes = Lister::new().list_all();
    let info = probes
        .iter()
        .find(|probe| probe.serial_number.as_deref() == Some(args[1].as_str()))
        .context("probe not found")?;
    let mut probe = info.open()?;
    probe.set_speed(speed)?;
    let mut session = probe.attach("STM32F405RGTx", Permissions::default())?;
    let mut core = session.core(0)?;
    ensure!(core.read_word_32(DHCSR)? & (1 << 17) == 0, "core halted");
    ensure!(
        core.read_word_32(TIM1_CCER)? & 0x555 == 0,
        "attach while Stopped"
    );
    let mut output = std::io::BufWriter::new(std::fs::File::create_new(&args[4])?);
    eprintln!("attached");
    let start = Instant::now();
    let duration = Duration::from_secs_f64(seconds);
    let mut buffer = vec![0u32; block];
    let (mut total, mut active) = (0u64, 0u64);
    loop {
        let before = core.read_word_32(TIM1_CCER)?;
        let finished = start.elapsed() >= duration || stop.load(Ordering::Acquire);
        if finished && before & 0x555 == 0 {
            break;
        }
        ensure!(
            start.elapsed() < duration + Duration::from_secs(2),
            "phase outputs remain enabled after the sampling deadline"
        );
        for slot in &mut buffer {
            *slot = core.read_word_32(DWT_PCSR)?;
        }
        let after = core.read_word_32(TIM1_CCER)?;
        let driven = u32::from(before & 0x555 != 0 && after & 0x555 != 0);
        for pc in &buffer {
            output.write_all(&pc.to_le_bytes())?;
            output.write_all(&driven.to_le_bytes())?;
        }
        total += block as u64;
        active += u64::from(driven) * block as u64;
    }
    output.flush()?;
    // A timeout caller must command Stop before this session is dropped too.
    ensure!(
        core.read_word_32(TIM1_CCER)? & 0x555 == 0,
        "sampling ended before Stop"
    );
    eprintln!(
        "samples={total} driven={active} seconds={:.3}",
        start.elapsed().as_secs_f64()
    );
    Ok(())
}
