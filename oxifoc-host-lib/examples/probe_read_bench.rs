//! Read-only SRAM throughput and target-progress benchmark. Never resets/runs/halts.

#[cfg(not(feature = "desktop"))]
fn main() {
    eprintln!("probe_read_bench requires the desktop feature");
}

#[cfg(feature = "desktop")]
fn main() -> anyhow::Result<()> {
    bench::run()
}

#[cfg(feature = "desktop")]
mod bench {
    use anyhow::{Context, Result, bail, ensure};
    use object::{Object, ObjectSection, ObjectSymbol, SectionKind};
    use probe_rs::{
        Core, MemoryInterface, Permissions,
        probe::{WireProtocol, list::Lister},
    };
    use serde_json::{Value, json};
    use std::{
        path::PathBuf,
        time::{Duration, Instant, SystemTime, UNIX_EPOCH},
    };

    const RAM: std::ops::Range<u64> = 0x2000_0000..0x2002_0000;
    const HELP: &str = "probe_read_bench --elf PATH [--speed-khz 24000] [--block-bytes 4096] [--seconds 3] [--idle-seconds 1] [--warmup-seconds 2] [--probe VID:PID:SERIAL] [--chip STM32F405RGTx] [--address 0x...] [--seq-address 0x...]\nReads only; one attached session runs idle-before, read-load, idle-after. No reset/halt/run.";

    struct Args {
        elf: PathBuf,
        speed: u32,
        block: usize,
        seconds: Duration,
        idle: Duration,
        warmup: Duration,
        probe: Option<String>,
        chip: String,
        address: Option<u64>,
        sequence: Option<u64>,
    }

    fn number(value: &str) -> Result<u64> {
        Ok(if let Some(hex) = value.strip_prefix("0x") {
            u64::from_str_radix(hex, 16)?
        } else {
            value.parse()?
        })
    }

    fn parse() -> Result<Option<Args>> {
        let mut result = Args {
            elf: PathBuf::new(),
            speed: 24_000,
            block: 4096,
            seconds: Duration::from_secs(3),
            idle: Duration::from_secs(1),
            warmup: Duration::from_secs(2),
            probe: None,
            chip: "STM32F405RGTx".into(),
            address: None,
            sequence: None,
        };
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            if arg == "--help" || arg == "-h" {
                println!("{HELP}");
                return Ok(None);
            }
            let value = args
                .next()
                .with_context(|| format!("missing value for {arg}"))?;
            match arg.as_str() {
                "--elf" => result.elf = value.into(),
                "--speed-khz" => result.speed = value.parse()?,
                "--block-bytes" => result.block = value.parse()?,
                "--seconds" => result.seconds = Duration::try_from_secs_f64(value.parse()?)?,
                "--idle-seconds" => result.idle = Duration::try_from_secs_f64(value.parse()?)?,
                "--warmup-seconds" => result.warmup = Duration::try_from_secs_f64(value.parse()?)?,
                "--probe" => result.probe = Some(value),
                "--chip" => result.chip = value,
                "--address" => result.address = Some(number(&value)?),
                "--seq-address" => result.sequence = Some(number(&value)?),
                _ => bail!("unknown argument {arg}; {HELP}"),
            }
        }
        ensure!(!result.elf.as_os_str().is_empty(), "--elf required; {HELP}");
        ensure!(
            result.speed > 0 && result.block > 0 && result.block.is_multiple_of(4),
            "speed and aligned block must be positive"
        );
        ensure!(
            !result.seconds.is_zero() && !result.idle.is_zero(),
            "phase durations must be positive"
        );
        Ok(Some(result))
    }

    fn unix_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }

    fn progress(core: &mut Core<'_>, sequence: Option<u64>, origin: Instant) -> Result<Value> {
        let host_begin = origin.elapsed().as_nanos() as u64;
        let unix_begin = unix_ns();
        let tim5_before = core.read_word_32(0x4000_0c24)?;
        let adc_sequence = sequence
            .map(|address| core.read_word_32(address))
            .transpose()?;
        let dwt_cycles = core.read_word_32(0xe000_1004)?;
        let tim5_after = core.read_word_32(0x4000_0c24)?;
        let dhcsr = core.read_word_32(0xe000_edf0)?;
        ensure!(dhcsr & (1 << 17) == 0, "core halted; refusing to resume it");
        let dwt_ctrl = core.read_word_32(0xe000_1000)?;
        let demcr = core.read_word_32(0xe000_edfc)?;
        let apb1_freeze = core.read_word_32(0xe004_2008)?;
        Ok(json!({
            "host_begin_ns":host_begin,"host_end_ns":origin.elapsed().as_nanos() as u64,
            "unix_begin_ns":unix_begin,"unix_end_ns":unix_ns(),
            "tim5_before":tim5_before,"adc_sequence":adc_sequence,
            "dwt_cycles":dwt_cycles,"tim5_after":tim5_after,"dhcsr":dhcsr,
            "dwt_ctrl":dwt_ctrl,"demcr":demcr,"dbgmcu_apb1fz":apb1_freeze,
            "progress_qualified":adc_sequence.is_some() && apb1_freeze & (1 << 3) == 0
                && dwt_cycles != 0 && dwt_ctrl & 1 != 0 && demcr & (1 << 24) != 0,
        }))
    }

    fn phase(
        core: &mut Core<'_>,
        args: &Args,
        sequence: Option<u64>,
        address: u64,
        block: &mut [u32],
        origin: Instant,
        phase: (&str, bool),
    ) -> Result<Value> {
        let (name, load) = phase;
        let before = progress(core, sequence, origin)?;
        let duration = if load { args.seconds } else { args.idle };
        let unix_begin = unix_ns();
        let began = Instant::now();
        let monotonic_begin = began.duration_since(origin).as_nanos() as u64;
        let mut reads = 0_u64;
        let mut latency_sum = 0_u128;
        let mut latency_min = u128::MAX;
        let mut latency_max = 0;
        if load {
            while began.elapsed() < duration {
                let read_start = Instant::now();
                core.read_32(address, block)?;
                let latency = read_start.elapsed().as_nanos();
                latency_sum += latency;
                latency_min = latency_min.min(latency);
                latency_max = latency_max.max(latency);
                reads += 1;
            }
        } else {
            std::thread::sleep(duration);
        }
        let elapsed = began.elapsed();
        let monotonic_end = origin.elapsed().as_nanos() as u64;
        let unix_end = unix_ns();
        let after = progress(core, sequence, origin)?;
        let bytes = reads * args.block as u64;
        Ok(json!({
            "name":name,"status":"ok","read_load":load,
            "unix_begin_ns":unix_begin,"unix_end_ns":unix_end,
            "monotonic_begin_ns":monotonic_begin,"monotonic_end_ns":monotonic_end,
            "host_elapsed_s":elapsed.as_secs_f64(),"reads":reads,"bytes":bytes,
            "bytes_per_second":bytes as f64 / elapsed.as_secs_f64(),
            "latency_ns":if reads==0 {Value::Null} else {json!({
                "min":latency_min as u64,"mean":(latency_sum / u128::from(reads)) as u64,
                "max":latency_max as u64})},
            "progress_qualified":before["progress_qualified"] == true && after["progress_qualified"] == true,
            "before":before,"after":after,
        }))
    }

    pub fn run() -> Result<()> {
        let Some(args) = parse()? else {
            return Ok(());
        };
        let data = std::fs::read(&args.elf).context("read exact firmware ELF")?;
        let elf = object::File::parse(data.as_slice()).context("parse firmware ELF")?;
        let address = args
            .address
            .or_else(|| {
                elf.sections()
                    .filter(|section| {
                        matches!(
                            section.kind(),
                            SectionKind::Data | SectionKind::UninitializedData
                        )
                    })
                    .filter(|section| {
                        RAM.contains(&section.address())
                            && section
                                .address()
                                .checked_add(section.size())
                                .is_some_and(|end| end <= RAM.end)
                    })
                    .filter(|section| {
                        section.size() >= args.block as u64 && section.address().is_multiple_of(4)
                    })
                    .max_by_key(|section| section.size())
                    .map(|section| section.address())
            })
            .context("no static SRAM slice large enough; provide --address")?;
        ensure!(
            address.is_multiple_of(4)
                && RAM.contains(&address)
                && address
                    .checked_add(args.block as u64)
                    .is_some_and(|end| end <= RAM.end),
            "read slice outside aligned STM32F405 SRAM"
        );
        let mut candidates: Vec<_> = elf
            .symbols()
            .filter(|symbol| {
                symbol.size() == 4
                    && RAM.contains(&symbol.address())
                    && symbol
                        .name()
                        .is_ok_and(|name| name.contains("ADC_SEQUENCE"))
            })
            .map(|symbol| symbol.address())
            .collect();
        candidates.sort_unstable();
        candidates.dedup();
        let sequence = match (args.sequence, candidates.as_slice()) {
            (Some(address), _) => Some(address),
            (None, [address]) => Some(*address),
            (None, []) => None,
            _ => bail!("ambiguous ADC_SEQUENCE; provide --seq-address"),
        };
        ensure!(
            sequence
                .is_none_or(|address| address.is_multiple_of(4)
                    && (RAM.start..RAM.end - 3).contains(&address)),
            "sequence address outside SRAM"
        );
        let probes = Lister::new().list_all();
        let matches: Vec<_> = probes
            .iter()
            .filter(|probe| {
                args.probe.as_ref().is_none_or(|selector| {
                    let short = format!("{:04x}:{:04x}", probe.vendor_id, probe.product_id);
                    selector.eq_ignore_ascii_case(&short)
                        || selector
                            == &format!(
                                "{short}:{}",
                                probe.serial_number.as_deref().unwrap_or_default()
                            )
                })
            })
            .collect();
        ensure!(
            matches.len() == 1,
            "select exactly one probe with --probe; found {} matches",
            matches.len()
        );
        let identity = format!("{:?}", matches[0]);
        let mut probe = matches[0].open().context("open probe")?;
        probe.select_protocol(WireProtocol::Swd)?;
        let actual_speed = probe.set_speed(args.speed)?;
        let mut session = probe
            .attach(args.chip.as_str(), Permissions::default())
            .context("normal attach (no reset)")?;
        let mut core = session.core(0)?;
        ensure!(
            !core.core_halted()?,
            "core already halted; refusing to resume it"
        );
        // Existing firmware may re-arm DWT from its low-rate task after attach.
        // Observe that state instead of enabling counters through SWD.
        std::thread::sleep(args.warmup);
        let registers = json!({
            "tim1_arr":core.read_word_32(0x4001_002c)?,
            "tim1_psc":core.read_word_32(0x4001_0028)?,
            "tim5_psc":core.read_word_32(0x4000_0c28)?,
            "dbgmcu_apb2fz":core.read_word_32(0xe004_200c)?,
        });
        let mut block = vec![0_u32; args.block / 4];
        let origin = Instant::now();
        let mut phases = Vec::new();
        let mut failure = None;
        for (name, load) in [
            ("idle_before", false),
            ("read_load", true),
            ("idle_after", false),
        ] {
            match phase(
                &mut core,
                &args,
                sequence,
                address,
                &mut block,
                origin,
                (name, load),
            ) {
                Ok(result) => phases.push(result),
                Err(error) => {
                    failure = Some(format!("{error:#}"));
                    break;
                }
            }
        }
        println!(
            "{}",
            serde_json::to_string_pretty(&json!({
                "schema_version":1,"status":if failure.is_some(){"error"}else{"ok"},"error":failure,
                "probe":identity,"protocol":"SWD","requested_speed_khz":args.speed,"actual_speed_khz":actual_speed,
                "elf":args.elf,"elf_bytes":data.len(),"address":address,"sequence_address":sequence,
                "block_bytes":args.block,"read_seconds":args.seconds.as_secs_f64(),"idle_seconds":args.idle.as_secs_f64(),
                "warmup_seconds":args.warmup.as_secs_f64(),"target_registers":registers,
                "phases":phases,
            }))?
        );
        if let Some(error) = failure {
            bail!("{error}");
        }
        Ok(())
    }
}
