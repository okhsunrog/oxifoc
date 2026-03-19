mod fault;
mod sim;
mod storage;
mod tcp_server;

use clap::Parser;
use tracing_subscriber::EnvFilter;

// Platform state globals
oxifoc_core::define_platform_state!(fault::VirtualFault);

// Runtime config shared between config server and simulation
static RUNTIME_CONFIG: critical_section::Mutex<
    core::cell::RefCell<oxifoc_core::storage::RuntimeConfig>,
> = critical_section::Mutex::new(core::cell::RefCell::new(
    oxifoc_core::storage::RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: None,
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
    },
));

#[derive(Parser)]
#[command(name = "oxifoc-virtual")]
#[command(about = "Virtual motor controller with ergot protocol over TCP")]
struct Args {
    /// TCP port to listen on
    #[arg(short, long, default_value_t = 2025)]
    port: u16,
    /// Simulated FOC frequency in Hz
    #[arg(short, long, default_value_t = 20_000)]
    foc_freq: u32,
    /// Simulation batch size (steps per sleep)
    #[arg(short, long, default_value_t = 100)]
    batch: usize,
    /// DC bus voltage (V)
    #[arg(long, default_value_t = 24.0)]
    vbus: f32,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let args = Args::parse();
    tracing::info!(
        "oxifoc-virtual: port={}, foc={}Hz, batch={}, vbus={}V",
        args.port,
        args.foc_freq,
        args.batch,
        args.vbus
    );

    // Storage worker uses !Send futures (sequential-storage internals),
    // so run it on a dedicated thread with a LocalSet.
    std::thread::spawn(|| {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(storage::storage_worker());
    });

    let loaded_config = oxifoc_core::storage::CONFIG_LOADED.wait().await;
    critical_section::with(|cs| {
        *RUNTIME_CONFIG.borrow(cs).borrow_mut() = loaded_config;
    });
    tracing::info!("Config loaded from mock flash");

    // Spawn simulation loop
    tokio::spawn(sim::foc_loop(
        args.foc_freq,
        args.batch,
        args.vbus,
        &STATE,
        &FAULT_REGISTRY,
    ));

    // Run TCP server (blocks on accept loop)
    tcp_server::run(args.port, &STATE, &FAULT_REGISTRY, &RUNTIME_CONFIG).await
}
