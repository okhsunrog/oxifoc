mod detect;
mod sim;
mod storage;
mod tcp_server;
mod udp_server;

pub struct TokioTimer;

impl Timer for TokioTimer {
    async fn after_millis(ms: u64) {
        tokio::time::sleep(std::time::Duration::from_millis(ms)).await;
    }

    async fn after_micros(us: u64) {
        tokio::time::sleep(std::time::Duration::from_micros(us)).await;
    }
}

use clap::Parser;
use oxifoc_core::storage::{CONFIG_LOADED, MotorParamsConfig, RuntimeConfig};
use oxifoc_core::timer::Timer;
use oxifoc_core::virtual_motor::MotorParams;
use tracing_subscriber::EnvFilter;

// Platform state globals
oxifoc_core::define_platform_state!(oxifoc_core::foc::fault::StandardFault);

/// Synthetic board electrical constants reported to the host (for telemetry
/// enrichment) and used by the sim to encode currents into raw ADC counts —
/// generic B-G431B-ESC1-like values so the enrich round-trip is meaningful.
pub const VIRTUAL_CALIB: oxifoc_core::types::BoardCalib = oxifoc_core::types::BoardCalib {
    shunt_ohms: 0.003,
    amp_gain: 64.0 / 7.0,
    adc_vref_mv: 3300,
    adc_max_counts: 4095,
    invert_current_sign: true,
    vbus_divider_ratio: 10.39,
};

// Runtime config shared between config server and simulation
static RUNTIME_CONFIG: critical_section::Mutex<core::cell::RefCell<RuntimeConfig>> =
    critical_section::Mutex::new(core::cell::RefCell::new(RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: None,
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
        failsafe: None,
        velocity: None,
        derating: None,
    }));

#[derive(Clone, Copy, Debug, PartialEq, clap::ValueEnum)]
enum Transport {
    Tcp,
    Udp,
}

#[derive(Parser)]
#[command(name = "oxifoc-virtual")]
#[command(about = "Virtual motor controller with ergot protocol")]
struct Args {
    /// Transport protocol
    #[arg(short, long, default_value = "tcp")]
    transport: Transport,
    /// Listen port
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
    /// Motor pole pairs
    #[arg(long, default_value_t = 7)]
    pole_pairs: u8,
    /// Maximum phase current (A)
    #[arg(long, default_value_t = 40.0)]
    max_current: f32,
    /// Load torque in N·m (opposes rotation, creates steady-state speed)
    #[arg(long, default_value_t = 0.0)]
    load: f32,
}

fn config_from_motor_params(params: MotorParams, max_current_a: f32) -> MotorParamsConfig {
    MotorParamsConfig {
        resistance_ohm: params.r,
        inductance_d_h: params.ld,
        inductance_q_h: params.lq,
        flux_linkage_wb: params.lambda,
        pole_pairs: params.pole_pairs,
        max_current_a,
        max_power_loss_w: 0.0,
        ..Default::default()
    }
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
        "oxifoc-virtual: {:?} port={}, foc={}Hz, batch={}, vbus={}V, pp={}, max_i={}A",
        args.transport,
        args.port,
        args.foc_freq,
        args.batch,
        args.vbus,
        args.pole_pairs,
        args.max_current,
    );

    // One motor parameter set for both the live sim and the detection
    // backend — CLI flags override the defaults here, nowhere else.
    let motor_params = MotorParams {
        pole_pairs: args.pole_pairs,
        ..Default::default()
    };

    // Storage worker uses !Send futures (sequential-storage internals),
    // so run it on a dedicated thread with a LocalSet.
    std::thread::spawn(|| {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(storage::storage_worker());
    });

    let mut loaded_config = CONFIG_LOADED.wait().await;
    if loaded_config.motor_params.is_none() {
        loaded_config.motor_params = Some(config_from_motor_params(motor_params, args.max_current));
    }
    critical_section::with(|cs| {
        *RUNTIME_CONFIG.borrow(cs).borrow_mut() = loaded_config;
    });
    tracing::info!("Config loaded from mock flash");

    // Spawn simulation loop
    tokio::spawn(sim::foc_loop(
        args.foc_freq,
        args.batch,
        args.vbus,
        args.load,
        motor_params,
        &STATE,
        &FAULT_REGISTRY,
    ));

    // Run server (blocks on accept/recv loop)
    match args.transport {
        Transport::Tcp => {
            tcp_server::run(
                args.port,
                args.foc_freq,
                args.max_current,
                args.vbus,
                motor_params,
                &STATE,
                &FAULT_REGISTRY,
                &RUNTIME_CONFIG,
            )
            .await
        }
        Transport::Udp => {
            // Box::pin: ~6 KB future; the 2 KB large_futures threshold is
            // tuned for firmware, on the host we just heap it.
            Box::pin(udp_server::run(
                args.port,
                args.foc_freq,
                args.max_current,
                args.vbus,
                motor_params,
                &STATE,
                &FAULT_REGISTRY,
                &RUNTIME_CONFIG,
            ))
            .await
        }
    }
}
