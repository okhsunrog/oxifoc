//! Motor parameter detection report against VirtualMotor.
//!
//! Runs `run_full_detection()` across a catalog of simulated motors,
//! printing a summary table with ground truth comparison.
//!
//! ```sh
//! cargo run -p oxifoc-core --example detection_report --features virtual-motor,std
//! ```

use oxifoc_core::foc::detection::sweep::{
    DetectionParams, measure_flux_linkage, measure_flux_linkage_magnitude,
};
use oxifoc_core::foc::detection::types::{FluxLinkageParams, MotorSize};
use oxifoc_core::foc::detection::virtual_harness::{
    VirtualHardware, VirtualTimer, block_on, run_detection, with_sim,
};
use oxifoc_core::units::quantity;
use oxifoc_core::units::{Electrical, amps, erpm, hertz, volts};
use oxifoc_core::virtual_motor::MotorParams;

// ── Motor catalog ──────────────────────────────────────────────────────────

struct MotorDef {
    name: &'static str,
    params: MotorParams,
    vbus: f32,
    motor_size: MotorSize,
    openloop_erpm: f32,
}

fn motor_catalog() -> Vec<MotorDef> {
    vec![
        MotorDef {
            name: "Default (hobby BLDC)",
            params: MotorParams {
                j: 5e-4,
                ..MotorParams::default()
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
            openloop_erpm: 1400.0,
        },
        MotorDef {
            name: "Micro gimbal",
            params: MotorParams {
                r: quantity(8.0),
                ld: quantity(3e-3),
                lq: quantity(3e-3),
                lambda: quantity(0.005),
                pole_pairs: 11,
                j: 5e-6,
                friction_b: 1e-5,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 12.0,
            motor_size: MotorSize::Mini,
            openloop_erpm: 1400.0,
        },
        MotorDef {
            name: "5010 drone motor",
            params: MotorParams {
                r: quantity(0.12),
                ld: quantity(2e-4),
                lq: quantity(2e-4),
                lambda: quantity(0.008),
                pole_pairs: 7,
                j: 3e-4,
                friction_b: 5e-5,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
            openloop_erpm: 1400.0,
        },
        MotorDef {
            name: "6354 eskate",
            params: MotorParams {
                r: quantity(0.035),
                ld: quantity(1.5e-5),
                lq: quantity(1.5e-5),
                lambda: quantity(0.0085),
                pole_pairs: 7,
                j: 1e-3,
                friction_b: 1e-3,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
            openloop_erpm: 700.0,
        },
        MotorDef {
            name: "8308 ebike hub",
            params: MotorParams {
                r: quantity(0.05),
                ld: quantity(4e-5),
                lq: quantity(4e-5),
                lambda: quantity(0.015),
                pole_pairs: 20,
                j: 5e-3,
                friction_b: 5e-3,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 72.0,
            motor_size: MotorSize::Large,
            openloop_erpm: 700.0,
        },
        // IPM motors (Ld ≠ Lq) — saliency test
        MotorDef {
            name: "IPM servo (mild)",
            params: MotorParams {
                r: quantity(0.3),
                ld: quantity(3e-4),
                lq: quantity(5e-4),
                lambda: quantity(0.012),
                pole_pairs: 4,
                j: 5e-4,
                friction_b: 1e-4,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
            openloop_erpm: 700.0,
        },
        MotorDef {
            name: "IPM traction (strong)",
            params: MotorParams {
                r: quantity(0.02),
                ld: quantity(1e-4),
                lq: quantity(3e-4),
                lambda: quantity(0.02),
                pole_pairs: 4,
                j: 2e-3,
                friction_b: 1e-3,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 96.0,
            motor_size: MotorSize::Large,
            openloop_erpm: 700.0,
        },
        // High-friction: spin-down fails, forces driven fallback
        MotorDef {
            name: "Robot joint (geared)",
            params: MotorParams {
                r: quantity(0.1),
                ld: quantity(1e-4),
                lq: quantity(1e-4),
                lambda: quantity(0.015),
                pole_pairs: 7,
                j: 5e-5,
                friction_b: 0.004,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
            openloop_erpm: 1400.0,
        },
        MotorDef {
            name: "Direct-drive gripper",
            params: MotorParams {
                r: quantity(0.5),
                ld: quantity(4e-4),
                lq: quantity(4e-4),
                lambda: quantity(0.015),
                pole_pairs: 7,
                j: 2e-5,
                friction_b: 0.005,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
            openloop_erpm: 1400.0,
        },
        MotorDef {
            name: "NEMA23 stepper-servo",
            params: MotorParams {
                r: quantity(1.2),
                ld: quantity(2e-3),
                lq: quantity(2e-3),
                lambda: quantity(0.04),
                pole_pairs: 50,
                j: 2e-4,
                friction_b: 1e-3,
                hall_offset: quantity(0.0),
                ..MotorParams::default()
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
            openloop_erpm: 5000.0,
        },
    ]
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn err_pct(measured: f32, expected: f32) -> f32 {
    if expected.abs() < 1e-12 {
        0.0
    } else {
        (measured - expected) / expected * 100.0
    }
}

fn fmt_err(measured: f32, expected: f32) -> String {
    let e = err_pct(measured, expected);
    if e.abs() < 0.05 {
        " 0.0%".to_string()
    } else {
        format!("{e:+5.1}%")
    }
}

fn det_params(def: &MotorDef) -> DetectionParams {
    DetectionParams {
        motor_size: def.motor_size,
        pole_pairs: def.params.pole_pairs,
        current_max: amps(10.0),
        max_power_loss: def.motor_size.max_power_loss(),
        pwm_freq: hertz(20_000.0),
        vbus: volts(def.vbus),
        openloop_speed: Some(erpm(def.openloop_erpm)),
    }
}

/// Non-ideal plant variant: sub-stepped integration (the rotor moves
/// *within* the FOC period — the ideal table is partly self-confirmation),
/// dead-time distortion (300 ns at 20 kHz; the harness configures the
/// matching duty-domain compensation, as the firmware does), and a 12-bit
/// ±31 A current sensor with 1 LSB of uniform noise (B-G431B-ESC1-class).
///
/// The one-cycle actuation pipeline (`actuation_delay: 1.0`) is ON:
/// the detection now measures the command→apply depth in place
/// (`probe_hfi_pipeline_lag`) instead of assuming one cycle, pairs the
/// demod through a command-history ring, cross-checks the demod L against
/// the latency-immune |Z| magnitude, and the pulse fallback self-aligns
/// on the actual application edge — so this table is the regression net
/// for the whole pipeline-skew fix.
fn nonideal(def: &MotorDef) -> MotorParams {
    const ADC_LSB_A: f32 = 62.0 / 4096.0; // 12-bit over ±31 A ≈ 15 mA
    MotorParams {
        substeps: 10,
        dead_time_voltage: volts(300e-9 * 20_000.0 * def.vbus),
        adc_lsb: amps(ADC_LSB_A),
        adc_noise: amps(ADC_LSB_A),
        actuation_delay: 1.0,
        ..def.params
    }
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let catalog = motor_catalog();

    println!("Motor Parameter Detection Report (VirtualMotor Simulation)");
    println!("==========================================================");
    println!();

    // ── Ground truth table ─────────────────────────────────────────────
    println!("Ground truth:");
    println!(
        "  {:<22} {:>7} {:>8} {:>8} {:>9} {:>4} {:>9}",
        "Motor", "R(Ohm)", "Ld(uH)", "Lq(uH)", "lm(mWb)", "pp", "J(g*m2)"
    );
    println!(
        "  {:-<22} {:->7} {:->8} {:->8} {:->9} {:->4} {:->9}",
        "", "", "", "", "", "", ""
    );
    for def in &catalog {
        let p = &def.params;
        println!(
            "  {:<22} {:>7.4} {:>8.1} {:>8.1} {:>9.3} {:>4} {:>9.2}",
            def.name,
            p.r.value,
            p.ld.value * 1e6,
            p.lq.value * 1e6,
            p.lambda.value * 1e3,
            p.pole_pairs,
            p.j * 1e3,
        );
    }
    println!();

    // ── Full detection results ─────────────────────────────────────────
    println!("Full detection results (run_full_detection, error vs ground truth):");
    println!(
        "  {:<22} {:>9} {:>9} {:>9} {:>12}",
        "Motor", "R", "Ld", "Lq", "lambda"
    );
    println!("  {:-<22} {:->9} {:->9} {:->9} {:->12}", "", "", "", "", "");

    for def in &catalog {
        let p = def.params;
        let result = run_detection(p, def.vbus, det_params(def));

        let (r_s, ld_s, lq_s, lam_s) = match result {
            Ok(det) => (
                fmt_err(det.params.resistance.value, p.r.value),
                fmt_err(det.params.inductance_d.value, p.ld.value),
                fmt_err(det.params.inductance_q.value, p.lq.value),
                fmt_err(det.params.flux_linkage.value, p.lambda.value),
            ),
            Err(_) => (
                " FAIL".into(),
                " FAIL".into(),
                " FAIL".into(),
                "  FAIL".into(),
            ),
        };

        println!(
            "  {:<22} {:>9} {:>9} {:>9} {:>12}",
            def.name, r_s, ld_s, lq_s, lam_s,
        );
    }
    println!();

    // ── Full detection, non-ideal plant ────────────────────────────────
    println!(
        "Non-ideal plant (substeps=10, dead-time 300ns@20kHz, 12-bit ADC + noise, 1-cycle delay):"
    );
    println!(
        "  {:<22} {:>9} {:>9} {:>9} {:>12}",
        "Motor", "R", "Ld", "Lq", "lambda"
    );
    println!("  {:-<22} {:->9} {:->9} {:->9} {:->12}", "", "", "", "", "");

    for def in &catalog {
        let p = def.params;
        let result = run_detection(nonideal(def), def.vbus, det_params(def));

        let (r_s, ld_s, lq_s, lam_s) = match result {
            Ok(det) => (
                fmt_err(det.params.resistance.value, p.r.value),
                fmt_err(det.params.inductance_d.value, p.ld.value),
                fmt_err(det.params.inductance_q.value, p.lq.value),
                fmt_err(det.params.flux_linkage.value, p.lambda.value),
            ),
            Err(e) => {
                eprintln!("  [{}] detection error: {e:?}", def.name);
                (
                    " FAIL".into(),
                    " FAIL".into(),
                    " FAIL".into(),
                    "  FAIL".into(),
                )
            }
        };

        println!(
            "  {:<22} {:>9} {:>9} {:>9} {:>12}",
            def.name, r_s, ld_s, lq_s, lam_s,
        );
    }
    println!();

    // ── HFI frequency benchmark (hfi-detect only) ──────────────────────
    #[cfg(feature = "hfi-detect")]
    {
        use oxifoc_core::foc::detection::sweep::measure_inductance;
        use oxifoc_core::foc::detection::types::InductanceParams;
        println!("HFI injection frequency (Ld/Lq error at 1kHz vs 2kHz vs 5kHz):");
        println!(
            "  {:<22} {:>14} {:>14} {:>14}",
            "Motor", "1kHz", "2kHz", "5kHz (cur)"
        );
        println!("  {:-<22} {:->14} {:->14} {:->14}", "", "", "", "");
        for def in &catalog {
            let p = def.params;
            let r_val = p.r;
            let max_hold = (def.vbus * 0.577 * 0.6) / r_val.max(quantity(0.001));
            let hold = 2.0f32.min(max_hold.value).max(0.1);

            let mut results = Vec::new();
            for freq in [1000.0f32, 2000.0, 5000.0] {
                let l = with_sim(p, def.vbus, |hw| {
                    let params = InductanceParams {
                        motor_size: def.motor_size,
                        resistance: r_val,
                        hold_current: amps(hold),
                        hfi_frequency: hertz(freq),
                        vbus: Some(volts(def.vbus)),
                        ..Default::default()
                    };
                    block_on(measure_inductance::<
                        VirtualHardware,
                        VirtualTimer,
                        oxifoc_core::foc::trig::LibmSinCos,
                    >(hw, &params, hertz(20_000.0)))
                });
                match l {
                    Ok((ld, lq)) => {
                        results.push(format!(
                            "{:+.1}/{:+.1}%",
                            err_pct(ld.value, p.ld.value),
                            err_pct(lq.value, p.lq.value)
                        ));
                    }
                    Err(_) => results.push("FAIL".to_string()),
                }
            }
            println!(
                "  {:<22} {:>14} {:>14} {:>14}",
                def.name, results[0], results[1], results[2]
            );
        }
        println!();
    } // end #[cfg(feature = "hfi-detect")] HFI frequency benchmark

    // ── Driven flux benchmark ──────────────────────────────────────────
    println!("Driven flux fallback: q-axis vs magnitude (prod: magnitude-first):");
    println!(
        "  {:<22} {:>10} {:>10} {:>10}",
        "Motor", "q-axis", "magnit.", "produced"
    );
    println!("  {:-<22} {:->10} {:->10} {:->10}", "", "", "", "");
    for def in &catalog {
        let p = def.params;
        // Same parameterization as run_full_detection: per-motor speed cap
        // from openloop_erpm and a back-EMF rise target of 0.2·vbus.
        let flux_params = FluxLinkageParams {
            motor_size: def.motor_size,
            resistance: p.r,
            inductance: Some((p.ld + p.lq) / 2.0),
            pole_pairs: p.pole_pairs,
            spin_speed: erpm(def.openloop_erpm).to_mechanical(p.pole_pairs),
            v_target: Some(volts(0.2 * def.vbus)),
            ..Default::default()
        };

        let qaxis = with_sim(p, def.vbus, |hw| {
            block_on(measure_flux_linkage::<VirtualHardware, VirtualTimer>(
                hw,
                &flux_params,
            ))
        });
        let q_s = match qaxis {
            Ok(v) => format!("{:+.1}%", err_pct(v.value, p.lambda.value)),
            Err(_) => "FAIL".to_string(),
        };

        let mag = with_sim(p, def.vbus, |hw| {
            block_on(measure_flux_linkage_magnitude::<
                VirtualHardware,
                VirtualTimer,
            >(hw, &flux_params))
        });
        let m_s = match mag {
            Ok(v) => format!("{:+.1}%", err_pct(v.value, p.lambda.value)),
            Err(_) => "FAIL".to_string(),
        };

        let prod_s = match (mag, qaxis) {
            (Ok(m), _) => format!("{:+.1}%", err_pct(m.value, p.lambda.value)),
            (Err(_), Ok(q)) => format!("{:+.1}%", err_pct(q.value, p.lambda.value)),
            _ => "FAIL".to_string(),
        };

        println!("  {:<22} {:>10} {:>10} {:>10}", def.name, q_s, m_s, prod_s);
    }

    println!();
    println!("Positive = overestimate, negative = underestimate.");
}
