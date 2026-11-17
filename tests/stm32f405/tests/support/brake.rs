//! Full-grant and q-only authority costs with frozen-reference comparisons.
use super::dissipation_reference as reference;
use super::dwt::{Stats, batch};
use core::hint::black_box;
use oxifoc_core::foc::{
    controller::{Decoupling, FocController},
    phase::{PhaseInput, PhaseOutput, PhaseProvider, RotorVelocityEstimate},
    pwm::{PhasePwm, PhaseState, SvpwmModulator},
    sensors::CurrentSensor,
    trig::FastSinCos,
};
use oxifoc_core::motor::dissipation::{self, RegenBudget, WindingModel};
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::motor::{ControlMode, FocDriver};

struct InertPwm;

impl PhasePwm for InertPwm {
    fn max_duty(&self) -> u16 {
        5600
    }
    fn set_duties(&mut self, duties: [u16; 3]) {
        black_box(duties);
    }
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        black_box(states);
    }
}

struct ZeroCurrent;

impl CurrentSensor for ZeroCurrent {
    fn read_currents(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
    fn read_raw(&self) -> (u16, u16, u16) {
        (2048, 2048, 2048)
    }
    fn is_calibrated(&self) -> bool {
        true
    }
    fn get_offsets(&self) -> (f32, f32, f32) {
        (2048.0, 2048.0, 2048.0)
    }
    fn set_offsets(&mut self, _: (f32, f32, f32)) {}
}

struct FixedRotor(f32);

impl PhaseProvider for FixedRotor {
    fn get(&self) -> PhaseOutput {
        PhaseOutput {
            angle: 0.0,
            velocity: self.0,
        }
    }
    fn update(&mut self, _: &PhaseInput, _: u64) {}
    fn rotor_velocity(&self) -> Option<RotorVelocityEstimate> {
        RotorVelocityEstimate::new(self.0)
    }
}

#[inline(never)]
fn driver_batch(seed: u32) -> u64 {
    let heat = black_box(match seed >> 16 {
        0 => 0.0,
        1 => 0.05,
        _ => 1.0,
    });
    let mut controller = FocController::<SvpwmModulator, FastSinCos>::from_motor_params(
        black_box(0.064_200_59),
        black_box(40.991_37e-6),
        black_box(24.0),
    );
    controller.set_decoupling(Some(black_box(Decoupling {
        ld_h: 31.216_663e-6,
        lq_h: 50.766_08e-6,
        flux_linkage_wb: 0.003_976_720_4,
        resistance_ohm: 0.064_200_59,
    })));
    let mut driver = FocDriver::new(
        controller,
        InertPwm,
        ZeroCurrent,
        FixedRotor(black_box(450.0)),
        1.0 / 20_000.0,
    );
    driver.set_current_limits(CurrentLimits {
        bus_in_max_a: 5.0,
        bus_regen_max_a: 0.0,
        max_dissipation_w: heat,
        ..CurrentLimits::from_max_current(30.0 / 1.3)
    });
    driver.set_mode(ControlMode::VelocityControl {
        target_vel: black_box(450.0),
    });
    let mut checksum = 0u64;
    for step in 1..=64 {
        let output = black_box(&mut driver).step(step * 1000);
        defmt::assert!(output.is_ok());
        if let Ok(out) = output {
            checksum = checksum.wrapping_add(u64::from(black_box(out.vq).to_bits()));
        }
    }
    checksum
}

fn input(mode: u32, index: u32) -> (WindingModel, f32, f32, f32, f32) {
    let n = (index & 63) as f32;
    match mode {
        0 => (
            WindingModel {
                r_ohm: 0.064_200_59,
                flux_wb: 0.003_976_720_4,
                saliency_h: 1.954_941_7e-5,
            },
            -(30.0 / 1.3),
            420.0 + n,
            0.720_565,
            30.0 / 1.3,
        ),
        1 => (
            WindingModel {
                r_ohm: 0.036,
                flux_wb: 0.00292,
                saliency_h: 0.0002,
            },
            -(0.1 + n * 0.01),
            120.0,
            8.0,
            30.0,
        ),
        _ => (
            WindingModel {
                r_ohm: 0.036,
                flux_wb: 0.0102,
                saliency_h: 0.0002,
            },
            -(25.0 + n * 0.001),
            100.0,
            20.0,
            30.0,
        ),
    }
}

const BUDGET: RegenBudget = RegenBudget {
    frac: 1.0,
    max_w: 0.0,
};

#[inline(never)]
fn reference_batch(seed: u32) -> u64 {
    let mut sum = 0u64;
    for index in 0..64 {
        let (m, iq, omega, heat, current) = black_box(input(seed >> 16, seed + index));
        let model = reference::WindingModel {
            r_ohm: m.r_ohm,
            flux_wb: m.flux_wb,
            saliency_h: m.saliency_h,
        };
        let budget = reference::RegenBudget {
            frac: BUDGET.frac,
            max_w: BUDGET.max_w,
        };
        let value = black_box(reference::dissipate(
            iq, omega, &model, &budget, heat, current,
        ));
        sum = sum.wrapping_add(value.map_or(0, |(d, q)| {
            (u64::from(d.to_bits()) << 32) | u64::from(q.to_bits())
        }));
    }
    sum
}

#[inline(never)]
fn full_batch(seed: u32) -> u64 {
    let mut sum = 0u64;
    for index in 0..64 {
        let (model, iq, omega, heat, current) = black_box(input(seed >> 16, seed + index));
        let value = black_box(dissipation::dissipate(
            iq, omega, &model, &BUDGET, heat, current,
        ));
        sum = sum.wrapping_add(value.map_or(0, |(d, q)| {
            (u64::from(d.to_bits()) << 32) | u64::from(q.to_bits())
        }));
    }
    sum
}

#[inline(never)]
fn q_only_batch(seed: u32) -> u64 {
    let mut sum = 0u64;
    for index in 0..64 {
        let (model, iq, omega, heat, current) = black_box(input(seed >> 16, seed + index));
        let value = black_box(dissipation::dissipate_iq(
            iq, omega, &model, &BUDGET, heat, current,
        ));
        sum = sum.wrapping_add(value.map_or(0, |q| u64::from(q.to_bits())));
    }
    sum
}

pub fn run() {
    for mode in 0..3 {
        let mut driver_stats = Stats::new();
        for round in 0..33 {
            let cycles = batch(driver_batch, (mode << 16) | round);
            if round > 0 {
                driver_stats.record(cycles);
            }
        }
        driver_stats.report(
            [
                "velocity-due-plain",
                "velocity-due-heat005",
                "velocity-due-heat1",
            ][mode as usize],
            32,
            64,
        );
    }
    let mut stats = [[Stats::new(); 3]; 3];
    let functions: [fn(u32) -> u64; 3] = [reference_batch, full_batch, q_only_batch];
    for mode in 0..3 {
        for round in 0..33 {
            for index in 0..3 {
                let variant = (index + round as usize) % 3;
                let cycles = batch(functions[variant], (mode << 16) | round);
                if round != 0 {
                    stats[mode as usize][variant].record(cycles);
                }
            }
        }
    }
    let mut state = 0x74d5_192bu32;
    let mut option_mismatches = 0u32;
    let mut full_mismatches = 0u32;
    let mut q_mismatches = 0u32;
    let mut max_q_delta = 0.0f32;
    for _ in 0..30000 {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let current = 1.0 + (state & 1023) as f32 * 0.06;
        let heat = current * ((state >> 10) & 255) as f32 / 255.0;
        let omega = 1.0 + ((state >> 18) & 2047) as f32;
        let q = -current * ((state >> 9) & 511) as f32 / 511.0;
        let model = WindingModel {
            r_ohm: 0.005 + (state & 255) as f32 * 0.001,
            flux_wb: 0.001 + ((state >> 8) & 255) as f32 * 0.00004,
            saliency_h: if state & 1 == 0 { 0.0 } else { 0.0002 },
        };
        let budget = RegenBudget {
            frac: ((state >> 17) & 3).min(2) as f32 * 0.5,
            max_w: if state & 2 == 0 { 0.0 } else { 5.0 },
        };
        let r = reference::dissipate(
            q,
            omega,
            &reference::WindingModel {
                r_ohm: model.r_ohm,
                flux_wb: model.flux_wb,
                saliency_h: model.saliency_h,
            },
            &reference::RegenBudget {
                frac: budget.frac,
                max_w: budget.max_w,
            },
            heat,
            current,
        );
        let full = dissipation::dissipate(q, omega, &model, &budget, heat, current);
        let projected = dissipation::dissipate_iq(q, omega, &model, &budget, heat, current);
        match (r, full, projected) {
            (Some((rd, rq)), Some((d, iq)), Some(projected)) => {
                full_mismatches +=
                    u32::from(rd.to_bits() != d.to_bits() || rq.to_bits() != iq.to_bits());
                q_mismatches += u32::from(rq.to_bits() != projected.to_bits());
                max_q_delta = max_q_delta.max((rq - projected).abs());
            }
            (None, None, None) => {}
            _ => option_mismatches += 1,
        }
    }
    let names = [
        [
            "brake-authority-reference",
            "brake-authority-full",
            "brake-authority-q-only",
        ],
        [
            "brake-small-reference",
            "brake-small-full",
            "brake-small-q-only",
        ],
        [
            "brake-circle-reference",
            "brake-circle-full",
            "brake-circle-q-only",
        ],
    ];
    for mode in 0..3 {
        for variant in 0..3 {
            stats[mode][variant].report(names[mode][variant], 32, 64);
        }
    }
    defmt::info!(
        "BRAKECHECK n=30000 option_mismatch={} full_bit_mismatch={} q_bit_mismatch={} max_q_delta_a={}",
        option_mismatches,
        full_mismatches,
        q_mismatches,
        max_q_delta
    );
}
