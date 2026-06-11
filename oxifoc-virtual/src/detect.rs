//! Virtual detection backend — runs the real detection algorithms against a
//! `VirtualMotor` simulation, plugged into the shared `detect_server`.
//!
//! Each measurement runs the actual sweep functions from
//! `oxifoc_core::foc::detection::sweep` against a freshly initialised
//! `VirtualHardware` / `VirtualTimer` simulation on a blocking thread (timer
//! delays advance the simulation instantly — no real time elapses).

use ergot::net_stack::NetStackHandle;
use ergot::net_stack::endpoints::Endpoints;
use oxifoc_core::foc::detection::sweep::{
    calibrate_hall, measure_flux_linkage_auto, measure_inductance_auto, measure_resistance,
};
use oxifoc_core::foc::detection::types::{
    DetectionError, FluxLinkageParams, InductanceParams, ResistanceParams,
};
use oxifoc_core::foc::detection::virtual_harness::{
    VirtualHallReader, VirtualTimer, block_on, with_sim,
};
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::foc::trig::LibmSinCos;
use oxifoc_core::runtime::detect::{DetectionBackend, detect_server as core_detect_server};
use oxifoc_core::virtual_motor::MotorParams;

/// Detection backend backed by the virtual-motor simulation.
struct VirtualBackend {
    vbus: f32,
    /// Simulated motor under test — same parameter set the live sim runs
    /// (CLI overrides like --pole-pairs land here, not in defaults).
    params: MotorParams,
}

impl DetectionBackend for VirtualBackend {
    fn vbus(&self) -> f32 {
        self.vbus
    }

    async fn measure_resistance(
        &mut self,
        params: &ResistanceParams,
    ) -> Result<f32, DetectionError> {
        let params = *params;
        let vbus = self.vbus;
        let params_m = self.params;
        tokio::task::spawn_blocking(move || {
            with_sim(params_m, vbus, |hw| {
                block_on(measure_resistance::<_, VirtualTimer>(hw, &params))
            })
        })
        .await
        .unwrap_or(Err(DetectionError::HardwareFault))
    }

    async fn measure_inductance(
        &mut self,
        params: &InductanceParams,
        pwm_freq_hz: f32,
    ) -> Result<(f32, f32), DetectionError> {
        let params = *params;
        let vbus = self.vbus;
        let params_m = self.params;
        tokio::task::spawn_blocking(move || {
            with_sim(params_m, vbus, |hw| {
                // HFI with voltage-pulse fallback — same ladder the boards run.
                block_on(measure_inductance_auto::<_, VirtualTimer, LibmSinCos>(
                    hw,
                    &params,
                    pwm_freq_hz,
                ))
            })
        })
        .await
        .unwrap_or(Err(DetectionError::HardwareFault))
    }

    async fn measure_flux(&mut self, params: &FluxLinkageParams) -> Result<f32, DetectionError> {
        let params = *params;
        let vbus = self.vbus;
        let params_m = self.params;
        tokio::task::spawn_blocking(move || {
            with_sim(params_m, vbus, |hw| {
                // Same ladder the boards run: spin-down first (the virtual
                // harness has coast telemetry), driven fallback otherwise.
                block_on(measure_flux_linkage_auto::<_, VirtualTimer>(hw, &params))
            })
        })
        .await
        .unwrap_or(Err(DetectionError::HardwareFault))
    }

    async fn calibrate_hall(
        &mut self,
        params: HallCalibrationParams,
    ) -> Result<HallCalibrationResult, DetectionError> {
        let vbus = self.vbus;
        let params_m = self.params;
        tokio::task::spawn_blocking(move || {
            with_sim(params_m, vbus, |hw| {
                let hall_reader = VirtualHallReader;
                block_on(calibrate_hall::<_, VirtualTimer, _>(
                    hw,
                    &hall_reader,
                    params,
                ))
            })
        })
        .await
        .unwrap_or(Err(DetectionError::HardwareFault))
    }
}

/// Serve detection for the virtual platform (delegates to the shared server,
/// which handles dedup, the safe-current formulas, and Hall persistence).
pub async fn detect_server<NS: NetStackHandle>(
    endpoints: Endpoints<NS>,
    vbus: f32,
    max_current_a: f32,
    foc_freq_hz: u32,
    motor_params: MotorParams,
) {
    core_detect_server(
        endpoints,
        VirtualBackend {
            vbus,
            params: motor_params,
        },
        max_current_a,
        foc_freq_hz,
        Some(&crate::RUNTIME_CONFIG),
    )
    .await
}
