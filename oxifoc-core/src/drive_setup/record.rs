//! Frozen flash representation. Evolve by appending a variant, never by
//! changing V1 or embedding mutable wire/config structs in its layout.
use super::{DriveCurrent, DriveSetup, Feedback, SetupError, ValidatedDriveSetup};
use crate::storage::{HallCalibrationConfig, HallTransitionGeometryConfig, MotorParamsConfig};
use crate::types::commissioning::SourceLimits;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SetupRecord {
    V1(SetupV1),
}
impl sequential_storage::map::PostcardValue<'_> for SetupRecord {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SetupV1 {
    resistance: f32,
    ld: f32,
    lq: f32,
    flux: f32,
    pole_pairs: u8,
    power_loss: f32,
    ld_fundamental: f32,
    lq_fundamental: f32,
    feedback: FeedbackV1,
    input_a: f32,
    regen_a: f32,
    current: CurrentV1,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
enum FeedbackV1 {
    Sensorless,
    Hall {
        angles: [f32; 8],
        valid: [bool; 8],
        geometry: Option<GeometryV1>,
    },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
struct GeometryV1 {
    entry_angles: [f32; 8],
    next_states: [u8; 8],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
enum CurrentV1 {
    Auto,
    Override(f32),
}

impl From<&ValidatedDriveSetup> for SetupRecord {
    fn from(value: &ValidatedDriveSetup) -> Self {
        let setup = value.setup();
        let motor = &setup.motor;
        Self::V1(SetupV1 {
            resistance: motor.resistance_ohm,
            ld: motor.inductance_d_h,
            lq: motor.inductance_q_h,
            flux: motor.flux_linkage_wb,
            pole_pairs: motor.pole_pairs,
            power_loss: motor.max_power_loss_w,
            ld_fundamental: motor.ld_fundamental_h,
            lq_fundamental: motor.lq_fundamental_h,
            feedback: match &setup.feedback {
                Feedback::Sensorless => FeedbackV1::Sensorless,
                Feedback::Hall(h) => FeedbackV1::Hall {
                    angles: h.angles,
                    valid: h.valid,
                    geometry: h.geometry.map(|g| GeometryV1 {
                        entry_angles: g.entry_angles,
                        next_states: g.next_states,
                    }),
                },
            },
            input_a: setup.source.input_a,
            regen_a: setup.source.regen_a,
            current: match setup.current {
                DriveCurrent::Auto => CurrentV1::Auto,
                DriveCurrent::Override(a) => CurrentV1::Override(a),
            },
        })
    }
}
impl SetupRecord {
    /// Recompute derived values against this board's protection at every boot.
    pub fn validate(
        self,
        board_trip_a: f32,
        configured_trip_a: f32,
    ) -> Result<ValidatedDriveSetup, SetupError> {
        let Self::V1(v) = self;
        DriveSetup {
            motor: MotorParamsConfig {
                resistance_ohm: v.resistance,
                inductance_d_h: v.ld,
                inductance_q_h: v.lq,
                flux_linkage_wb: v.flux,
                pole_pairs: v.pole_pairs,
                max_power_loss_w: v.power_loss,
                ld_fundamental_h: v.ld_fundamental,
                lq_fundamental_h: v.lq_fundamental,
                // Derived by validation, not stored twice.
                max_current_a: 0.0,
            },
            feedback: match v.feedback {
                FeedbackV1::Sensorless => Feedback::Sensorless,
                FeedbackV1::Hall {
                    angles,
                    valid,
                    geometry,
                } => Feedback::Hall(HallCalibrationConfig {
                    angles,
                    valid,
                    geometry: geometry.map(|g| HallTransitionGeometryConfig {
                        entry_angles: g.entry_angles,
                        next_states: g.next_states,
                    }),
                }),
            },
            source: SourceLimits {
                input_a: v.input_a,
                regen_a: v.regen_a,
            },
            current: match v.current {
                CurrentV1::Auto => DriveCurrent::Auto,
                CurrentV1::Override(a) => DriveCurrent::Override(a),
            },
        }
        .validate(board_trip_a, configured_trip_a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn ready<T>(f: impl Future<Output = T>) -> T {
        let mut f = core::pin::pin!(f);
        match f.as_mut().poll(&mut core::task::Context::from_waker(
            core::task::Waker::noop(),
        )) {
            core::task::Poll::Ready(v) => v,
            _ => panic!("mock flash yielded"),
        }
    }

    #[test]
    fn interrupted_replacement_recovers_one_complete_setup() {
        use crate::storage::{ConfigKey, InstalledDrive, RuntimeConfig};
        use sequential_storage::{
            cache::Cache,
            map::{MapConfig, MapStorage},
            mock_flash::{MockFlashBase, WriteCountCheck},
        };
        type Flash = MockFlashBase<4, 4, 128>;
        let old = DriveSetup {
            motor: MotorParamsConfig {
                resistance_ohm: 0.063,
                inductance_d_h: 31e-6,
                inductance_q_h: 51e-6,
                flux_linkage_wb: 0.004,
                pole_pairs: 7,
                max_power_loss_w: 120.0,
                ..Default::default()
            },
            feedback: Feedback::Hall(HallCalibrationConfig::default()),
            current: DriveCurrent::Auto,
            source: SourceLimits {
                input_a: 5.0,
                regen_a: 0.0,
            },
        }
        .validate(70.0, 30.0)
        .unwrap();
        let mut next = old.setup().clone();
        next.feedback = Feedback::Sensorless;
        next.motor.resistance_ohm = 0.05;
        next.source = SourceLimits {
            input_a: 12.0,
            regen_a: 2.0,
        };
        let next = next.validate(70.0, 30.0).unwrap();
        let mut store = MapStorage::<ConfigKey, _, _>::new(
            Flash::new(WriteCountCheck::TwiceWithZero, None, true),
            MapConfig::new(Flash::FULL_FLASH_RANGE),
            Cache::new_uncached(),
        );
        let old_record = InstalledDrive::new(&old);
        let next_record = InstalledDrive::new(&next);
        let key = ConfigKey::InstalledDrive;
        let mut buf = [0; 512];
        let mut interrupted = 0;
        let mut completed = 0;
        // Exercise fresh append and successive page positions, including reclaim.
        for _ in 0..12 {
            ready(store.store_item(&mut buf, &key, &old_record)).unwrap();
            let image = store.flash().clone();
            for cut in 0..1024 {
                let mut flash = image.clone();
                flash.bytes_until_shutoff = Some(cut);
                let mut attempt = MapStorage::<ConfigKey, _, _>::new(
                    flash,
                    MapConfig::new(Flash::FULL_FLASH_RANGE),
                    Cache::new_uncached(),
                );
                if ready(attempt.store_item(&mut buf, &key, &next_record)).is_ok() {
                    completed += 1;
                } else {
                    interrupted += 1;
                }
                let (mut flash, _) = attempt.destroy();
                flash.bytes_until_shutoff = None;
                let mut reboot = MapStorage::<ConfigKey, _, _>::new(
                    flash,
                    MapConfig::new(Flash::FULL_FLASH_RANGE),
                    Cache::new_uncached(),
                );
                let loaded = ready(reboot.fetch_item::<InstalledDrive>(&mut buf, &key))
                    .unwrap()
                    .unwrap();
                let mut config = RuntimeConfig::default();
                assert!(loaded.project(&mut config, 70.0));
                let matches = |setup: &ValidatedDriveSetup| {
                    config.motor_params.as_ref() == Some(&setup.setup().motor)
                        && config.current_limits.as_ref() == Some(setup.limits())
                        && config.hall_calibration == setup.hall()
                };
                assert!(matches(&old) || matches(&next), "mixed setup at cut {cut}");
            }
        }
        assert!(interrupted > 0 && completed > 0);
    }

    #[test]
    fn record_roundtrip_preserves_all_choices_and_revalidates_protection() {
        for feedback in [
            Feedback::Sensorless,
            Feedback::Hall(HallCalibrationConfig::default()),
        ] {
            for current in [DriveCurrent::Auto, DriveCurrent::Override(20.0)] {
                let setup = DriveSetup {
                    motor: MotorParamsConfig {
                        resistance_ohm: 0.063,
                        inductance_d_h: 31e-6,
                        inductance_q_h: 51e-6,
                        flux_linkage_wb: 0.004,
                        pole_pairs: 7,
                        max_power_loss_w: 120.0,
                        ..Default::default()
                    },
                    feedback: feedback.clone(),
                    current,
                    source: SourceLimits {
                        input_a: 5.0,
                        regen_a: 0.0,
                    },
                }
                .validate(70.0, 30.0)
                .unwrap();
                let mut buf = [0; 256];
                let bytes = postcard::to_slice(&SetupRecord::from(&setup), &mut buf).unwrap();
                assert_eq!(bytes[0], 0); // V1 is permanently variant zero.
                let loaded: SetupRecord = postcard::from_bytes(bytes).unwrap();
                assert_eq!(
                    loaded.clone().validate(70.0, 30.0).unwrap().setup(),
                    setup.setup()
                );
                let lowered = loaded.validate(70.0, 13.0);
                match current {
                    DriveCurrent::Auto => assert_eq!(lowered.unwrap().limits().max_iq_a, 10.0),
                    DriveCurrent::Override(_) => {
                        assert!(matches!(lowered, Err(SetupError::Current)));
                    }
                }
                for end in 0..bytes.len() {
                    assert!(postcard::from_bytes::<SetupRecord>(&bytes[..end]).is_err());
                }
            }
        }
        assert!(postcard::from_bytes::<SetupRecord>(&[99]).is_err());
    }
}
