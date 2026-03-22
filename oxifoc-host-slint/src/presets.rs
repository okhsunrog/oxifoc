/// Motor preset for pre-filling detection parameters.
///
/// Values match VESC Tool's FOC detection dialog presets.
pub struct MotorPreset {
    pub name: &'static str,
    /// Number of magnetic pole pairs (poles / 2)
    pub pole_pairs: u8,
    /// Maximum power dissipation during detection (W)
    pub max_power_loss_w: f32,
    /// Open-loop ERPM for sensorless startup
    pub openloop_erpm: f32,
    /// ERPM threshold for sensorless transition
    pub sensorless_erpm: f32,
}

pub static PRESETS: &[MotorPreset] = &[
    MotorPreset {
        name: "Mini Outrunner (~75g)",
        pole_pairs: 7,
        max_power_loss_w: 20.0,
        openloop_erpm: 1400.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Small Outrunner (~200g)",
        pole_pairs: 7,
        max_power_loss_w: 50.0,
        openloop_erpm: 1400.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Medium Outrunner (~750g)",
        pole_pairs: 7,
        max_power_loss_w: 120.0,
        openloop_erpm: 700.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Large Outrunner (~2kg)",
        pole_pairs: 7,
        max_power_loss_w: 400.0,
        openloop_erpm: 700.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Small Inrunner (~200g)",
        pole_pairs: 1,
        max_power_loss_w: 50.0,
        openloop_erpm: 1400.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Medium Inrunner (~750g)",
        pole_pairs: 2,
        max_power_loss_w: 140.0,
        openloop_erpm: 1400.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "Large Inrunner (~2kg)",
        pole_pairs: 2,
        max_power_loss_w: 400.0,
        openloop_erpm: 1000.0,
        sensorless_erpm: 4000.0,
    },
    MotorPreset {
        name: "E-Bike DD Hub Motor",
        pole_pairs: 23,
        max_power_loss_w: 150.0,
        openloop_erpm: 300.0,
        sensorless_erpm: 2000.0,
    },
    MotorPreset {
        name: "EDF Inrunner Small (~200g)",
        pole_pairs: 3,
        max_power_loss_w: 110.0,
        openloop_erpm: 1400.0,
        sensorless_erpm: 4000.0,
    },
];

/// Preset names for UI ComboBox, including "Custom" as last entry
pub fn preset_names() -> Vec<&'static str> {
    let mut names: Vec<&str> = PRESETS.iter().map(|p| p.name).collect();
    names.push("Custom");
    names
}
