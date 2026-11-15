use slint::wgpu_30::WGPUSettings;

pub fn required_wgpu_settings(capacity: usize, channels: usize) -> WGPUSettings {
    let mut settings = WGPUSettings::default();
    settings.device_required_features = oxifoc_plot::required_features();
    settings.device_required_limits = oxifoc_plot::required_limits(capacity, channels);
    settings
}
