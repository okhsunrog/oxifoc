mod buffer;
mod renderer;

pub use buffer::PlotBuffer;
pub use renderer::{PlotConfig, PlotRenderer};

use slint::wgpu_30::{WGPUSettings, wgpu};

/// Maximum number of channels supported by the shader.
pub const MAX_CHANNELS: usize = 8;

/// Build the [`WGPUSettings`] required by this library.
///
/// - `max_capacity`  — largest ring-buffer capacity you will use across all charts.
/// - `max_channels`  — largest number of channels in any single chart.
///
/// Pass the returned settings to [`slint::BackendSelector::require_wgpu_30`].
pub fn required_wgpu_settings(max_capacity: usize, max_channels: usize) -> WGPUSettings {
    let mut s = WGPUSettings::default();
    s.device_required_features = wgpu::Features::IMMEDIATES;
    s.device_required_limits.max_immediate_size = size_of::<renderer::PlotParams>() as u32;
    s.device_required_limits
        .max_storage_buffers_per_shader_stage = 1;
    s.device_required_limits.max_storage_buffer_binding_size =
        (max_capacity * max_channels * size_of::<f32>()) as u64;
    s
}
