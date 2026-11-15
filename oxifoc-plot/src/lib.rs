//! UI-independent WGPU telemetry plotting.
mod buffer;
mod renderer;
pub use buffer::PlotBuffer;
pub use renderer::{PlotConfig, PlotRenderer, RenderOutput};
pub const MAX_CHANNELS: usize = 8;

#[cfg(test)]
mod gpu_tests;

/// Device capabilities shared by every frontend.
pub fn required_features() -> wgpu::Features {
    wgpu::Features::IMMEDIATES
}

pub fn required_limits(max_capacity: usize, max_channels: usize) -> wgpu::Limits {
    wgpu::Limits {
        max_immediate_size: size_of::<renderer::PlotParams>() as u32,
        max_storage_buffer_binding_size: ((max_capacity * max_channels * 4) as u64)
            .max(32768 * max_channels as u64 * 16),
        ..wgpu::Limits::default()
    }
}
