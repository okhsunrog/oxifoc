use crate::{PlotBuffer, PlotConfig, PlotRenderer, required_wgpu_settings};
use slint::wgpu_30::wgpu;

fn pixels(device: &wgpu::Device, queue: &wgpu::Queue, texture: &wgpu::Texture) -> Vec<u8> {
    let row = (texture.width() * 4).div_ceil(256) * 256;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: u64::from(row * texture.height()),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row),
                rows_per_image: None,
            },
        },
        texture.size(),
    );
    queue.submit(Some(encoder.finish()));
    let (tx, rx) = std::sync::mpsc::channel();
    buffer
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    rx.recv().unwrap().unwrap();
    buffer.slice(..).get_mapped_range().unwrap().to_vec()
}

/// Run explicitly on a machine with a Vulkan/Metal/DX12 adapter.
#[test]
#[ignore = "requires a GPU adapter"]
fn peak_tail_line_mode_reset_and_cache() {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&Default::default())).unwrap();
    let settings = required_wgpu_settings(32768, 1);
    let (device, queue) = pollster::block_on(
        adapter.request_device(&wgpu::DeviceDescriptor {
            required_features: settings.device_required_features,
            required_limits: settings
                .device_required_limits
                .using_resolution(adapter.limits()),
            ..Default::default()
        }),
    )
    .unwrap();
    let buffer = PlotBuffer::new(1, 32768);
    let mut data = vec![0.0; 32768];
    data[500] = 0.8; // First column's tail: beyond the old 256-iteration limit.
    buffer.push_batch(&data);
    let mut renderer = PlotRenderer::new(
        &device,
        &queue,
        PlotConfig {
            num_channels: 1,
            capacity: 32768,
            y_min: -1.0,
            y_max: 1.0,
            auto_range: false,
            channel_colors: vec![[1.0, 0.0, 0.0, 1.0]],
        },
    );
    let texture = renderer.render(&buffer, 64, 64, 32768, 0, 1.0).texture;
    let peak = pixels(&device, &queue, &texture);
    assert!(
        peak[6 * 256 + 3] > 128,
        "peak beyond sample 256 must remain visible"
    );
    let texture = renderer
        .render(&buffer, 64, 64, 32768, u32::MAX, 1.0)
        .texture;
    assert_eq!(
        peak,
        pixels(&device, &queue, &texture),
        "pan must clamp to available history"
    );
    buffer.clear();
    let texture = renderer.render(&buffer, 64, 64, 32768, 0, 1.0).texture;
    assert!(
        pixels(&device, &queue, &texture)
            .chunks_exact(4)
            .all(|p| p[3] == 0)
    );
    buffer.push_batch(&[0.0, 0.0]);
    let texture = renderer.render(&buffer, 64, 64, 2, 0, 1.0).texture;
    let line = pixels(&device, &queue, &texture);
    assert!(line[32 * 256 + 32 * 4 + 3] > 128);
    let texture = renderer.render(&buffer, 64, 64, 32768, 0, 1.0).texture;
    let sparse = pixels(&device, &queue, &texture);
    assert!(sparse[32 * 256 + 63 * 4 + 3] > 128);
    assert_eq!(sparse[32 * 256 + 3], 0);

    // Projection must use the pixel metric on a wide, non-square plot.
    buffer.clear();
    buffer.push_batch(&[-0.5, 0.5]);
    let texture = renderer.render(&buffer, 320, 64, 2, 0, 1.0).texture;
    let diagonal = pixels(&device, &queue, &texture);
    assert!(diagonal[32 * 1280 + 160 * 4 + 3] > 128);
    assert!(diagonal[35 * 1280 + 160 * 4 + 3] < 16);
}
