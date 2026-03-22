use crate::{MAX_CHANNELS, PlotBuffer};
use slint::wgpu_28::wgpu;

// Matches the `PlotParams` struct in shader.wgsl exactly.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct PlotParams {
    write_pos: u32,
    num_samples: u32,
    y_min: f32,
    y_max: f32,
    num_channels: u32,
    visible_samples: u32,
    texture_width: u32,
    texture_height: u32,
    view_offset: u32,
    _pad: [u32; 3], // align to 16 bytes for GPU
}

// Matches the `Colors` struct in shader.wgsl.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct ColorsUniform {
    data: [[f32; 4]; MAX_CHANNELS],
}

/// Construction-time configuration for a [`PlotRenderer`].
pub struct PlotConfig {
    pub num_channels: usize,
    pub capacity: usize,
    /// Fallback Y-axis minimum (used when auto_range has no valid data)
    pub y_min: f32,
    /// Fallback Y-axis maximum (used when auto_range has no valid data)
    pub y_max: f32,
    /// Automatically compute Y range from visible data each frame
    pub auto_range: bool,
    /// RGBA colour per channel; length must equal `num_channels`.
    pub channel_colors: Vec<[f32; 4]>,
}

/// GPU renderer for one chart.  Create one instance per chart via
/// [`PlotRenderer::new`] inside Slint's `RenderingState::RenderingSetup`
/// callback.
pub struct PlotRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    texture: wgpu::Texture,
    samples_buffer: wgpu::Buffer,
    _colors_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// Reused scratch space for the CPU→GPU copy; allocated once.
    scratch: Vec<f32>,
    config: PlotConfig,
    /// Last write_pos seen — skip upload+render when unchanged.
    last_write_pos: u32,
    /// Track whether we need to re-render due to resize (even if data unchanged).
    last_width: u32,
    last_height: u32,
    last_visible: u32,
    last_view_offset: u32,
}

impl PlotRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: PlotConfig) -> Self {
        assert_eq!(
            config.channel_colors.len(),
            config.num_channels,
            "channel_colors length must equal num_channels"
        );

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("plot_shader"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!(
                "shader.wgsl"
            ))),
        });

        let samples_size =
            (config.capacity * config.num_channels * std::mem::size_of::<f32>()) as u64;
        let samples_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("plot_samples"),
            size: samples_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let mut colors_data = ColorsUniform {
            data: [[0.0; 4]; MAX_CHANNELS],
        };
        for (i, c) in config.channel_colors.iter().enumerate() {
            colors_data.data[i] = *c;
        }
        let colors_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("plot_colors"),
            size: std::mem::size_of::<ColorsUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&colors_buffer, 0, bytemuck::bytes_of(&colors_data));

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plot_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plot_bg"),
            layout: &bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: samples_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: colors_buffer.as_entire_binding(),
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("plot_pipeline_layout"),
            bind_group_layouts: &[&bgl],
            immediate_size: std::mem::size_of::<PlotParams>() as u32,
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("plot_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::TextureFormat::Rgba8UnormSrgb.into())],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let texture = Self::make_texture(device, 1, 1);

        Self {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            texture,
            samples_buffer,
            _colors_buffer: colors_buffer,
            bind_group,
            scratch: Vec::with_capacity(config.capacity * config.num_channels),
            config,
            last_write_pos: u32::MAX, // force first render
            last_width: 0,
            last_height: 0,
            last_visible: 0,
            last_view_offset: u32::MAX,
        }
    }

    fn make_texture(device: &wgpu::Device, width: u32, height: u32) -> wgpu::Texture {
        device.create_texture(&wgpu::TextureDescriptor {
            label: Some("plot_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
    }

    /// Render `buffer` into a texture of the requested pixel size.
    ///
    /// `visible_samples` is clamped to `[2, buffer.capacity]`.
    /// Call this from Slint's `RenderingState::BeforeRendering` on the main thread.
    ///
    /// Returns `(texture, actual_y_min, actual_y_max)`. When `auto_range` is true,
    /// y_min/y_max are computed from the visible data with 10% margin.
    pub fn render(
        &mut self,
        buffer: &PlotBuffer,
        width: u32,
        height: u32,
        visible_samples: u32,
        view_offset: u32,
    ) -> (wgpu::Texture, f32, f32) {
        let width = width.max(1);
        let height = height.max(1);
        let vis = visible_samples.clamp(2, buffer.capacity as u32);
        let wp = buffer.write_pos();

        // Check if anything changed since last render
        let needs_resize = width != self.last_width || height != self.last_height;
        let needs_render = wp != self.last_write_pos
            || vis != self.last_visible
            || view_offset != self.last_view_offset
            || needs_resize;

        if needs_resize {
            self.texture = Self::make_texture(&self.device, width, height);
        }

        if !needs_render {
            return (self.texture.clone(), self.config.y_min, self.config.y_max);
        }

        self.last_write_pos = wp;
        self.last_width = width;
        self.last_height = height;
        self.last_visible = vis;
        self.last_view_offset = view_offset;

        buffer.copy_to(&mut self.scratch);
        self.queue
            .write_buffer(&self.samples_buffer, 0, bytemuck::cast_slice(&self.scratch));

        // Compute Y range from visible window only (auto-range)
        let (y_min, y_max) = if self.config.auto_range {
            let nch = buffer.num_channels;
            let cap = buffer.capacity;
            let wp = buffer.write_pos() as usize;
            let mut lo = f32::INFINITY;
            let mut hi = f32::NEG_INFINITY;

            // Only scan the visible portion of the ring buffer (accounting for view_offset)
            let total_offset = (vis as usize + view_offset as usize) % cap;
            let start = (wp + cap - total_offset) % cap;
            for i in 0..vis as usize {
                let frame_idx = (start + i) % cap;
                let base = frame_idx * nch;
                for ch in 0..nch {
                    let v = self.scratch[base + ch];
                    if v.is_finite() {
                        lo = lo.min(v);
                        hi = hi.max(v);
                    }
                }
            }

            if !lo.is_finite() || !hi.is_finite() {
                // No valid data at all — use config defaults
                (self.config.y_min, self.config.y_max)
            } else if (hi - lo).abs() < 1e-6 {
                // Constant value — center with reasonable margin
                let center = lo;
                let margin = center.abs() * 0.1;
                let margin = margin.max(0.5);
                (center - margin, center + margin)
            } else {
                let margin = (hi - lo) * 0.1;
                let margin = margin.max(0.01);
                (lo - margin, hi + margin)
            }
        } else {
            (self.config.y_min, self.config.y_max)
        };

        let params = PlotParams {
            write_pos: buffer.write_pos(),
            num_samples: buffer.capacity as u32,
            y_min,
            y_max,
            num_channels: buffer.num_channels as u32,
            visible_samples: vis,
            texture_width: width,
            texture_height: height,
            view_offset,
            _pad: [0; 3],
        };

        let view = self
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("plot_encoder"),
            });
        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("plot_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.0,
                            b: 0.0,
                            a: 0.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            rpass.set_pipeline(&self.pipeline);
            rpass.set_bind_group(0, &self.bind_group, &[]);
            rpass.set_immediates(0, bytemuck::bytes_of(&params));
            rpass.draw(0..3, 0..1);
        }
        self.queue.submit(Some(encoder.finish()));
        // Cache Y range for skip-render path
        self.config.y_min = y_min;
        self.config.y_max = y_max;
        (self.texture.clone(), y_min, y_max)
    }

    /// Update the Y-axis range at runtime (e.g. for auto-scaling).
    pub fn set_y_range(&mut self, y_min: f32, y_max: f32) {
        self.config.y_min = y_min;
        self.config.y_max = y_max;
    }
}
