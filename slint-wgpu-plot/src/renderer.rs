use crate::{MAX_CHANNELS, PlotBuffer};
use slint::wgpu_30::wgpu;

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
    peak_pipeline: wgpu::ComputePipeline,
    peak_bind_group: wgpu::BindGroup,
    snapshot: crate::buffer::Snapshot,
    texture: wgpu::Texture,
    samples_buffer: wgpu::Buffer,
    _colors_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// Reused scratch space for the CPU→GPU copy; allocated once.
    scratch: Vec<f32>,
    config: PlotConfig,
    /// Last content generation seen — skip upload+render when unchanged.
    last_generation: u64,
    /// Effective range used for the cached texture.
    last_y_min: f32,
    last_y_max: f32,
    /// Track whether we need to re-render due to resize (even if data unchanged).
    last_width: u32,
    last_height: u32,
    last_visible: u32,
    last_view_offset: u32,
}

impl PlotRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: PlotConfig) -> Self {
        assert!((1..=MAX_CHANNELS).contains(&config.num_channels));
        assert!(config.capacity >= 2);
        assert!(
            config.y_min.is_finite() && config.y_max.is_finite() && config.y_min < config.y_max
        );
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

        let samples_size = (config.capacity * config.num_channels * size_of::<f32>()) as u64;
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
            size: size_of::<ColorsUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&colors_buffer, 0, bytemuck::bytes_of(&colors_data));

        let peaks_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("plot_peaks"),
            size: u64::from(device.limits().max_texture_dimension_2d)
                * config.num_channels as u64
                * 16,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let mut entries = vec![
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
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
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plot_bgl"),
            entries: &entries,
        });
        entries[2].visibility = wgpu::ShaderStages::COMPUTE;
        entries[2].ty = wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: false },
            has_dynamic_offset: false,
            min_binding_size: None,
        };
        let peak_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plot_peak_bgl"),
            entries: &entries,
        });
        let peak_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plot_peak_bg"),
            layout: &peak_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: samples_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: colors_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: peaks_buffer.as_entire_binding(),
                },
            ],
        });
        let peak_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("plot_peak_layout"),
            bind_group_layouts: &[Some(&peak_bgl)],
            immediate_size: size_of::<PlotParams>() as u32,
        });
        let peak_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("plot_reduce"),
            source: wgpu::ShaderSource::Wgsl(include_str!("reduce.wgsl").into()),
        });
        let peak_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("plot_reduce"),
            layout: Some(&peak_layout),
            module: &peak_shader,
            entry_point: Some("reduce"),
            compilation_options: Default::default(),
            cache: None,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plot_bg"),
            layout: &bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: peaks_buffer.as_entire_binding(),
                },
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
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: size_of::<PlotParams>() as u32,
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
        let fallback_y_min = config.y_min;
        let fallback_y_max = config.y_max;

        Self {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            peak_pipeline,
            peak_bind_group,
            snapshot: Default::default(),
            texture,
            samples_buffer,
            _colors_buffer: colors_buffer,
            bind_group,
            scratch: Vec::with_capacity(config.capacity * config.num_channels),
            config,
            last_generation: u64::MAX, // force first render
            last_y_min: fallback_y_min,
            last_y_max: fallback_y_max,
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
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
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
        assert_eq!(buffer.capacity, self.config.capacity);
        assert_eq!(buffer.num_channels, self.config.num_channels);
        let uploads = buffer.sync_to(&mut self.scratch, &mut self.snapshot);
        for range in &uploads {
            self.queue.write_buffer(
                &self.samples_buffer,
                (range.start * size_of::<f32>()) as u64,
                bytemuck::cast_slice(&self.scratch[range.clone()]),
            );
        }
        let generation = self.snapshot.generation;
        if !uploads.is_empty() {
            self.last_generation = u64::MAX;
        }
        let view_offset = view_offset.min(self.snapshot.available.saturating_sub(vis));

        let reduce_peaks = (vis as f32 / width as f32) > 8.0
            && (generation != self.last_generation
                || vis != self.last_visible
                || view_offset != self.last_view_offset
                || width != self.last_width);

        // Check if anything changed since last render
        let needs_resize = width != self.last_width || height != self.last_height;
        let needs_render = generation != self.last_generation
            || vis != self.last_visible
            || view_offset != self.last_view_offset
            || needs_resize;

        if needs_resize {
            self.texture = Self::make_texture(&self.device, width, height);
        }

        if !needs_render {
            return (self.texture.clone(), self.last_y_min, self.last_y_max);
        }

        self.last_generation = generation;
        self.last_width = width;
        self.last_height = height;
        self.last_visible = vis;
        self.last_view_offset = view_offset;

        // Compute Y range from visible window only (auto-range)
        let (y_min, y_max) = if self.config.auto_range {
            let nch = buffer.num_channels;
            let cap = buffer.capacity;
            let wp = self.snapshot.write_pos as usize;
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
            write_pos: self.snapshot.write_pos,
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
        if reduce_peaks {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("plot_reduce"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.peak_pipeline);
            pass.set_bind_group(0, &self.peak_bind_group, &[]);
            pass.set_immediates(0, bytemuck::bytes_of(&params));
            pass.dispatch_workgroups((width * params.num_channels).div_ceil(64), 1, 1);
        }
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
        self.last_y_min = y_min;
        self.last_y_max = y_max;
        (self.texture.clone(), y_min, y_max)
    }

    /// Update the Y-axis range at runtime (e.g. for auto-scaling).
    pub fn set_y_range(&mut self, y_min: f32, y_max: f32) {
        assert!(y_min.is_finite() && y_max.is_finite() && y_min < y_max);
        self.config.y_min = y_min;
        self.config.y_max = y_max;
        self.last_generation = u64::MAX;
    }
}
