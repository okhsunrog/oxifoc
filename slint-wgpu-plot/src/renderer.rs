use crate::{MAX_CHANNELS, PlotBuffer};
use slint::wgpu_30::wgpu;
use std::path::Path;

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
    scale: f32,
    _pad: [u32; 2], // align to 16 bytes for GPU
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

/// Result of one [`PlotRenderer::render`] call: the texture plus the axis
/// range and tick count actually used (nice-snapped to 1-2-5 steps).
pub struct RenderOutput {
    pub texture: wgpu::Texture,
    pub y_min: f32,
    pub y_max: f32,
    pub y_divisions: u32,
    /// `false` when the cached texture was returned unchanged — the caller
    /// can skip updating UI properties and scheduling another frame.
    pub rendered: bool,
}

/// GPU renderer for one chart.  Create one instance per chart via
/// [`PlotRenderer::new`] inside Slint's `RenderingState::RenderingSetup`
/// callback.
pub struct PlotRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    peak_pipeline: wgpu::ComputePipeline,
    peak_bind_group: wgpu::BindGroup,
    peak_width_capacity: u32,
    snapshot: crate::buffer::Snapshot,
    texture: wgpu::Texture,
    samples_buffer: wgpu::Buffer,
    colors_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// Reused scratch space for the CPU→GPU copy; allocated once.
    scratch: Vec<f32>,
    config: PlotConfig,
    visible_channels: Vec<bool>,
    /// Last content generation seen — skip upload+render when unchanged.
    last_generation: u64,
    /// Raw auto-range target from the visible data (with margin).
    target_lo: f32,
    target_hi: f32,
    /// Smoothed range: expands to the target instantly, shrinks gradually.
    smooth_lo: f32,
    smooth_hi: f32,
    /// Nice-snapped range used for the cached texture.
    last_y_min: f32,
    last_y_max: f32,
    last_divisions: u32,
    /// Track whether we need to re-render due to resize (even if data unchanged).
    last_width: u32,
    last_height: u32,
    last_visible: u32,
    last_view_offset: u32,
    last_scale: f32,
}

/// Round `raw_step` up to the nearest "nice" 1-2-5×10ⁿ value.
fn nice_step(raw_step: f32) -> f32 {
    let mag = 10f32.powf(raw_step.log10().floor());
    let norm = raw_step / mag;
    let step = if norm < 1.5 {
        1.0
    } else if norm < 3.5 {
        2.0
    } else if norm < 7.5 {
        5.0
    } else {
        10.0
    };
    step * mag
}

/// Expand `[lo, hi]` outward to multiples of a nice step so grid lines and
/// labels land on round values. Returns `(y_min, y_max, divisions)`.
fn nice_axis(lo: f32, hi: f32, target_divisions: u32) -> (f32, f32, u32) {
    let span = (hi - lo).max(1e-9);
    let step = nice_step(span / target_divisions as f32);
    let y_lo = (lo / step).floor() * step;
    let y_hi = (hi / step).ceil() * step;
    let divisions = (((y_hi - y_lo) / step).round() as u32).max(1);
    (y_lo, y_hi, divisions)
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

        // Slint requests the adapter's full texture resolution, which can be
        // larger than our storage binding limit (e.g. 65536 columns on Mali).
        let peak_width_capacity = device.limits().max_texture_dimension_2d.min(
            (device.limits().max_storage_buffer_binding_size / (config.num_channels as u64 * 16))
                as u32,
        );
        let peaks_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("plot_peaks"),
            size: u64::from(peak_width_capacity) * config.num_channels as u64 * 16,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let mut entries = vec![
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX
                    | wgpu::ShaderStages::FRAGMENT
                    | wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
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

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("plot_lines"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8UnormSrgb,
                    blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let texture = Self::make_texture(device, 1, 1);
        let y_min = config.y_min;
        let y_max = config.y_max;

        Self {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            line_pipeline,
            peak_pipeline,
            peak_bind_group,
            peak_width_capacity,
            snapshot: Default::default(),
            texture,
            samples_buffer,
            colors_buffer,
            bind_group,
            scratch: Vec::with_capacity(config.capacity * config.num_channels),
            visible_channels: vec![true; config.num_channels],
            config,
            last_generation: u64::MAX, // force first render
            target_lo: y_min,
            target_hi: y_max,
            smooth_lo: y_min,
            smooth_hi: y_max,
            last_y_min: y_min,
            last_y_max: y_max,
            last_divisions: 1,
            last_width: 0,
            last_height: 0,
            last_visible: 0,
            last_view_offset: u32::MAX,
            last_scale: 0.0,
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

    /// Min/max of the visible window (with margin), from the CPU-side copy.
    fn scan_target(&self, buffer: &PlotBuffer, vis: u32, view_offset: u32) -> (f32, f32) {
        let nch = buffer.num_channels;
        let cap = buffer.capacity;
        let wp = self.snapshot.write_pos as usize;
        let mut lo = f32::INFINITY;
        let mut hi = f32::NEG_INFINITY;

        let total_offset = (vis as usize + view_offset as usize) % cap;
        let start = (wp + cap - total_offset) % cap;
        for i in 0..vis as usize {
            let frame_idx = (start + i) % cap;
            let base = frame_idx * nch;
            for ch in 0..nch {
                if !self.visible_channels[ch] {
                    continue;
                }
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
            let margin = (lo.abs() * 0.1).max(0.5);
            (lo - margin, lo + margin)
        } else {
            let margin = ((hi - lo) * 0.1).max(0.01);
            (lo - margin, hi + margin)
        }
    }

    /// Show a subset of channels without discarding the recorded samples.
    /// Hidden channels do not contribute to auto-range or the GPU waveform.
    pub fn set_channel_visibility(&mut self, visible: &[bool]) {
        assert_eq!(visible.len(), self.config.num_channels);
        if self.visible_channels == visible {
            return;
        }
        self.visible_channels.copy_from_slice(visible);
        let mut colors = ColorsUniform {
            data: [[0.0; 4]; MAX_CHANNELS],
        };
        for (index, color) in self.config.channel_colors.iter().enumerate() {
            colors.data[index] = *color;
            if !visible[index] {
                colors.data[index][3] = 0.0;
            }
        }
        self.queue
            .write_buffer(&self.colors_buffer, 0, bytemuck::bytes_of(&colors));
        self.last_generation = u64::MAX;
    }

    /// Render `buffer` into a texture of the requested pixel size.
    ///
    /// `visible_samples` is clamped to `[2, buffer.capacity]`; `scale_factor`
    /// is the window's physical-per-logical pixel ratio (hidpi).
    /// Call this from Slint's `RenderingState::BeforeRendering` on the main thread.
    ///
    /// When `auto_range` is on, the Y range follows the visible data:
    /// it expands instantly, shrinks gradually, and is snapped outward to
    /// nice 1-2-5 grid steps so axis labels stay round.
    pub fn render(
        &mut self,
        buffer: &PlotBuffer,
        width: u32,
        height: u32,
        visible_samples: u32,
        view_offset: u32,
        scale_factor: f32,
    ) -> RenderOutput {
        let width = width.clamp(1, self.peak_width_capacity);
        let height = height.clamp(1, self.device.limits().max_texture_dimension_2d);
        let scale_factor = if scale_factor > 0.0 {
            scale_factor
        } else {
            1.0
        };
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

        let data_changed = generation != self.last_generation;
        let view_changed = vis != self.last_visible || view_offset != self.last_view_offset;
        let needs_resize = width != self.last_width || height != self.last_height;
        let scale_changed = scale_factor != self.last_scale;

        let reduce_peaks = (vis as f32 / width as f32) * scale_factor > 8.0
            && (generation != self.last_generation
                || vis != self.last_visible
                || view_offset != self.last_view_offset
                || width != self.last_width
                || scale_factor != self.last_scale);

        if data_changed {
            self.last_generation = generation;
        }

        if self.config.auto_range {
            if data_changed || view_changed {
                (self.target_lo, self.target_hi) = self.scan_target(buffer, vis, view_offset);
            }
            // Hysteresis: grow to the target immediately (clipping is worse
            // than a jump), shrink at a gentle per-frame rate, and snap once
            // close enough so the render cache can settle.
            let span = (self.target_hi - self.target_lo).max(1e-6);
            let snap = span * 0.005;
            self.smooth_lo = if self.target_lo < self.smooth_lo {
                self.target_lo
            } else {
                let next = self.smooth_lo + (self.target_lo - self.smooth_lo) * 0.08;
                if self.target_lo - next < snap {
                    self.target_lo
                } else {
                    next
                }
            };
            self.smooth_hi = if self.target_hi > self.smooth_hi {
                self.target_hi
            } else {
                let next = self.smooth_hi + (self.target_hi - self.smooth_hi) * 0.08;
                if next - self.target_hi < snap {
                    self.target_hi
                } else {
                    next
                }
            };
        } else {
            self.smooth_lo = self.config.y_min;
            self.smooth_hi = self.config.y_max;
        }

        let (y_min, y_max, divisions) = nice_axis(self.smooth_lo, self.smooth_hi, 6);
        let range_changed = y_min != self.last_y_min || y_max != self.last_y_max;

        if needs_resize {
            self.texture = Self::make_texture(&self.device, width, height);
        }

        if !(data_changed || view_changed || needs_resize || scale_changed || range_changed) {
            return RenderOutput {
                texture: self.texture.clone(),
                y_min: self.last_y_min,
                y_max: self.last_y_max,
                y_divisions: self.last_divisions,
                rendered: false,
            };
        }

        self.last_width = width;
        self.last_height = height;
        self.last_visible = vis;
        self.last_view_offset = view_offset;
        self.last_scale = scale_factor;
        self.last_y_min = y_min;
        self.last_y_max = y_max;
        self.last_divisions = divisions;

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
            scale: scale_factor,
            _pad: [0; 2],
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
            let use_lines = vis as f32 / width as f32 * scale_factor <= 8.0;
            rpass.set_pipeline(if use_lines {
                &self.line_pipeline
            } else {
                &self.pipeline
            });
            rpass.set_bind_group(0, &self.bind_group, &[]);
            rpass.set_immediates(0, bytemuck::bytes_of(&params));
            if use_lines {
                rpass.draw(0..6, 0..((vis - 1) * params.num_channels));
            } else {
                rpass.draw(0..3, 0..1);
            }
        }
        self.queue.submit(Some(encoder.finish()));
        RenderOutput {
            texture: self.texture.clone(),
            y_min,
            y_max,
            y_divisions: divisions,
            rendered: true,
        }
    }

    /// Update the Y-axis range at runtime (used when `auto_range` is off).
    pub fn set_y_range(&mut self, y_min: f32, y_max: f32) {
        assert!(y_min.is_finite() && y_max.is_finite() && y_min < y_max);
        self.config.y_min = y_min;
        self.config.y_max = y_max;
        self.last_y_min = f32::NAN; // force re-render
    }

    /// Save the last rendered texture as a PNG, composited over `background`
    /// (linear-ish sRGB triplet, 0..1) since the plot itself is transparent.
    pub fn export_png(&self, path: &Path, background: [f32; 3]) -> Result<(), String> {
        let width = self.texture.size().width;
        let height = self.texture.size().height;
        let unpadded_row = width * 4;
        let padded_row = unpadded_row.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;

        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("plot_png_staging"),
            size: u64::from(padded_row) * u64::from(height),
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("plot_png_encoder"),
            });
        encoder.copy_texture_to_buffer(
            self.texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(padded_row),
                    rows_per_image: None,
                },
            },
            self.texture.size(),
        );
        self.queue.submit(Some(encoder.finish()));

        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| format!("GPU poll failed: {e}"))?;
        rx.recv()
            .map_err(|_| "map_async callback dropped".to_string())?
            .map_err(|e| format!("buffer map failed: {e}"))?;

        let bg = background.map(|c| {
            if c <= 0.0 {
                0.0
            } else if c >= 1.0 {
                255.0
            } else {
                c * 255.0
            }
        });
        let data = slice
            .get_mapped_range()
            .map_err(|e| format!("mapped range failed: {e}"))?;
        let mut rgb = Vec::with_capacity((width * height * 3) as usize);
        for row in 0..height {
            let start = (row * padded_row) as usize;
            for px in data[start..start + unpadded_row as usize].chunks_exact(4) {
                let a = f32::from(px[3]) / 255.0;
                for ch in 0..3 {
                    rgb.push((f32::from(px[ch]) * a + bg[ch] * (1.0 - a)).round() as u8);
                }
            }
        }
        drop(data);
        staging.unmap();

        let file = std::fs::File::create(path).map_err(|e| format!("create {path:?}: {e}"))?;
        let mut png_encoder = png::Encoder::new(std::io::BufWriter::new(file), width, height);
        png_encoder.set_color(png::ColorType::Rgb);
        png_encoder.set_depth(png::BitDepth::Eight);
        png_encoder
            .write_header()
            .and_then(|mut w| w.write_image_data(&rgb))
            .map_err(|e| format!("PNG encode failed: {e}"))?;
        Ok(())
    }
}
