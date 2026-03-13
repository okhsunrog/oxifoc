// ── Vertex shader ────────────────────────────────────────────────────────────
// Full-screen triangle; renders into the texture via a single draw(0..3).

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0)       uv:       vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0,  3.0),
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
    );
    let pos = positions[vi];
    return VertexOutput(
        vec4<f32>(pos.x, -pos.y, 0.0, 1.0),
        vec2<f32>(pos.x * 0.5 + 0.5, 0.5 - pos.y * 0.5),
    );
}

// ── Types & bindings ─────────────────────────────────────────────────────────

struct PlotParams {
    write_pos:       u32,
    num_samples:     u32,   // == ring-buffer capacity
    y_min:           f32,
    y_max:           f32,
    num_channels:    u32,
    visible_samples: u32,
    texture_width:   u32,
    texture_height:  u32,
};

struct Colors {
    data: array<vec4<f32>, 8>,  // one entry per channel; MAX_CHANNELS = 8
};

var<immediate> params: PlotParams;

@group(0) @binding(0) var<storage, read> samples:        array<f32>;
@group(0) @binding(1) var<uniform>       channel_colors: Colors;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Read the value of `channel` at logical sample index `index` within the
// visible window, honouring the ring-buffer wrap.
fn get_sample(channel: u32, index: u32) -> f32 {
    let start  = (params.write_pos + params.num_samples - params.visible_samples)
                 % params.num_samples;
    let actual = (start + index) % params.num_samples;
    return samples[actual * params.num_channels + channel];
}

// Map a data value to a normalised Y coordinate in [0, 1].
fn value_to_y(v: f32) -> f32 {
    return (v - params.y_min) / (params.y_max - params.y_min);
}

// ── Fragment shader ───────────────────────────────────────────────────────────
//
// For every pixel column the shader finds the min/max of all samples that
// map to that column and draws a vertical line segment, exactly like an
// oscilloscope in "peak-detect" mode.  This handles the case where
// visible_samples >> texture_width (many samples per pixel).

@fragment
fn fs_main(@location(0) uv: vec2<f32>) -> @location(0) vec4<f32> {
    var color = vec3<f32>(0.0);
    var alpha = 0.0;

    let px_y = 1.0 / f32(params.texture_height);
    let vis  = params.visible_samples;

    let samples_per_pixel = f32(vis) / f32(params.texture_width);
    let sample_center     = uv.x * f32(vis - 1u);

    // At least one sample on each side of the column centre for continuity.
    let half_span  = max(samples_per_pixel * 0.5, 0.5);
    let s_start    = u32(clamp(floor(sample_center - half_span), 0.0, f32(vis - 1u)));
    let s_end      = u32(clamp(ceil( sample_center + half_span), 0.0, f32(vis - 1u)));
    let iter_count = min(s_end - s_start + 1u, 256u);

    for (var ch = 0u; ch < params.num_channels; ch++) {
        var val_min = get_sample(ch, s_start);
        var val_max = val_min;

        for (var i = 1u; i < iter_count; i++) {
            let v   = get_sample(ch, s_start + i);
            val_min = min(val_min, v);
            val_max = max(val_max, v);
        }

        // Screen-space Y (0 = top, 1 = bottom).
        let y_top = 1.0 - value_to_y(val_max);
        let y_bot = 1.0 - value_to_y(val_min);

        // Distance from the current pixel to the vertical line segment.
        var dist: f32;
        if      uv.y < y_top { dist = y_top - uv.y; }
        else if uv.y > y_bot { dist = uv.y - y_bot; }
        else                 { dist = 0.0; }

        let line_col      = channel_colors.data[ch].rgb;
        let line_alpha    = smoothstep(px_y * 2.0, 0.0, dist);
        let glow_alpha    = smoothstep(px_y * 6.0, 0.0, dist) * 0.25;

        color = mix(color, line_col * 0.35, glow_alpha);
        alpha = max(alpha, glow_alpha);
        color = mix(color, line_col, line_alpha);
        alpha = max(alpha, line_alpha);
    }

    return vec4<f32>(color, alpha);
}
