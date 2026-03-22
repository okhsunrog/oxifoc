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
    view_offset:     u32,   // samples to shift back from write_pos (for pan)
    _pad0:           u32,
    _pad1:           u32,
    _pad2:           u32,
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
    let start  = (params.write_pos + params.num_samples - params.visible_samples - params.view_offset)
                 % params.num_samples;
    let actual = (start + index) % params.num_samples;
    return samples[actual * params.num_channels + channel];
}

// Map a data value to a normalised Y coordinate in [0, 1].
fn value_to_y(v: f32) -> f32 {
    return (v - params.y_min) / (params.y_max - params.y_min);
}

// Distance from point p to the line segment from a to b,
// measured in pixel space (accounting for aspect ratio).
fn dist_to_segment_px(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
    let closest = a + t * ab;
    let diff = p - closest;
    // Scale to pixel space: X in pixels, Y in pixels
    let diff_px = vec2<f32>(
        diff.x * f32(params.texture_width),
        diff.y * f32(params.texture_height)
    );
    return length(diff_px);
}

// ── Fragment shader ───────────────────────────────────────────────────────────
//
// Hybrid rendering:
// - When samples_per_pixel <= 4: LINE mode — connects consecutive samples
//   with anti-aliased line segments for a smooth waveform.
// - When samples_per_pixel > 4: PEAK-DETECT mode — finds min/max per pixel
//   column and draws vertical bars, like an oscilloscope.

@fragment
fn fs_main(@location(0) uv: vec2<f32>) -> @location(0) vec4<f32> {
    var color = vec3<f32>(0.0);
    var alpha = 0.0;

    let px_y = 1.0 / f32(params.texture_height);
    let px_x = 1.0 / f32(params.texture_width);
    let vis  = params.visible_samples;

    let samples_per_pixel = f32(vis) / f32(params.texture_width);
    let use_lines = samples_per_pixel <= 8.0;

    if use_lines {
        // ── LINE MODE ────────────────────────────────────────────────────
        // Check all line segments that could contribute to this pixel.
        // At close zoom (< 1 sample/pixel), each segment spans many pixels,
        // so we need enough margin to find the segment passing through us.
        let sample_center = uv.x * f32(vis - 1u);
        let half_span = max(samples_per_pixel * 0.5 + 1.0, 2.0);
        let s_start = u32(clamp(floor(sample_center - half_span), 0.0, f32(vis - 2u)));
        let s_end   = u32(clamp(ceil(sample_center + half_span), 1.0, f32(vis - 1u)));

        let pixel_pos = vec2<f32>(uv.x, uv.y);

        for (var ch = 0u; ch < params.num_channels; ch++) {
            var min_dist = 1e30f;

            for (var s = s_start; s < s_end; s++) {
                let va = get_sample(ch, s);
                let vb_val = get_sample(ch, s + 1u);
                if va == va && vb_val == vb_val {
                    let pa = vec2<f32>(f32(s) / f32(vis - 1u), 1.0 - value_to_y(va));
                    let pb = vec2<f32>(f32(s + 1u) / f32(vis - 1u), 1.0 - value_to_y(vb_val));
                    min_dist = min(min_dist, dist_to_segment_px(pixel_pos, pa, pb));
                }
            }

            // min_dist is already in pixel space
            let dist_px = min_dist;

            let line_col   = channel_colors.data[ch].rgb;
            let line_alpha = smoothstep(1.5, 0.0, dist_px);
            let glow_alpha = smoothstep(4.0, 0.0, dist_px) * 0.2;

            color = mix(color, line_col * 0.35, glow_alpha);
            alpha = max(alpha, glow_alpha);
            color = mix(color, line_col, line_alpha);
            alpha = max(alpha, line_alpha);
        }
    } else {
        // ── PEAK-DETECT MODE ─────────────────────────────────────────────
        // For every pixel column, find min/max of all samples mapping to it
        // and draw a vertical line segment (oscilloscope style).
        let sample_center = uv.x * f32(vis - 1u);
        let half_span  = max(samples_per_pixel * 0.5, 0.5);
        let s_start    = u32(clamp(floor(sample_center - half_span), 0.0, f32(vis - 1u)));
        let s_end      = u32(clamp(ceil( sample_center + half_span), 0.0, f32(vis - 1u)));
        let iter_count = min(s_end - s_start + 1u, 256u);

        for (var ch = 0u; ch < params.num_channels; ch++) {
            var val_min = 1e30f;
            var val_max = -1e30f;
            var valid = false;

            for (var i = 0u; i < iter_count; i++) {
                let v = get_sample(ch, s_start + i);
                if v == v {
                    val_min = min(val_min, v);
                    val_max = max(val_max, v);
                    valid = true;
                }
            }

            if !valid { continue; }

            let y_top = 1.0 - value_to_y(val_max);
            let y_bot = 1.0 - value_to_y(val_min);

            var dist: f32;
            if      uv.y < y_top { dist = y_top - uv.y; }
            else if uv.y > y_bot { dist = uv.y - y_bot; }
            else                 { dist = 0.0; }

            let line_col   = channel_colors.data[ch].rgb;
            let line_alpha = smoothstep(px_y * 2.0, 0.0, dist);
            let glow_alpha = smoothstep(px_y * 6.0, 0.0, dist) * 0.25;

            color = mix(color, line_col * 0.35, glow_alpha);
            alpha = max(alpha, glow_alpha);
            color = mix(color, line_col, line_alpha);
            alpha = max(alpha, line_alpha);
        }
    }

    return vec4<f32>(color, alpha);
}
