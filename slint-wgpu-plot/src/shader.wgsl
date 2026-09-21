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
        vec4<f32>(pos.x, pos.y, 0.0, 1.0),
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
@group(0) @binding(2) var<storage, read> peaks: array<vec4<f32>>;

@group(0) @binding(0) var<storage, read> samples:        array<f32>;
@group(0) @binding(1) var<uniform>       channel_colors: Colors;

fn finite(v: f32) -> bool {
    return (bitcast<u32>(v) & 0x7f800000u) != 0x7f800000u;
}

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

// Peak rendering only reads precomputed envelopes.
@fragment
fn fs_main(@location(0) uv: vec2<f32>) -> @location(0) vec4<f32> {
    var color = vec3<f32>(0.0);
    var alpha = 0.0;
    let px_y = 1.0 / f32(params.texture_height);
    let column = min(u32(uv.x * f32(params.texture_width)), params.texture_width - 1u);
    for (var ch = 0u; ch < params.num_channels; ch++) {
        let envelope = peaks[column * params.num_channels + ch];
        if envelope.z == 0.0 { continue; }
        let val_min = envelope.x;
        let val_max = envelope.y;

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
    return vec4<f32>(color, alpha);
}

struct LineVertex {
    @builtin(position) position: vec4<f32>,
    @location(0) @interpolate(flat) a: vec2<f32>,
    @location(1) @interpolate(flat) b: vec2<f32>,
    @location(2) @interpolate(flat) color: vec4<f32>,
};

// One oriented, glow-width quad per segment. Only pixels close to the
// waveform run the distance calculation, rather than the entire texture.
@vertex
fn vs_line(@builtin(vertex_index) vertex: u32, @builtin(instance_index) instance: u32) -> LineVertex {
    let segments = params.visible_samples - 1u;
    let channel = instance / segments;
    let sample_index = instance % segments;
    let va = get_sample(channel, sample_index);
    let vb = get_sample(channel, sample_index + 1u);
    if !finite(va) || !finite(vb) {
        return LineVertex(vec4<f32>(2.0, 2.0, 0.0, 1.0), vec2<f32>(0.0), vec2<f32>(0.0), vec4<f32>(0.0));
    }
    let size = vec2<f32>(f32(params.texture_width), f32(params.texture_height));
    let a = vec2<f32>(f32(sample_index) / f32(segments), 1.0 - value_to_y(va)) * size;
    let b = vec2<f32>(f32(sample_index + 1u) / f32(segments), 1.0 - value_to_y(vb)) * size;
    let direction = normalize(b - a);
    let normal = vec2<f32>(-direction.y, direction.x);
    let radius = 4.0 * 1.0;
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(0.0, 1.0),
        vec2<f32>(0.0, 1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0)
    );
    let corner = corners[vertex];
    let point = mix(a, b, corner.x) + direction * ((corner.x * 2.0 - 1.0) * radius) + normal * corner.y * radius;
    let clip = point / size;
    return LineVertex(vec4<f32>(clip.x * 2.0 - 1.0, 1.0 - clip.y * 2.0, 0.0, 1.0),
        a, b, channel_colors.data[channel]);
}

@fragment
fn fs_line(input: LineVertex) -> @location(0) vec4<f32> {
    let ab = input.b - input.a;
    let ap = input.position.xy - input.a;
    let t = clamp(dot(ap, ab) / max(dot(ab, ab), 1e-20), 0.0, 1.0);
    let distance = length(ap - t * ab);
    let core = 1.0 - smoothstep(0.0, 1.5 * 1.0, distance);
    let glow = (1.0 - smoothstep(0.0, 4.0 * 1.0, distance)) * 0.2;
    let alpha = max(core, glow) * input.color.a;
    let color = input.color.rgb * mix(0.35, 1.0, core);
    return vec4<f32>(color * alpha, alpha);
}
