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
    scale:           f32,   // physical pixels per logical pixel (hidpi)
    _pad0:           u32,
    _pad1:           u32,
};


var<immediate> params: PlotParams;
@group(0) @binding(0) var<storage, read> samples: array<f32>;
@group(0) @binding(2) var<storage, read_write> peaks: array<vec4<f32>>;

// One invocation per physical column/channel. No truncation: every sample
// contributing to the column is included, even for very narrow plots.
@compute @workgroup_size(64)
fn reduce(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = id.x;
    if index >= params.texture_width * params.num_channels { return; }
    let column = index / params.num_channels;
    let channel = index % params.num_channels;
    let center = (f32(column) + 0.5) / f32(params.texture_width) * f32(params.visible_samples - 1u);
    let half_span = max(f32(params.visible_samples) / f32(params.texture_width) * 0.5, 0.5);
    let first = u32(max(floor(center - half_span), 0.0));
    let last = u32(min(ceil(center + half_span), f32(params.visible_samples - 1u)));
    let start = (params.write_pos + params.num_samples - params.visible_samples - params.view_offset) % params.num_samples;
    var lo = 3.402823e38f;
    var hi = -3.402823e38f;
    var valid = 0.0;
    for (var i = first; i <= last; i++) {
        let v = samples[((start + i) % params.num_samples) * params.num_channels + channel];
        if (bitcast<u32>(v) & 0x7f800000u) != 0x7f800000u {
            lo = min(lo, v);
            hi = max(hi, v);
            valid = 1.0;
        }
    }
    peaks[index] = vec4<f32>(lo, hi, valid, 0.0);
}
