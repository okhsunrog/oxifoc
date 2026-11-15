// WebGL2 adaptation of oxifoc-plot's instanced distance-antialiased segments.
// WebGL lacks storage buffers and compute: use an R32F ring texture and CPU
// min/max reduction for the wide-window path. No canvas image readback per frame.
const config = await dioxus.recv();
if (config.bench) document.body.classList.add("bench");
const canvas = document.getElementById("plot");
const gl = canvas.getContext("webgl2", { antialias: false, alpha: false });
if (!gl) throw new Error("WebGL2 is unavailable");
const CAP = 65536,
  CH = 3,
  WIDTH = 802,
  HEIGHT = 400;
let running = false;
function wake() {
  if (!running) {
    running = true;
    requestAnimationFrame(frame);
  }
}
let visible = config.visible,
  paused = config.paused && !config.bench,
  written = 0,
  dirty = true;
const ring = new Float32Array(CAP * CH);
ring.fill(NaN);
const colors = [
  [0.05, 0.68, 0.58, 1],
  [0.37, 0.39, 0.91, 1],
  [0.94, 0.53, 0.2, 1],
];
const vs = `#version 300 es
precision highp float; precision highp int;
uniform sampler2D samples; uniform int head, visible, capacity, channel, peakMode; uniform vec2 size;
out vec2 a; out vec2 b;
float sampleAt(int i){int pos=(head+capacity-visible+i)%capacity; return texelFetch(samples,ivec2(pos%1024,(pos/1024)*3+channel),0).r;}
void main(){
 vec2 corners[6]=vec2[6](vec2(0,-1),vec2(1,-1),vec2(0,1),vec2(0,1),vec2(1,-1),vec2(1,1));
 float va,vb; int n=gl_InstanceID;
 if(peakMode==1){va=texelFetch(samples,ivec2(n,channel*2),0).r;vb=texelFetch(samples,ivec2(n,channel*2+1),0).r;a=vec2(float(n)+.5,(va+4.)/8.*size.y);b=vec2(float(n)+.5,(vb+4.)/8.*size.y);}
 else{va=sampleAt(n);vb=sampleAt(n+1);a=vec2(float(n)/float(visible-1)*size.x,(va+4.)/8.*size.y);b=vec2(float(n+1)/float(visible-1)*size.x,(vb+4.)/8.*size.y);}
 if(isnan(va)||isnan(vb)){gl_Position=vec4(2,2,0,1);return;}
 vec2 direction=(length(b-a)>0.)?normalize(b-a):vec2(0,1);vec2 normal=vec2(-direction.y,direction.x);vec2 corner=corners[gl_VertexID];
 vec2 point=mix(a,b,corner.x)+direction*((corner.x*2.-1.)*2.)+normal*corner.y*2.;
 gl_Position=vec4(point/size*2.-1.,0,1);
}`;
const fs = `#version 300 es
precision highp float;in vec2 a;in vec2 b;uniform vec4 color;out vec4 outColor;
void main(){vec2 ab=b-a;vec2 ap=gl_FragCoord.xy-a;float t=clamp(dot(ap,ab)/max(dot(ab,ab),1e-20),0.,1.);float alpha=1.-smoothstep(0.,1.5,length(ap-t*ab));outColor=vec4(color.rgb,alpha);}`;
function shader(type, source) {
  const s = gl.createShader(type);
  gl.shaderSource(s, source);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
    throw Error(gl.getShaderInfoLog(s));
  return s;
}
const program = gl.createProgram();
gl.attachShader(program, shader(gl.VERTEX_SHADER, vs));
gl.attachShader(program, shader(gl.FRAGMENT_SHADER, fs));
gl.linkProgram(program);
if (!gl.getProgramParameter(program, gl.LINK_STATUS))
  throw Error(gl.getProgramInfoLog(program));
gl.useProgram(program);
const loc = Object.fromEntries(
  ["head", "visible", "capacity", "channel", "peakMode", "size", "color"].map(
    (k) => [k, gl.getUniformLocation(program, k)],
  ),
);
function texture(w, h) {
  const t = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, t);
  gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R32F, w, h);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  return t;
}
const tex = texture(1024, 64 * CH),
  peakTex = texture(WIDTH, CH * 2);
const upload = new Float32Array(1024 * CH);
const peaks = new Float32Array(WIDTH * CH * 2);
gl.bindVertexArray(gl.createVertexArray());
gl.enable(gl.BLEND);
gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
gl.viewport(0, 0, WIDTH, HEIGHT);
gl.uniform2f(loc.size, WIDTH, HEIGHT);
gl.uniform1i(loc.capacity, CAP);
function append(data) {
  let first = written;
  for (let i = 0; i < data.length; i += CH) {
    const base = (written % CAP) * CH;
    for (let c = 0; c < CH; c++) ring[base + c] = data[i + c];
    written++;
  }
  gl.bindTexture(gl.TEXTURE_2D, tex);
  // Upload only changed columns, splitting at texture-row and ring boundaries.
  for (let pos = first; pos < written;) {
    const index = pos % CAP,
      row = Math.floor(index / 1024),
      column = index % 1024,
      count = Math.min(1024 - column, written - pos);
    for (let c = 0; c < CH; c++)
      for (let i = 0; i < count; i++)
        upload[c * count + i] = ring[(index + i) * CH + c];
    gl.texSubImage2D(
      gl.TEXTURE_2D,
      0,
      column,
      row * CH,
      count,
      CH,
      gl.RED,
      gl.FLOAT,
      upload.subarray(0, count * CH),
    );
    pos += count;
  }
  dirty ||= data.length > 0;
}
function draw() {
  const wide = visible > WIDTH * 2;
  gl.clearColor(0.965, 0.98, 0.985, 1);
  gl.clear(gl.COLOR_BUFFER_BIT);
  if (wide) {
    for (let c = 0; c < CH; c++)
      for (let x = 0; x < WIDTH; x++) {
        let lo = Infinity,
          hi = -Infinity;
        const center = ((x + 0.5) / WIDTH) * (visible - 1),
          half = Math.max((visible / WIDTH) * 0.5, 0.5),
          start = Math.max(0, Math.floor(center - half)),
          end = Math.min(visible - 1, Math.ceil(center + half));
        for (let i = start; i <= end; i++) {
          const v =
            ring[(((written % CAP) + CAP - visible + i) % CAP) * CH + c];
          if (Number.isFinite(v)) {
            lo = Math.min(lo, v);
            hi = Math.max(hi, v);
          }
        }
        peaks[c * 2 * WIDTH + x] = lo === Infinity ? NaN : lo;
        peaks[(c * 2 + 1) * WIDTH + x] = hi === -Infinity ? NaN : hi;
      }
    gl.bindTexture(gl.TEXTURE_2D, peakTex);
    gl.texSubImage2D(
      gl.TEXTURE_2D,
      0,
      0,
      0,
      WIDTH,
      CH * 2,
      gl.RED,
      gl.FLOAT,
      peaks,
    );
  } else gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.uniform1i(loc.head, written % CAP);
  gl.uniform1i(loc.visible, visible);
  gl.uniform1i(loc.peakMode, wide ? 1 : 0);
  for (let c = 0; c < CH; c++) {
    gl.uniform1i(loc.channel, c);
    gl.uniform4fv(loc.color, colors[c]);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, 6, wide ? WIDTH : visible - 1);
  }
}
const controls = document.getElementById("controls");
controls.innerHTML =
  '<button id="pause">Pause</button><button id="short">1,200 samples</button><button id="long">60,000 samples</button>' +
  (config.bench
    ? ""
    : '<button id="start">Apply 1 A</button><button id="stop">Stop drive</button>');
document.getElementById("pause").onclick = () => {
  paused = !paused;
  document.getElementById("pause").textContent = paused ? "Resume" : "Pause";
  wake();
};
for (const [id, n] of [
  ["short", 1200],
  ["long", 60000],
])
  document.getElementById(id).onclick = () => {
    visible = n;
    dirty = true;
    wake();
  };
if (!config.bench)
  for (const command of ["start", "stop"])
    document.getElementById(command).onclick = () => dioxus.send({ command });
canvas.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    visible = Math.max(
      32,
      Math.min(CAP, Math.round(visible * Math.exp(event.deltaY * 0.002))),
    );
    dirty = true;
    wake();
  },
  { passive: false },
);
let last = performance.now(),
  mark = last,
  intervals = [],
  costs = [],
  frames = 0,
  callbacks = 0,
  batches = 0,
  emptyBatches = 0,
  largestBatch = 0;
const percentile = (xs, p) =>
  xs.slice().sort((a, b) => a - b)[
    Math.min(xs.length - 1, Math.floor(xs.length * p))
  ] || 0;
let inFlight = false,
  pending = null,
  latestStatus = "",
  lastStatusUpdate = 0;
async function requestBatch() {
  if (inFlight) return;
  inFlight = true;
  dioxus.send({ request: true });
  try {
    pending = await dioxus.recv();
  } catch (e) {
    dioxus.send({ error: String(e) });
    paused = true;
  }
  inFlight = false;
}
function frame(now) {
  callbacks++;
  if (!paused) {
    if (pending) {
      batches++;
      if (!pending.samples.length) emptyBatches++;
      largestBatch = Math.max(largestBatch, pending.samples.length / CH);
      latestStatus = pending.status;
      append(pending.samples);
      if (!config.bench && now - lastStatusUpdate >= 100) {
        document.getElementById("telemetry").textContent = pending.status;
        lastStatusUpdate = now;
      }
      pending = null;
    }
    // IPC runs concurrently; do not postpone the next rAF until its reply.
    requestBatch();
  }
  const start = performance.now();
  if (dirty) {
    draw();
    dirty = false;
    frames++;
    costs.push(performance.now() - start);
    intervals.push(now - last);
    last = now;
  }
  if (now - mark >= 1000) {
    const stats = {
      kind: "stats",
      fresh_hz: (frames * 1000) / (now - mark),
      callbacks_hz: (callbacks * 1000) / (now - mark),
      batches,
      empty_batches: emptyBatches,
      largest_batch: largestBatch,
      interval_p95_ms: percentile(intervals, 0.95),
      render_cpu_mean_ms:
        costs.reduce((a, b) => a + b, 0) / (costs.length || 1),
      width: WIDTH,
      height: HEIGHT,
      visible,
      paused,
      samples: written,
    };
    if (!config.bench) stats.status = latestStatus;
    dioxus.send(stats);
    if (!config.bench)
      document.getElementById("metrics").textContent =
        `${stats.fresh_hz.toFixed(1)} updates/s · ${visible} samples · ${stats.render_cpu_mean_ms.toFixed(2)} ms JS draw`;
    frames = 0;
    callbacks = batches = emptyBatches = largestBatch = 0;
    intervals = [];
    costs = [];
    mark = now;
  }
  running = false;
  if (!paused) wake();
}
// Compile/draw validation and pixel readback happen once, outside benchmark windows.
const initialVisible = visible;
visible = 1200;
append(Array.from({ length: 1200 * CH }, (_, i) => [1, 0, -1][i % 3]));
draw();
const check = new Uint8Array(4);
gl.readPixels(401, 250, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, check);
ring.fill(NaN);
written = 0;
visible = initialVisible;
gl.bindTexture(gl.TEXTURE_2D, tex);
gl.texSubImage2D(
  gl.TEXTURE_2D,
  0,
  0,
  0,
  1024,
  64 * CH,
  gl.RED,
  gl.FLOAT,
  new Float32Array(1024 * 64 * CH).fill(NaN),
);
draw();
dioxus.send({
  pixel: [...check],
  kind: "render-check",
  error: gl.getError(),
  timerQuery: !!gl.getExtension("EXT_disjoint_timer_query_webgl2"),
});
dioxus.send({
  kind: "layout",
  body: getComputedStyle(document.body).fontSize,
  padding: getComputedStyle(document.querySelector("main")).padding,
  width: canvas.getBoundingClientRect().width,
  dpr: devicePixelRatio,
});
if (config.paused && config.bench)
  setTimeout(() => {
    paused = true;
    dioxus.send({ kind: "paused-after-warmup", samples: written });
  }, 4000);
wake();
// Keep the eval channel alive while animation callbacks exchange telemetry.
await new Promise(() => {});
