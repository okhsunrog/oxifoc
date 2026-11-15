const report = {
  ua: navigator.userAgent,
  origin: location.origin,
  secure: isSecureContext,
  webgpu: !!navigator.gpu,
};
for (const kind of ["webgl", "webgl2"]) {
  const c = document.createElement("canvas");
  const gl = c.getContext(kind);
  if (!gl) {
    report[kind] = false;
    continue;
  }
  const ext = gl.getExtension("WEBGL_debug_renderer_info");
  gl.clearColor(0.25, 0.5, 0.75, 1);
  gl.clear(gl.COLOR_BUFFER_BIT);
  const pixel = new Uint8Array(4);
  gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel);
  report[kind] = {
    version: gl.getParameter(gl.VERSION),
    renderer: ext
      ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL)
      : gl.getParameter(gl.RENDERER),
    pixel: [...pixel],
  };
  gl.getExtension("WEBGL_lose_context")?.loseContext();
}
if (navigator.gpu) {
  try {
    const a = await navigator.gpu.requestAdapter();
    report.adapter = !!a;
    if (a) {
      const d = await a.requestDevice();
      report.device = !!d;
      d.destroy();
    }
  } catch (e) {
    report.gpuError = String(e);
  }
}
document.getElementById("status").textContent = JSON.stringify(report, null, 2);
dioxus.send(report);
