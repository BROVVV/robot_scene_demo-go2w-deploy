/* Occupancy-grid display. This reader never requests or renders a point cloud. */
(function () {
  "use strict";
  var canvas = document.getElementById("slam-map-2d");
  if (!canvas) return;
  var context = canvas.getContext("2d", { alpha: false });
  var state = document.getElementById("slam2d-state");
  var meta = document.getElementById("slam2d-meta");
  var light = document.getElementById("light-slam");
  var snapshot = null, bitmap = null, bitmapKey = "", epoch = "";
  var zoom = 1, pan = [0, 0], trace = [], hits = [], selected = "";
  var pointer = null, dragged = false, busy = false;

  function finite(value) { return typeof value === "number" && isFinite(value); }
  function gridPoint(grid, x, y) {
    var origin = grid.origin || {}, angle = Number(origin.yaw || 0);
    return [Number(origin.x || 0) + x * Math.cos(angle) - y * Math.sin(angle),
      Number(origin.y || 0) + x * Math.sin(angle) + y * Math.cos(angle)];
  }
  function makeBitmap(grid) {
    var imageCanvas = document.createElement("canvas");
    imageCanvas.width = grid.width; imageCanvas.height = grid.height;
    var target = imageCanvas.getContext("2d"), image = target.createImageData(grid.width, grid.height);
    for (var i = 0; i < grid.data.length; i += 1) {
      var value = Number(grid.data[i]), shade = value < 0 ? 88 : value >= 50 ? 27 : 242 - value * 2;
      image.data[i * 4] = shade; image.data[i * 4 + 1] = shade;
      image.data[i * 4 + 2] = shade; image.data[i * 4 + 3] = 255;
    }
    target.putImageData(image, 0, 0);
    return imageCanvas;
  }
  function render() {
    var ratio = Math.min(2, window.devicePixelRatio || 1);
    var width = Math.max(320, Math.round(canvas.clientWidth * ratio));
    var height = Math.max(220, Math.round(canvas.clientHeight * ratio));
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    context.fillStyle = "#101923"; context.fillRect(0, 0, width, height); hits = [];
    if (!snapshot || !snapshot.grid || !bitmap) return;
    var grid = snapshot.grid, w = grid.width * grid.resolution, h = grid.height * grid.resolution;
    var corners = [gridPoint(grid, 0, 0), gridPoint(grid, w, 0), gridPoint(grid, 0, h), gridPoint(grid, w, h)];
    var xs = corners.map(function (p) { return p[0]; }), ys = corners.map(function (p) { return p[1]; });
    var xmin = Math.min.apply(null, xs), xmax = Math.max.apply(null, xs);
    var ymin = Math.min.apply(null, ys), ymax = Math.max.apply(null, ys);
    var center = [(xmin + xmax) / 2, (ymin + ymax) / 2];
    var scale = Math.min(width / Math.max(.1, xmax - xmin), height / Math.max(.1, ymax - ymin)) * .90 * zoom;
    function project(x, y) { return [width / 2 + pan[0] + (x - center[0]) * scale,
      height / 2 + pan[1] - (y - center[1]) * scale]; }
    context.save(); context.translate(width / 2 + pan[0], height / 2 + pan[1]);
    context.scale(scale, -scale); context.translate(-center[0], -center[1]);
    var origin = grid.origin || {};
    context.translate(Number(origin.x || 0), Number(origin.y || 0));
    context.rotate(Number(origin.yaw || 0)); context.imageSmoothingEnabled = false;
    context.drawImage(bitmap, 0, 0, w, h); context.restore();
    if (trace.length > 1) {
      context.beginPath(); trace.forEach(function (p, i) {
        var v = project(p[0], p[1]); if (i) context.lineTo(v[0], v[1]); else context.moveTo(v[0], v[1]);
      }); context.strokeStyle = "#22c55e"; context.lineWidth = 2 * ratio; context.stroke();
    }
    (snapshot.objects || []).forEach(function (item) {
      if (!finite(item.x) || !finite(item.y)) return;
      var p = project(item.x, item.y), active = String(item.object_id || "") === selected;
      context.beginPath(); context.arc(p[0], p[1], (active ? 7 : 5) * ratio, 0, Math.PI * 2);
      context.fillStyle = item.is_hazard ? "#ef4444" : item.is_target_confirmed ? "#facc15" : "#38bdf8";
      context.fill(); context.strokeStyle = active ? "#fff" : "#0f172a";
      context.lineWidth = ratio; context.stroke();
      context.font = 11 * ratio + "px sans-serif"; context.fillStyle = "#fff";
      context.fillText(String(item.label_zh || "未分类物体").slice(0, 40), p[0] + 8 * ratio, p[1] - 6 * ratio);
      hits.push({ x: p[0], y: p[1], id: String(item.object_id || "") });
    });
    var pose = snapshot.pose;
    if (pose && finite(pose.x) && finite(pose.y)) {
      var p = project(pose.x, pose.y); context.save(); context.translate(p[0], p[1]);
      context.rotate(-Number(pose.yaw || 0)); context.beginPath();
      context.moveTo(11 * ratio, 0); context.lineTo(-7 * ratio, -6 * ratio);
      context.lineTo(-4 * ratio, 0); context.lineTo(-7 * ratio, 6 * ratio); context.closePath();
      context.fillStyle = "#22c55e"; context.fill(); context.strokeStyle = "#052e16";
      context.lineWidth = ratio; context.stroke(); context.restore();
    }
  }
  function update(data) {
    if (!data.available || !data.grid) {
      state.textContent = "等待二维地图…"; if (light) light.className = "light amber"; return;
    }
    var grid = data.grid;
    if (!(grid.width > 0 && grid.height > 0 && grid.resolution > 0) ||
        !Array.isArray(grid.data) || grid.data.length !== grid.width * grid.height) throw new Error("二维地图数据无效");
    if (epoch !== String(data.map_epoch || "")) { epoch = String(data.map_epoch || ""); trace = []; zoom = 1; pan = [0, 0]; }
    var key = [epoch, grid.stamp, grid.width, grid.height, grid.resolution,
      JSON.stringify(grid.origin), data.map_revision].join(":");
    if (key !== bitmapKey) { bitmap = makeBitmap(grid); bitmapKey = key; }
    snapshot = data;
    var pose = data.pose, last = trace[trace.length - 1];
    if (pose && finite(pose.x) && finite(pose.y) && (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) > .03)) {
      trace.push([pose.x, pose.y]); if (trace.length > 2000) trace.shift();
    }
    state.textContent = data.localization_valid ? "二维地图 · 定位正常" : "二维地图 · 定位待恢复";
    state.classList.toggle("stale", !data.localization_valid);
    if (light) light.className = "light " + (data.localization_valid ? "green" : "amber");
    meta.textContent = grid.width + "×" + grid.height + " · " + Math.round(grid.resolution * 100) + " cm/格";
    render();
  }
  function poll() {
    if (busy) return; busy = true;
    var controller = new AbortController(), timer = setTimeout(function () { controller.abort(); }, 4000);
    fetch("/api/slam/map2d", { cache: "no-store", signal: controller.signal })
      .then(function (response) { if (!response.ok) throw new Error("地图连接中断"); return response.json(); })
      .then(update).catch(function () { state.textContent = "地图连接中断，正在重连…";
        state.classList.add("stale"); if (light) light.className = "light red"; })
      .finally(function () { clearTimeout(timer); busy = false; });
  }
  canvas.addEventListener("pointerdown", function (event) { pointer = [event.clientX, event.clientY]; dragged = false;
    canvas.setPointerCapture(event.pointerId); });
  canvas.addEventListener("pointermove", function (event) {
    if (!pointer) return;
    var dx = event.clientX - pointer[0], dy = event.clientY - pointer[1];
    if (Math.abs(dx) + Math.abs(dy) > 2) dragged = true;
    pan[0] += dx * canvas.width / Math.max(1, canvas.clientWidth);
    pan[1] += dy * canvas.height / Math.max(1, canvas.clientHeight);
    pointer = [event.clientX, event.clientY]; render();
  });
  canvas.addEventListener("pointerup", function () { pointer = null; });
  canvas.addEventListener("pointercancel", function () { pointer = null; });
  canvas.addEventListener("wheel", function (event) { event.preventDefault();
    zoom = Math.max(.25, Math.min(12, zoom * (event.deltaY > 0 ? .88 : 1.14))); render(); }, { passive: false });
  canvas.addEventListener("dblclick", function () { zoom = 1; pan = [0, 0]; render(); });
  canvas.addEventListener("click", function (event) {
    if (dragged) return;
    var rect = canvas.getBoundingClientRect(), x = (event.clientX - rect.left) * canvas.width / rect.width;
    var y = (event.clientY - rect.top) * canvas.height / rect.height, best = null, distance = 30;
    hits.forEach(function (item) { var d = Math.hypot(item.x - x, item.y - y); if (d < distance) { best = item; distance = d; } });
    if (best) window.dispatchEvent(new CustomEvent("semantic-object-select", { detail: { objectId: best.id, source: "map2d" } }));
  });
  window.addEventListener("semantic-object-select", function (event) { selected = String((event.detail || {}).objectId || ""); render(); });
  window.addEventListener("resize", render); render(); poll(); setInterval(poll, 500);
})();
