/* Synchronised local recording for the WebUI visualisations.
 *
 * One operator click starts two or three MediaRecorders from the same
 * browser clock:
 *   1. MJPEG camera + the YOLOE detection overlay;
 *   2. the semantic topology SVG exactly in its current object/navigation view;
 *   3. the 2D map canvas, or the existing workstation map canvas.
 *
 * Frames never leave the browser. Stopping creates local files with a
 * shared timestamp and keeps explicit download links in case the browser
 * blocks automatic multiple downloads. */
(function () {
  "use strict";

  var camera = document.getElementById("search-camera");
  var overlay = document.getElementById("search-overlay");
  var topologySvg = document.getElementById("search-map");
  var mapCanvas = document.getElementById("slam-map-2d") || document.getElementById("slam-map-3d");
  var startButton = document.getElementById("btn-camera-record");
  var stopButton = document.getElementById("btn-camera-record-stop");
  var status = document.getElementById("camera-record-status");
  var downloads = document.getElementById("camera-record-downloads");
  if (!camera || !overlay || !topologySvg ||
      !startButton || !stopButton || !status) return;

  var channels = [];
  var drawTimer = null;
  var statusTimer = null;
  var startedAt = 0;
  var sessionStamp = "";
  var stopping = false;
  var recordingFailed = false;
  var pendingStops = 0;
  var savedUrls = [];
  var topologyImage = null;
  var topologyMarkup = "";
  var topologyLoadPending = false;

  function setStatus(text, recording) {
    status.textContent = text;
    status.classList.toggle("recording", !!recording);
  }

  function formatDuration(seconds) {
    var whole = Math.max(0, Math.floor(seconds));
    var minutes = Math.floor(whole / 60);
    var remainder = whole % 60;
    return String(minutes).padStart(2, "0") + ":" +
      String(remainder).padStart(2, "0");
  }

  function selectMimeType() {
    var candidates = [
      "video/webm;codecs=vp9",
      "video/webm;codecs=vp8",
      "video/webm",
      "video/mp4"
    ];
    if (!window.MediaRecorder) return "";
    for (var i = 0; i < candidates.length; i += 1) {
      if (MediaRecorder.isTypeSupported(candidates[i])) return candidates[i];
    }
    return "";
  }

  function timestampName() {
    var now = new Date();
    function two(value) { return String(value).padStart(2, "0"); }
    return now.getFullYear() + two(now.getMonth() + 1) + two(now.getDate()) + "_" +
      two(now.getHours()) + two(now.getMinutes()) + two(now.getSeconds());
  }

  function extensionFor(mimeType) {
    return mimeType.indexOf("mp4") >= 0 ? "mp4" : "webm";
  }

  function drawContainedSource(context, source, x, y, width, height, sourceWidth, sourceHeight) {
    var scale = Math.min(width / sourceWidth, height / sourceHeight);
    var drawWidth = sourceWidth * scale;
    var drawHeight = sourceHeight * scale;
    var drawX = x + (width - drawWidth) / 2;
    var drawY = y + (height - drawHeight) / 2;
    context.drawImage(source, drawX, drawY, drawWidth, drawHeight);
  }

  function drawContainedImage(context, image, width, height) {
    var sourceWidth = image.naturalWidth || width;
    var sourceHeight = image.naturalHeight || height;
    context.fillStyle = "#000";
    context.fillRect(0, 0, width, height);
    drawContainedSource(context, image, 0, 0, width, height, sourceWidth, sourceHeight);
  }

  function drawHeader(context, canvas, title) {
    context.fillStyle = "#111827";
    context.fillRect(0, 0, canvas.width, 58);
    context.fillStyle = "#f8fafc";
    context.font = "700 25px system-ui, sans-serif";
    context.textBaseline = "middle";
    context.fillText(title, 22, 29);
    context.fillStyle = "#94a3b8";
    context.font = "16px system-ui, sans-serif";
    context.textAlign = "right";
    context.fillText(
      "同步录制 " + formatDuration((Date.now() - startedAt) / 1000),
      canvas.width - 22,
      29
    );
    context.textAlign = "left";
    context.textBaseline = "alphabetic";
  }

  function drawCameraFrame(channel) {
    var context = channel.canvas.getContext("2d", { alpha: false });
    try {
      drawContainedImage(context, camera, channel.canvas.width, channel.canvas.height);
      // search-overlay contains the exact boxes, names and confidence values
      // currently displayed over the live camera.
      context.drawImage(overlay, 0, 0, channel.canvas.width, channel.canvas.height);
    } catch (error) {
      // An MJPEG reconnect may leave one transient frame unavailable. Keep
      // the common recording clock alive; the next tick draws the recovery.
    }
  }

  function serializeTopology() {
    var clone = topologySvg.cloneNode(true);
    clone.setAttribute("xmlns", "http://www.w3.org/2000/svg");
    var rect = topologySvg.getBoundingClientRect();
    clone.setAttribute("width", String(Math.max(600, Math.round(rect.width || 600))));
    clone.setAttribute("height", String(Math.max(400, Math.round(rect.height || 400))));
    // SVG text otherwise depends on stylesheet inheritance that is no longer
    // present after serialisation into a Blob image.
    var style = document.createElementNS("http://www.w3.org/2000/svg", "style");
    style.textContent = "text{font-family:system-ui,-apple-system,'Noto Sans CJK SC',sans-serif}";
    clone.insertBefore(style, clone.firstChild);
    return new XMLSerializer().serializeToString(clone);
  }

  function refreshTopologyImage() {
    if (topologyLoadPending) return;
    var markup;
    try {
      markup = serializeTopology();
    } catch (error) {
      return;
    }
    if (markup === topologyMarkup) return;
    topologyMarkup = markup;
    topologyLoadPending = true;
    var blob = new Blob([markup], { type: "image/svg+xml;charset=utf-8" });
    var url = URL.createObjectURL(blob);
    var image = new Image();
    image.addEventListener("load", function () {
      topologyImage = image;
      topologyLoadPending = false;
      URL.revokeObjectURL(url);
    });
    image.addEventListener("error", function () {
      topologyLoadPending = false;
      URL.revokeObjectURL(url);
    });
    image.src = url;
  }

  function drawTopologyFrame(channel) {
    refreshTopologyImage();
    var context = channel.canvas.getContext("2d", { alpha: false });
    context.fillStyle = "#070b10";
    context.fillRect(0, 0, channel.canvas.width, channel.canvas.height);
    drawHeader(context, channel.canvas, "GO2-W 实时语义拓扑");
    if (topologyImage && topologyImage.naturalWidth && topologyImage.naturalHeight) {
      drawContainedSource(
        context,
        topologyImage,
        0,
        58,
        channel.canvas.width,
        channel.canvas.height - 58,
        topologyImage.naturalWidth,
        topologyImage.naturalHeight
      );
    } else {
      context.fillStyle = "#94a3b8";
      context.font = "24px system-ui, sans-serif";
      context.fillText("等待拓扑首帧…", 40, 115);
    }
  }

  function drawMapFrame(channel) {
    var context = channel.canvas.getContext("2d", { alpha: false });
    context.fillStyle = "#070b10";
    context.fillRect(0, 0, channel.canvas.width, channel.canvas.height);
    drawHeader(context, channel.canvas, "GO2-W 二维地图与物体节点");
    try {
      drawContainedSource(
        context,
        mapCanvas,
        0,
        58,
        channel.canvas.width,
        channel.canvas.height - 58,
        Math.max(1, mapCanvas.width),
        Math.max(1, mapCanvas.height)
      );
    } catch (error) {
      context.fillStyle = "#94a3b8";
      context.font = "24px system-ui, sans-serif";
      context.fillText("等待二维地图首帧…", 40, 115);
    }
  }

  function drawAllFrames() {
    channels.forEach(function (channel) { channel.draw(channel); });
  }

  function makeChannel(key, label, width, height, draw) {
    var canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    return {
      key: key,
      label: label,
      canvas: canvas,
      draw: draw,
      stream: null,
      recorder: null,
      chunks: [],
      blob: null,
      finished: false,
      error: ""
    };
  }

  function buildChannels() {
    var cameraWidth = Math.max(640, (overlay.width || 640) * 2);
    var cameraHeight = Math.max(480, (overlay.height || 480) * 2);
    var result = [
      makeChannel("camera_boxes", "相机＋YOLOE识别框", cameraWidth, cameraHeight, drawCameraFrame),
      makeChannel("topology", "二维拓扑图", 1200, 800, drawTopologyFrame)
    ];
    if (mapCanvas) result.push(
      makeChannel("2d_map_objects", "二维地图＋物体节点", 1280, 720, drawMapFrame)
    );
    return result;
  }

  function resetUi() {
    startButton.disabled = false;
    stopButton.disabled = false;
    startButton.classList.remove("hidden");
    stopButton.classList.add("hidden");
    stopping = false;
  }

  function releaseCapture() {
    if (drawTimer !== null) window.clearInterval(drawTimer);
    if (statusTimer !== null) window.clearInterval(statusTimer);
    drawTimer = null;
    statusTimer = null;
    channels.forEach(function (channel) {
      if (channel.stream) {
        channel.stream.getTracks().forEach(function (track) { track.stop(); });
      }
      channel.stream = null;
      channel.recorder = null;
    });
  }

  function clearSavedDownloads() {
    savedUrls.forEach(function (url) { URL.revokeObjectURL(url); });
    savedUrls = [];
    if (downloads) {
      downloads.textContent = "";
      downloads.classList.add("hidden");
    }
  }

  function exposeDownloads(mimeType) {
    var extension = extensionFor(mimeType);
    var totalBytes = 0;
    clearSavedDownloads();
    if (downloads) downloads.classList.remove("hidden");
    channels.forEach(function (channel, index) {
      if (!channel.blob || !channel.blob.size) return;
      totalBytes += channel.blob.size;
      var url = URL.createObjectURL(channel.blob);
      savedUrls.push(url);
      var link = document.createElement("a");
      link.href = url;
      link.download = "go2w_" + channel.key + "_" + sessionStamp + "." + extension;
      link.textContent = "下载" + channel.label;
      link.className = "camera-record-download";
      if (downloads) downloads.appendChild(link);
      // Keep the explicit links above as a fallback. Chrome/Edge may ask the
      // operator once for permission to allow these three automatic files.
      window.setTimeout(function () { link.click(); }, index * 220);
    });
    setStatus(
      "已生成 " + channels.length + " 个同步视频（" + (totalBytes / 1048576).toFixed(1) +
        " MB）；如浏览器拦截请点右侧下载链接",
      false
    );
  }

  function finishChannel(channel, mimeType) {
    if (channel.finished) return;
    channel.finished = true;
    channel.blob = new Blob(channel.chunks, { type: mimeType });
    pendingStops -= 1;
    if (recordingFailed) return;
    if (pendingStops > 0) return;
    releaseCapture();
    resetUi();
    exposeDownloads(mimeType);
  }

  function failRecording(message) {
    if (recordingFailed) return;
    recordingFailed = true;
    channels.forEach(function (channel) {
      if (channel.recorder && channel.recorder.state !== "inactive") {
        try { channel.recorder.stop(); } catch (ignore) { /* already failed */ }
      }
    });
    releaseCapture();
    resetUi();
    setStatus("同步录像失败：" + message, false);
  }

  function configureRecorder(channel, mimeType) {
    channel.stream = channel.canvas.captureStream(20);
    channel.recorder = new MediaRecorder(channel.stream, {
      mimeType: mimeType,
      videoBitsPerSecond: channel.key === "camera_boxes" ? 6000000 : 4500000
    });
    channel.recorder.addEventListener("dataavailable", function (event) {
      if (event.data && event.data.size > 0) channel.chunks.push(event.data);
    });
    channel.recorder.addEventListener("error", function (event) {
      var detail = event.error && event.error.message ? event.error.message : "未知错误";
      channel.error = detail;
      failRecording(channel.label + "：" + detail);
    });
    channel.recorder.addEventListener("stop", function () {
      finishChannel(channel, mimeType);
    });
  }

  function startRecording() {
    if (channels.some(function (channel) {
      return channel.recorder && channel.recorder.state !== "inactive";
    })) return;
    if (!window.MediaRecorder || !HTMLCanvasElement.prototype.captureStream) {
      setStatus("浏览器不支持画布录像，请使用 Chrome/Edge", false);
      return;
    }
    if (!camera.complete || !camera.naturalWidth || !camera.naturalHeight) {
      setStatus("相机画面尚未就绪", false);
      return;
    }
    var mimeType = selectMimeType();
    if (!mimeType) {
      setStatus("浏览器没有可用的视频编码器", false);
      return;
    }

    clearSavedDownloads();
    sessionStamp = timestampName();
    startedAt = Date.now();
    topologyImage = null;
    topologyMarkup = "";
    topologyLoadPending = false;
    channels = buildChannels();
    drawAllFrames();
    recordingFailed = false;
    pendingStops = channels.length;
    try {
      channels.forEach(function (channel) { configureRecorder(channel, mimeType); });
      // Starting all tracks in the same JavaScript task keeps their media
      // timelines aligned to within one event-loop turn.
      channels.forEach(function (channel) { channel.recorder.start(1000); });
    } catch (error) {
      failRecording("初始化失败：" + error.message);
      return;
    }

    stopping = false;
    drawTimer = window.setInterval(drawAllFrames, 50);
    statusTimer = window.setInterval(function () {
      setStatus(
        "● " + channels.length + " 路同步录像中 " + formatDuration((Date.now() - startedAt) / 1000),
        true
      );
    }, 250);
    startButton.classList.add("hidden");
    stopButton.classList.remove("hidden");
    setStatus("● " + channels.length + " 路同步录像中 00:00", true);
  }

  function stopRecording() {
    if (!channels.length || stopping) return;
    var active = channels.filter(function (channel) {
      return channel.recorder && channel.recorder.state !== "inactive";
    });
    if (!active.length) return;
    stopping = true;
    stopButton.disabled = true;
    setStatus("正在封装 " + channels.length + " 个同步视频…", true);
    active.forEach(function (channel) { channel.recorder.stop(); });
  }

  startButton.addEventListener("click", startRecording);
  stopButton.addEventListener("click", stopRecording);
  window.addEventListener("beforeunload", function (event) {
    var active = channels.some(function (channel) {
      return channel.recorder && channel.recorder.state !== "inactive";
    });
    if (!active) return;
    event.preventDefault();
    event.returnValue = "同步录像尚未结束，离开页面会丢失本次录像。";
  });
  window.addEventListener("unload", clearSavedDownloads);
}());
