/* Lightweight dependency-free 3D point-cloud viewer for plain_slam.
 * The browser only consumes the display snapshot API; it never publishes a
 * command and cannot affect motion or ROS frame authority.
 */
(function () {
  "use strict";

  // 画幅合理性上限（米）。轨迹被发散的 odom 污染时 focus_bounds 会到 1e19
  // （2026-09-11 实测：span≈1.7e20 → scale≈0 地图缩成一个像素 = 看不见，且
  // drawGrid 的网格循环要迭代 3.5e19 次把浏览器主线程卡死）。这种画幅一律不认。
  var FRAME_ABS_LIMIT_M = 1000;

  function saneFrame(frame) {
    if (!frame || !frame.min || !frame.max) return null;
    for (var i = 0; i < 3; i++) {
      var low = Number(frame.min[i]);
      var high = Number(frame.max[i]);
      if (!isFinite(low) || !isFinite(high)) return null;
      if (Math.abs(low) > FRAME_ABS_LIMIT_M || Math.abs(high) > FRAME_ABS_LIMIT_M) return null;
    }
    return frame;
  }

  // 物体框标签的**唯一**来源是 label_zh（中文显示名，由后端统一解析器给出）。
  // 刻意不读 raw_label：那是关联键/取证字段，未知类别时它就是英文原文，
  // 画到 canvas 上等于把整改前的"bin / barrel / speaker"原样漏回页面。
  // 后端解析不出来时会填「未分类物体」，这里只兜它漏填空串的情况。
  var OBJECT_LABEL_FALLBACK = "未分类物体";

  function objectLabel(item) {
    var text = item && item.label_zh != null ? String(item.label_zh).trim() : "";
    return text || OBJECT_LABEL_FALLBACK;
  }

  // 目标例外（视觉复核过的目标）换个颜色画，好和普通候选区分开。
  function objectVerified(item) {
    return !!item && item.decision === "TARGET_VERIFIED";
  }

  // 状态栏那行过滤统计。后端没给（老快照）时返回空串，不显示"0/0"这种
  // 看着像"门把所有东西都滤掉了"的假数字。
  function objectFilterSummary(stats) {
    if (!stats || typeof stats.input !== "number") return "";
    var shown = typeof stats.displayed === "number" ? stats.displayed : 0;
    var filtered = typeof stats.filtered === "number"
      ? stats.filtered : Math.max(0, stats.input - shown);
    return "物体框 " + shown + " 个 / 已过滤 " + filtered + " 个";
  }

  // 供 headless 单测加载（同 search_map.js 的 window.SvgViewport）。
  // 放在这个位置是有意的：下面 canvas 取不到时会 return，之后的行不会执行。
  window.Slam3dFrame = {
    saneFrame: saneFrame,
    centerAndSpan: centerAndSpan,
    objectLabel: objectLabel,
    objectVerified: objectVerified,
    objectFilterSummary: objectFilterSummary,
    gridWorldPoint: gridWorldPoint
  };

  var canvas = document.getElementById("slam-map-3d");
  if (!canvas) return;
  var context = canvas.getContext("2d", { alpha: false });
  var meta = document.getElementById("slam3d-meta");
  var state = document.getElementById("slam3d-state");
  var capacity = document.getElementById("slam3d-capacity");
  var previewToggle = document.getElementById("slam3d-preview-toggle");
  var light = document.getElementById("light-slam");
  var points = [];
  var occupancyGrid = null;
  var occupancyBitmap = null;
  var currentPose = null;
  var selectedObjectId = "";
  var last2dView = null;
  var last2dScale = 1;
  var objects = [];
  var preview = [];
  var previewDrawable = false;
  var bounds = null;
  var focusBounds = null;
  var yaw = -0.72;
  // 默认接近正俯视。原来的 0.63 rad（36°）斜视下，投影式
  // vertical = cos(pitch)*dz - sin(pitch)*ry 里 sin(0.63)=0.59 把"远处的低点"
  // 和"近处的高点"挤进同一片屏幕区域 —— 用户 2026-09-11 描述的"地图像把很多次
  // 建的图重叠在一起、角度没对齐"就是这个，不是数据里有重影（分层渲染已证）。
  var pitch = 1.35;
  var zoom = 1.0;
  var dragging = false;
  var lastX = 0;
  var lastY = 0;
  var requestInFlight = false;

  function setLight(kind) {
    if (light) light.className = "light " + kind;
  }

  function resize() {
    var ratio = Math.min(2, window.devicePixelRatio || 1);
    var width = Math.max(320, Math.round(canvas.clientWidth * ratio));
    var height = Math.max(220, Math.round(canvas.clientHeight * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
  }

  function centerAndSpan(focus, fallback) {
    // focus_bounds（轨迹 bbox + 边距，由 webui_map_snapshot_writer.py 给）是
    // "狗真正建过图的区域"。用它定画幅，房间占约 60%；用点云 bbox 只有 26%，
    // 剩下的画幅全被 12 m 内的走廊占掉，细节看不清。
    var frame = saneFrame(focus) || fallback;
    var min = frame && frame.min || [-5, -5, -1];
    var max = frame && frame.max || [5, 5, 2];
    var center = [
      (Number(min[0]) + Number(max[0])) / 2,
      (Number(min[1]) + Number(max[1])) / 2,
      (Number(min[2]) + Number(max[2])) / 2
    ];
    var span = Math.max(
      2, Number(max[0]) - Number(min[0]),
      Number(max[1]) - Number(min[1]),
      (Number(max[2]) - Number(min[2])) * 1.8
    );
    return { center: center, span: span };
  }

  function project(point, center, scale) {
    var dx = Number(point[0]) - center[0];
    var dy = Number(point[1]) - center[1];
    var dz = Number(point[2]) - center[2];
    var cy = Math.cos(yaw), sy = Math.sin(yaw);
    var cp = Math.cos(pitch), sp = Math.sin(pitch);
    var rx = cy * dx - sy * dy;
    var ry = sy * dx + cy * dy;
    var vertical = cp * dz - sp * ry;
    var depth = sp * dz + cp * ry;
    return [canvas.width * 0.5 + rx * scale, canvas.height * 0.54 - vertical * scale, depth];
  }

  function gridWorldPoint(grid, column, row) {
    var origin = grid && grid.origin || {};
    var yawValue = Number(origin.yaw || 0);
    var resolution = Number(grid.resolution);
    var localX = Number(column) * resolution;
    var localY = Number(row) * resolution;
    var cosine = Math.cos(yawValue), sine = Math.sin(yawValue);
    return [Number(origin.x) + cosine * localX - sine * localY,
            Number(origin.y) + sine * localX + cosine * localY];
  }

  function gridBounds(grid) {
    if (!grid) return null;
    var corners = [gridWorldPoint(grid, 0, 0),
      gridWorldPoint(grid, grid.width, 0),
      gridWorldPoint(grid, 0, grid.height),
      gridWorldPoint(grid, grid.width, grid.height)];
    return {
      min: [Math.min.apply(null, corners.map(function (p) { return p[0]; })),
            Math.min.apply(null, corners.map(function (p) { return p[1]; })), 0],
      max: [Math.max.apply(null, corners.map(function (p) { return p[0]; })),
            Math.max.apply(null, corners.map(function (p) { return p[1]; })), 0]
    };
  }

  function project2d(x, y, view, scale) {
    return [canvas.width * 0.5 + (Number(x) - view.center[0]) * scale,
            canvas.height * 0.5 - (Number(y) - view.center[1]) * scale];
  }

  function buildOccupancyBitmap(grid) {
    var bitmap = document.createElement("canvas");
    bitmap.width = Number(grid.width); bitmap.height = Number(grid.height);
    var bitmapContext = bitmap.getContext("2d", { alpha: false });
    var image = bitmapContext.createImageData(bitmap.width, bitmap.height);
    for (var row = 0; row < bitmap.height; row++) {
      for (var column = 0; column < bitmap.width; column++) {
        var value = Number(grid.data[row * bitmap.width + column]);
        var shade = value < 0 ? 42 : Math.round(238 - Math.max(0, Math.min(100, value)) * 2.18);
        var targetRow = bitmap.height - 1 - row;
        var offset = (targetRow * bitmap.width + column) * 4;
        image.data[offset] = shade;
        image.data[offset + 1] = value < 0 ? shade + 4 : shade;
        image.data[offset + 2] = value < 0 ? shade + 8 : shade;
        image.data[offset + 3] = 255;
      }
    }
    bitmapContext.putImageData(image, 0, 0);
    return bitmap;
  }

  function drawOccupancyGrid(grid, view, scale) {
    if (!occupancyBitmap) occupancyBitmap = buildOccupancyBitmap(grid);
    var resolution = Number(grid.resolution);
    var origin = grid.origin || {};
    var angle = Number(origin.yaw || 0);
    var cosine = Math.cos(angle), sine = Math.sin(angle);
    var topLeft = gridWorldPoint(grid, 0, Number(grid.height));
    var screen = project2d(topLeft[0], topLeft[1], view, scale);
    context.save();
    context.imageSmoothingEnabled = false;
    context.setTransform(cosine * resolution * scale, -sine * resolution * scale,
                         sine * resolution * scale, cosine * resolution * scale,
                         screen[0], screen[1]);
    context.drawImage(occupancyBitmap, 0, 0);
    context.restore();
  }

  function drawObject2d(item, view, scale) {
    var p = project2d(item.x, item.y, view, scale);
    var lifecycle = String(item.lifecycle_state || "active");
    context.globalAlpha = lifecycle === "stale" ? 0.48 : (lifecycle === "missing_candidate" ? 0.7 : 1);
    context.setLineDash(lifecycle === "missing_candidate" ? [3, 3] : []);
    context.beginPath(); context.arc(p[0], p[1], objectVerified(item) ? 7 : 5.5, 0, Math.PI * 2);
    context.fillStyle = item.is_hazard ? "#ef4444" : (objectVerified(item) ? "#facc15" :
      (item.mobility_class === "semi_static" ? "#fb923c" : "#0ea5e9"));
    context.fill(); context.strokeStyle = "#ffffff"; context.lineWidth = 1.2; context.stroke();
    if (String(item.object_id || "") === selectedObjectId) {
      context.beginPath(); context.arc(p[0], p[1], 11, 0, Math.PI * 2);
      context.strokeStyle = "#f8fafc"; context.lineWidth = 2.2; context.stroke();
    }
    context.font = "12px system-ui, sans-serif"; context.fillStyle = "#f8fafc";
    context.fillText(objectLabel(item), p[0] + 9, p[1] - 7);
    context.globalAlpha = 1; context.setLineDash([]);
  }

  function drawRobot2d(pose, view, scale) {
    if (!pose) return;
    var p = project2d(pose.x, pose.y, view, scale);
    var angle = -Number(pose.yaw || 0);
    context.save(); context.translate(p[0], p[1]); context.rotate(angle);
    context.beginPath(); context.moveTo(11, 0); context.lineTo(-7, -6); context.lineTo(-4, 0);
    context.lineTo(-7, 6); context.closePath(); context.fillStyle = "#22c55e"; context.fill();
    context.strokeStyle = "#dcfce7"; context.stroke(); context.restore();
  }

  function heightColor(z, zMin, zMax, alpha) {
    var t = zMax > zMin ? Math.max(0, Math.min(1, (z - zMin) / (zMax - zMin))) : 0.5;
    var r = Math.round(30 + 220 * t);
    var g = Math.round(210 - 75 * Math.abs(t - 0.45));
    var b = Math.round(250 - 190 * t);
    return "rgba(" + r + "," + g + "," + b + "," + alpha + ")";
  }

  function drawObjectDot(item, view, scale) {
    var p = project([Number(item.x), Number(item.y),
                     item.z == null ? 0.5 : Number(item.z)], view.center, scale);
    if (p[0] < -40 || p[0] > canvas.width + 40 ||
        p[1] < -40 || p[1] > canvas.height + 40) return;
    var verified = objectVerified(item);
    context.beginPath();
    context.arc(p[0], p[1], 5.5, 0, Math.PI * 2);
    context.fillStyle = verified
      ? "rgba(250, 204, 21, 0.95)" : "rgba(239, 68, 68, 0.95)";
    context.fill();
    context.lineWidth = 1.3;
    context.strokeStyle = "rgba(255, 255, 255, 0.9)";
    context.stroke();
    context.font = "12px system-ui, sans-serif";
    context.fillStyle = verified
      ? "rgba(254, 249, 195, 0.97)" : "rgba(255, 243, 243, 0.96)";
    context.fillText(objectLabel(item), p[0] + 9, p[1] - 7);
  }

  function drawObjectBox(item, view, scale) {
    var cx = Number(item.x), cy = Number(item.y), cz = Number(item.z);
    var hw = Number(item.size[0]) / 2;
    var hd = Number(item.size[1]) / 2;
    var hh = Number(item.size[2]) / 2;
    if (hh <= 0) hh = 0.12;  // 同深度平面无高度时给一个最小可见厚度
    var corners = [
      [cx - hw, cy - hd, cz - hh], [cx + hw, cy - hd, cz - hh],
      [cx + hw, cy + hd, cz - hh], [cx - hw, cy + hd, cz - hh],
      [cx - hw, cy - hd, cz + hh], [cx + hw, cy - hd, cz + hh],
      [cx + hw, cy + hd, cz + hh], [cx - hw, cy + hd, cz + hh]
    ].map(function (c) { return project(c, view.center, scale); });
    var edges = [
      [0, 1], [1, 2], [2, 3], [3, 0],
      [4, 5], [5, 6], [6, 7], [7, 4],
      [0, 4], [1, 5], [2, 6], [3, 7]
    ];
    var verified = objectVerified(item);
    context.lineWidth = 1.5;
    context.strokeStyle = verified
      ? "rgba(250, 204, 21, 0.92)" : "rgba(56, 189, 248, 0.92)";
    context.beginPath();
    edges.forEach(function (e) {
      var a = corners[e[0]], b = corners[e[1]];
      if (a[0] < -200 || a[0] > canvas.width + 200 ||
          b[0] < -200 || b[0] > canvas.width + 200) return;
      context.moveTo(a[0], a[1]);
      context.lineTo(b[0], b[1]);
    });
    context.stroke();
    // 顶部中心小标记点，便于定位。
    var top = corners[5];
    context.beginPath();
    context.arc(top[0], top[1], 3, 0, Math.PI * 2);
    context.fillStyle = verified
      ? "rgba(250, 204, 21, 0.95)" : "rgba(56, 189, 248, 0.95)";
    context.fill();
    var conf = item.confidence != null ? " " + Number(item.confidence).toFixed(2) : "";
    context.font = "12px system-ui, sans-serif";
    context.fillStyle = verified
      ? "rgba(254, 249, 195, 0.97)" : "rgba(224, 242, 254, 0.97)";
    context.fillText(objectLabel(item) + conf, top[0] + 7, top[1] - 6);
  }

  function drawGrid(center, scale, span) {
    var half = Math.max(2, Math.ceil(span / 2));
    var step = span > 25 ? 5 : (span > 10 ? 2 : 1);
    context.lineWidth = Math.max(1, canvas.width / 1200);
    context.strokeStyle = "rgba(56, 189, 248, 0.13)";
    for (var value = -half; value <= half; value += step) {
      [[[-half, value, 0], [half, value, 0]], [[value, -half, 0], [value, half, 0]]].forEach(function (line) {
        var a = project([center[0] + line[0][0], center[1] + line[0][1], 0], center, scale);
        var b = project([center[0] + line[1][0], center[1] + line[1][1], 0], center, scale);
        context.beginPath(); context.moveTo(a[0], a[1]); context.lineTo(b[0], b[1]); context.stroke();
      });
    }
  }

  function render() {
    resize();
    context.fillStyle = "#070b10";
    context.fillRect(0, 0, canvas.width, canvas.height);
    var view = centerAndSpan(focusBounds, bounds);
    var scale = Math.min(canvas.width, canvas.height) * 0.78 / view.span * zoom;
    if (occupancyGrid) {
      last2dView = view; last2dScale = scale;
      drawOccupancyGrid(occupancyGrid, view, scale);
      objects.forEach(function (item) { drawObject2d(item, view, scale); });
      drawRobot2d(currentPose, view, scale);
      return;
    }
    drawGrid(view.center, scale, view.span);
    var size = Math.max(1.1, Math.min(3.2, canvas.width / 720));
    if (points.length) {
      var minZ = bounds && bounds.min ? Number(bounds.min[2]) : -1;
      var maxZ = bounds && bounds.max ? Number(bounds.max[2]) : 2;
      // Depth buckets preserve enough occlusion to read room geometry without
      // an O(n log n) sort for every interaction frame.
      var buckets = [[], [], [], [], [], []];
      points.forEach(function (point) {
        var p = project(point, view.center, scale);
        var bucket = Math.max(0, Math.min(5, Math.floor((p[2] / view.span + 0.5) * 6)));
        buckets[bucket].push([p[0], p[1], Number(point[2])]);
      });
      buckets.forEach(function (bucket, index) {
        var alpha = 0.48 + index * 0.08;
        bucket.forEach(function (p) {
          if (p[0] < 0 || p[0] > canvas.width || p[1] < 0 || p[1] > canvas.height) return;
          context.fillStyle = heightColor(p[2], minZ, maxZ, alpha);
          context.fillRect(p[0], p[1], size, size);
        });
      });
    }
    // 语义物体标注层：快照的 objects 字段。**这里画的全是已经过了显示门的
    // 物体** —— 低置信、非物体标签（purple/floor/corner）在写端就被滤掉了，
    // 前端不做也不该做二次判定（判据只有一份，在 app/spatial/semantic_object_gate.py）。
    // 有合法 size（3D box）时画 AABB 线框 + 中文标签；否则退化为点。
    if (objects.length) {
      objects.forEach(function (item) {
        // size 非法时写端已置 null，所以这一个判断同时覆盖"没有"和"不可信"。
        var hasSize = Array.isArray(item.size) && item.size.length === 3;
        if (hasSize) {
          drawObjectBox(item, view, scale);
        } else {
          drawObjectDot(item, view, scale);
        }
      });
    }
    // 当前 scan 是调试图层：颜色不同、画在上面，永远不进永久地图。
    if (previewDrawable && preview.length && previewToggle && previewToggle.checked) {
      context.fillStyle = "rgba(251, 191, 36, 0.85)";
      preview.forEach(function (point) {
        var p = project(point, view.center, scale);
        if (p[0] < 0 || p[0] > canvas.width || p[1] < 0 || p[1] > canvas.height) return;
        context.fillRect(p[0], p[1], size * 1.3, size * 1.3);
      });
    }
  }

  // 快照里没有的字段一律显示 "--"。原来的 `Number(value || 0)` 会把 undefined
  // 编成 0，页脚于是出现"全局缓存体素 0/0""地图范围 0.0×0.0×0.0m"这种看着像故障
  // 的假数 —— 2026-09-14 就是因为这个，写端发 v1 词表、前端按 v2 读，整条页脚全假。
  function count(value) {
    return value == null ? "--" : Number(value).toLocaleString();
  }

  function age(value) {
    if (value == null) return "--";
    var seconds = Number(value);
    return seconds >= 60 ? (seconds / 60).toFixed(1) + "分" : seconds.toFixed(1) + "s";
  }

  // 对象与地图的配对状态（后端 app/manual_web_demo/slam_map_snapshot.py 的口径）。
  // OK / 缺字段 时返回空串 —— 正常情况不占状态栏。
  function overlayStatusNote(status) {
    if (status === "MAP_EPOCH_MISMATCH") return "物体属于另一张地图（epoch 不同）";
    if (status === "FRAME_MISMATCH") return "物体与地图坐标系不同";
    if (status === "NO_OBJECT_SNAPSHOT") return "本模式没有语义建图进程";
    return "";
  }

  function applySnapshot(snapshot) {
    if (!snapshot || !Array.isArray(snapshot.points)) {
      state.textContent = "3D 点云接口返回了无法解析的快照";
      state.className = "slam3d-state error";
      setLight("red");
      return;
    }
    var health = snapshot.mapping_health || "UNKNOWN";
    var reason = snapshot.health_reason || snapshot.reason || "";
    // §16.3：地图那一格要的是 LIVE / FROZEN_LAST_GOOD。后端在 §13.5 冻结时写
    // map_status（还有第三种 SUSPECT：判据说已发散、但没有 last-good 可退）。
    // 后端一个都不写时（判据没在跑）退回原来的 fresh 判断。
    var mapStatus = snapshot.map_status || (snapshot.fresh ? "LIVE" : "STALE");
    points = snapshot.points;
    occupancyGrid = snapshot.map_kind === "occupancy_grid_2d" ? snapshot.grid : null;
    occupancyBitmap = occupancyGrid ? buildOccupancyBitmap(occupancyGrid) : null;
    objects = Array.isArray(snapshot.objects) ? snapshot.objects : [];
    bounds = occupancyGrid ? gridBounds(occupancyGrid) : (snapshot.bounds || bounds);
    focusBounds = occupancyGrid ? bounds : (snapshot.focus_bounds || null);
    currentPose = snapshot.pose || null;
    var scan = snapshot.preview || {};
    // §9.1：pslam_odom 的 scan 和 pslam_map 的地图没有带时间戳的变换之前不许
    // 画在同一个坐标系里，所以只有两个 frame 相同时才允许显示调试层。
    previewDrawable = !!scan.frame_id && scan.frame_id === snapshot.canonical_frame;
    preview = previewDrawable && Array.isArray(scan.points) ? scan.points : [];
    if (!snapshot.available || (!occupancyGrid && !points.length)) {
      state.textContent = "等待固定世界地图 · " + health + (reason ? " · " + reason : "");
      state.className = "slam3d-state error";
      setLight("red");
    } else {
      state.textContent = (occupancyGrid ? "二维占据栅格" : mapStatus) + " · " + health +
        " · 地图 " + age(snapshot.map_age_seconds) + "前 · scan " +
        age(snapshot.preview_age_seconds) + "前" + (reason ? " · " + reason : "");
      state.className = "slam3d-state" + (snapshot.fresh && health === "HEALTHY" ? "" : " stale");
      setLight(snapshot.fresh && health === "HEALTHY" ? "green" : "yellow");
    }
    // 现场核验用：显示的/被滤的各几个。"门生效了"和"这次本来就没检出东西"
    // 在页面上长得一样，不给这个数就只能靠猜。
    // §2.3：物体只有在 map_epoch 与 frame_id 都与本地图相等时才会被合成进来。
    // 不相等时后端给空 objects 并说明原因 —— 页面必须把这件事说出来，否则
    // "这张图还没有物体"和"物体属于另一张地图"看起来完全一样。
    var overlayNote = overlayStatusNote(snapshot.object_overlay_status);
    var objectSummary = overlayNote || objectFilterSummary(snapshot.object_filter_stats);
    meta.textContent = count(snapshot.web_display_points) + (occupancyGrid ? " 个栅格 · " : " 显示点 · ") +
      (snapshot.canonical_frame || "pslam_map") + " · voxel " +
      Number(snapshot.global_voxel_size_m || 0).toFixed(2) + "m · session " +
      count(snapshot.mapping_session_id) + " · r" + count(snapshot.map_revision) +
      (objectSummary ? " · " + objectSummary : "");
    var extent = snapshot.map_extent_m || [0, 0, 0];
    if (capacity) {
      // "全局缓存体素"那层只有 §10 的桥才有（GlobalVoxelCloud + 上限）。写端
      // 没有这一层时整层不显示，而不是显示成 "N/--"。
      var layers = ["SLAM 源点 " + count(snapshot.source_map_points)];
      if (occupancyGrid && occupancyGrid.counts) {
        layers = ["未知 " + count(occupancyGrid.counts.unknown),
          "自由 " + count(occupancyGrid.counts.free),
          "占用 " + count(occupancyGrid.counts.occupied)];
      }
      if (snapshot.max_global_voxels != null) {
        layers.push("全局缓存体素 " + count(snapshot.global_cached_voxels) +
          "/" + count(snapshot.max_global_voxels));
      }
      if (!occupancyGrid) layers.push("网页显示 " + count(snapshot.web_display_points) +
        "/" + count(snapshot.max_web_points));
      capacity.textContent = layers.join(" → ") +
        " · 地图范围 " + Number(extent[0] || 0).toFixed(1) + "×" +
        Number(extent[1] || 0).toFixed(1) + "×" + Number(extent[2] || 0).toFixed(1) + "m" +
        (snapshot.capacity_limited ? " · 已整体降分辨率" : "") +
        // 没有 scan 时不能报"坐标系不同"—— 那是另一回事（写端根本没订阅 scan）。
        " · 当前 scan " + (scan.frame_id
          ? count(scan.point_count) + " 点(" + scan.frame_id + ")"
          : "未接入") +
        (previewDrawable || !scan.frame_id ? "" : "，与地图坐标系不同故不叠加显示") +
        (snapshot.last_rejected_reason ? " · 最近拒绝 " + snapshot.last_rejected_reason : "");
      capacity.className = "slam3d-capacity" +
        (health === "HEALTHY" ? "" : " degraded");
    }
    render();
  }

  function poll() {
    if (requestInFlight) return;
    requestInFlight = true;
    fetch("/api/slam/map3d", { cache: "no-store" })
      .then(function (response) { return response.json(); })
      .then(applySnapshot)
      .catch(function () {
        state.textContent = "3D 点云接口暂不可用";
        state.className = "slam3d-state error";
        setLight("red");
      })
      .finally(function () { requestInFlight = false; });
  }

  canvas.addEventListener("pointerdown", function (event) {
    dragging = true; lastX = event.clientX; lastY = event.clientY;
    canvas.classList.add("dragging"); canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", function (event) {
    if (!dragging) return;
    if (occupancyGrid) return;
    yaw += (event.clientX - lastX) * 0.008;
    pitch = Math.max(-1.35, Math.min(1.35, pitch + (event.clientY - lastY) * 0.006));
    lastX = event.clientX; lastY = event.clientY; render();
  });
  canvas.addEventListener("pointerup", function () { dragging = false; canvas.classList.remove("dragging"); });
  canvas.addEventListener("pointercancel", function () { dragging = false; canvas.classList.remove("dragging"); });
  canvas.addEventListener("wheel", function (event) {
    event.preventDefault();
    zoom = Math.max(0.25, Math.min(8, zoom * (event.deltaY > 0 ? 0.88 : 1.14)));
    render();
  }, { passive: false });
  canvas.addEventListener("click", function (event) {
    if (!occupancyGrid || !last2dView) return;
    var rect = canvas.getBoundingClientRect();
    var ratioX = canvas.width / Math.max(1, rect.width);
    var ratioY = canvas.height / Math.max(1, rect.height);
    var px = (event.clientX - rect.left) * ratioX;
    var py = (event.clientY - rect.top) * ratioY;
    var best = null, bestDistance = 18 * Math.max(ratioX, ratioY);
    objects.forEach(function (item) {
      var point = project2d(item.x, item.y, last2dView, last2dScale);
      var distance = Math.hypot(point[0] - px, point[1] - py);
      if (distance < bestDistance) { best = item; bestDistance = distance; }
    });
    if (best) window.dispatchEvent(new CustomEvent("semantic-object-select", {
      detail: { objectId: String(best.object_id || ""), source: "map2d" }
    }));
  });
  window.addEventListener("semantic-object-select", function (event) {
    selectedObjectId = String(event.detail && event.detail.objectId || "");
    render();
  });
  canvas.addEventListener("dblclick", function () { yaw = -0.72; pitch = 1.35; zoom = 1; render(); });
  if (previewToggle) previewToggle.addEventListener("change", render);
  window.addEventListener("resize", render);
  render();
  poll();
  // 这里只是网页展示轮询，不是雷达或 SLAM 的更新周期。二维遥控建图桥每 0.5 s
  // 写一次展示快照，三维遥控建图/自主搜索的桥默认每 1.0 s 写一次；网页取 1 s，
  // 因而可能跳过一份二维展示快照，但不会降低 SLAM 内部建图频率。
  // 850 ms 一次比绝大多数写端还密
  // —— 拉回来的 JSON 每次都是同一份（读取端按 mtime 缓存解析，但传输和渲染照样
  // 白做）。取 1 s：对最快的写端不慢，对 5 s 的写端也只是多空转几次。
  // 地图数据本身没变时 applySnapshot 仍是幂等的（render 结果不变）。
  window.setInterval(poll, 1000);
})();
