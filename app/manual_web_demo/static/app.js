/* Go2-W Manual WASD+QE Web Demo front-end.
 *
 * Responsibilities only: keyboard events, WebSocket, status rendering,
 * object polling, control buttons (plan book §30). No framework.
 */
(function () {
  "use strict";

  var KEY_MAP = { w: "w", s: "s", a: "a", d: "d", q: "q", e: "e" };
  var INTERCEPT = { w: 1, s: 1, a: 1, d: 1, q: 1, e: 1, " ": 1, Escape: 1 };

  var ws = null;
  var seq = 0;
  var controlEnabled = false;
  var pressed = {}; // key -> true
  var estopLatched = false;
  var lastMotionCommand = "stop";

  var els = {
    camera: document.getElementById("camera"),
    camFps: document.getElementById("cam-fps"),
    camAge: document.getElementById("cam-age"),
    camKey: document.getElementById("cam-key"),
    camCmd: document.getElementById("cam-cmd"),
    stale: document.getElementById("camera-stale"),
    lightCamera: document.getElementById("light-camera"),
    lightMotion: document.getElementById("light-motion"),
    lightLlm: document.getElementById("light-llm"),
    btnEstop: document.getElementById("btn-estop"),
    btnEstopReset: document.getElementById("btn-estop-reset"),
    btnEnable: document.getElementById("btn-enable"),
    btnDisable: document.getElementById("btn-disable"),
    motionState: document.getElementById("motion-state"),
    blockedNote: document.getElementById("blocked-note"),
    objectTbody: document.getElementById("object-tbody"),
    sceneSummary: document.getElementById("scene-summary"),
    llmLast: document.getElementById("llm-last"),
    llmStatus: document.getElementById("llm-status"),
    llmError: document.getElementById("llm-error"),
    btnLlm: document.getElementById("btn-llm"),
  };

  // ------------------------------------------------------------------ //
  // WebSocket                                                          //
  // ------------------------------------------------------------------ //
  function connect() {
    var proto = location.protocol === "https:" ? "wss:" : "ws:";
    ws = new WebSocket(proto + "//" + location.host + "/ws/control");
    ws.onopen = function () {
      send({ type: "hello" });
    };
    ws.onmessage = function (event) {
      try {
        var list = JSON.parse(event.data);
        var messages = Array.isArray(list) ? list : [list];
        messages.forEach(handleServerMessage);
      } catch (err) { /* ignore malformed frames */ }
    };
    ws.onclose = function () {
      releaseAllLocal();
      setControlEnabled(false);
      ws = null;
      setTimeout(connect, 1500);
    };
    ws.onerror = function () { try { ws.close(); } catch (e) {} };
  }

  function send(message) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  function handleServerMessage(message) {
    var type = message.type;
    if (type === "state" || type === "enable_result") {
      if (message.state) {
        lastMotionCommand = message.state.command || "stop";
        renderMotionState(message.state);
      }
      if (type === "enable_result") {
        if (message.ok) {
          setControlEnabled(true);
          note(null);
        } else {
          setControlEnabled(false);
          note(message.reason || "无法启用控制");
        }
      }
    } else if (type === "motion_blocked") {
      note(message.reason || "方向被安全门阻断");
    } else if (type === "motion_finished") {
      // Result handling is cosmetic; the next status poll reconciles state.
      renderMotionState({ command: lastMotionCommand, motion_in_flight: false });
    } else if (type === "deadman") {
      note("连接超时，已自动停止");
      setControlEnabled(false);
    }
  }

  // ------------------------------------------------------------------ //
  // Control enable / disable / estop                                    //
  // ------------------------------------------------------------------ //
  function setControlEnabled(value) {
    controlEnabled = value;
    els.btnEnable.classList.toggle("hidden", value);
    els.btnDisable.classList.toggle("hidden", !value);
    if (!value) {
      pressed = {};
      els.camKey.textContent = "按键: -";
    }
  }

  function enableControl() {
    send({ type: "enable_control" });
  }
  function disableControl() {
    releaseAllLocal();
    send({ type: "release_all" });
    setControlEnabled(false);
  }
  function emergencyStop() {
    estopLatched = true;
    pressed = {};
    send({ type: "estop" });
    note("急停已触发，需要重新启用控制");
    setControlEnabled(false);
    renderMotionState({ command: "stop", motion_in_flight: false });
  }

  function resetEmergencyStop() {
    fetch("/api/estop/reset", { method: "POST" })
      .then(function (response) { return response.json(); })
      .then(function (data) {
        if (!data.ok) {
          note(data.error || "解除急停前置检查未通过");
          return;
        }
        estopLatched = false;
        setControlEnabled(false);
        note("急停已解除；如需手动控制请重新启用，搜索可重新开始");
      })
      .catch(function () { note("解除急停请求失败"); });
  }

  // ------------------------------------------------------------------ //
  // Keyboard                                                           //
  // ------------------------------------------------------------------ //
  function isTypingTarget(event) {
    var node = event.target;
    if (!node) return false;
    var tag = (node.tagName || "").toLowerCase();
    return tag === "input" || tag === "textarea" || tag === "select" || node.isContentEditable;
  }

  document.addEventListener("keydown", function (event) {
    if (isTypingTarget(event)) return;
    var key = event.key;
    if (!(key in INTERCEPT)) return;
    event.preventDefault();

    if (key === "Escape") { emergencyStop(); return; }
    if (key === " ") { send({ type: "release_all" }); pressed = {}; return; }

    var mapped = KEY_MAP[key.toLowerCase()];
    if (!mapped) return;
    if (!controlEnabled) return;

    if (!pressed[mapped]) {
      pressed[mapped] = true;
      send({ type: "key_down", key: mapped });
    }
  });

  document.addEventListener("keyup", function (event) {
    if (isTypingTarget(event)) return;
    var key = event.key;
    if (!(key in INTERCEPT)) return;
    var mapped = KEY_MAP[key.toLowerCase()];
    if (!mapped) return;
    if (!controlEnabled) return;
    delete pressed[mapped];
    send({ type: "key_up", key: mapped });
  });

  // ------------------------------------------------------------------ //
  // Blur / hidden / unload → release_all (deadman, plan book §11)      //
  // ------------------------------------------------------------------ //
  function releaseAllLocal() {
    pressed = {};
  }
  window.addEventListener("blur", function () {
    releaseAllLocal();
    send({ type: "release_all" });
  });
  document.addEventListener("visibilitychange", function () {
    if (document.hidden) {
      releaseAllLocal();
      send({ type: "release_all" });
    }
  });
  window.addEventListener("beforeunload", function () {
    try { send({ type: "release_all" }); } catch (e) {}
  });

  // ------------------------------------------------------------------ //
  // Heartbeat (100ms)                                                   //
  // ------------------------------------------------------------------ //
  setInterval(function () {
    seq += 1;
    send({
      type: "heartbeat",
      seq: seq,
      pressed: Object.keys(pressed),
      control_enabled: controlEnabled,
    });
    if (controlEnabled) {
      var active = Object.keys(pressed);
      els.camKey.textContent = "按键: " + (active.join(",") || "-");
    }
  }, 100);

  // ------------------------------------------------------------------ //
  // Touch buttons (plan book §32) — reuse the same keyboard logic       //
  // ------------------------------------------------------------------ //
  function bindTouch(button) {
    var key = button.dataset.key;
    var down = function (event) {
      event.preventDefault();
      if (!controlEnabled || estopLatched) return;
      if (!pressed[key]) {
        pressed[key] = true;
        send({ type: "key_down", key: key });
      }
    };
    var up = function (event) {
      event.preventDefault();
      delete pressed[key];
      send({ type: "key_up", key: key });
    };
    button.addEventListener("pointerdown", down);
    button.addEventListener("pointerup", up);
    button.addEventListener("pointercancel", up);
    button.addEventListener("pointerleave", up);
  }

  document.querySelectorAll(".keybtn[data-key]").forEach(bindTouch);

  // ------------------------------------------------------------------ //
  // Polling: status (500ms) + objects (1000ms)                          //
  // ------------------------------------------------------------------ //
  function pollStatus() {
    fetch("/api/status").then(function (response) { return response.json(); })
      .then(renderStatus)
      .catch(function () {});
  }

  function renderStatus(status) {
    var camera = status.camera || {};
    var motion = status.motion || {};
    var llm = status.llm || {};
    var analysis = llm.analysis || {};
    var directions = status.directions || {};

    // Camera light.
    if (camera.fresh) setLight(els.lightCamera, "green");
    else if (camera.available) setLight(els.lightCamera, "red");
    else setLight(els.lightCamera, "gray");
    els.stale.classList.toggle("hidden", camera.fresh !== false || !camera.available);
    els.camAge.textContent = "帧年龄 " + fmtAge(camera.age_seconds);

    // Motion light.
    if (motion.available) {
      if (motion.control_enabled) setLight(els.lightMotion, "green");
      else setLight(els.lightMotion, "yellow");
    } else {
      setLight(els.lightMotion, "gray");
    }
    els.btnEstop.disabled = !motion.available;
    var owner = status.owner || {};
    var estopActive = owner.owner === "ESTOP" || motion.state === "ESTOP";
    els.btnEstopReset.disabled = !estopActive || !motion.available;
    if (!estopActive) estopLatched = false;
    renderMotionState(motion);

    // LLM light + toggle state.
    var llmEnabled = status.llm && status.llm.enabled;
    els.btnLlm.classList.toggle("on", !!llmEnabled);
    els.btnLlm.textContent = llmEnabled ? "大模型分析: 开" : "大模型分析: 关";
    if (!llmEnabled) setLight(els.lightLlm, "gray");
    else if (analysis.status === "running") setLight(els.lightLlm, "yellow");
    else if (analysis.status === "error") setLight(els.lightLlm, "red");
    else if (analysis.status === "ok") setLight(els.lightLlm, "green");
    else setLight(els.lightLlm, "gray");

    // Direction availability labels (plan book §53).
    document.querySelectorAll(".keybtn[data-key]").forEach(function (btn) {
      var key = btn.dataset.key;
      var dir = dirForKey(key);
      var info = directions[dir] || {};
      btn.classList.toggle("blocked", info.allowed === false);
      btn.title = info.allowed === false ? (info.reason || "不可用") : "";
    });

    // Blocked note from controller state.
    if (motion.blocked_reason) note(motion.blocked_reason);
  }

  function pollObjects() {
    fetch("/api/objects").then(function (response) { return response.json(); })
      .then(renderObjects)
      .catch(function () {});
  }

  function renderObjects(data) {
    els.sceneSummary.textContent = data.scene_summary || "—";
    els.llmStatus.textContent = statusText(data.status);
    els.llmLast.textContent = data.analysis_finished_at
      ? new Date(data.analysis_finished_at * 1000).toLocaleTimeString()
      : "--";
    els.llmError.textContent = data.error || "";
    els.llmError.classList.toggle("hidden", !data.error);

    var tbody = els.objectTbody;
    var objects = data.objects || [];
    if (!objects.length) {
      tbody.innerHTML = '<tr><td colspan="4" class="empty">等待识别…</td></tr>';
      return;
    }
    var html = "";
    objects.forEach(function (obj) {
      html +=
        "<tr>" +
        "<td>" + esc(obj.name_zh || obj.name_en || "") + "</td>" +
        "<td>" + (obj.count === null || obj.count === undefined ? "—" : esc(String(obj.count))) + "</td>" +
        "<td>" + esc(obj.position || "—") + "</td>" +
        '<td class="conf-' + esc(obj.confidence || "medium") + '">' + esc(obj.confidence || "medium") + "</td>" +
        "</tr>";
    });
    tbody.innerHTML = html;
  }

  function statusText(s) {
    return { idle: "空闲", running: "识别中", ok: "识别完成", error: "识别失败" }[s] || s || "--";
  }

  function dirForKey(key) {
    return { w: "forward", s: "backward", a: "strafe_left", d: "strafe_right",
             q: "turn_left", e: "turn_right" }[key] || "";
  }

  // ------------------------------------------------------------------ //
  // Rendering helpers                                                   //
  // ------------------------------------------------------------------ //
  function renderMotionState(motion) {
    var command = motion.command || lastMotionCommand || "stop";
    var label = { forward: "前进", backward: "后退", strafe_left: "左移",
                  strafe_right: "右移", turn_left: "左转", turn_right: "右转",
                  stop: "STOP" }[command] || String(command).toUpperCase();
    if (motion.motion_in_flight) label += " …";
    els.motionState.textContent = "当前：" + label;
    els.camCmd.textContent = "当前: " + label;
  }

  function setLight(el, cls) {
    el.className = "light " + cls;
  }

  function note(text) {
    els.blockedNote.textContent = text || "";
  }

  function fmtAge(seconds) {
    if (seconds === null || seconds === undefined) return "--";
    return seconds.toFixed(1) + "s";
  }

  function esc(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  // ------------------------------------------------------------------ //
  // Camera FPS                                                          //
  // ------------------------------------------------------------------ //
  var lastFrameAt = null;
  var fps = 0;
  els.camera.addEventListener("load", function () {
    var now = performance.now();
    if (lastFrameAt !== null) {
      var dt = (now - lastFrameAt) / 1000;
      if (dt > 0.01) fps = 1 / dt;
    }
    lastFrameAt = now;
    els.camFps.textContent = Math.round(fps) + " FPS";
  });

  // ------------------------------------------------------------------ //
  // Buttons                                                             //
  // ------------------------------------------------------------------ //
  els.btnEnable.addEventListener("click", enableControl);
  els.btnDisable.addEventListener("click", disableControl);
  els.btnEstop.addEventListener("click", emergencyStop);
  els.btnEstopReset.addEventListener("click", resetEmergencyStop);
  els.btnLlm.addEventListener("click", function () {
    var on = els.btnLlm.classList.contains("on");
    fetch(on ? "/api/llm/disable" : "/api/llm/enable", { method: "POST" })
      .then(function (r) { return r.json(); })
      .then(function (data) {
        els.btnLlm.classList.toggle("on", data.enabled);
        els.btnLlm.textContent = data.enabled ? "大模型分析: 开" : "大模型分析: 关";
      })
      .catch(function () {});
  });

  setInterval(pollStatus, 500);
  setInterval(pollObjects, 1000);
  pollStatus();
  pollObjects();
  connect();
})();
