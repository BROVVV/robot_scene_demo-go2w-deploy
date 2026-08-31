/* Autonomous Semantic Search WebUI front-end (plan book §5-§9, §63-§70).
 *
 * Responsibilities only: target entry, start/pause/resume/stop buttons,
 * /ws/search (snapshot + incremental SearchEvents), status / evidence /
 * decision / candidates / timeline rendering, canvas overlay and the SVG map
 * driver.  NONE of the search decisions are made in the browser.
 */
(function () {
  "use strict";

  var appState = {
    search: {},
    observation: {},
    objects: {},
    targetMatch: {},
    selectedGoal: null,
    task: {},
    decisions: [],
    nextMotionCommand: null,
    candidates: [],
    map: {},
    events: [],
    health: {},
  };

  var STARTUP_STAGE_ZH = {
    "IDLE": "空闲",
    "SPAWN_WORKER": "正在启动搜索进程",
    "WORKER_READY": "工作进程就绪",
    "LOAD_PIPELINE": "正在初始化 SemanticNavigation",
    "WAIT_RGBD": "正在等待 D435 RGB-D",
    "WAIT_SPATIAL_PROVIDER": "正在连接 RTAB-Map",
    "START_EXPLORER": "正在启动探索器",
    "RUNNING": "运行中",
    "FAILED": "启动异常",
    "INIT": "初始化",
  };

  var ws = null;
  var wsState = "OFFLINE";
  var wsAttempt = 0;
  var wsTimer = null;
  var lastDetectionFrame = null;
  var timelineCount = 0;
  var viewingHistoryId = null;
  var currentLiveState = null;

  var els = {
    // taskbar
    target: document.getElementById("search-target"),
    btnStart: document.getElementById("btn-search-start"),
    btnPause: document.getElementById("btn-search-pause"),
    btnResume: document.getElementById("btn-search-resume"),
    btnStop: document.getElementById("btn-search-stop"),
    btnEstop: document.getElementById("btn-search-estop"),
    chkDebug: document.getElementById("chk-debug"),
    chkMotion: document.getElementById("chk-motion"),
    sessionInfo: document.getElementById("search-session-info"),
    banner: document.getElementById("search-banner"),
    errorDetail: document.getElementById("search-error-detail"),
    activeControls: document.getElementById("search-active-controls"),
    btnHistoryCurrent: document.getElementById("btn-history-current"),
    history: document.getElementById("search-history"),
    // camera
    cam: document.getElementById("search-camera"),
    overlay: document.getElementById("search-overlay"),
    scamFps: document.getElementById("scam-fps"),
    scamAge: document.getElementById("scam-age"),
    scamLabel: document.getElementById("scam-label"),
    scamCycle: document.getElementById("scam-cycle"),
    camStale: document.getElementById("search-camera-stale"),
    // status
    stTarget: document.getElementById("st-target"),
    stPhase: document.getElementById("st-phase"),
    stCycle: document.getElementById("st-cycle"),
    stElapsed: document.getElementById("st-elapsed"),
    stMatch: document.getElementById("st-match"),
    stAnchor: document.getElementById("st-anchor"),
    stAction: document.getElementById("st-action"),
    stPose: document.getElementById("st-pose"),
    stEvidence: document.getElementById("st-evidence"),
    taskUnderstanding: document.getElementById("task-understanding"),
    // observation
    obsCurrent: document.getElementById("obs-current"),
    obsSeen: document.getElementById("obs-seen"),
    obsObjects: document.getElementById("obs-objects"),
    obsObjectsMeta: document.getElementById("obs-objects-meta"),
    // decision
    decIntent: document.getElementById("dec-intent"),
    decReason: document.getElementById("dec-reason"),
    decScores: document.getElementById("dec-scores"),
    decCandidates: document.getElementById("dec-candidates"),
    decMotion: document.getElementById("dec-motion"),
    decHistory: document.getElementById("dec-history"),
    // map
    mapMeta: document.getElementById("map-meta"),
    mapNodeDetail: document.getElementById("map-node-detail"),
    mapViewToggle: document.getElementById("map-view-toggle"),
    mapLegend: document.getElementById("map-legend"),
    // timeline
    timeline: document.getElementById("search-timeline"),
    tlWsState: document.getElementById("tl-ws-state"),
    // lights
    lightSearch: document.getElementById("light-search"),
    lightRobot: document.getElementById("light-robot"),
    lightWs: document.getElementById("light-ws"),
    // system tab
    sysCamera: document.getElementById("sys-camera"),
    sysWorker: document.getElementById("sys-worker"),
    sysMotion: document.getElementById("sys-motion"),
    sysOwner: document.getElementById("sys-owner"),
    sysSearch: document.getElementById("sys-search"),
    sysLlm: document.getElementById("sys-llm"),
    sysReadiness: document.getElementById("sys-readiness"),
    sysHistory: document.getElementById("sys-history"),
    // debug
    debugPanel: document.getElementById("debug-panel"),
    debugGoalGraph: document.getElementById("debug-goal_graph"),
    debugSceneGraph: document.getElementById("debug-scene_graph"),
    debugCandidates: document.getElementById("debug-candidates"),
    debugRaw: document.getElementById("debug-raw"),
  };

  var mapRenderer = new window.SearchMapRenderer(
    document.getElementById("search-map"),
    els.mapNodeDetail
  );

  // ------------------------------------------------------------------ //
  // WebSocket /ws/search (plan book §22-§24, §65)                       //
  // ------------------------------------------------------------------ //
  function connect() {
    wsAttempt += 1;
    var proto = location.protocol === "https:" ? "wss:" : "ws:";
    ws = new WebSocket(proto + "//" + location.host + "/ws/search");
    ws.onopen = function () {
      wsAttempt = 0;
      setWsState("LIVE");
    };
    ws.onmessage = function (event) {
      try {
        var message = JSON.parse(event.data);
        handleWsMessage(message);
      } catch (err) { /* ignore malformed */ }
    };
    ws.onclose = function () {
      setWsState("RECONNECTING");
      ws = null;
      scheduleReconnect();
    };
    ws.onerror = function () {
      try { ws.close(); } catch (e) {}
    };
  }

  function scheduleReconnect() {
    if (wsTimer) return;
    var delays = [1, 2, 5];
    var delay = delays[Math.min(wsAttempt - 1, delays.length - 1)] || 5;
    wsTimer = setTimeout(function () {
      wsTimer = null;
      if (ws === null) connect();
    }, delay * 1000);
  }

  function setWsState(state) {
    wsState = state;
    els.tlWsState.textContent = state;
    setLight(els.lightWs, state === "LIVE" ? "green" : state === "RECONNECTING" ? "yellow" : "red");
  }

  function handleWsMessage(message) {
    var type = message.type;
    if (type === "snapshot") {
      if (message.state) {
        currentLiveState = message.state;
        if (!viewingHistoryId) applyStateSnapshot(message.state);
      }
    } else if (type === "events") {
      if (!viewingHistoryId) {
        // The snapshot is authoritative.  Replaying SESSION_CREATED and old
        // deltas after it would clear accumulated decisions/topology on every
        // F5.  Historical events only seed the timeline; new live events are
        // still applied normally through the singular "event" message.
        appState.events = (message.events || []).map(function (event) {
          return { event_type: event.event_type, timestamp: event.timestamp, cycle: event.cycle };
        });
        renderTimeline();
      }
    } else if (type === "event") {
      if (!viewingHistoryId) applySearchEvent(message.event);
    } else if (type === "heartbeat") {
      /* keepalive only */
    }
  }

  // ------------------------------------------------------------------ //
  // State application                                                   //
  // ------------------------------------------------------------------ //
  // 合并后端 spatial：只要出现了非空 object_topology 就用新的；否则保留上一次
  // 非空图形（搜索结束 IDLE 也不清空），仅当 sessionId 变化时清空。
  function mergeSpatialGraph(freshSpatial, sessionChanged) {
    var out = Object.assign({}, freshSpatial || {});
    if (sessionChanged) return out;
    var ot = (out.semantic_graph || {}).object_topology;
    var oldGraph = appState._lastSemanticGraph ||
      (appState.spatial && appState.spatial.semantic_graph) || null;
    var keep = null;
    if (ot && Array.isArray(ot.nodes) && ot.nodes.length) keep = out.semantic_graph;
    else if (oldGraph && oldGraph.object_topology &&
             Array.isArray(oldGraph.object_topology.nodes) &&
             oldGraph.object_topology.nodes.length) keep = oldGraph;
    if (keep) {
      out = Object.assign({}, out, { semantic_graph: keep });
      appState._lastSemanticGraph = keep;
    }
    return out;
  }

  // 兜底轮询：每 2.5s 拉一次后端 state，保证拓扑图/物体列表“从第一次识别就开始
  // 实时更新”，即使单个 WS 事件偶尔掉线也不会漏。
  var statePollTimer = null;
  function startStatePolling() {
    if (statePollTimer) return;
    statePollTimer = window.setInterval(function () {
      fetch("/api/search/state", { method: "GET", headers: { "Accept": "application/json" } })
        .then(function (r) { return r.json(); })
        .then(function (state) {
          if (!state || !state.session_id) return;
          currentLiveState = state;
          if (viewingHistoryId) return;
          var sessionChanged = !!(appState.search.session_id &&
            state.session_id !== appState.search.session_id);
          if (!appState.search.session_id) appState.search.session_id = state.session_id;
          appState.spatial = mergeSpatialGraph(state.spatial || {}, sessionChanged);
          renderAll();
        })
        .catch(function () { /* 后端暂时不可达时忽略，保持上次画面 */ });
    }, 2500);
  }
  startStatePolling();

  function applyStateSnapshot(state) {
    if (!state || !state.session_id) return;
    if (["MAX_STEPS_REACHED", "MAX_PLANNING_CYCLES_REACHED"].indexOf(
        state.result || state.finish_reason || "") >= 0) {
      state.status = "FINISHED";
      state.phase = "FINISHED";
      state.error = null;
      state.last_error = "";
    }
    if (state.status === "FINISHED" && !state.error &&
        els.banner && els.banner.classList.contains("error")) {
      els.banner.textContent = "";
      els.banner.className = "search-banner hidden";
    }
    var sessionChanged = !!(appState.search.session_id && state.session_id !== appState.search.session_id);
    appState.search = state;
    appState.observation = state.observation || {};
    appState.objects = state.objects || {};
    appState.targetMatch = state.target_match || state.targetMatch || {};
    appState.selectedGoal = state.selected_goal || null;
    appState.task = state.task || {};
    appState.decisions = state.decisions || [];
    appState.nextMotionCommand = state.next_motion_command || null;
    appState.candidates = state.candidates || [];
    appState.map = state.map || {};
    appState.spatial = mergeSpatialGraph(state.spatial || {}, sessionChanged);
    if (sessionChanged) appState._lastSemanticGraph = null;
    state.timeline = state.timeline || [];
    renderAll();
  }

  function applySearchEvent(event) {
    if (!event || !event.event_type) return;
    // stale-event guard: ignore events from a previous session
    if (appState.search.session_id && event.session_id &&
        appState.search.session_id !== event.session_id) {
      return;
    }
    var type = event.event_type;
    var payload = event.payload || {};
    timelineCount += 1;

    switch (type) {
      case "SESSION_CREATED":
        viewingHistoryId = null;
        appState.search = {};
        appState.search.target = payload.target;
        appState.search.session_id = event.session_id;
        appState.search.status = "STARTING";
        appState.search.phase = payload.phase || "STARTING";
        appState.task = payload.task || {};
        appState.decisions = [];
        appState.nextMotionCommand = null;
        appState.targetMatch = {};
        appState.selectedGoal = null;
        appState.candidates = [];
        appState.map = {};
        break;
      case "TASK_UNDERSTANDING":
        appState.task = payload.task || {};
        appState.search.task = appState.task;
        appState.search.phase = "TASK_UNDERSTANDING";
        break;
      case "TASK_REJECTED":
        appState.search.status = "TASK_REJECTED";
        appState.search.phase = "TASK_REJECTED";
        showBanner("任务不可执行: " + (payload.reason || "请修改任务描述"), "error");
        break;
      case "SESSION_STARTED":
        appState.search.status = "RUNNING";
        appState.search.phase = payload.phase || "BOOTSTRAP";
        break;
      case "SEARCH_STATE_CHANGED":
        appState.search.phase = payload.phase || appState.search.phase || "RUNNING";
        if (payload.phase_detail !== undefined) {
          appState.search.phase_detail = payload.phase_detail || "";
        }
        if (payload.phase_started_at !== undefined) {
          appState.search.phase_started_at = payload.phase_started_at;
        }
        if (payload.startup) appState.search.startup = payload.startup;
        if (payload.warning) appState.search.last_warning = payload.warning;
        break;
      case "OBSERVATION_UPDATED":
        appState.observation = {
          bundle_id: payload.bundle_id,
          timestamp: payload.timestamp,
          objects: payload.scene_objects || payload.objects || [],
          detections: payload.detections || [],
          target_present: payload.target_present,
          heading_sector: payload.heading_sector,
          pose: payload.pose,
          sensor_health: payload.sensor_health || {},
        };
        lastDetectionFrame = payload.detections || null;
        break;
      case "OBJECTS_UPDATED":
        appState.objects.current = payload.current || [];
        appState.objects.target_evidence = payload.target_evidence || {};
        break;
      case "TARGET_MATCH_UPDATED":
        appState.targetMatch = {
          level: payload.target_match_level || "none",
          target_score: payload.target_score,
          anchor_labels: payload.anchor_labels || [],
          explicit_anchor_found: payload.explicit_anchor_found,
          directive: payload.directive,
          graph_match: payload.graph_match,
        };
        appState.search.goal_graph = payload.goal_graph || appState.search.goal_graph;
        break;
      case "VERIFICATION_STARTED":
        appState.search.phase = "VERIFY";
        appState.verifying = payload;
        break;
      case "VERIFICATION_FINISHED":
        appState.search.phase = "VERIFY";
        appState.verification = payload;
        break;
      case "TARGET_CONFIRMED":
        appState.search.status = "TARGET_FOUND";
        appState.search.phase = "TARGET_FOUND";
        showBanner("✓ TARGET FOUND", "found");
        break;
      case "MEMORY_UPDATED":
        appState.search.phase = payload.phase || "UPDATE_MEMORY";
        break;
      case "CANDIDATES_GENERATED":
        appState.candidates = payload.candidates || [];
        break;
      case "GOAL_SELECTED":
        appState.selectedGoal = {
          goal: payload.goal || {},
          score: payload.score,
          components: payload.components || {},
          reasons: payload.reasons || [],
          planning_cycles: payload.planning_cycles,
        };
        break;
      case "ACTION_STARTED":
        appState.search.phase = "EXECUTE";
        appState.search.phase_detail = payload.phase_detail || "正在等待动作服务器执行并回传结果";
        appState.robotAction = "EXECUTING";
        if (payload.next_motion_command) appState.nextMotionCommand = payload.next_motion_command;
        break;
      case "DECISION_RECORDED":
        var decision = payload.decision || payload;
        appState.nextMotionCommand = decision.next_motion_command || appState.nextMotionCommand;
        appState.decisions = appState.decisions || [];
        var did = decision.decision_id;
        appState.decisions = appState.decisions.filter(function (item) {
          return !did || item.decision_id !== did;
        });
        appState.decisions.push(decision);
        break;
      case "ACTION_FINISHED":
        appState.robotAction = payload.status === "succeeded" ? "SUCCEEDED" : "FAILED";
        appState.search.phase = "WAIT_RESULT";
        appState.search.phase_detail = payload.phase_detail || payload.message || "动作已结束";
        if (payload.status !== "succeeded" && /MOTION_(ACCEPT|RESULT)_TIMEOUT/.test(payload.message || "")) {
          showBanner("动作响应超时，已取消并安全停止；搜索将重新规划。原因：" + payload.message, "error");
        }
        break;
      case "REPLAN":
        appState.search.phase = "RECOVER";
        break;
      case "PAUSED":
        appState.search.status = "PAUSED";
        appState.search.phase = "PAUSED";
        break;
      case "RESUMED":
        appState.search.status = "RUNNING";
        appState.search.phase = "OBSERVE";
        break;
      case "SEARCH_EXHAUSTED":
        appState.search.status = "SEARCH_EXHAUSTED";
        showBanner("搜索空间已穷尽（SEARCH_EXHAUSTED）", "exhausted");
        break;
      case "OPERATOR_STOP":
        appState.search.status = "OPERATOR_STOP";
        showBanner("操作员停止（OPERATOR_STOP）", "error");
        break;
      case "ERROR":
        appState.search.status = "FAILED";
        appState.search.error = payload;
        showBanner("错误: " + (payload.message || payload.error_type || "unknown"), "error");
        break;
      case "MAP_UPDATED":
        appState.map = {
          revision: payload.revision,
          map_mode: payload.map_mode || "topological",
          current_node_id: payload.current_node_id,
          robot: payload.robot,
          nodes: (payload.graph || {}).nodes || [],
          edges: (payload.graph || {}).edges || [],
        };
        // Semantic topology view reads state.spatial.semantic_graph.object_topology.
        // Prefer semantic_navigation_graph; otherwise accept a graph that itself
        // carries object_topology.  Exploration-only graphs stay ignored so the
        // topology list never flips to empty.
        appState.spatial = appState.spatial || {};
        var topoGraph = payload.semantic_navigation_graph ||
          ((payload.graph && payload.graph.object_topology) ? payload.graph : null);
        if (topoGraph) {
          appState.spatial.semantic_graph = topoGraph;
          appState._lastSemanticGraph = topoGraph;
        }
        break;
      case "RGBD_FRAME_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.rgbd_frame = payload;
        break;
      case "SPATIAL_POSE_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.spatial_pose = payload.pose || null;
        break;
      case "SPATIAL_MAP_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.spatial_map = payload.map || null;
        break;
      case "FRONTIERS_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.frontiers = payload.frontiers || [];
        break;
      case "PLACE_CREATED":
      case "PLACE_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.place_graph = appState.spatial.place_graph || { places: [], edges: [] };
        var place = payload.place || {};
        var places = appState.spatial.place_graph.places || [];
        var idx = places.findIndex(function (p) { return p.place_id === place.place_id; });
        if (idx >= 0) { places[idx] = place; } else { places.push(place); }
        appState.spatial.place_graph.places = places;
        break;
      case "SEMANTIC_OBJECT_LOCALIZED":
        appState.spatial = appState.spatial || {};
        appState.spatial.semantic_objects = appState.spatial.semantic_objects || [];
        var obj = payload.object || {};
        var objs = appState.spatial.semantic_objects;
        var oidx = objs.findIndex(function (o) { return o.object_id === obj.object_id; });
        if (oidx >= 0) { objs[oidx] = obj; } else { objs.push(obj); }
        break;
      case "PSG_PRIOR_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.psg_prior = payload.prior || null;
        break;
      case "SEMANTIC_REGION_CREATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.psg_prior = appState.spatial.psg_prior || { region_hypotheses: [] };
        appState.spatial.psg_prior.region_hypotheses = appState.spatial.psg_prior.region_hypotheses || [];
        appState.spatial.psg_prior.region_hypotheses.push(payload.region || {});
        break;
      case "LONG_TERM_GOAL_SELECTED":
        appState.spatial = appState.spatial || {};
        appState.spatial.long_term_goal = payload.intent || null;
        break;
      case "LOCAL_GOAL_PROGRESS":
        appState.spatial = appState.spatial || {};
        appState.spatial.local_goal_progress = payload.progress || null;
        break;
      case "SEARCH_FINISHED":
        appState.search.status = payload.result === "TARGET_FOUND" ? "TARGET_FOUND"
          : payload.result === "OPERATOR_STOP" ? "OPERATOR_STOP"
          : payload.result === "SEARCH_EXHAUSTED" ? "SEARCH_EXHAUSTED"
          : "FINISHED";
        appState.search.result = payload.result;
        appState.search.finish_reason = payload.finish_reason;
        appState.search.summary = payload;
        appState.search.last_error = payload.error || payload.reason || "";
        var failed = payload.result === "FAILED" || payload.result === "TIMEOUT"
          || payload.result === "BACKEND_FAILURE" || payload.result === "PERCEPTION_FAILURE";
        if (payload.result === "TARGET_FOUND") {
          showBanner("搜索成功：已找到目标！", "success");
        } else if (failed) {
          var reason = payload.error || payload.reason || payload.finish_reason || "FAILED";
          var hint = "";
          if (/No module named|ModuleNotFoundError|ImportError/.test(reason)) {
            hint = "（worker 缺少依赖，请检查 outputs/autonomous_search/logs/search_worker.log；或用 --mock 离线演示）";
          } else if (/rclpy|import.*rclpy/.test(reason)) {
            hint = "（当前 worker 解释器没有 ROS rclpy：真机请确认用 ROS Python 启服务；本机演示请用 --mock 模式）";
          }
          showBanner("搜索失败: " + reason.slice(0, 220) + hint, "error");
        } else {
          showBanner("搜索结束: " + (payload.result || "FINISHED"), "exhausted");
        }
        break;
      default:
        break;
    }
    pushTimeline(event);
    renderAll();
  }

  // ------------------------------------------------------------------ //
  // Controls                                                            //
  // ------------------------------------------------------------------ //
  function api(path, body) {
    return fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body ? JSON.stringify(body) : undefined,
    }).then(function (r) { return r.json(); });
  }

  function startSearch() {
    viewingHistoryId = null;
    doStartSearch();
  }

  function doStartSearch() {
    var target = els.target.value.trim();
    if (!target) { showBanner("请输入搜索目标", "error"); return; }
    api("/api/search/start", {
      task_text: target,
      // One-release compatibility alias for an already-running old worker.
      target: target
    }).then(function (data) {
      if (!data.ok) {
        // The server automatically finalizes dead/stalled workers.  A healthy
        // active task must never be stopped just because a refreshed browser
        // briefly showed an enabled Start button.
        if (/already active/.test(data.error || "")) {
          showErrorRetry(data.error, data.error_detail);
          pollSearchState();
          return;
        }
        if (data.error === "emergency_stop_latched") {
          showBanner("无法开始：请先点击顶部“解除急停”，确认状态正常后再搜索", "error");
        } else {
          showErrorRetry(data.error, data.error_detail);
        }
        return;
      }
      // Refresh the status bar immediately instead of waiting for the WS
      // snapshot/event round-trip (the worker can be slow to reach STARTING).
      appState.search = appState.search || {};
      appState.search.session_id = data.session_id;
      appState.search.status = data.status || "STARTING";
      appState.search.phase = "STARTING";
      showBanner("已开始搜索（等待后端就绪）…", "");
      renderStatus();
      renderButtons();
    }).catch(function (err) {
      showErrorRetry(String(err || "network error"), {
        code: "WEBUI_NETWORK_ERROR", title: "WebUI 无法连接搜索服务",
        message: String(err || "network error"),
        cause: "浏览器到 WebUI 后端的请求失败。",
        suggestion: "确认 Web 服务仍在运行并检查浏览器网络连接。",
        source: "browser", stage: "START", recoverable: true
      });
    });
  }

  function showErrorRetry(error, detail) {
    appState.search = appState.search || {};
    appState.search.error = detail || { message: error };
    showBanner("无法开始: " + (error || "unknown") + "（已自动尝试释放占用，可再点一次开始）", "error");
    renderErrorDetail();
  }

  els.btnStart.addEventListener("click", startSearch);
  els.target.addEventListener("keydown", function (event) {
    if (event.key === "Enter") startSearch();
  });
  els.btnPause.addEventListener("click", function () {
    api("/api/search/pause").catch(function () {});
  });
  els.btnResume.addEventListener("click", function () {
    api("/api/search/resume").catch(function () {});
  });
  els.btnStop.addEventListener("click", function () {
    api("/api/search/stop").then(function (data) {
      if (data && data.status === "IDLE") {
        appState.search = appState.search || {};
        appState.search.status = "IDLE";
        appState.search.phase = "IDLE";
        renderStatus();
        renderButtons();
        showBanner("已停止当前会话", "error");
      } else {
        showBanner("已请求停止…", "error");
      }
    }).catch(function () {});
  });
  els.btnEstop.addEventListener("click", function () {
    api("/api/search/estop").then(function () {
      showBanner("急停已触发", "error");
    }).catch(function () {});
  });

  // Map view toggle: 语义拓扑 (default) / 空间地图 (plan §17).
  if (els.mapViewToggle) {
    els.mapViewToggle.addEventListener("click", function (event) {
      var btn = event.target.closest ? event.target.closest("button") : null;
      if (!btn || !btn.dataset || !btn.dataset.mode) return;
      var mode = btn.dataset.mode === "spatial_map" ? "spatial_map" : "semantic_topology";
      Array.prototype.forEach.call(els.mapViewToggle.querySelectorAll(".vtab"), function (b) {
        b.classList.toggle("active", b.dataset.mode === mode);
      });
      mapRenderer.setMode(mode);
      if (els.mapLegend) {
        els.mapLegend.style.display = mode === "spatial_map" ? "" : "none";
      }
      renderAll();
    });
  }
  // Default: semantic topology; hide the metric legend by default.
  if (els.mapLegend) els.mapLegend.style.display = "none";

  // ------------------------------------------------------------------ //
  // Rendering                                                           //
  // ------------------------------------------------------------------ //
  function renderObjects() {
    // 识别物体列表：与语义拓扑同源（spatial.semantic_graph.object_topology），
    // 从第一次识别到物体就开始显示，并随每次 memory_update 实时刷新。
    var spatial = appState.spatial || {};
    var ot = ((spatial.semantic_graph || {}).object_topology) || null;
    var nodes = ot && Array.isArray(ot.nodes) ? ot.nodes : [];
    if (els.obsObjectsMeta) {
      els.obsObjectsMeta.textContent = nodes.length ? nodes.length + " 个" : "";
    }
    if (!els.obsObjects) return;
    if (!nodes.length) {
      els.obsObjects.innerHTML =
        '<div class="empty">等待首次识别…（识别到第一个物体后自动开始建物体列表 / 拓扑图）</div>';
      return;
    }
    var statusColor = { CONFIRMED: "#34d399", TENTATIVE: "#38bdf8", STALE: "#8b95a3" };
    var html = nodes.slice().sort(function (a, b) {
      return String(a.node_id).localeCompare(String(b.node_id));
    }).map(function (n) {
      var color = statusColor[n.status] || "#8b95a3";
      var star = n.is_target_confirmed ? "★ " : (n.is_target_candidate ? "◎ " : "");
      var meta = String(n.status || "TENTATIVE");
      if (n.observation_count != null) meta += " ×" + n.observation_count;
      var label = esc(n.label || "");
      return '<div class="obj-item">' +
        '<b style="color:' + color + '">' + star + esc(n.node_id) + '</b> ' +
        '<span>' + label + '</span>' +
        '<span class="obj-meta">' + esc(meta) + '</span>' +
        '</div>';
    }).join("");
    els.obsObjects.innerHTML = html;
  }

  function renderAll() {
    renderTaskUnderstanding();
    renderStatus();
    renderObservation();
    renderObjects();
    renderDecision();
    renderCandidates();
    renderTimeline();
    renderButtons();
    renderDebug();
    renderErrorDetail();
    var spatial = appState.spatial || null;
    var hasOldMap = appState.map && Array.isArray(appState.map.nodes) && appState.map.nodes.length;
    var hasPlaces = spatial && spatial.place_graph &&
      Array.isArray(spatial.place_graph.places) && spatial.place_graph.places.length;
    var mode = mapRenderer.getMode();
    if (mode === "semantic_topology") {
      // Semantic topology projection: show only topology stats, never metric
      // map / place metrics (they belong to the spatial map view).
      var hasGraph = !!(spatial && spatial.semantic_graph && spatial.semantic_graph.object_topology);
      var topology = (spatial && spatial.semantic_graph && spatial.semantic_graph.object_topology) || null;
      var nodes = (topology && topology.nodes) || [];
      var edges = (topology && topology.edges) || [];
      var empty = (nodes.length === 0);
      els.mapMeta.textContent = empty
        ? (hasGraph ? "已建图 0 节点 · 等待识别到物体" : "等待首次物体识别…识别后自动开始建拓扑")
        : "拓扑节点 " + nodes.length + " · 关系 " + edges.length +
          " · rev " + ((topology && topology.revision) || 0);
      mapRenderer.render(appState.map || {}, spatial);
    } else {
      if (hasOldMap || hasPlaces) {
        mapRenderer.render(appState.map || {}, spatial);
        var placeCount = hasPlaces ? spatial.place_graph.places.length : 0;
        els.mapMeta.textContent = "Map revision " + ((appState.map || {}).revision || 0) +
          " · Place " + placeCount +
          " · Frontier " + ((spatial && spatial.frontiers) || []).length;
      }
    }
    drawOverlay();
  }

  function renderStatus() {
    var s = appState.search || {};
    els.stTarget.textContent = s.target || "--";
    var phaseText = s.phase || (s.status || "IDLE");
    if (s.phase_detail) phaseText += " · " + s.phase_detail;
    els.stPhase.textContent = phaseText;
    els.stCycle.textContent = String(s.cycle || 0);
    els.stElapsed.textContent = fmtDuration(s.elapsed_seconds);

    // Startup stage (plan §4): render explicit progress and disable the start
    // button while a search is STARTING/RUNNING to avoid the double-start
    // conflict.
    var startup = s.startup || {};
    var startupWrap = document.getElementById("startup-stage-wrap");
    var startupStageEl = document.getElementById("st-startup-stage");
    var startupErrorEl = document.getElementById("st-startup-error");
    if (startupWrap && startupStageEl && startupErrorEl) {
      if (s.status === "STARTING" || s.status === "RUNNING" || s.status === "PAUSED") {
        startupWrap.classList.remove("hidden");
        var stageZh = STARTUP_STAGE_ZH[String(startup.stage || "")] || String(startup.stage || "INIT");
        startupStageEl.textContent = stageZh;
        startupErrorEl.textContent = startup.last_error ? (" " + esc(String(startup.last_error))) : "";
      } else {
        startupWrap.classList.add("hidden");
      }
    }
    // Start button gating.
    var startBtn = document.getElementById("btn-search-start");
    if (startBtn) {
      var activeStates = ["STARTING", "RUNNING", "PAUSED", "STOPPING"];
      startBtn.disabled = activeStates.indexOf(s.status || "IDLE") >= 0;
    }
    var m = appState.targetMatch || {};
    els.stMatch.textContent = m.level || "none";
    els.stAnchor.textContent = (m.anchor_labels || []).length
      ? m.anchor_labels.join(", ") : (m.explicit_anchor_found ? "(found)" : "--");
    els.stAction.textContent = appState.robotAction || "IDLE";
    var obs = appState.observation || {};
    els.stPose.textContent = obs.pose ? "relative" : "--";
    els.scamCycle.textContent = "cycle " + (s.cycle || "--");
    // search light
    var status = s.status || "IDLE";
    var light = status === "TARGET_FOUND" ? "green"
      : (status === "RUNNING" || status === "STARTING") ? "yellow"
      : (status === "FAILED" || status === "OPERATOR_STOP") ? "red"
      : (status === "PAUSED") ? "yellow"
      : "gray";
    setLight(els.lightSearch, light);
    // robot light from status poll (set in pollStatus)
    // evidence
    var evidence = (appState.objects.target_evidence) || {};
    var html = "";
    html += "<div><span class='" + (evidence.target_confirmed ? "ok" : "no") + "'>" +
      (evidence.target_confirmed ? "✓ 目标已确认 TARGET CONFIRMED" : "✕ 目标尚未确认") + "</span></div>";
    html += "<div>目标匹配等级: <b>" + esc(m.level || "none") + "</b></div>";
    html += "<div>锚点: " + ((m.anchor_labels || []).map(function (a) { return esc(a); }).join(", ") || "未发现") + "</div>";
    html += "<div>目标分数: " + (m.target_score == null ? "--" : Number(m.target_score).toFixed(2)) + "</div>";
    (obs.objects || []).forEach(function (item) {
      var label = item.label_zh || item.label || item.name || "";
      if (label && (m.anchor_labels || []).some(function (a) { return String(label).indexOf(a) >= 0 || a.indexOf(label) >= 0; })) {
        html += "<div class='ok'>✓ 锚点物体: " + esc(label) + "</div>";
      }
    });
    html += "<div class='pend'>关系验证: " + (appState.verification ? esc(String(appState.verification.reason_zh || "")) : "pending") + "</div>";
    els.stEvidence.innerHTML = html;
  }

  function renderObservation() {
    var obs = appState.observation || {};
    var current = (obs.objects || []);
    var html = "";
    if (!current.length) {
      html = '<div class="muted">等待观察…</div>';
    } else {
      current.forEach(function (item) {
        var label = item.label_zh || item.label || item.name || "object";
        var conf = item.confidence == null ? "" : " (" + Number(item.confidence).toFixed(2) + ")";
        var pos = item.position_2d ? " " + esc(String(item.position_2d)) : "";
        html += '<div class="row"><span>' + esc(label) + conf + "</span><span class='cnt'>" +
          (item.mask_area_ratio ? Math.round(item.mask_area_ratio * 100) + "%" : "") +
          esc(pos) + "</span></div>";
      });
    }
    els.obsCurrent.innerHTML = html;
    var seen = (appState.objects.session_seen || []);
    if (!seen.length) {
      els.obsSeen.textContent = "--";
    } else {
      els.obsSeen.innerHTML = seen.map(function (item) {
        return '<div class="row"><span>' + esc(item.label) + "</span><span class='cnt'>" +
          item.observations + " 次</span></div>";
      }).join("");
    }
  }

  function renderDecision() {
    var goal = appState.selectedGoal;
    var command = appState.nextMotionCommand ||
      (appState.search || {}).next_motion_command ||
      ((appState.search || {}).last_decision || {}).next_motion_command;
    var instruction = command && (command.instruction_zh || command.instruction);
    els.decMotion.textContent = instruction || "等待决策…";
    renderDecisionHistory();
    if (!goal) {
      els.decIntent.textContent = "--";
      els.decReason.textContent = "";
      els.decScores.textContent = "";
      return;
    }
    var g = goal.goal || {};

    // Prefer the real structured reason_zh when available (plan §13.2).
    var lastDecision = (appState.search || {}).last_decision || null;
    var structuredReason = lastDecision && lastDecision.reason_zh;
    var structuredBreakdown = lastDecision && lastDecision.score_breakdown;
    var selectedIntent = lastDecision && lastDecision.selected_intent;

    els.decIntent.textContent = (selectedIntent && selectedIntent.intent_type) || g.goal_type || "--";
    var detail = goalDetailText(g);
    if (structuredReason) {
      els.decReason.textContent = structuredReason;
    } else {
      els.decReason.textContent = detail;
      var reasons = (goal.reasons || []).join("；");
      if (reasons) els.decReason.textContent += (detail ? " — " : "") + reasons;
    }
    // Score breakdown: prefer the structured decision breakdown, fall back to
    // the selected-goal components.
    var comps = structuredBreakdown || goal.components || {};
    var scoreRows = [
      ["semantic_relevance", "语义相关度"],
      ["spatial_gain", "空间信息增益"],
      ["psg_prior", "PSG 先验"],
      ["novelty", "新颖度"],
      ["continuity", "连续性"],
      ["route_cost_penalty", "路径代价"],
      ["visited_penalty", "已访问惩罚"],
      ["negative_evidence_penalty", "负证据惩罚"],
      ["score", "综合"],
    ];
    els.decScores.innerHTML = scoreRows.map(function (row) {
      var key = row[0];
      var value = comps[key];
      if (value === undefined || value === null) return "";
      return '<div class="score-row"><span>' + row[1] + "</span><b>" +
        Number(value).toFixed(2) + "</b></div>";
    }).join("");

    // Alternatives with real rejection reasons (plan §13.4).
    var alternatives = (lastDecision && lastDecision.alternatives) || [];
    var altEl = document.getElementById("dec-alternatives");
    if (altEl) {
      if (!alternatives.length) {
        altEl.textContent = "无其他候选";
      } else {
        altEl.innerHTML = alternatives.map(function (alt) {
          return '<div class="alt-row"><b>' + esc(alt.candidate_id || "候选") +
            "</b> " + (alt.score == null ? "" : " " + Number(alt.score).toFixed(2)) +
            '<br><span class="muted">' + esc(alt.rejected_reason_zh || "") + "</span></div>";
        }).join("");
      }
    }
  }

  function renderTaskUnderstanding() {
    var task = appState.task || (appState.search || {}).task || {};
    if (!els.taskUnderstanding) return;
    if (!task || !Object.keys(task).length) {
      els.taskUnderstanding.textContent = "等待任务解析…";
      return;
    }
    var target = task.canonical_target || task.raw_text || "--";
    var intent = task.intent || "--";
    var attrs = task.target_attributes || {};
    var relations = task.target_relations || [];
    var constraints = task.constraints || [];
    var status = task.executable === false ? "不可执行" : "可执行";
    var html = "<div><b>规范目标：</b>" + esc(target) + "</div>";
    html += "<div><b>意图：</b>" + esc(intent) + " · " + esc(status) + "</div>";
    if (Object.keys(attrs).length) html += "<div><b>属性：</b>" + esc(pretty(attrs)) + "</div>";
    if (relations.length) html += "<div><b>关系：</b>" + esc(pretty(relations)) + "</div>";
    if (constraints.length) html += "<div><b>约束：</b>" + esc(pretty(constraints)) + "</div>";
    if (task.rejection_reason) html += "<div class='no'><b>拒绝原因：</b>" + esc(task.rejection_reason) + "</div>";
    els.taskUnderstanding.innerHTML = html;
  }

  function renderDecisionHistory() {
    if (!els.decHistory) return;
    var decisions = (appState.decisions || []).slice(-12).reverse();
    if (!decisions.length) {
      els.decHistory.innerHTML = '<div class="muted">暂无决策记录</div>';
      return;
    }
    els.decHistory.innerHTML = decisions.map(function (item) {
      var command = item.next_motion_command || {};
      var text = command.instruction_zh || command.instruction || item.decision_id || "决策";
      var result = item.execution_status ? " · " + item.execution_status : "";
      return '<div class="decision-history-row"><span>' + esc(text) +
        '</span><span class="cnt">' + esc(String(item.decision_id || "")) +
        esc(result) + '</span></div>';
    }).join("");
  }

  function goalDetailText(g) {
    if (g.semantic_reason) return g.semantic_reason;
    switch (g.goal_type) {
      case "ROTATE_VIEW": {
        var dyaw = g.relative_dyaw;
        if (dyaw === undefined || dyaw === null) return "旋转观察";
        return "旋转观察 " + Math.abs(Number(dyaw)).toFixed(0) + "° " + (dyaw > 0 ? "右侧" : "左侧");
      }
      case "RELATIVE_MOVE": {
        var dx = g.relative_dx;
        return "前进 " + (dx == null ? "--" : Number(dx).toFixed(2)) + " m";
      }
      case "INSPECT_ANCHOR":
        return "检查锚点" + (g.semantic_anchor ? "「" + g.semantic_anchor + "」" : "");
      case "REVISIT_NODE":
        return "重访节点 " + (g.target_node_id || "");
      case "REOBSERVE":
        return "重新观察当前视角";
      case "STOP":
        return "停止";
      default:
        return g.goal_type || "";
    }
  }

  function renderCandidates() {
    var list = appState.candidates || [];
    if (!list.length) {
      els.decCandidates.textContent = "";
      return;
    }
    var s = appState.selectedGoal || {};
    var selectedId = (s.goal || {}).goal_id;
    els.decCandidates.innerHTML = list.map(function (c) {
      var g = c.goal || {};
      var sel = selectedId && g.goal_id === selectedId;
      var score = c.score == null ? "" : " 总分 " + Number(c.score).toFixed(2);
      var label = goalDetailText(g) || g.goal_type;
      return '<div class="cand' + (sel ? " selected" : "") + '">' +
        esc(g.goal_type) + " · " + esc(label) +
        "<span class='cscore'>" + score + (c.selected ? " ★" : "") + "</span></div>";
    }).join("");
  }

  function pushTimeline(event) {
    appState.events.push({
      event_type: event.event_type,
      timestamp: event.timestamp,
      cycle: event.cycle,
    });
    if (appState.events.length > 400) appState.events = appState.events.slice(-400);
  }

  function renderTimeline() {
    var items = appState.events.slice(-120).reverse();
    if (!items.length) {
      els.timeline.innerHTML = '<div class="muted">等待事件…</div>';
      return;
    }
    els.timeline.innerHTML = items.map(function (item) {
      var t = new Date(item.timestamp * 1000).toTimeString().slice(0, 8);
      var meta = [];
      if (item.cycle !== undefined && item.cycle !== null) meta.push("c" + item.cycle);
      return '<div class="tline"><span class="ttype">' + esc(item.event_type) +
        '</span><span class="tmeta">' + t + (meta.length ? " · " + meta.join(",") : "") + "</span></div>";
    }).join("");
  }

  function renderButtons() {
    var status = (appState.search || {}).status || "IDLE";
    var running = status === "RUNNING" || status === "STARTING";
    var paused = status === "PAUSED";
    var finished = ["TARGET_FOUND", "SEARCH_EXHAUSTED", "OPERATOR_STOP", "FINISHED", "FAILED"].indexOf(status) >= 0;
    els.btnStart.disabled = running || paused;
    els.btnPause.disabled = !running;
    els.btnResume.disabled = !paused;
    els.btnStop.disabled = !(running || paused);
    els.btnEstop.disabled = false;
    if (els.activeControls) {
      els.activeControls.classList.toggle(
        "hidden", !(running || paused || status === "STOPPING")
      );
    }
    if (els.btnHistoryCurrent) {
      els.btnHistoryCurrent.classList.toggle("hidden", !viewingHistoryId);
    }
    els.sessionInfo.textContent = viewingHistoryId
      ? "正在回看 " + viewingHistoryId
      : ((appState.search && appState.search.session_id)
        ? appState.search.session_id : "输入自然语言目标即可开始");
  }

  function renderErrorDetail() {
    if (!els.errorDetail) return;
    var error = (appState.search || {}).error || null;
    if (!error || !(error.message || error.code || error.error_type)) {
      els.errorDetail.classList.add("hidden");
      els.errorDetail.innerHTML = "";
      return;
    }
    var rows = [
      ["错误代码", error.code || error.error_type || "SEARCH_ERROR", true],
      ["直接原因", error.message || "--"],
      ["原因分类", error.cause || error.category || "--"],
      ["发生位置", [error.source, error.stage].filter(Boolean).join(" / ") || "--"],
      ["处理建议", error.suggestion || "查看本次会话事件和 worker 日志后重试。"],
      ["相关日志", error.log_ref || "outputs/autonomous_search/logs/search_worker.log", true]
    ];
    els.errorDetail.innerHTML = "<h3>" + esc(error.title || "搜索错误详情") +
      "</h3><div class='error-grid'>" + rows.map(function (row) {
        var value = row[2] ? "<code>" + esc(row[1]) + "</code>" : esc(row[1]);
        return "<div class='error-key'>" + esc(row[0]) + "</div><div>" + value + "</div>";
      }).join("") + "</div>";
    els.errorDetail.classList.remove("hidden");
  }

  function renderDebug() {
    if (!els.debugPanel.classList.contains("hidden")) {
      var s = appState.search || {};
      els.debugGoalGraph.textContent = pretty(s.goal_graph);
      els.debugSceneGraph.textContent = pretty({
        objects: (appState.observation || {}).objects || [],
        relations: (appState.observation || {}).relations || [],
      });
      els.debugCandidates.textContent = pretty(appState.candidates);
      els.debugRaw.textContent = pretty(appState);
    }
  }

  function drawOverlay() {
    var canvas = els.overlay;
    var ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var detections = lastDetectionFrame;
    if (!detections || !detections.length) return;
    detections.forEach(function (item) {
      var bbox = item.bbox_2d || item.bbox;
      if (!Array.isArray(bbox) || bbox.length < 4) return;
      var x1 = bbox[0] * canvas.width;
      var y1 = bbox[1] * canvas.height;
      var x2 = bbox[2] * canvas.width;
      var y2 = bbox[3] * canvas.height;
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      var label = String(item.label || item.name || "object");
      var score = item.score == null ? "" : " " + Number(item.score).toFixed(2);
      ctx.fillStyle = "rgba(56, 189, 248, 0.9)";
      ctx.font = "bold 13px sans-serif";
      var text = label + score;
      var tw = ctx.measureText(text).width;
      ctx.fillRect(x1, Math.max(0, y1 - 18), tw + 8, 18);
      ctx.fillStyle = "#06121b";
      ctx.fillText(text, x1 + 4, Math.max(13, y1 - 5));
    });
  }

  function showBanner(text, kind) {
    els.banner.textContent = text;
    els.banner.className = "search-banner " + kind;
  }

  function setLight(el, cls) {
    if (el) el.className = "light " + cls;
  }

  function pretty(value) {
    try {
      return JSON.stringify(value, null, 2) || "null";
    } catch (e) {
      return String(value);
    }
  }

  function esc(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function fmtDuration(seconds) {
    if (seconds === undefined || seconds === null) return "00:00";
    var s = Math.max(0, Math.floor(seconds));
    var m = Math.floor(s / 60);
    var h = Math.floor(m / 60);
    m = m % 60;
    s = s % 60;
    return (h > 0 ? String(h).padStart(2, "0") + ":" : "") +
      String(m).padStart(2, "0") + ":" + String(s).padStart(2, "0");
  }

  // ------------------------------------------------------------------ //
  // Tabs                                                                //
  // ------------------------------------------------------------------ //
  document.querySelectorAll("#tabs .tab").forEach(function (btn) {
    btn.addEventListener("click", function () {
      document.querySelectorAll("#tabs .tab").forEach(function (b) { b.classList.remove("active"); });
      btn.classList.add("active");
      var tab = btn.dataset.tab;
      document.querySelectorAll(".tab-pane").forEach(function (pane) { pane.classList.remove("active"); });
      var pane = document.getElementById("tab-" + tab);
      if (pane) pane.classList.add("active");
    });
  });

  // Debug toggle
  els.chkDebug.addEventListener("change", function () {
    els.debugPanel.classList.toggle("hidden", !els.chkDebug.checked);
  });
  document.querySelectorAll(".dtab").forEach(function (btn) {
    btn.addEventListener("click", function () {
      document.querySelectorAll(".dtab").forEach(function (b) { b.classList.remove("active"); });
      btn.classList.add("active");
      document.querySelectorAll(".debug-pre").forEach(function (pre) { pre.classList.add("hidden"); });
      var target = document.getElementById("debug-" + btn.dataset.dtab);
      if (target) target.classList.remove("hidden");
    });
  });

  // Map node detail dismiss
  document.getElementById("search-map").addEventListener("click", function () {
    els.mapNodeDetail.classList.add("hidden");
  });

  // ------------------------------------------------------------------ //
  // Camera FPS                                                          //
  // ------------------------------------------------------------------ //
  var lastFrameAt = null;
  var fps = 0;
  els.cam.addEventListener("load", function () {
    var now = performance.now();
    if (lastFrameAt !== null) {
      var dt = (now - lastFrameAt) / 1000;
      if (dt > 0.01) fps = 1 / dt;
    }
    lastFrameAt = now;
    els.scamFps.textContent = Math.round(fps) + " FPS";
  });

  // ------------------------------------------------------------------ //
  // Polling: /api/status (lights) + /api/search/state (reconcile)       //
  // ------------------------------------------------------------------ //
  function pollStatus() {
    fetch("/api/status").then(function (response) { return response.json(); })
      .then(function (status) {
        var camera = status.camera || {};
        els.scamAge.textContent = "帧年龄 " + fmtAge(camera.age_seconds);
        els.camStale.classList.toggle("hidden", camera.fresh !== false || !camera.available);
        var motion = status.motion || {};
        setLight(els.lightRobot, motion.available ? (motion.state === "ESTOP" ? "red" : "green") : "gray");
        if (status.owner) {
          renderOwner(status.owner);
          if (status.owner.owner === "AUTONOMOUS" && !(appState.search || {}).session_id) {
            showBanner("自主搜索正在后端运行，页面刷新后已恢复会话", "exhausted");
          }
        }
        renderSystem(status);
      })
      .catch(function () {});
  }

  function pollSearchState() {
    fetch("/api/search/state").then(function (response) { return response.json(); })
      .then(function (state) {
        if (state && state.session_id) currentLiveState = state;
        if (viewingHistoryId) return;
        if (state.session_id) {
          if (!appState.search.session_id || appState.search.session_id === state.session_id) {
            var fresh = appState.search.session_id !== state.session_id;
            appState.search = {};
            appState.search.session_id = state.session_id;
            applyStateSnapshot(state);
            if (fresh) mapRenderer.render(appState.map, appState.spatial);
          }
        } else if (appState.search.session_id && !ws) {
          appState.search = {};
          renderAll();
        }
      })
      .catch(function () {});
  }

  function pollHistory() {
    fetch("/api/search/history?limit=10").then(function (response) { return response.json(); })
      .then(function (data) { renderHistory(data.sessions || []); })
      .catch(function () {
        if (els.history) els.history.innerHTML = '<div class="muted">历史记录暂时不可用</div>';
      });
  }

  function renderHistory(sessions) {
    if (!els.history) return;
    if (!sessions.length) {
      els.history.innerHTML = '<div class="muted">完成首次搜索后会在这里保留完整记录</div>';
      return;
    }
    els.history.innerHTML = sessions.map(function (item) {
      var when = item.created_at || item.updated_at;
      var timeText = when ? new Date(when * 1000).toLocaleString() : "";
      var active = viewingHistoryId === item.session_id ? " active" : "";
      return '<button type="button" class="history-item' + active + '" data-session-id="' +
        esc(item.session_id) + '"><span class="history-result">' +
        esc(item.result || item.status || "--") + '</span><div class="history-target">' +
        esc(item.task_text || item.target || "未命名任务") + '</div><div class="history-meta">' +
        esc(timeText) + ' · ' + esc(item.session_id) + '</div></button>';
    }).join("");
  }

  function loadHistorySession(sessionId) {
    fetch("/api/search/history/" + encodeURIComponent(sessionId))
      .then(function (response) { return response.json(); })
      .then(function (record) {
        if (!record.ok || !record.state) {
          showErrorRetry(record.error || "历史记录读取失败", record.error_detail);
          return;
        }
        viewingHistoryId = sessionId;
        applyStateSnapshot(record.state);
        appState.events = (record.events || []).map(function (event) {
          return { event_type: event.event_type, timestamp: event.timestamp, cycle: event.cycle };
        });
        if (els.target) {
          els.target.value = ((record.session || {}).task_text) || record.state.target || "";
        }
        showBanner("正在回看完整搜索记录；实时任务仍在后端保持运行。", "exhausted");
        renderAll();
        pollHistory();
      }).catch(function (err) {
        showErrorRetry(String(err || "history request failed"));
      });
  }

  if (els.history) {
    els.history.addEventListener("click", function (event) {
      var button = event.target.closest ? event.target.closest("[data-session-id]") : null;
      if (button) loadHistorySession(button.getAttribute("data-session-id"));
    });
  }
  if (els.btnHistoryCurrent) {
    els.btnHistoryCurrent.addEventListener("click", function () {
      viewingHistoryId = null;
      if (currentLiveState && currentLiveState.session_id) {
        applyStateSnapshot(currentLiveState);
      } else {
        pollSearchState();
      }
      showBanner("已返回当前搜索状态", "");
      pollHistory();
    });
  }

  function renderOwner(owner) {
    var el = document.getElementById("sys-owner");
    if (el) el.textContent = pretty(owner);
  }

  function renderSystem(status) {
    els.sysCamera.textContent = pretty(status.camera);
    els.sysWorker.textContent = pretty(status.worker);
    els.sysMotion.textContent = pretty(status.motion);
    els.sysSearch.textContent = pretty(status.search);
    els.sysLlm.textContent = pretty({
      enabled: status.llm && status.llm.enabled,
      analysis_status: status.llm && status.llm.analysis && status.llm.analysis.status,
      model: status.llm && status.llm.analysis && status.llm.analysis.model,
      error: status.llm && status.llm.analysis && status.llm.analysis.error,
    });
    fetch("/api/search/readiness").then(function (r) { return r.json(); })
      .then(function (data) { els.sysReadiness.textContent = pretty(data); })
      .catch(function () {});
    fetch("/api/search/history").then(function (r) { return r.json(); })
      .then(function (data) { els.sysHistory.textContent = pretty(data.sessions || []); })
      .catch(function () {});
  }

  function fmtAge(seconds) {
    if (seconds === null || seconds === undefined) return "--";
    return seconds.toFixed(1) + "s";
  }

  // ------------------------------------------------------------------ //
  // init                                                                //
  // ------------------------------------------------------------------ //
  pollStatus();
  pollSearchState();
  pollHistory();
  setInterval(pollStatus, 1000);
  setInterval(pollSearchState, 3000);
  setInterval(pollHistory, 5000);
  connect();
})();
