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
    semantic: {},
    objects: {},
    targetMatch: {},
    selectedGoal: null,
    task: {},
    decisions: [],
    nextMotionCommand: null,
    turnProgress: null,
    candidates: [],
    spatial: {},
    // 唯一一张图（state.graph），由服务端 web_snapshot() 投影。
    graph: null,
    events: [],
    health: {},
  };

  var STARTUP_STAGE_ZH = {
    "IDLE": "空闲",
    "SPAWN_WORKER": "正在启动搜索进程",
    "WORKER_READY": "工作进程就绪",
    "LOAD_PIPELINE": "正在初始化 SemanticNavigation",
    "WAIT_RGBD": "正在等待 D435 RGB-D",
    "WAIT_SPATIAL_PROVIDER": "正在连接地图与定位",
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
  var mappingMode = false;
  var mappingState = null;
  var mappingSoundEnabled = false;
  var mappingAudio = null;
  var mappingLiveCamera = false;
  var lastSoundByEvent = {};
  // 回放面板状态：会话 id、当前选中倍速、轮询定时器、最近一次状态。
  var replaySessionId = null;
  var replaySpeed = null;
  var replayPollTimer = null;
  var replayLastStatus = null;

  var els = {
    // taskbar
    target: document.getElementById("search-target"),
    btnStart: document.getElementById("btn-search-start"),
    btnPause: document.getElementById("btn-search-pause"),
    btnResume: document.getElementById("btn-search-resume"),
    btnStop: document.getElementById("btn-search-stop"),
    btnEstop: document.getElementById("btn-search-estop"),
    chkDebug: document.getElementById("chk-debug"),
    detector: document.getElementById("search-detector"),
    chkMotion: document.getElementById("chk-motion"),
    sessionInfo: document.getElementById("search-session-info"),
    banner: document.getElementById("search-banner"),
    errorDetail: document.getElementById("search-error-detail"),
    activeControls: document.getElementById("search-active-controls"),
    btnHistoryCurrent: document.getElementById("btn-history-current"),
    history: document.getElementById("search-history"),
    // replay panel (history detail)
    replayPanel: document.getElementById("replay-panel"),
    replayStatus: document.getElementById("replay-status"),
    replayProgress: document.getElementById("replay-progress"),
    replayVideo: document.getElementById("replay-video"),
    replayPlaceholder: document.getElementById("replay-placeholder"),
    replayReasons: document.getElementById("replay-reasons"),
    replayDownload: document.getElementById("replay-download"),
    replayFinalMap: document.getElementById("replay-final-map"),
    replayCover: document.getElementById("replay-cover"),
    replayRegenerate: document.getElementById("replay-regenerate"),
    replayClose: document.getElementById("replay-close"),
    replaySpeeds: document.getElementById("replay-speeds"),
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
    stSemantic: document.getElementById("st-semantic"),
    stSemanticDetail: document.getElementById("st-semantic-detail"),
    stEvidence: document.getElementById("st-evidence"),
    stEmbodiment: document.getElementById("st-embodiment"),
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
    // task agent panel (plan §5.1) — explanation layer, never replaces the
    // candidate / map / safety areas below it.
    agentMode: document.getElementById("agent-mode"),
    agentSafety: document.getElementById("agent-safety"),
    agentPlan: document.getElementById("agent-plan"),
    agentChoice: document.getElementById("agent-choice"),
    agentDownstream: document.getElementById("agent-downstream"),
    agentCandidates: document.getElementById("agent-candidates"),
    agentKnowledge: document.getElementById("agent-knowledge"),
    agentInputs: document.getElementById("agent-inputs"),
    agentUncertainties: document.getElementById("agent-uncertainties"),
    // map
    mapMeta: document.getElementById("map-meta"),
    mapNodeDetail: document.getElementById("map-node-detail"),
    topologyViewToggle: document.getElementById("topology-view-toggle"),
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
    } else if (type === "event") {
      if (!viewingHistoryId) applySearchEvent(message.event);
    } else if (type === "heartbeat") {
      /* keepalive only */
    }
  }

  // ------------------------------------------------------------------ //
  // State application                                                   //
  // ------------------------------------------------------------------ //
  // 计划书 §2.1：图只有一个来源 —— `GET /api/search/state` 的 `state.graph`。
  // 客户端不再本地拼第二张图、不再做“保留上一次非空图”的补丁（保留由服务端
  // 的 store 负责）。`MAP_UPDATED` 只带 revision，收到后拉一次 state。
  var graphPullPending = false;
  function pullGraphState() {
    if (mappingMode) return;
    if (graphPullPending) return;
    graphPullPending = true;
    fetch("/api/search/state", { method: "GET", headers: { "Accept": "application/json" } })
      .then(function (r) { return r.json(); })
      .then(function (state) {
        if (!state || !state.session_id) return;
        currentLiveState = state;
        if (viewingHistoryId) return;
        applyStateSnapshot(state);
      })
      .catch(function () { /* 后端暂时不可达时忽略，保持上次画面 */ })
      .then(function () { graphPullPending = false; });
  }

  // 兜底轮询：每 2.5s 拉一次后端 state，保证拓扑图/物体列表“从第一次识别就开始
  // 实时更新”，即使单个 WS 事件偶尔掉线也不会漏。
  var statePollTimer = null;
  function startStatePolling() {
    if (statePollTimer) return;
    statePollTimer = window.setInterval(function () {
      if (mappingMode) return;
      fetch("/api/search/state", { method: "GET", headers: { "Accept": "application/json" } })
        .then(function (r) { return r.json(); })
        .then(function (state) {
          if (!state || !state.session_id) return;
          currentLiveState = state;
          if (viewingHistoryId) return;
          if (!appState.search.session_id) appState.search.session_id = state.session_id;
          appState.spatial = state.spatial || {};
          appState.graph = state.graph || null;
          if (state.embodiment) appState.embodiment = state.embodiment;
          // 兜底轮询也要带上 Agent trace，否则 WS 掉线时面板会停在旧结论上。
          if (state.agent) appState.search.agent = state.agent;
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
    // A poll started before STOP can arrive after the request event. Retain
    // the intent for this session while still accepting all new telemetry.
    if (state.session_id === appState.search.session_id &&
        ["TARGET_FOUND", "SEARCH_EXHAUSTED", "OPERATOR_STOP", "FINISHED",
          "FAILED", "TASK_REJECTED"].indexOf(state.status) < 0) {
      restoreStopLifecycle(state, captureStopLifecycle(appState.search));
    }
    if (state.status === "FINISHED" && !state.error &&
        els.banner && els.banner.classList.contains("error")) {
      els.banner.textContent = "";
      els.banner.className = "search-banner hidden";
    }
    appState.search = state;
    appState.observation = state.observation || {};
    appState.objects = state.objects || {};
    appState.targetMatch = state.target_match || state.targetMatch || {};
    appState.selectedGoal = state.selected_goal || null;
    appState.task = state.task || {};
    appState.decisions = state.decisions || [];
    appState.nextMotionCommand = state.next_motion_command || null;
    appState.candidates = state.candidates || [];
    appState.embodiment = state.embodiment || {};
    appState.spatial = state.spatial || {};
    appState.graph = state.graph || null;
    state.timeline = state.timeline || [];
    appState.events = state.timeline.slice();
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
    var terminalEvent = ["OPERATOR_STOP", "SEARCH_FINISHED", "SEARCH_EXHAUSTED",
      "ERROR", "TASK_REJECTED"].indexOf(type) >= 0;
    var stopLifecycle = terminalEvent ? null : captureStopLifecycle(appState.search);
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
        appState.graph = null;
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
        if (payload.embodiment) mergeEmbodiment(payload.embodiment);
        break;
      case "OBSERVATION_UPDATED":
        appState.observation = {
          bundle_id: payload.bundle_id,
          timestamp: payload.timestamp,
          objects: payload.scene_objects || payload.objects || [],
          detections: payload.detections || [],
          target_present: payload.target_present,
          target_state: payload.target_state || (payload.target_present ? "PRESENT" : "ABSENT"),
          heading_sector: payload.heading_sector,
          navigation_heading_sector: payload.navigation_heading_sector,
          semantic_status: payload.semantic_status,
          semantic_quality: payload.semantic_quality,
          semantic_source_frame_id: payload.semantic_source_frame_id,
          semantic_age_ms: payload.semantic_age_ms,
          spatial_quality: payload.spatial_quality,
          pose: payload.pose,
          sensor_health: payload.sensor_health || {},
        };
        lastDetectionFrame = payload.detections || null;
        break;
      case "SEMANTIC_STATUS":
        // 计划书 §13：语义健康状态（Quick VLM / Full Semantic / frame / age）。
        appState.semantic = payload || {};
        break;
      case "OBJECTS_UPDATED":
        appState.objects.current = payload.current || [];
        appState.objects.target_evidence = payload.target_evidence || {};
        break;
      case "TARGET_MATCH_UPDATED":
        appState.targetMatch = {
          level: payload.target_match_level || "none",
          target_state: payload.target_state || "ABSENT",
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
        if (!stopLifecycle) showBanner("✓ TARGET FOUND", "found");
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
        appState.turnProgress = null;
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
        appState.robotAction = payload.dry_run && payload.path_checked &&
          payload.candidate_decision && payload.candidate_decision.accepted ? "PLANNED" :
          (payload.status === "succeeded" ? "SUCCEEDED" : "FAILED");
        appState.search.phase = "WAIT_RESULT";
        appState.search.phase_detail = payload.phase_detail || payload.message || "动作已结束";
        appState.turnProgress = turnProgressText(payload) || null;
        if (payload.status !== "succeeded" && /MOTION_(ACCEPT|RESULT)_TIMEOUT/.test(payload.message || "")) {
          showBanner("动作响应超时，已取消并安全停止；搜索将重新规划。原因：" + payload.message, "error");
        }
        if (payload.status !== "succeeded" && /ROTATION_NOT_CONFIRMED/.test(payload.message || "")) {
          showBanner("转向未被里程计确认，逻辑目标判定失败：" + payload.message, "error");
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
      case "STOP_REQUESTED":
        if (["STARTING", "RUNNING", "PAUSED", "STOPPING"].indexOf(
            appState.search.status) >= 0) {
          if (appState.search.stop_requested_at == null) {
            appState.search.stop_requested_at = event.timestamp;
            appState.search.phase_started_at = event.timestamp;
          }
          appState.search.status = "STOPPING";
          appState.search.phase = "STOPPING";
          appState.search.phase_detail = "正在停止任务，等待执行端确认";
        }
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
        // 计划书 §2.1：MAP_UPDATED 只带 revision，不携带图；拉一次 state 拿新图。
        pullGraphState();
        break;
      case "RGBD_FRAME_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.rgbd_frame = payload;
        break;
      case "SPATIAL_POSE_UPDATED":
        appState.spatial = appState.spatial || {};
        appState.spatial.spatial_pose = payload.pose || null;
        break;
      // SPATIAL_MAP_UPDATED / FRONTIERS_UPDATED / PLACE_CREATED / PLACE_UPDATED /
      // SEMANTIC_OBJECT_LOCALIZED 曾各自在客户端拼一份图；计划书 §2.2 起它们只
      // 记录时间线，图统一由 state.graph 提供，这里不再有分支。
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
          if (payload.cause) reason += " · cause=" + payload.cause;
          if (payload.attempts != null) reason += " · attempts=" + payload.attempts;
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
    if (stopLifecycle) {
      restoreStopLifecycle(appState.search, stopLifecycle);
    } else if (terminalEvent && appState.search.stop_requested_at != null) {
      appState.search.phase = appState.search.status;
      appState.search.phase_detail = "";
      appState.search.phase_started_at = event.timestamp;
    }
    pushTimeline(event);
    renderAll();
  }

  function captureStopLifecycle(state) {
    if (state.status !== "STOPPING" && state.stop_requested_at == null) return null;
    var value = {};
    ["status", "phase", "phase_detail", "phase_started_at", "result",
      "finish_reason", "finished_at", "summary", "stop_requested_at"].forEach(function (key) {
      if (Object.prototype.hasOwnProperty.call(state, key)) value[key] = state[key];
    });
    return value;
  }

  function restoreStopLifecycle(state, value) {
    if (!value) return;
    ["status", "phase", "phase_detail", "phase_started_at", "result",
      "finish_reason", "finished_at", "summary", "stop_requested_at"].forEach(function (key) {
      if (Object.prototype.hasOwnProperty.call(value, key)) state[key] = value[key];
      else delete state[key];
    });
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
      detector: (els.detector && els.detector.value) || "grounded_sam",
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

  // 语义拓扑页的投影视图：物体拓扑（默认）/ 导航拓扑。
  // 计划1 §7.3 要"只有物体"，计划3 步骤 2.1 要保留 PLACE 中转站才连得成一张图，
  // 计划2 §11.2 给过出路（"或至少提供 toggle"）——这里就是那个 toggle。
  if (els.topologyViewToggle) {
    els.topologyViewToggle.addEventListener("click", function (event) {
      var btn = event.target.closest ? event.target.closest("button") : null;
      if (!btn || !btn.dataset || !btn.dataset.topologyView) return;
      var view = btn.dataset.topologyView === "navigation" ? "navigation" : "objects";
      Array.prototype.forEach.call(
        els.topologyViewToggle.querySelectorAll(".vtab"), function (b) {
          b.classList.toggle("active", b.dataset.topologyView === view);
        });
      mapRenderer.setTopologyView(view);
      renderAll();
    });
  }

  // ------------------------------------------------------------------ //
  // Rendering                                                           //
  // ------------------------------------------------------------------ //
  // 计划书 §7.3：物体列表、计数和拓扑图共用同一个投影，
  // 保证界面上的节点数就是画出来的节点数，绝不退回导航图。
  // 投影跟着当前视图走：物体拓扑下 P1 不计入"N 个物体"。
  function objectTopology(graph) {
    var view = mapRenderer && mapRenderer.getTopologyView
      ? mapRenderer.getTopologyView() : "objects";
    var raw = graph
      ? (view === "navigation"
        ? (graph.integrated_topology || graph.object_topology || null)
        : (graph.object_topology || null))
      : null;
    if (window.TopologyLayout) {
      var project = view === "navigation"
        ? window.TopologyLayout.objectOnly
        : window.TopologyLayout.objectStrict;
      if (project) return project(raw);
    }
    return raw;
  }

  function renderObjects() {
    // 识别物体列表：与语义拓扑同源（state.graph.object_topology），
    // 从第一次识别到物体就开始显示，并随每次 memory_update 实时刷新。
    var ot = objectTopology(appState.graph);
    var nodes = ot && Array.isArray(ot.nodes) ? ot.nodes : [];
    if (els.obsObjectsMeta) {
      els.obsObjectsMeta.textContent = nodes.length ? nodes.length + " 个" : "";
    }
    if (!els.obsObjects) return;
    if (mappingMode) {
      var detections = (mappingState && mappingState.current_detections) || [];
      var currentEl = document.getElementById("obs-current");
      if (currentEl) currentEl.innerHTML = detections.length ? detections.map(function (d) {
        var cls = d.mobility_class === "fixed" ? "固定地标" :
          (d.mobility_class === "semi_static" ? "暂时静止" :
            (d.mobility_class === "dynamic" ? "动态" : "待判定"));
        return '<div class="obj-item' + (d.is_hazard ? ' mapping-hazard' : '') + '">' +
          esc(d.label_zh || d.label || "物体") + ' · ' + cls +
          (d.is_hazard ? ' ⚠ ' + esc(d.hazard_label || "危险") : '') +
          (d.hazard_position_status ? ' · ' + esc(d.hazard_position_status) :
            (d.has_position ? '' : ' · 位置待确认')) + '</div>';
      }).join("") : '<div class="empty">当前帧未检出物体</div>';
    }
    if (!nodes.length) {
      els.obsObjects.innerHTML =
        '<div class="empty">等待首次识别…（识别到第一个物体后自动开始建物体列表 / 拓扑图）</div>';
      return;
    }
    var statusColor = { CONFIRMED: "#34d399", TENTATIVE: "#38bdf8", STALE: "#8b95a3" };
    var html = nodes.slice().sort(function (a, b) {
      return String(a.node_id).localeCompare(String(b.node_id));
    }).map(function (n) {
      var color = n.is_hazard ? "#f87171" : (n.mobility_class === "fixed" ? "#38bdf8" :
        (n.mobility_class === "semi_static" ? "#fb923c" : (statusColor[n.status] || "#8b95a3")));
      var star = n.is_target_confirmed ? "★ " : (n.is_target_candidate ? "◎ " : "");
      var meta = (n.mobility_class === "fixed" ? "固定地标" :
        (n.mobility_class === "semi_static" ? "暂时静止" : "待复核"));
      if (n.is_hazard) meta += " ⚠ 危险";
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
    renderEmbodiment();
    renderObservation();
    renderObjects();
    renderAgentPanel();
    renderDecision();
    renderCandidates();
    renderTimeline();
    renderButtons();
    renderDebug();
    renderErrorDetail();
    // Semantic topology projection: show only topology stats, never metric
    // map / place metrics. 计量"空间地图"视图已于 2026-08-26 移除，
    // 2026-09-15 起连入口按钮和模式开关也一并删掉，这里不再分支。
    var topology = objectTopology(appState.graph);
    var nodes = (topology && topology.nodes) || [];
    var edges = (topology && topology.edges) || [];
    var empty = (nodes.length === 0);
    els.mapMeta.textContent = empty
      ? (appState.graph ? "已建图 0 节点 · 等待识别到物体" : "等待首次物体识别…识别后自动开始建拓扑")
      : "拓扑节点 " + nodes.length + " · 关系 " + edges.length +
        " · rev " + ((topology && topology.revision) || 0);
    mapRenderer.render(appState.graph, appState.spatial || null);
    drawOverlay();
  }

  function renderStatus() {
    var s = appState.search || {};
    els.stTarget.textContent = s.target || "--";
    var phaseText = s.phase || (s.status || "IDLE");
    if (appState.turnProgress) phaseText += " · " + appState.turnProgress;
    if (s.phase_detail) phaseText += " · " + s.phase_detail;
    els.stPhase.textContent = phaseText;
    var detectorEl = document.getElementById("st-detector");
    if (detectorEl) detectorEl.textContent = s.detector_backend || "--";
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
    if (els.detector) {
      els.detector.disabled = ["STARTING", "RUNNING", "PAUSED", "STOPPING"].indexOf(s.status || "IDLE") >= 0;
      if (s.detector_backend && !els.detector.disabled) els.detector.value = s.detector_backend;
    }
    var m = appState.targetMatch || {};
    els.stMatch.textContent = m.target_state && m.target_state !== "ABSENT"
      ? m.target_state : (m.level || "none");
    els.stAnchor.textContent = (m.anchor_labels || []).length
      ? m.anchor_labels.join(", ") : (m.explicit_anchor_found ? "(found)" : "--");
    els.stAction.textContent = appState.robotAction === "PLANNED" ? "规划通过（只读）" : (appState.robotAction || "IDLE");
    var obs = appState.observation || {};
    els.stPose.textContent = obs.pose ? "relative" : "--";
    // 计划书 §13：语义健康状态（Quick VLM / Full Semantic / frame / age）。
    var sem = appState.semantic || obs || {};
    var semStatus = sem.semantic_status || "--";
    var semText = semStatus;
    if (semStatus === "fresh_full") semText = "Full 新鲜";
    else if (semStatus === "fresh_quick_scene") semText = "轻量场景";
    else if (semStatus === "pending") semText = "Full 后台运行中";
    else if (semStatus === "stale") semText = "旧帧兜底";
    else if (semStatus === "timeout") semText = "Full 超时";
    else if (semStatus === "error") semText = "Full 错误";
    else if (semStatus === "unavailable") semText = "语义不可用";
    if (els.stSemantic) els.stSemantic.textContent = semText;
    if (els.stSemanticDetail) {
      var parts = [];
      var sframe = sem.semantic_source_frame_id || sem.frame_id;
      if (sframe) parts.push("source=" + sframe);
      if (sem.semantic_age_ms != null) parts.push("age=" + Math.round(sem.semantic_age_ms) + "ms");
      if (sem.semantic_object_count != null) parts.push("objects=" + sem.semantic_object_count);
      if (sem.spatial_quality) parts.push("spatial=" + sem.spatial_quality);
      if (sem.navigation_heading_sector != null) parts.push("nav_sector=" + sem.navigation_heading_sector);
      if (semStatus === "timeout" || semStatus === "error" || semStatus === "unavailable") {
        els.stSemanticDetail.className = "semantic-detail no";
        els.stSemanticDetail.textContent = "Full Semantic unavailable · " +
          (parts.join(" · ") || "使用轻量场景物体");
      } else if (semStatus === "fresh_quick_scene") {
        els.stSemanticDetail.className = "semantic-detail";
        els.stSemanticDetail.textContent = "Using lightweight scene objects" +
          (parts.length ? " · " + parts.join(" · ") : "");
      } else {
        els.stSemanticDetail.className = "semantic-detail";
        els.stSemanticDetail.textContent = parts.join(" · ") || "--";
      }
    }
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

  // 计划书 §29/§30：目标接近跨多个事件累积，同名 dict 合并而不是整体覆盖。
  function mergeEmbodiment(patch) {
    var current = appState.embodiment || {};
    Object.keys(patch).forEach(function (key) {
      var value = patch[key];
      var previous = current[key];
      if (value && typeof value === "object" && !Array.isArray(value) &&
          previous && typeof previous === "object" && !Array.isArray(previous)) {
        var merged = {};
        Object.keys(previous).forEach(function (k) { merged[k] = previous[k]; });
        Object.keys(value).forEach(function (k) { merged[k] = value[k]; });
        current[key] = merged;
      } else {
        current[key] = value;
      }
    });
    appState.embodiment = current;
  }

  function fmtMeters(value) {
    return (value === null || value === undefined) ? "--" : Number(value).toFixed(2) + "m";
  }

  // 计划书 §30：净空 / 卡住 / 选中视点 / 脱困原因 / 目标接近状态必须可见，
  // 且接近中的状态绝不能显示成"已完成"。
  function renderEmbodiment() {
    if (!els.stEmbodiment) return;
    var e = appState.embodiment || {};
    var rows = [];
    var safety = e.local_safety;
    if (safety) {
      rows.push("<div>净空 前 " + fmtMeters(safety.front_clearance_m) +
        " · 左 " + fmtMeters(safety.left_clearance_m) +
        " · 右 " + fmtMeters(safety.right_clearance_m) +
        " · 后 " + fmtMeters(safety.rear_clearance_m) + "</div>");
      rows.push('<div class="muted">前扇区 ' + esc(safety.front_state || "--") +
        " · 可前进 " + (safety.forward_safe ? "是" : "否") +
        " · LiDAR " + (safety.lidar_fresh ? "新鲜" : "不新鲜") +
        " · 几何 " + (safety.geometry_validated ? "已校验" : "未校验") +
        " · 安全转向 左" + Number(safety.safe_turn_left_deg || 0).toFixed(0) +
        "° 右" + Number(safety.safe_turn_right_deg || 0).toFixed(0) + "°</div>");
    }
    var perception = e.perception_quality;
    if (perception) {
      rows.push('<div class="muted">相机 ' +
        (perception.camera_blocked ? '<span class="no">贴障/死角</span>' : "正常") +
        " · 信息量 " + Number(perception.estimated_information_gain || 0).toFixed(2) +
        (perception.reason ? " · " + esc(perception.reason) : "") + "</div>");
    }
    if (e.stuck_state) {
      rows.push("<div>卡住状态 <b>" + esc(e.stuck_state) + "</b></div>");
    }
    var viewpoint = e.selected_viewpoint;
    if (viewpoint) {
      rows.push("<div>选中视点 " + esc(viewpoint.viewpoint_id || "--") +
        '<span class="muted"> ← ' + esc(viewpoint.frontier_id || "--") + "</span></div>");
      rows.push('<div class="muted">路径最小净空 ' +
        fmtMeters(viewpoint.path_min_clearance_m) + " · 站位净空 " +
        fmtMeters(viewpoint.local_clearance_m) + " · 视野质量 " +
        Number(viewpoint.view_quality || 0).toFixed(2) + "</div>");
    }
    if (Array.isArray(e.safety_rejections) && e.safety_rejections.length) {
      rows.push('<div class="muted">安全拒绝 ' + e.safety_rejections.length +
        " 个候选：" + esc((e.safety_rejections[0] || {}).reason || "") + "</div>");
    }
    var recovery = e.recovery;
    if (recovery) {
      rows.push("<div>脱困 " + esc(recovery.stage || "") + " · " +
        esc(recovery.trigger || recovery.stuck_state || "") +
        '<span class="muted"> ' + esc(recovery.reason || "") + "</span></div>");
      rows.push('<div class="muted">后退 ' +
        (recovery.backward_dx_m == null ? "不允许" : fmtMeters(recovery.backward_dx_m)) +
        " · 脱困转向 " + (recovery.escape_rotation_deg == null ? "--" :
          Number(recovery.escape_rotation_deg).toFixed(0) + "°") +
        " · 已执行 " + esc(recovery.executed || "无") + "</div>");
    }
    var approach = e.target_approach;
    if (approach) {
      rows.push("<div>目标接近 <b>" + esc(approach.status || "--") + "</b>" +
        (approach.reached ? ' <span class="ok">已到达</span>'
                          : ' <span class="muted">未到达</span>') + "</div>");
      rows.push('<div class="muted">距离 ' + fmtMeters(approach.depth_m) +
        " · 方位 " + (approach.bearing_deg == null ? "--" :
          Number(approach.bearing_deg).toFixed(1) + "°") +
        " · 空间质量 " + esc(approach.spatial_quality || "--") +
        " · 期望 standoff " + fmtMeters(approach.desired_standoff_m) + "</div>");
      rows.push('<div class="muted">第 ' + (approach.steps || 0) + " 步 · " +
        esc(approach.action || "") + " · " +
        esc(approach.reason || approach.decision_reason || "") + "</div>");
    }
    els.stEmbodiment.innerHTML = rows.length ? rows.join("")
      : '<span class="muted">等待局部安全证据…</span>';
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

  // ---- 任务 Agent 解释层（计划书 §5） ------------------------------------
  // 展示规则与所有文案都在 agent_trace.js 里（可被 headless node 钉住），
  // 这里只负责把 HTML 挂到 DOM 上，不重复实现任何判断。
  function renderAgentPanel() {
    var agent = (appState.search || {}).agent || null;
    var html = window.AgentTrace.render(agent && agent.trace, agent);
    els.agentMode.textContent = "";
    els.agentMode.innerHTML = html.mode;
    els.agentSafety.innerHTML = html.safety;
    els.agentPlan.innerHTML = html.plan;
    els.agentChoice.innerHTML = html.choice;
    els.agentDownstream.innerHTML = html.downstream;
    els.agentCandidates.innerHTML = html.candidates;
    els.agentKnowledge.innerHTML = html.knowledge;
    els.agentInputs.innerHTML = html.inputs;
    els.agentUncertainties.innerHTML = html.uncertainties;
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
      // §5.4：每轮历史带上 Agent 依据 —— trace ID、扫描序号、当前子任务、
      // 选中偏好、reason code、关键证据引用、后续 decision ID 与结果。
      // 这些字段随决定一起存在已有的 decisions 列表里，没有第二份缓冲区。
      var agentLine = window.AgentTrace.renderHistoryLine(item.agent);
      return '<div class="decision-history-row"><span>' + esc(text) +
        '</span><span class="cnt">' + esc(String(item.decision_id || "")) +
        esc(result) + '</span>' + agentLine + '</div>';
    }).join("");
  }

  // 计划书 §5.3: a logical turn goal is executed as several bounded primitives.
  // Show the logical goal, the segment progress and the accumulated measured
  // yaw as one line instead of mixing the logical goal with one primitive.
  function turnProgressText(payload) {
    var req = payload.requested_motion || {};
    var obs = payload.observed_motion || {};
    var total = req.requested_total_deg;
    if (total === undefined || total === null) return "";
    var count = Number(req.segment_count || 0);
    var index = Number(req.segment_index || 0);
    var done = obs.observed_total_deg;
    var text = (Number(total) >= 0 ? "左转" : "右转") +
      Math.abs(Number(total)).toFixed(0) + "°";
    if (count > 0) text += "（" + index + "/" + count + "";
    if (done !== undefined && done !== null) {
      text += (count > 0 ? "，" : "（") + "已完成" +
        Math.abs(Number(done)).toFixed(1) + "°";
    }
    if (count > 0 || (done !== undefined && done !== null)) text += "）";
    return text;
  }

  function goalDetailText(g) {
    if (g.semantic_reason) return g.semantic_reason;
    switch (g.goal_type) {
      case "ROTATE_VIEW": {
        var dyaw = g.relative_dyaw;
        if (dyaw === undefined || dyaw === null) return "旋转观察";
        // Positive relative_dyaw increases yaw, i.e. a LEFT turn.
        return "旋转观察 " + (Number(dyaw) >= 0 ? "左转" : "右转") +
          Math.abs(Number(dyaw)).toFixed(0) + "°";
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
      var normalized = Math.max.apply(null, bbox.map(Math.abs)) <= 1.5;
      var sourceW = normalized ? 1 : (els.cam.naturalWidth || canvas.width);
      var sourceH = normalized ? 1 : (els.cam.naturalHeight || canvas.height);
      var x1 = bbox[0] / sourceW * canvas.width;
      var y1 = bbox[1] / sourceH * canvas.height;
      var x2 = bbox[2] / sourceW * canvas.width;
      var y2 = bbox[3] / sourceH * canvas.height;
      ctx.strokeStyle = item.target_watch_status === "CONFIRMED" ? "#22c55e" :
        (item.target_watch_status === "VERIFYING" ? "#3b82f6" :
          (item.is_target_candidate ? "#eab308" :
            (item.is_hazard ? "#f87171" :
              (item.mobility_class === "fixed" ? "#38bdf8" :
                (item.mobility_class === "semi_static" ? "#fb923c" : "#a78bfa")))));
      ctx.lineWidth = 2;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      var label = String(item.label_zh || item.label || item.name || "object") +
        (item.is_hazard ? " ⚠" : "") +
        (item.is_target_candidate ? " · " + (item.target_watch_status || "候选") : "");
      var score = item.score == null ? "" : " " + Number(item.score).toFixed(2);
      ctx.fillStyle = item.is_hazard ? "rgba(248, 113, 113, 0.9)" : "rgba(56, 189, 248, 0.9)";
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
    if (mappingMode && !mappingLiveCamera) {
      els.scamFps.textContent = "已分析帧";
      drawOverlay();
      return;
    }
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
        if (status.control && status.control.display_only) {
          mappingMode = true;
          document.body.classList.add("mapping-mode");
          var searchTab = document.querySelector('[data-tab="search"]');
          if (searchTab) searchTab.textContent = "遥控建图";
          var brand = document.querySelector(".brand");
          if (brand) brand.textContent = "Go2-W 实体遥控建图";
          document.title = "Go2-W 实体遥控建图";
          var toolsEl = document.getElementById("mapping-tools");
          if (toolsEl) toolsEl.classList.remove("hidden");
          var watchEl = document.getElementById("mapping-target-watch");
          if (watchEl) watchEl.classList.remove("hidden");
        }
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
    if (mappingMode) return;
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
            if (fresh) mapRenderer.render(appState.graph, appState.spatial);
          }
        } else if (appState.search.session_id && !ws) {
          appState.search = {};
          renderAll();
        }
      })
      .catch(function () {});
  }

  function playMappingAlert() {
    if (!mappingSoundEnabled) return;
    if (!mappingAudio) mappingAudio = new (window.AudioContext || window.webkitAudioContext)();
    var oscillator = mappingAudio.createOscillator();
    var gain = mappingAudio.createGain();
    oscillator.type = "sine";
    oscillator.frequency.value = 880;
    gain.gain.setValueAtTime(0.0001, mappingAudio.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.12, mappingAudio.currentTime + 0.03);
    gain.gain.exponentialRampToValueAtTime(0.0001, mappingAudio.currentTime + 0.35);
    oscillator.connect(gain); gain.connect(mappingAudio.destination);
    oscillator.start(); oscillator.stop(mappingAudio.currentTime + 0.36);
  }

  function pollMappingState() {
    if (!mappingMode) return;
    fetch("/api/mapping/state", { cache: "no-store" }).then(function (r) { return r.json(); })
      .then(function (state) {
        if (viewingHistoryId) return;
        if (!state || !state.session_id) return;
        var changed = !mappingState || mappingState.session_id !== state.session_id || mappingState.revision !== state.revision ||
          ((mappingState.target_watch || {}).revision !== (state.target_watch || {}).revision);
        mappingState = state;
        var age = Math.max(0, Date.now() / 1000 - (state.generated_at || 0));
        var runtimeEl = document.getElementById("mapping-runtime");
        if (runtimeEl) runtimeEl.textContent = "处理 " + (state.processing_latency_seconds == null ? "--" : state.processing_latency_seconds.toFixed(1) + " s") +
          " · 待处理 " + (state.queue_depth || 0) + " 帧 · 快照 " + (age < 60 ? Math.round(age) + " 秒前" : "较早前") +
          " · 检测帧 " + (state.last_detection_at ? Math.round(Math.max(0, Date.now()/1000-state.last_detection_at)) + " 秒前" : "未产生");
        if (runtimeEl && state.last_error) runtimeEl.textContent += " · 识别失败：" + state.last_error;
        renderTargetWatch(state.target_watch || {});
        if (changed) {
          appState.search = { session_id: state.session_id, status: "RUNNING", phase: "实体遥控建图" };
          appState.graph = state.graph || null;
          lastDetectionFrame = mappingLiveCamera ? [] : (state.current_detections || []);
          if (!mappingLiveCamera && state.frame_available) {
            els.cam.src = "/api/mapping/frame?revision=" + state.revision;
            els.scamLabel.textContent = "识别帧 · " + new Date(state.last_detection_at * 1000).toLocaleTimeString();
          } else if (!mappingLiveCamera) {
            els.cam.src = "/api/camera.mjpeg";
            els.scamLabel.textContent = "实时画面（暂无对应识别帧）";
            lastDetectionFrame = [];
          }
          renderAll();
        }
        var events = state.hazard_events || [];
        var active = events.filter(function (event) { return Date.now() / 1000 - event.last_seen < 30; });
        var alertEl = document.getElementById("mapping-alert");
        if (alertEl) {
          alertEl.classList.toggle("hidden", !active.length);
          alertEl.textContent = active.length ? "⚠ " + active.map(function (event) {
            return (event.status === "confirmed" ? "模拟泄漏危险信号" :
              (event.status === "review_pending" ? "模拟异常待复查" : "疑似模拟泄漏信号")) + "：" +
              event.hazard_label + "（" + event.position_status + "）";
          }).join("；") : "";
        }
        var list = document.getElementById("mapping-hazards");
        if (list) list.innerHTML = events.length ? events.map(function (event) {
          return '<div class="obj-item mapping-hazard">' + esc(event.status === "confirmed" ? "⚠ 模拟泄漏危险信号" :
            (event.status === "review_pending" ? "模拟异常待复查" : "疑似模拟泄漏信号")) +
            ' · ' + esc(event.hazard_label) + ' · ' + esc(event.position_status) +
            ' · 最近观测 ' + new Date(event.last_seen * 1000).toLocaleTimeString() + '</div>';
        }).join("") : "暂无告警";
        active.forEach(function (event) {
          if (event.status !== "confirmed") return;
          var previous = lastSoundByEvent[event.hazard_event_id] || 0;
          if (Date.now() / 1000 - previous >= (state.sound_cooldown_s || 30) && changed) {
            playMappingAlert();
            lastSoundByEvent[event.hazard_event_id] = Date.now() / 1000;
          }
        });
      }).catch(function () {});
  }

  var soundButton = document.getElementById("mapping-sound");
  if (soundButton) soundButton.addEventListener("click", function () {
    mappingSoundEnabled = !mappingSoundEnabled;
    if (mappingSoundEnabled && !mappingAudio) mappingAudio = new (window.AudioContext || window.webkitAudioContext)();
    soundButton.textContent = mappingSoundEnabled ? "静音" : "启用声音";
  });
  var cameraModeButton = document.getElementById("mapping-camera-mode");
  if (cameraModeButton) cameraModeButton.addEventListener("click", function () {
    mappingLiveCamera = !mappingLiveCamera;
    cameraModeButton.textContent = mappingLiveCamera ? "切到识别帧" : "切到实时画面";
    if (mappingLiveCamera) {
      els.cam.src = "/api/camera.mjpeg";
      els.scamLabel.textContent = "实时画面（检测框已隐藏）";
      lastDetectionFrame = [];
      drawOverlay();
    } else if (mappingState && mappingState.frame_available) {
      els.cam.src = "/api/mapping/frame?revision=" + mappingState.revision;
      els.scamLabel.textContent = "识别帧 · " + new Date(mappingState.last_detection_at * 1000).toLocaleTimeString();
      lastDetectionFrame = mappingState.current_detections || [];
      drawOverlay();
    }
  });
  var resetButton = document.getElementById("mapping-layout-reset");
  if (resetButton) resetButton.addEventListener("click", function () { mapRenderer.resetTopologyLayout(); });

  function targetWatchStateLabel(state) {
    return ({IDLE:"未启用", PARSING:"解析中", ARMED:"等待候选", CANDIDATE:"候选待复核",
      VERIFYING:"VLM 复核中", CONFIRMED:"已确认", REJECTED:"最近候选已排除",
      UNKNOWN:"复核未知", DEGRADED:"VLM 降级"})[state] || state || "未启用";
  }

  function renderTargetWatch(watch) {
    var badge = document.getElementById("target-watch-state");
    var summary = document.getElementById("target-watch-summary");
    var evidence = document.getElementById("target-watch-evidence");
    if (!badge || !summary || !evidence) return;
    badge.textContent = (watch.automatic_hazard_enabled && (watch.state || "IDLE") === "IDLE") ?
      "自动危险源监看中" : targetWatchStateLabel(watch.state);
    badge.dataset.state = watch.state || "IDLE";
    var profile = watch.profile || {};
    var candidate = watch.last_candidate || {};
    var verify = watch.verification || {};
    var stats = watch.statistics || {};
    var parts = [];
    if (watch.automatic_hazard_enabled) {
      parts.push("自动危险源：已启用（史莱姆泥 / 管道模拟泄漏，识别候选后自动 VLM 复核）");
    }
    parts.push("人工目标：" + esc(watch.raw_query || "未设置"));
    if (watch.enabled) parts.push("标准化：" + esc(profile.canonical_name_zh || "--"));
    if (profile.resolver_source) parts.push("解析：" + esc(profile.resolver_source) + (profile.resolver_error ? "（已回退）" : ""));
    if (profile.target_terms_en) parts.push("目标词：" + esc(profile.target_terms_en.join(", ")));
    if (profile.anchor_terms_en && profile.anchor_terms_en.length) parts.push("锚点词：" + esc(profile.anchor_terms_en.join(", ")));
    if (profile.relation_constraints && profile.relation_constraints.length) parts.push("关系：" + esc(profile.relation_constraints.join("；")));
    if (candidate.candidate_id) parts.push((candidate.automatic_hazard ? "自动危险候选：" : "最近候选：") + esc(candidate.matched_term || candidate.label || "object") + " / " + Number(candidate.detector_score || 0).toFixed(2) + " / frame " + esc(candidate.frame_id || "--"));
    if (verify.status) parts.push("复核：" + esc(verify.status) + (verify.confidence == null ? "" : " / " + Number(verify.confidence).toFixed(2)) + (verify.reason_zh ? " / " + esc(verify.reason_zh) : ""));
    if (verify.latency_ms != null) parts.push("耗时：" + Math.round(verify.latency_ms) + " ms");
    if (candidate.camera_xyz) parts.push("camera_xyz：" + candidate.camera_xyz.map(Number).map(function(v){ return v.toFixed(2); }).join(", "));
    parts.push("map_xyz：" + (candidate.map_xyz ? candidate.map_xyz.map(Number).map(function(v){ return v.toFixed(2); }).join(", ") : "无可靠地图位置"));
    parts.push("统计：帧 " + (stats.frames_evaluated || 0) + " / 候选 " + (stats.candidates_generated || 0) + " / VLM " + (stats.verify_started || 0));
    if (watch.last_error) parts.push("错误：" + esc(watch.last_error));
    summary.innerHTML = parts.join("<br>");
    var verificationId = verify.verification_id || candidate.verification_id;
    if (verificationId) {
      evidence.src = "/api/mapping/target-watch/evidence/" + encodeURIComponent(verificationId) + "?sequence=" + (verify.sequence || 0);
      evidence.classList.remove("hidden");
    } else {
      evidence.classList.add("hidden");
      evidence.removeAttribute("src");
    }
  }

  function setTargetWatch() {
    var input = document.getElementById("target-watch-input");
    var target = input ? input.value.trim() : "";
    if (!target) { showBanner("请输入监看目标", "error"); return; }
    fetch("/api/mapping/target-watch", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({target:target})})
      .then(function(response){ return response.json().then(function(body){ return {ok:response.ok, body:body}; }); })
      .then(function(result){ if (!result.ok) throw new Error(result.body.error || "设置失败"); showBanner("目标监看已提交，等待检测候选", "info"); pollMappingState(); })
      .catch(function(error){ showBanner("目标监看设置失败：" + error.message, "error"); });
  }
  var targetWatchSet = document.getElementById("target-watch-set");
  var targetWatchInput = document.getElementById("target-watch-input");
  var targetWatchClear = document.getElementById("target-watch-clear");
  if (targetWatchSet) targetWatchSet.addEventListener("click", setTargetWatch);
  if (targetWatchInput) targetWatchInput.addEventListener("keydown", function(event){ if (event.key === "Enter") { event.preventDefault(); setTargetWatch(); } });
  if (targetWatchClear) targetWatchClear.addEventListener("click", function(){
    fetch("/api/mapping/target-watch", {method:"DELETE"})
      .then(function(response){ return response.json().then(function(body){ return {ok:response.ok, body:body}; }); })
      .then(function(result){ if (!result.ok) throw new Error(result.body.error || "清除失败"); showBanner("人工目标已清除；自动危险源监看继续运行", "info"); pollMappingState(); })
      .catch(function(error){ showBanner("目标监看清除失败：" + error.message, "error"); });
  });

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
        loadReplay(sessionId);
      }).catch(function (err) {
        showErrorRetry(String(err || "history request failed"));
      });
  }

  // ------------------------------------------------------------------ //
  // replay panel (history detail)                                      //
  // ------------------------------------------------------------------ //
  var REPLAY_ACTIVE = {
    STARTING: 1, RECORDING: 1, FINALIZING: 1, QUEUED: 1, RENDERING: 1,
  };
  var REPLAY_LABELS = {
    DISABLED: "回放未启用",
    STARTING: "准备录制",
    RECORDING: "录制中",
    FINALIZING: "收尾中",
    QUEUED: "排队等待生成",
    RENDERING: "正在生成",
    READY: "可播放",
    PARTIAL: "可播放（有降级说明）",
    FAILED: "生成失败",
  };

  function replayApiBase(sessionId) {
    return "/api/search/history/" + encodeURIComponent(sessionId) + "/replay";
  }

  function stopReplayPoll() {
    if (replayPollTimer) {
      clearTimeout(replayPollTimer);
      replayPollTimer = null;
    }
  }

  function showReplayPanel(sessionId) {
    if (!els.replayPanel) return;
    els.replayPanel.classList.remove("hidden");
    if (els.replayStatus) els.replayStatus.textContent = "读取中…";
    if (replaySessionId !== sessionId && els.replayVideo) {
      els.replayVideo.pause();
      els.replayVideo.classList.add("hidden");
      els.replayVideo.removeAttribute("src");
      // data-src 是"已经加载过哪个地址"的标记，必须一起清掉。只清 src 的话
      // 切回同一个会话时 attachReplayVideo 会认为地址没变、拒绝重新加载，
      // 播放器就永远停在无源状态。
      els.replayVideo.removeAttribute("data-src");
      els.replayVideo.load();
    }
    replaySessionId = sessionId;
  }

  function hideReplay() {
    stopReplayPoll();
    replaySessionId = null;
    replayLastStatus = null;
    if (els.replayVideo) {
      els.replayVideo.pause();
      els.replayVideo.classList.add("hidden");
    }
    if (els.replayPanel) els.replayPanel.classList.add("hidden");
  }

  function loadReplay(sessionId) {
    if (!sessionId) return;
    showReplayPanel(sessionId);
    fetch(replayApiBase(sessionId))
      .then(function (response) { return response.json(); })
      .then(function (status) {
        if (replaySessionId !== sessionId) return;
        renderReplayStatus(status);
      })
      .catch(function () {
        if (replaySessionId !== sessionId) return;
        if (els.replayStatus) els.replayStatus.textContent = "回放状态读取失败";
        renderReplayPlaceholder("回放接口暂时不可用，稍后会自动重试。");
        scheduleReplayPoll(sessionId);
      });
  }

  function renderReplayStatus(status) {
    replayLastStatus = status;
    var state = (status && status.status) || "DISABLED";
    var label = REPLAY_LABELS[state] || state;
    if (els.replayStatus) {
      els.replayStatus.textContent = label +
        (status && status.speed ? " · " + status.speed + "×" : "");
    }
    updateReplaySpeedButtons(status);
    if (els.replayProgress) {
      if (REPLAY_ACTIVE[state] && state === "RENDERING") {
        els.replayProgress.textContent =
          Math.round((status.progress || 0) * 100) + "%";
      } else if (REPLAY_ACTIVE[state]) {
        els.replayProgress.textContent = "请稍候";
      } else {
        els.replayProgress.textContent = "";
      }
    }
    var playable = !!(status && status.playable && status.urls && status.urls.video);
    if (playable) {
      attachReplayVideo(status);
      if (els.replayPlaceholder) els.replayPlaceholder.classList.add("hidden");
    } else {
      if (els.replayVideo) els.replayVideo.classList.add("hidden");
      renderReplayPlaceholder(replayPlaceholderText(status, state, label));
    }
    renderReplayLinks(status, playable);
    renderReplayReasons(status);
    if (REPLAY_ACTIVE[state] || (!playable && status && status.status === "QUEUED")) {
      scheduleReplayPoll(replaySessionId);
    } else {
      stopReplayPoll();
    }
  }

  function replayPlaceholderText(status, state, label) {
    if (!status) return "回放状态不可用。";
    if (status.status === "FAILED") {
      return "回放生成失败：" + ((status.reasons || [])[0] || "未知原因");
    }
    if (status.status === "DISABLED") {
      return "本次会话没有回放记录（可能是回放未启用或采集器未启动）。";
    }
    if (REPLAY_ACTIVE[state]) {
      return label + "：后台正在处理，完成后这里会自动出现播放器。";
    }
    return label + "。可以点「生成 / 重新生成」重试。";
  }

  function renderReplayPlaceholder(text) {
    if (!els.replayPlaceholder) return;
    els.replayPlaceholder.textContent = text;
    els.replayPlaceholder.classList.remove("hidden");
  }

  function attachReplayVideo(status) {
    if (!els.replayVideo) return;
    var base = status.urls.video;
    var version = [
      status.recording_id || "", status.speed || "",
      status.size_bytes || "", status.duration || "",
    ].join("-");
    var src = base + "?v=" + encodeURIComponent(version);
    // 没有 src 时即便 data-src 相同也必须重新加载（面板切换会清掉 src）。
    var needsLoad = !els.replayVideo.getAttribute("src")
      || els.replayVideo.getAttribute("data-src") !== src;
    if (needsLoad) {
      els.replayVideo.setAttribute("data-src", src);
      els.replayVideo.src = src;
      els.replayVideo.load();
    }
    els.replayVideo.classList.remove("hidden");
  }

  function renderReplayLinks(status, playable) {
    var artifacts = (status && status.artifacts) || {};
    if (els.replayDownload) {
      if (playable) {
        els.replayDownload.href = status.urls.download;
        els.replayDownload.classList.remove("hidden");
      } else {
        els.replayDownload.classList.add("hidden");
      }
    }
    if (els.replayFinalMap) {
      if (artifacts["final_map.png"]) {
        els.replayFinalMap.href = status.urls.final_map;
        els.replayFinalMap.classList.remove("hidden");
      } else {
        els.replayFinalMap.classList.add("hidden");
      }
    }
    if (els.replayCover) {
      if (artifacts["cover.jpg"]) {
        els.replayCover.href = status.urls.cover;
        els.replayCover.classList.remove("hidden");
      } else {
        els.replayCover.classList.add("hidden");
      }
    }
    if (els.replayRegenerate) {
      els.replayRegenerate.disabled = false;
    }
  }

  function renderReplayReasons(status) {
    if (!els.replayReasons) return;
    var items = ((status && status.reasons) || []).concat((status && status.notes) || []);
    if (!items.length) {
      els.replayReasons.classList.add("hidden");
      els.replayReasons.textContent = "";
      return;
    }
    els.replayReasons.classList.remove("hidden");
    els.replayReasons.innerHTML = items.map(function (item) {
      return "<div>· " + esc(item) + "</div>";
    }).join("");
  }

  function updateReplaySpeedButtons(status) {
    if (!els.replaySpeeds) return;
    var current = replaySpeed || (status && status.speed) || 10;
    var buttons = els.replaySpeeds.querySelectorAll("[data-replay-speed]");
    for (var i = 0; i < buttons.length; i += 1) {
      var value = parseInt(buttons[i].getAttribute("data-replay-speed"), 10);
      buttons[i].classList.toggle("active", value === current);
    }
  }

  function scheduleReplayPoll(sessionId) {
    stopReplayPoll();
    replayPollTimer = setTimeout(function () {
      replayPollTimer = null;
      if (replaySessionId === sessionId) loadReplay(sessionId);
    }, 2000);
  }

  function requestReplayRender(speed, force) {
    var sessionId = replaySessionId;
    if (!sessionId) return;
    replaySpeed = speed;
    updateReplaySpeedButtons(replayLastStatus);
    if (els.replayStatus) els.replayStatus.textContent = "提交生成请求…";
    fetch(replayApiBase(sessionId) + "/render", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ speed: speed, force: !!force }),
    }).then(function (response) { return response.json(); })
      .then(function (data) {
        if (!data.ok) {
          renderReplayPlaceholder("无法生成：" + (data.error || "未知原因"));
          stopReplayPoll();
          return;
        }
        scheduleReplayPoll(sessionId);
        setTimeout(function () {
          if (replaySessionId === sessionId) loadReplay(sessionId);
        }, 400);
      })
      .catch(function () {
        renderReplayPlaceholder("生成请求失败，稍后重试。");
      });
  }

  if (els.replaySpeeds) {
    els.replaySpeeds.addEventListener("click", function (event) {
      var button = event.target.closest
        ? event.target.closest("[data-replay-speed]") : null;
      if (!button || !replaySessionId) return;
      var speed = parseInt(button.getAttribute("data-replay-speed"), 10);
      var status = replayLastStatus;
      if (status && status.playable && status.speed === speed
          && els.replayVideo && !els.replayVideo.classList.contains("hidden")) {
        replaySpeed = speed;
        updateReplaySpeedButtons(status);
        return;
      }
      requestReplayRender(speed, false);
    });
  }
  if (els.replayRegenerate) {
    els.replayRegenerate.addEventListener("click", function () {
      var status = replayLastStatus;
      var speed = replaySpeed || (status && status.speed) || 10;
      var force = !!(status && status.playable && status.speed === speed);
      requestReplayRender(speed, force);
    });
  }
  if (els.replayClose) {
    els.replayClose.addEventListener("click", function () { hideReplay(); });
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
      hideReplay();
      if (mappingMode) {
        mappingState = null;
        pollMappingState();
        pollHistory();
        return;
      }
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
  setInterval(pollMappingState, 1000);
  setInterval(pollSearchState, 3000);
  setInterval(pollHistory, 5000);
  connect();
})();
