/* Current runtime data controls.  All safety and confirmation checks remain
 * server-side; this file only presents the operator workflow. */
(function () {
  "use strict";

  var panel = document.getElementById("runtime-admin-tools");
  var profile = document.getElementById("runtime-profile");
  var status = document.getElementById("runtime-admin-status");
  var clearObjects = document.getElementById("btn-clear-objects");
  var clearMap = document.getElementById("btn-clear-map");
  var searchDetector = document.getElementById("search-detector");
  if (!panel || !clearObjects || !clearMap) return;

  function setBusy(value) {
    clearObjects.disabled = value;
    clearMap.disabled = value;
  }

  function showError(error) {
    var labels = {
      search_must_be_stopped: "请先停止当前自主搜索",
      motion_must_be_idle: "请先停止运动并停用网页控制",
      reset_already_running: "已有清理/重启任务正在执行",
      confirmation_mismatch: "二次确认不匹配",
      runtime_admin_disabled: "当前启动模式未开放运行时清理",
    };
    status.textContent = labels[error] || error || "操作失败";
  }

  function waitForRestart() {
    var started = Date.now();
    var sawOffline = false;
    window.setTimeout(function poll() {
      fetch("/api/status", { cache: "no-store" })
        .then(function (response) {
          if (!response.ok) throw new Error("not ready");
          if (sawOffline || Date.now() - started > 12000) {
            window.location.reload();
            return;
          }
          window.setTimeout(poll, 1000);
        })
        .catch(function () {
          sawOffline = true;
          window.setTimeout(poll, 1000);
        });
    }, 2500);
  }

  function requestReset(action) {
    var mapReset = action === "map";
    var message = mapReset
      ? "这会归档当前地图与物体，重启二维 SLAM，并从空白地图重新建图。确定继续？"
      : "这会归档并清空当前所有物体、危险事件和物体拓扑，但保留二维地图。确定继续？";
    if (!window.confirm(message)) return;
    setBusy(true);
    status.textContent = "正在提交，WebUI 将自动重启…";
    fetch("/api/runtime/reset", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: action,
        confirmation: mapReset ? "CLEAR_CURRENT_MAP" : "CLEAR_CURRENT_OBJECTS",
      }),
    })
      .then(function (response) {
        return response.json().then(function (body) {
          if (!response.ok || !body.ok) throw new Error(body.error || "reset_failed");
          return body;
        });
      })
      .then(function () {
        status.textContent = "已接受；正在归档并重启，请勿重复点击…";
        waitForRestart();
      })
      .catch(function (error) {
        setBusy(false);
        showError(error.message);
      });
  }

  clearObjects.addEventListener("click", function () { requestReset("objects"); });
  clearMap.addEventListener("click", function () { requestReset("map"); });

  fetch("/api/runtime/admin", { cache: "no-store" })
    .then(function (response) { return response.json(); })
    .then(function (data) {
      if (!data.enabled) {
        panel.classList.add("hidden");
        return;
      }
      var mode = data.mode === "teleop_mapping" ? "被动遥操建图" : "自主搜索";
      var detector = data.detector === "yoloe_seg" ? "YOLOE-Seg" : "GroundingDINO + SAM2";
      profile.textContent = mode + " · " + detector;
      if (searchDetector && searchDetector.querySelector('option[value="' + data.detector + '"]')) {
        searchDetector.value = data.detector;
      }
      if (data.busy) {
        setBusy(true);
        status.textContent = "清理/重启任务正在执行";
      }
    })
    .catch(function () {
      status.textContent = "运行模式读取失败";
    });
}());
