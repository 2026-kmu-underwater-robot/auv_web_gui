const state = {
  websocket: null,
  reconnectTimer: null,
  ekf: {
    values: [],
    stateNames: [],
    size: 15,
  },
  bag: {
    topicsLoaded: false,
    loading: false,
    error: "",
    defaultTopics: [],
    topics: [],
    analysis: {
      running: false,
      result: null,
      error: "",
    },
  },
  test: {
    running: false,
    message: "Idle",
    steps: [],
  },
  path: {
    points: [],
    pose: { x: 0, y: 0, yaw: 0 },
    options: {
      viewMode: "fit",
      color: "#6ec6ff",
      width: 2.5,
      scale: 40,
      gridStep: 1,
      showGrid: true,
      showAxes: true,
      showRobot: true,
    },
  },
  control: {
    enabled: false,
    active: false,
    sendTimer: null,
    lastErrorAt: 0,
    axes: {
      forward: 0,
      lateral: 0,
      vertical: 0,
      yaw: 0,
    },
  },
};

const $ = (id) => document.getElementById(id);

function fmt(value, digits = 2) {
  if (typeof value !== "number" || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

function fmtUnit(value, unit, digits = 2) {
  const text = fmt(value, digits);
  return text === "--" ? "--" : `${text} ${unit}`;
}

function fmtPercent(value, digits = 1) {
  return typeof value === "number" && Number.isFinite(value)
    ? `${(value * 100).toFixed(digits)} %`
    : "--";
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function setPill(id, active, label) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("good", active);
  el.classList.toggle("warn", !active);
  el.classList.toggle("bad", false);
}

function setPillState(id, label, state) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("good", state === "good");
  el.classList.toggle("warn", state === "warn");
  el.classList.toggle("bad", state === "bad");
}

function setTelemetryState(id, label, state) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("state-good", state === "good");
  el.classList.toggle("state-warn", state === "warn");
  el.classList.toggle("state-bad", state === "bad");
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    let detail = `${path} failed: ${response.status}`;
    try {
      const body = await response.json();
      if (body.detail) detail = body.detail;
    } catch (_) {
      // Keep the status-only error when the body is not JSON.
    }
    throw new Error(detail);
  }
  return response.json();
}

async function getJson(path) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`${path} failed: ${response.status}`);
  }
  return response.json();
}

function bindControls() {
  $("start-stack").addEventListener("click", () => {
    postJson("/api/stack/start").catch(showError);
  });
  $("stop-stack").addEventListener("click", () => {
    postJson("/api/stack/stop").catch(showError);
  });
  $("start-bag").addEventListener("click", () => {
    startBag().catch(showError);
  });
  $("stop-bag").addEventListener("click", () => {
    postJson("/api/bag/stop").catch(showError);
  });
  $("bag-refresh-topics").addEventListener("click", () => {
    loadBagTopics().catch(showError);
  });
  $("analyze-bag").addEventListener("click", () => {
    analyzeBag().catch(showError);
  });
  $("test-start").addEventListener("click", () => {
    runLocalizationTest().catch(showError);
  });

  document.querySelectorAll("[data-dvl-command]").forEach((button) => {
    button.addEventListener("click", () => {
      const payload = {
        command: button.dataset.dvlCommand,
        parameter_name: button.dataset.dvlParam || "",
        parameter_value: button.dataset.dvlValue || "",
      };
      postJson("/api/dvl/command", payload).catch(showError);
    });
  });

  document.querySelectorAll("[data-tab]").forEach((button) => {
    button.addEventListener("click", () => {
      showTab(button.dataset.tab);
    });
  });

  $("ekf-reload").addEventListener("click", () => {
    loadEkfConfig().catch(showError);
  });
  $("ekf-save").addEventListener("click", () => {
    saveEkfConfig().catch(showError);
  });

  bindPathControls();
  bindWebControl();
}

function showTab(name) {
  document.querySelectorAll(".tab-button").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === name);
  });
  document.querySelectorAll(".tab-view").forEach((view) => {
    view.classList.toggle("active", view.id === `${name}-tab`);
  });
  if (name === "ekf" && state.ekf.values.length === 0) {
    loadEkfConfig().catch(showError);
  }
  if (name === "bag" && !state.bag.topicsLoaded) {
    loadBagTopics().catch(showError);
  }
}

function connectStatusSocket() {
  clearTimeout(state.reconnectTimer);
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  state.websocket = new WebSocket(`${protocol}//${window.location.host}/ws/status`);

  state.websocket.addEventListener("open", () => {
    $("connection-state").textContent = "Connected";
  });

  state.websocket.addEventListener("message", (event) => {
    renderStatus(JSON.parse(event.data));
  });

  state.websocket.addEventListener("close", () => {
    $("connection-state").textContent = "Disconnected. Reconnecting...";
    state.reconnectTimer = setTimeout(connectStatusSocket, 1000);
  });

  state.websocket.addEventListener("error", () => {
    $("connection-state").textContent = "Connection error";
  });
}

function renderStatus(payload) {
  const process = payload.process || {};
  const ros = payload.ros || {};
  const topics = ros.topics || {};
  const pose = ros.pose || {};
  const velocity = ros.velocity || {};
  const depth = ros.depth || {};
  const battery = ros.battery || {};
  const mavrosState = ros.mavros_state || {};
  const joy = ros.joy || {};
  const webControl = ros.web_control || {};
  const dvlConfig = ros.dvl_config || {};
  const dvlEvents = ros.dvl_events || [];

  setPill("stack-pill", process.stack_running, process.stack_running ? "STACK ON" : "STACK OFF");
  renderMavrosPill(mavrosState, topics.mavros_state);
  setPill("bag-pill", process.bag_running, process.bag_running ? "BAG ON" : "BAG OFF");
  setPill("joy-pill", topics.joy?.alive, topics.joy?.alive ? "JOY ON" : "JOY OFF");
  setPill("battery-pill", topics.battery?.alive, topics.battery?.alive ? "BAT ON" : "BAT OFF");

  $("pose-value").textContent = `${fmt(pose.x)}, ${fmt(pose.y)}, ${fmt(pose.z)}`;
  $("yaw-value").textContent = `${fmt((pose.yaw || 0) * 180 / Math.PI, 1)} deg`;
  $("velocity-value").textContent = `${fmt(velocity.x)}, ${fmt(velocity.y)}, ${fmt(velocity.z)}`;
  $("depth-value").textContent = `${fmt(depth.z)} m`;
  $("battery-voltage").textContent = fmtUnit(battery.voltage, "V");
  $("battery-current").textContent = fmtUnit(battery.current, "A");
  $("battery-soc").textContent =
    typeof battery.percentage === "number" && Number.isFinite(battery.percentage)
      ? `${fmt(battery.percentage * 100, 0)} %`
      : "--";
  $("battery-temp").textContent = fmtUnit(battery.temperature, "C", 1);
  renderMavrosState(mavrosState, topics.mavros_state);
  renderJoyGamepad(joy, topics.joy);
  renderDvl(dvlConfig, dvlEvents);
  renderTestState();
  renderBag(process);
  renderTopics(topics);
  renderWebControlStatus(webControl);
  state.path.points = ros.path || [];
  state.path.pose = {
    x: typeof pose.x === "number" ? pose.x : 0,
    y: typeof pose.y === "number" ? pose.y : 0,
    yaw: typeof pose.yaw === "number" ? pose.yaw : 0,
  };
  renderPath();
  $("log-output").textContent = (process.logs || []).join("\n");
}

function bindWebControl() {
  ["forward", "lateral", "vertical", "yaw"].forEach((name) => {
    $(`control-${name}`).addEventListener("input", () => {
      updateControlAxis(name, Number($(`control-${name}`).value));
    });
  });

  $("control-enable").addEventListener("change", () => {
    setWebControlEnabled($("control-enable").checked).catch(showError);
  });

  $("control-stop").addEventListener("click", () => {
    resetControlAxes();
    state.control.active = false;
    $("control-deadman").classList.remove("active");
    sendControlCommand().catch(showThrottledControlError);
  });

  $("control-deadman").addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (!state.control.enabled) return;
    state.control.active = true;
    $("control-deadman").classList.add("active");
    sendControlCommand().catch(showThrottledControlError);
  });

  ["pointerup", "pointercancel", "pointerleave"].forEach((eventName) => {
    $("control-deadman").addEventListener(eventName, () => {
      state.control.active = false;
      $("control-deadman").classList.remove("active");
      sendControlCommand().catch(showThrottledControlError);
    });
  });

  $("control-arm").addEventListener("click", () => {
    postJson("/api/control/arm", { armed: true }).catch(showError);
  });
  $("control-disarm").addEventListener("click", () => {
    postJson("/api/control/arm", { armed: false }).catch(showError);
  });

  document.querySelectorAll("[data-control-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      postJson("/api/control/mode", { mode: button.dataset.controlMode }).catch(showError);
    });
  });

  renderControlAxisValues();
}

async function setWebControlEnabled(enabled) {
  state.control.enabled = enabled;
  state.control.active = false;
  $("control-deadman").classList.remove("active");
  if (enabled) {
    startControlLoop();
  } else {
    stopControlLoop();
  }
  await postJson("/api/control/enable", { enabled });
  await sendControlCommand();
}

function startControlLoop() {
  if (state.control.sendTimer) return;
  state.control.sendTimer = window.setInterval(() => {
    if (!state.control.enabled) return;
    sendControlCommand().catch(showThrottledControlError);
  }, 100);
}

function stopControlLoop() {
  if (!state.control.sendTimer) return;
  window.clearInterval(state.control.sendTimer);
  state.control.sendTimer = null;
}

function updateControlAxis(name, value) {
  state.control.axes[name] = clamp(value, -1, 1);
  renderControlAxisValues();
  if (state.control.enabled) {
    sendControlCommand().catch(showThrottledControlError);
  }
}

function resetControlAxes() {
  Object.keys(state.control.axes).forEach((name) => {
    state.control.axes[name] = 0;
    $(`control-${name}`).value = 0;
  });
  renderControlAxisValues();
}

function renderControlAxisValues() {
  Object.entries(state.control.axes).forEach(([name, value]) => {
    $(`control-${name}-value`).textContent = fmt(value, 2);
  });
}

async function sendControlCommand() {
  if (!state.control.enabled) return;
  await postJson("/api/control/command", {
    active: state.control.active,
    axes: state.control.axes,
  });
}

function renderWebControlStatus(webControl) {
  if (!webControl.enabled) {
    setPillState("control-state", "WEB OFF", "warn");
    $("control-publish-state").textContent = "--";
    return;
  }
  if (webControl.active && webControl.fresh) {
    setPillState("control-state", "WEB DRIVE", "bad");
  } else {
    setPillState("control-state", "WEB READY", "good");
  }
  $("control-publish-state").textContent = webControl.last_publish
    ? `Last ${webControl.last_publish}`
    : "--";
}

function showThrottledControlError(error) {
  const now = Date.now();
  if (now - state.control.lastErrorAt < 2000) return;
  state.control.lastErrorAt = now;
  showError(error);
}

function renderMavrosPill(mavrosState, topic) {
  if (!topic?.alive) {
    setPillState("mavros-pill", "MAV OFF", "warn");
    return;
  }
  if (!mavrosState.connected) {
    setPillState("mavros-pill", "MAV NO FCU", "warn");
    return;
  }
  setPillState(
    "mavros-pill",
    mavrosState.armed ? "MAV ARMED" : "MAV SAFE",
    mavrosState.armed ? "bad" : "good",
  );
}

function renderMavrosState(mavrosState, topic) {
  const alive = Boolean(topic?.alive);
  const connected = alive && Boolean(mavrosState.connected);
  const armed = connected && Boolean(mavrosState.armed);
  const mode = connected && mavrosState.mode ? mavrosState.mode : "--";
  const control = connected
    ? [
        mavrosState.guided ? "GUIDED" : "NON-GUIDED",
        mavrosState.manual_input ? "MANUAL IN" : "NO MANUAL",
      ].join(" / ")
    : "--";

  setTelemetryState(
    "mavros-connected",
    connected ? "CONNECTED" : alive ? "NO FCU" : "NO TOPIC",
    connected ? "good" : "warn",
  );
  setTelemetryState(
    "mavros-armed",
    armed ? "ARMED" : connected ? "SAFE" : "--",
    connected ? (armed ? "bad" : "good") : "warn",
  );
  setTelemetryState("mavros-mode", mode, connected ? "good" : "warn");
  setTelemetryState("mavros-control", control, connected ? "good" : "warn");
}

function renderJoyGamepad(joy, topic) {
  const axes = Array.isArray(joy.axes) ? joy.axes : [];
  const buttons = Array.isArray(joy.buttons) ? joy.buttons : [];
  const alive = Boolean(topic?.alive);
  document.querySelector(".ps4-pad")?.classList.toggle("offline", !alive);

  setJoyButton("joy-btn-cross", buttonPressed(buttons, 0));
  setJoyButton("joy-btn-circle", buttonPressed(buttons, 1));
  setJoyButton("joy-btn-square", buttonPressed(buttons, 2));
  setJoyButton("joy-btn-triangle", buttonPressed(buttons, 3));
  setJoyButton("joy-btn-l1", buttonPressed(buttons, 4));
  setJoyButton("joy-btn-r1", buttonPressed(buttons, 5));
  setJoyButton("joy-btn-share", buttonPressed(buttons, 6));
  setJoyButton("joy-btn-options", buttonPressed(buttons, 7));
  setJoyButton("joy-btn-ps", buttonPressed(buttons, 8));
  setJoyButton("joy-btn-l3", buttonPressed(buttons, 9));
  setJoyButton("joy-btn-r3", buttonPressed(buttons, 10));
  setJoyButton("joy-btn-touchpad", buttonPressed(buttons, 13));

  const dpadX = axisValue(axes, 6);
  const dpadY = axisValue(axes, 7);
  setJoyButton("joy-dpad-left", dpadX > 0.5);
  setJoyButton("joy-dpad-right", dpadX < -0.5);
  setJoyButton("joy-dpad-up", dpadY > 0.5);
  setJoyButton("joy-dpad-down", dpadY < -0.5);

  setStickDot("joy-left-stick-dot", axisValue(axes, 0), axisValue(axes, 1));
  setStickDot("joy-right-stick-dot", axisValue(axes, 2), axisValue(axes, 3));
  const l2Amount = triggerAmount(axes, 4);
  const r2Amount = triggerAmount(axes, 5);
  setTriggerFill("joy-lt-fill", l2Amount);
  setTriggerFill("joy-rt-fill", r2Amount);
  setJoyButton("joy-btn-l2", buttonPressed(buttons, 11) || l2Amount > 0.5);
  setJoyButton("joy-btn-r2", buttonPressed(buttons, 12) || r2Amount > 0.5);

  const pressedCount = buttons.filter((value) => isPressedValue(value)).length;
  const leftStick = `${fmt(axisValue(axes, 0), 2)}, ${fmt(axisValue(axes, 1), 2)}`;
  const rightStick = `${fmt(axisValue(axes, 2), 2)}, ${fmt(axisValue(axes, 3), 2)}`;
  $("joy-gamepad-state").textContent = alive
    ? `Buttons ${pressedCount} · LS ${leftStick} · RS ${rightStick}`
    : "No Joy data";
}

function setJoyButton(id, pressed) {
  const el = $(id);
  if (!el) return;
  el.classList.toggle("pressed", Boolean(pressed));
}

function setStickDot(id, x, y) {
  const el = $(id);
  if (!el) return;
  const stickSize = el.parentElement?.getBoundingClientRect().width || 72;
  const maxTravel = clamp(stickSize * 0.28, 14, 24);
  const offsetX = clamp(x, -1, 1) * maxTravel;
  const offsetY = -clamp(y, -1, 1) * maxTravel;
  el.style.transform = `translate(-50%, -50%) translate(${offsetX}px, ${offsetY}px)`;
}

function setTriggerFill(id, amount) {
  const el = $(id);
  if (!el) return;
  el.style.width = `${clamp(amount, 0, 1) * 100}%`;
}

function buttonPressed(buttons, index) {
  return isPressedValue(buttons[index]);
}

function isPressedValue(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric !== 0;
}

function axisValue(axes, index) {
  const value = Number(axes[index]);
  return Number.isFinite(value) ? value : 0;
}

function triggerAmount(axes, index) {
  if (axes[index] === undefined) return 0;
  const value = axisValue(axes, index);
  // DualShock triggers commonly report 1.0 at rest and -1.0 when fully pressed.
  if (value < -0.05) return (1 - value) / 2;
  return 0;
}

function renderBag(process) {
  $("bag-state").textContent = process.bag_running ? "Recording" : "Stopped";
  $("bag-output").textContent = process.bag_output || "--";
  $("bag-log-output").textContent = (process.logs || [])
    .filter((line) => line.includes("[bag]") || line.includes("ros2 bag record"))
    .join("\n");
  renderBagAnalysis();
}

async function loadBagTopics() {
  state.bag.loading = true;
  state.bag.error = "";
  renderBagTopics();
  try {
    const payload = await getJson("/api/bag/topics");
    state.bag.defaultTopics = payload.default_topics || [];
    state.bag.topics = payload.topics || [];
    state.bag.topicsLoaded = true;
  } catch (error) {
    state.bag.error = error.message;
    throw error;
  } finally {
    state.bag.loading = false;
    renderBagTopics();
  }
}

function renderBagTopics() {
  if (state.bag.loading) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message">Loading topics...</div>`;
    return;
  }
  if (state.bag.error) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message bad">${state.bag.error}</div>`;
    return;
  }
  if (state.bag.topics.length === 0) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message">No topics discovered.</div>`;
    return;
  }

  const defaults = new Set(state.bag.defaultTopics);
  $("bag-topic-list").innerHTML = state.bag.topics
    .map((topic) => {
      const checked = defaults.has(topic) ? "checked" : "";
      return `
        <label>
          <input type="checkbox" value="${topic}" ${checked} />
          <span>${topic}</span>
        </label>`;
    })
    .join("");
}

async function startBag() {
  if (!state.bag.topicsLoaded) {
    await loadBagTopics();
  }
  const mode = document.querySelector('input[name="bag-mode"]:checked')?.value || "selected";
  const recordAll = mode === "all";
  const topics = Array.from(document.querySelectorAll("#bag-topic-list input:checked")).map(
    (input) => input.value,
  );
  await postJson("/api/bag/start", {
    record_all: recordAll,
    topics,
  });
}

async function analyzeBag() {
  state.bag.analysis.running = true;
  state.bag.analysis.error = "";
  renderBagAnalysis();
  try {
    const payload = await postJson("/api/bag/analyze", {});
    state.bag.analysis.result = payload.analysis || null;
    renderStatus(payload);
  } catch (error) {
    state.bag.analysis.error = error.message;
    throw error;
  } finally {
    state.bag.analysis.running = false;
    renderBagAnalysis();
  }
}

function renderBagAnalysis() {
  const analysis = state.bag.analysis;
  $("analyze-bag").disabled = analysis.running;
  if (analysis.running) {
    $("bag-analysis-state").textContent = "Analyzing";
    return;
  }

  const result = analysis.result;
  if (analysis.error) {
    $("bag-analysis-state").textContent = "Failed";
    $("bag-analysis-summary").innerHTML = `<div class="analysis-message bad">${escapeHtml(analysis.error)}</div>`;
    $("bag-analysis-output").textContent = analysis.error;
    drawAnalysisCanvas(null);
    return;
  }
  if (!result) {
    $("bag-analysis-state").textContent = "--";
    $("bag-analysis-summary").innerHTML = "";
    $("bag-analysis-output").textContent = "--";
    drawAnalysisCanvas(null);
    return;
  }

  const status = result.assessment?.status || "--";
  $("bag-analysis-state").textContent = status.toUpperCase();
  $("bag-analysis-summary").innerHTML = analysisSummaryHtml(result);
  $("bag-analysis-output").textContent = analysisText(result);
  drawAnalysisCanvas(result);
}

function analysisSummaryHtml(result) {
  const filtered = result.odometry?.filtered || {};
  const dvlOdom = result.odometry?.dvl_odom || {};
  const dvlDr = result.odometry?.dvl_dr || {};
  const dvlData = result.dvl?.data || {};
  const dvlTwist = result.dvl?.twist || {};
  return [
    analysisMetric("Filtered Length", fmtUnit(filtered.path_length_m, "m")),
    analysisMetric("Closure", fmtUnit(filtered.start_end_error_m, "m")),
    analysisMetric("Est. Laps", fmt(Math.abs(filtered.estimated_laps || 0), 2)),
    analysisMetric("DVL Valid", fmtPercent(dvlData.valid_rate)),
    analysisMetric("DVL FOM", fmt(dvlData.fom?.mean, 3)),
    analysisMetric("DVL Alt Min", fmtUnit(dvlData.altitude_m?.min, "m", 2)),
    analysisMetric("DVL Odom Length", fmtUnit(dvlOdom.path_length_m, "m")),
    analysisMetric("DVL DR Length", fmtUnit(dvlDr.path_length_m, "m")),
    analysisMetric("Twist Speed Max", fmtUnit(dvlTwist.speed_mps?.max, "m/s", 2)),
  ].join("");
}

function analysisMetric(label, value) {
  return `
    <div>
      <span>${escapeHtml(label)}</span>
      <strong>${escapeHtml(value || "--")}</strong>
    </div>`;
}

function analysisText(result) {
  const lines = [
    `Bag: ${result.bag_path || "--"}`,
    `Report Dir: ${result.report_dir || "--"}`,
    `Result File: ${result.result_path || "--"}`,
    `Image File: ${result.image_path || "--"}`,
    `Generated: ${result.generated_at || "--"}`,
    "",
    "Notes:",
  ];
  const notes = result.assessment?.notes || [];
  if (notes.length === 0) {
    lines.push("OK");
  } else {
    notes.forEach((note) => {
      lines.push(`${String(note.level || "info").toUpperCase()} ${note.message || ""}`);
    });
  }
  return lines.join("\n");
}

function drawAnalysisCanvas(result) {
  const canvas = $("analysis-canvas");
  const { ctx, width, height } = resizePathCanvas(canvas);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b0f0e";
  ctx.fillRect(0, 0, width, height);

  if (!result) return;

  const tracks = [
    { name: "filtered", label: "EKF", color: "#6ec6ff", points: result.samples?.filtered || [] },
    { name: "dvl_odom", label: "DVL Odom", color: "#52d273", points: result.samples?.dvl_odom || [] },
    { name: "dvl_dr", label: "DVL DR", color: "#f0b84f", points: result.samples?.dvl_dr || [] },
  ].filter((track) => track.points.length > 1);
  if (tracks.length === 0) return;

  const allPoints = tracks.flatMap((track) => track.points);
  const xs = allPoints.map((point) => point.x);
  const ys = allPoints.map((point) => point.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 1);
  const spanY = Math.max(maxY - minY, 1);
  const view = {
    centerX: (minX + maxX) / 2,
    centerY: (minY + maxY) / 2,
    pixelsPerMeter: clamp(0.78 * Math.min(width / spanX, height / spanY), 2, 240),
  };

  drawPathGrid(ctx, view, width, height, { gridStep: niceGridStep(Math.max(spanX, spanY)) });
  drawPathAxes(ctx, view, width, height);
  tracks.forEach((track) => drawAnalysisTrack(ctx, track, view, width, height));
  drawAnalysisLegend(ctx, tracks);
}

function drawAnalysisTrack(ctx, track, view, width, height) {
  ctx.save();
  ctx.lineWidth = track.name === "filtered" ? 2.5 : 1.8;
  ctx.strokeStyle = track.color;
  ctx.globalAlpha = track.name === "filtered" ? 1 : 0.78;
  ctx.beginPath();
  track.points.forEach((point, index) => {
    const screen = pathToScreen(point, view, width, height);
    if (index === 0) ctx.moveTo(screen.x, screen.y);
    else ctx.lineTo(screen.x, screen.y);
  });
  ctx.stroke();

  const first = pathToScreen(track.points[0], view, width, height);
  const last = pathToScreen(track.points[track.points.length - 1], view, width, height);
  ctx.globalAlpha = 1;
  ctx.fillStyle = track.color;
  ctx.beginPath();
  ctx.arc(first.x, first.y, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = track.color;
  ctx.strokeRect(last.x - 4, last.y - 4, 8, 8);
  ctx.restore();
}

function drawAnalysisLegend(ctx, tracks) {
  ctx.save();
  ctx.font = "12px sans-serif";
  ctx.textBaseline = "top";
  let x = 12;
  const y = 12;
  tracks.forEach((track) => {
    ctx.fillStyle = track.color;
    ctx.fillRect(x, y + 4, 16, 3);
    ctx.fillStyle = "#dce8e1";
    ctx.fillText(track.label, x + 22, y);
    x += ctx.measureText(track.label).width + 54;
  });
  ctx.restore();
}

function niceGridStep(span) {
  if (span <= 2) return 0.25;
  if (span <= 5) return 0.5;
  if (span <= 12) return 1;
  if (span <= 30) return 2;
  return 5;
}

async function runLocalizationTest() {
  state.test.running = true;
  state.test.message = "Running";
  state.test.steps = [];
  renderTestState();
  try {
    const payload = await postJson("/api/test/start", {});
    const test = payload.test || {};
    state.test.message = test.message || "Ready";
    state.test.steps = test.steps || [];
    renderStatus(payload);
  } catch (error) {
    state.test.message = "Failed";
    throw error;
  } finally {
    state.test.running = false;
    renderTestState();
  }
}

function renderTestState() {
  $("test-state").textContent = state.test.running ? "Running" : state.test.message;
  $("test-start").disabled = state.test.running;
}

function renderDvl(config, events) {
  const hasConfig = Object.keys(config).length > 0;
  $("dvl-updated").textContent = config.updated_at || "--";
  $("dvl-range").textContent = config.range_mode || "--";
  $("dvl-acoustic").textContent =
    typeof config.acoustic_enabled === "boolean" ? (config.acoustic_enabled ? "ON" : "OFF") : "--";
  $("dvl-dark").textContent =
    typeof config.dark_mode_enabled === "boolean" ? (config.dark_mode_enabled ? "ON" : "OFF") : "--";
  $("dvl-sound").textContent =
    typeof config.speed_of_sound === "number" ? `${config.speed_of_sound} m/s` : "--";
  $("dvl-rotation").textContent =
    typeof config.mounting_rotation_offset === "number" ? `${config.mounting_rotation_offset} deg` : "--";
  $("dvl-error").textContent = config.error_message || "--";
  $("dvl-config-json").textContent = hasConfig ? JSON.stringify(dvlConfigPayload(config), null, 2) : "--";

  const last = events[events.length - 1];
  if (last) {
    const name = last.parameter_name ? `${last.command}.${last.parameter_name}` : last.command;
    $("dvl-last").textContent = `${last.success ? "OK" : "FAIL"} ${name}`;
  } else {
    $("dvl-last").textContent = "--";
  }

  $("dvl-events").textContent = events
    .slice(-12)
    .map((event) => {
      const name = event.parameter_name ? `${event.command}.${event.parameter_name}` : event.command;
      const value = event.parameter_value ? ` ${event.parameter_value}` : "";
      const result = event.success ? "OK" : `FAIL ${event.error_message || ""}`.trim();
      const label = event.type === "config" ? "config received" : event.type;
      return `${event.time} ${label} ${name}${value} ${result}`;
    })
    .join("\n");
}

async function loadEkfConfig() {
  $("ekf-save-state").textContent = "Loading";
  const payload = await getJson("/api/ekf/process_noise");
  state.ekf.values = payload.values || [];
  state.ekf.stateNames = payload.state_names || [];
  state.ekf.size = payload.size || 15;
  $("ekf-path").textContent = payload.path || "--";
  $("ekf-save-state").textContent = "Loaded";
  renderEkfEditor();
}

async function saveEkfConfig() {
  const values = readEkfMatrix();
  $("ekf-save-state").textContent = "Saving";
  const payload = await postJson("/api/ekf/process_noise", { values });
  state.ekf.values = payload.values || values;
  $("ekf-save-state").textContent = "Saved";
  renderEkfEditor();
}

function renderEkfEditor() {
  const { values, stateNames, size } = state.ekf;
  if (values.length !== size * size) return;

  $("ekf-diagonal").innerHTML = stateNames
    .map((name, index) => {
      const value = values[index * size + index];
      return `
        <label>
          <span>${name}</span>
          <input type="number" step="0.001" data-ekf-row="${index}" data-ekf-col="${index}" value="${value}" />
        </label>`;
    })
    .join("");

  const header = [
    '<div class="ekf-cell ekf-corner"></div>',
    ...stateNames.map((name) => `<div class="ekf-cell ekf-heading">${name}</div>`),
  ].join("");

  const rows = [];
  for (let row = 0; row < size; row += 1) {
    rows.push(`<div class="ekf-cell ekf-heading">${stateNames[row]}</div>`);
    for (let col = 0; col < size; col += 1) {
      const value = values[row * size + col];
      rows.push(
        `<input class="ekf-cell ekf-input" type="number" step="0.001" data-ekf-row="${row}" data-ekf-col="${col}" value="${value}" />`,
      );
    }
  }
  $("ekf-matrix").innerHTML = header + rows.join("");

  document.querySelectorAll("[data-ekf-row]").forEach((input) => {
    input.addEventListener("change", syncEkfInputs);
  });
}

function syncEkfInputs(event) {
  const row = Number(event.target.dataset.ekfRow);
  const col = Number(event.target.dataset.ekfCol);
  const size = state.ekf.size;
  const value = Number(event.target.value);
  state.ekf.values[row * size + col] = Number.isFinite(value) ? value : 0;
  document.querySelectorAll(`[data-ekf-row="${row}"][data-ekf-col="${col}"]`).forEach((input) => {
    if (input !== event.target) input.value = event.target.value;
  });
}

function readEkfMatrix() {
  const { size } = state.ekf;
  const values = [...state.ekf.values];
  document.querySelectorAll("#ekf-matrix .ekf-input").forEach((input) => {
    const row = Number(input.dataset.ekfRow);
    const col = Number(input.dataset.ekfCol);
    const value = Number(input.value);
    values[row * size + col] = Number.isFinite(value) ? value : 0;
  });
  return values;
}

function dvlConfigPayload(config) {
  return {
    range_mode: config.range_mode,
    acoustic_enabled: config.acoustic_enabled,
    dark_mode_enabled: config.dark_mode_enabled,
    speed_of_sound: config.speed_of_sound,
    mounting_rotation_offset: config.mounting_rotation_offset,
    response_to: config.response_to,
    success: config.success,
    error_message: config.error_message,
    format: config.format,
    type: config.type,
    updated_at: config.updated_at,
  };
}

function renderTopics(topics) {
  const rows = Object.values(topics).map((topic) => {
    const age = typeof topic.age === "number" ? `${fmt(topic.age, 2)}s` : "--";
    const hz = `${fmt(topic.hz, 1)} Hz`;
    const cls = topic.alive ? "alive" : "stale";
    return `
      <div class="topic ${cls}">
        <strong>${topic.name}</strong>
        <span>${topic.alive ? "alive" : "stale"} · ${hz} · age ${age}</span>
      </div>`;
  });
  $("topic-list").innerHTML = rows.join("");
}

const PATH_OPTIONS_KEY = "kmu26-auv-web-gui-path-options";

function bindPathControls() {
  loadPathOptions();
  syncPathControls();

  [
    "path-view-mode",
    "path-color",
    "path-width",
    "path-scale",
    "path-grid-step",
    "path-show-grid",
    "path-show-axes",
    "path-show-robot",
  ].forEach((id) => {
    $(id).addEventListener("input", updatePathOptionsFromControls);
    $(id).addEventListener("change", updatePathOptionsFromControls);
  });

  $("path-clear").addEventListener("click", () => {
    state.path.points = [];
    renderPath();
    postJson("/api/path/clear").catch(showError);
  });
  $("path-set-origin").addEventListener("click", () => {
    postJson("/api/localization/set_origin").then(renderStatus).catch(showError);
  });

  window.addEventListener("resize", renderPath);
}

function loadPathOptions() {
  try {
    const saved = JSON.parse(localStorage.getItem(PATH_OPTIONS_KEY) || "{}");
    state.path.options = { ...state.path.options, ...saved };
  } catch (_) {
    // Keep defaults when saved options are not parseable.
  }
}

function savePathOptions() {
  try {
    localStorage.setItem(PATH_OPTIONS_KEY, JSON.stringify(state.path.options));
  } catch (_) {
    // The view still updates when browser storage is unavailable.
  }
}

function syncPathControls() {
  const options = state.path.options;
  $("path-view-mode").value = options.viewMode;
  $("path-color").value = options.color;
  $("path-width").value = options.width;
  $("path-scale").value = options.scale;
  $("path-grid-step").value = options.gridStep;
  $("path-show-grid").checked = options.showGrid;
  $("path-show-axes").checked = options.showAxes;
  $("path-show-robot").checked = options.showRobot;
  renderPathOptionValues();
}

function updatePathOptionsFromControls() {
  state.path.options = {
    viewMode: $("path-view-mode").value,
    color: $("path-color").value,
    width: clamp(Number($("path-width").value), 1, 12),
    scale: clamp(Number($("path-scale").value), 5, 200),
    gridStep: clamp(Number($("path-grid-step").value), 0.1, 20),
    showGrid: $("path-show-grid").checked,
    showAxes: $("path-show-axes").checked,
    showRobot: $("path-show-robot").checked,
  };
  renderPathOptionValues();
  savePathOptions();
  renderPath();
}

function renderPathOptionValues() {
  $("path-width-value").textContent = `${fmt(state.path.options.width, 1)} px`;
  $("path-scale-value").textContent = `${fmt(state.path.options.scale, 0)} px/m`;
}

function clamp(value, min, max) {
  if (!Number.isFinite(value)) return min;
  return Math.min(max, Math.max(min, value));
}

function resizePathCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function renderPath() {
  const points = state.path.points;
  const options = state.path.options;
  const canvas = $("path-canvas");
  const { ctx, width, height } = resizePathCanvas(canvas);
  const view = computePathView(points, state.path.pose, options, width, height);

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b0f0e";
  ctx.fillRect(0, 0, width, height);

  if (options.showGrid) drawPathGrid(ctx, view, width, height, options);
  if (options.showAxes) drawPathAxes(ctx, view, width, height);
  drawPathTrail(ctx, points, view, options);
  if (options.showRobot) drawRobotMarker(ctx, state.path.pose, view);

  $("path-point-count").textContent = `${points.length} points`;
  $("path-view-state").textContent = `${view.label} · ${fmt(view.pixelsPerMeter, 1)} px/m`;
}

function computePathView(points, pose, options, width, height) {
  if (options.viewMode === "fit" && points.length > 1) {
    const xs = points.map((p) => p.x);
    const ys = points.map((p) => p.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const spanX = Math.max(maxX - minX, options.gridStep);
    const spanY = Math.max(maxY - minY, options.gridStep);
    return {
      centerX: (minX + maxX) / 2,
      centerY: (minY + maxY) / 2,
      pixelsPerMeter: clamp(0.82 * Math.min(width / spanX, height / spanY), 5, 200),
      label: "Fit",
    };
  }

  if (options.viewMode === "follow" && points.length > 0) {
    const last = points[points.length - 1];
    return {
      centerX: last.x,
      centerY: last.y,
      pixelsPerMeter: options.scale,
      label: "Follow",
    };
  }

  return {
    centerX: 0,
    centerY: 0,
    pixelsPerMeter: options.scale,
    label: "Map",
  };
}

function pathToScreen(point, view, width, height) {
  return {
    x: width / 2 + (point.x - view.centerX) * view.pixelsPerMeter,
    y: height / 2 - (point.y - view.centerY) * view.pixelsPerMeter,
  };
}

function drawPathGrid(ctx, view, width, height, options) {
  const step = options.gridStep;
  const left = view.centerX - width / 2 / view.pixelsPerMeter;
  const right = view.centerX + width / 2 / view.pixelsPerMeter;
  const bottom = view.centerY - height / 2 / view.pixelsPerMeter;
  const top = view.centerY + height / 2 / view.pixelsPerMeter;
  const startX = Math.floor(left / step) * step;
  const startY = Math.floor(bottom / step) * step;

  ctx.lineWidth = 1;
  for (let x = startX; x <= right; x += step) {
    const screen = pathToScreen({ x, y: 0 }, view, width, height);
    const major = Math.abs(Math.round(x / step)) % 5 === 0;
    ctx.strokeStyle = major ? "#30413a" : "#22302b";
    ctx.beginPath();
    ctx.moveTo(screen.x, 0);
    ctx.lineTo(screen.x, height);
    ctx.stroke();
  }
  for (let y = startY; y <= top; y += step) {
    const screen = pathToScreen({ x: 0, y }, view, width, height);
    const major = Math.abs(Math.round(y / step)) % 5 === 0;
    ctx.strokeStyle = major ? "#30413a" : "#22302b";
    ctx.beginPath();
    ctx.moveTo(0, screen.y);
    ctx.lineTo(width, screen.y);
    ctx.stroke();
  }
}

function drawPathAxes(ctx, view, width, height) {
  const origin = pathToScreen({ x: 0, y: 0 }, view, width, height);
  ctx.lineWidth = 1.5;
  if (origin.y >= 0 && origin.y <= height) {
    ctx.strokeStyle = "#9d3343";
    ctx.beginPath();
    ctx.moveTo(0, origin.y);
    ctx.lineTo(width, origin.y);
    ctx.stroke();
  }
  if (origin.x >= 0 && origin.x <= width) {
    ctx.strokeStyle = "#2a8e65";
    ctx.beginPath();
    ctx.moveTo(origin.x, 0);
    ctx.lineTo(origin.x, height);
    ctx.stroke();
  }
}

function drawPathTrail(ctx, points, view, options) {
  if (points.length < 2) return;
  const canvas = $("path-canvas");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const stride = Math.max(1, Math.ceil(points.length / 20000));

  ctx.strokeStyle = options.color;
  ctx.lineWidth = options.width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  let started = false;
  points.forEach((point, index) => {
    if (index % stride !== 0 && index !== points.length - 1) return;
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;
    const screen = pathToScreen(point, view, width, height);
    if (!started) {
      ctx.moveTo(screen.x, screen.y);
      started = true;
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.stroke();
}

function drawRobotMarker(ctx, pose, view) {
  const canvas = $("path-canvas");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const center = pathToScreen(pose, view, width, height);
  const yaw = Number.isFinite(pose.yaw) ? pose.yaw : 0;
  const size = 9;

  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#52d273";
  ctx.strokeStyle = "#d8ffe3";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(size + 3, 0);
  ctx.lineTo(-size, -size * 0.65);
  ctx.lineTo(-size * 0.55, 0);
  ctx.lineTo(-size, size * 0.65);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function showError(error) {
  const line = `${new Date().toLocaleTimeString()} ${error.message}`;
  $("log-output").textContent = `${$("log-output").textContent}\n${line}`.trim();
}

bindControls();
connectStatusSocket();
