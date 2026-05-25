const statusGrid = document.getElementById("statusGrid");
const statusMeters = document.getElementById("statusMeters");
const stateStrip = document.getElementById("stateStrip");
const keyStats = document.getElementById("keyStats");
const nodeStatus = document.getElementById("nodeStatus");
const connection = document.getElementById("connection");
const liveClock = document.getElementById("liveClock");
const telloBadge = document.getElementById("telloBadge");
const jetsonBadge = document.getElementById("jetsonBadge");
const radar = document.getElementById("radar");
const radarCtx = radar.getContext("2d");
const radarReadout = document.getElementById("radarReadout");
const graphs = document.getElementById("graphs");
const graphCtx = graphs.getContext("2d");
const events = document.getElementById("events");
const graphLegend = document.getElementById("graphLegend");
const hitOverlay = document.getElementById("hitOverlay");
const hitOverlayDetail = document.getElementById("hitOverlayDetail");

const GRAPH_SERIES = [
  { key: "fps", label: "YOLO FPS", min: 0, max: 35, color: "#53a7ff" },
  { key: "tracking_error_px", label: "Tracking Error", min: 0, max: 720, color: "#ff8a4c" },
  { key: "latency_ms", label: "Latency", min: 0, max: 500, color: "#f0c84b" },
  { key: "audio_confidence", label: "Audio Confidence", min: 0, max: 1, color: "#40d8d0" },
];

const CAMERA_FOV_DEG = 90;
const CAMERA_FOV_HALF_DEG = CAMERA_FOV_DEG / 2;
const MOTOR_RING_SCALE = 0.9;
const PAN_TICK_RANGE = 4096;
const DEFAULT_FRONT_PAN_TICK = 2048;
const PAN_DIRECTION = 1;
const STATE_STEPS = ["IDLE", "SEARCH", "TRACK", "LOCK", "FIRE"];

const GRAPH_RENDER_INTERVAL_MS = 250;
const STATUS_RENDER_INTERVAL_MS = 50;

let reconnectTimer = null;
let lastSnapshot = null;
let pendingSnapshot = null;
let renderQueued = false;
let lastRenderAt = -Infinity;
let lastGraphRenderAt = -Infinity;
let lastHistorySignature = "";
let lastEventsSignature = "";
let lastMotorDirection = null;
let lastHitCount = null;
let lastLaserHit = false;
let hitFlashUntil = 0;
let lastHitDetail = "No hit recorded";

renderLegend();
tickClock();
setInterval(tickClock, 250);

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    connection.textContent = "WebSocket: connected";
  };

  ws.onmessage = (event) => {
    pendingSnapshot = JSON.parse(event.data);
    scheduleRender();
  };

  ws.onclose = () => {
    connection.textContent = "WebSocket: disconnected, reconnecting";
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, 1000);
    }
  };
}

function scheduleRender() {
  if (renderQueued) {
    return;
  }
  renderQueued = true;
  requestAnimationFrame(renderPendingSnapshot);
}

function renderPendingSnapshot() {
  renderQueued = false;
  if (!pendingSnapshot) {
    return;
  }
  const snapshot = pendingSnapshot;
  pendingSnapshot = null;
  lastSnapshot = snapshot;
  render(snapshot);
}

function render(snapshot) {
  const now = performance.now();
  updateHitVisualization(snapshot);
  if (now - lastRenderAt >= STATUS_RENDER_INTERVAL_MS || isHitFlashActive()) {
    updateStatus(snapshot);
    drawRadar(snapshot.tracking);
    lastRenderAt = now;
  }

  const history = snapshot.history || [];
  const historySignature = sampleSignature(history);
  if (historySignature !== lastHistorySignature && now - lastGraphRenderAt >= GRAPH_RENDER_INTERVAL_MS) {
    drawGraphs(history);
    lastHistorySignature = historySignature;
    lastGraphRenderAt = now;
  }

  const eventItems = snapshot.events || [];
  const eventsSignature = sampleSignature(eventItems);
  if (eventsSignature !== lastEventsSignature) {
    drawEvents(eventItems);
    lastEventsSignature = eventsSignature;
  }
}

function updateStatus(snapshot) {
  const tello = snapshot.tello || {};
  const tracking = snapshot.tracking || {};
  const audio = tracking.audio || {};
  const laser = tracking.laser || {};
  const latest = latestHistorySample(snapshot);
  const motorDirection = motorDirectionDeg(tracking);
  const telemetryRate = typeof latest.telemetry_rate_hz === "number" ? latest.telemetry_rate_hz : null;
  const confidence = numberOr(tracking.confidence, latest.confidence);
  const fps = numberOr(tracking.fps, latest.fps);
  const audioActive = audio.fallback_active === true;
  const audioConfidence = audioActive ? numberOr(audio.confidence, latest.audio_confidence) : null;
  const trackingError = trackingErrorPx(tracking, latest);
  const latency = numberOr(latest.latency_ms, null);
  const lockState = displayState(tracking, laser);
  const vision = visionState(tracking);
  const audioAssist = audioAssistState(tracking);
  const audioAlignment = audioActive ? angleDeltaDeg(audioTargetDeg(audio), motorDirection) : null;
  const fire = fireState(laser);
  const telemetry = telemetryState(snapshot);
  const bboxSize = bboxAreaRatio(tracking);
  const hitActive = isHitFlashActive() || Boolean(laser.hit_detected);
  const hitCount = snapshot.hit_count ?? latest.hit_count ?? 0;

  const meters = [
    statusMeter("Drone Battery", tello.battery, 0, 100, "%", "#f0c84b"),
    statusMeter("YOLO Confidence", confidence, 0, 1, "", "#43d17a"),
    statusMeter("Audio Confidence", audioConfidence, 0, 1, "", "#40d8d0"),
    statusMeter("Telemetry Freshness", telemetry.score, 0, 100, "", "#b3e36a", telemetry.detail),
  ];

  const rows = [
    statusText("Tello connected", yesNo(tello.connected)),
    statusText("Airborne", yesNo(tello.airborne)),
    statusText("Speed mode", formatValue(tello.speed, " cm/s")),
    statusText("Last command", tello.last_command || "-"),
    statusText("Jetson telemetry", telemetry.label),
    statusText("Jetson state", String(tracking.state || "-").toUpperCase()),
    statusText("Telemetry rate", formatValue(telemetryRate, " Hz")),
    statusText("Last received", age(snapshot.last_received_age)),
    statusText("Motor heading", formatDegrees(motorDirection)),
    statusText("Audio target", audioActive ? formatDegrees(audioTargetDeg(audio)) : "-"),
    statusText("Audio alignment", audioActive ? formatSignedDegrees(audioAlignment) : "-"),
    statusText("Audio status", audioAssist.detail),
    statusText("Fire result", fire.detail),
    statusText("Target size", formatRatio(bboxSize)),
  ];

  const stats = [
    keyStat("Vision", vision.label, vision.className),
    keyStat("Audio Assist", audioAssist.label, audioAssist.className),
    keyStat("Fire Result", fire.label, fire.className),
    keyStat("Laser Output", laser.hit_detected ? "HIT" : laser.armed ? "ON" : "OFF", hitActive ? "danger hit-pulse" : laser.armed ? "danger" : "muted"),
    keyStat("Hits", hitCount, hitActive ? "danger hit-pulse" : "success"),
    keyStat("Shots", snapshot.shot_count ?? latest.shot_count ?? 0, "warn"),
    keyStat("Error", formatValue(trackingError, " px"), trackingError !== null && trackingError <= 80 ? "success" : "warn"),
    keyStat("Latency", formatValue(latency, " ms"), latency !== null && latency <= 120 ? "success" : "warn"),
  ];

  statusMeters.innerHTML = meters.map(renderStatusMeter).join("");
  statusGrid.innerHTML = rows.map(renderStatusRow).join("");
  keyStats.innerHTML = stats.map(renderKeyStat).join("");
  stateStrip.innerHTML = STATE_STEPS.map((step) => renderStateStep(step, lockState)).join("");
  nodeStatus.innerHTML = renderNodes(snapshot, tracking);

  const telloConnected = Boolean(tello.connected);
  telloBadge.textContent = telloConnected ? "TELLO ON" : "TELLO OFF";
  telloBadge.className = `badge tello ${telloConnected ? "connected" : "disconnected"}`;

  const status = (snapshot.jetson_status || "DISCONNECTED").toLowerCase();
  jetsonBadge.textContent = telemetry.badge;
  jetsonBadge.className = `badge ${status}`;
}

function drawRadar(tracking) {
  const width = radar.width;
  const height = radar.height;
  const cx = width / 2;
  const cy = height / 2;
  const radius = Math.min(width, height) / 2 - 24;

  radarCtx.clearRect(0, 0, width, height);
  radarCtx.fillStyle = "#111820";
  radarCtx.fillRect(0, 0, width, height);

  drawRadarGrid(cx, cy, radius);
  drawFov(cx, cy, radius, tracking);
  drawRadarCenter(cx, cy, isHitFlashActive() || Boolean(tracking && tracking.laser && tracking.laser.hit_detected));

  if (!tracking) {
    radarReadout.textContent = `Motor - | FOV ${CAMERA_FOV_DEG} deg | Bearing only`;
    return;
  }

  const motorDirection = motorDirectionDeg(tracking);
  const audio = tracking.audio || {};
  const lockState = displayState(tracking, tracking.laser || {});
  const showAudioBearing = shouldShowAudioBearing(tracking);
  const audioTarget = audioTargetDeg(audio);

  if (showAudioBearing) {
    drawAudioBearing(audioTarget, audio.confidence, radius);
  }
  if (typeof motorDirection === "number") {
    drawMotorMarker(motorDirection, lockState, radius * MOTOR_RING_SCALE, tracking && tracking.laser && tracking.laser.hit_detected);
  }
  drawRadarLegend(showAudioBearing);

  radarReadout.textContent = showAudioBearing
    ? `Motor heading ${formatDegrees(motorDirection)} | FOV ${CAMERA_FOV_DEG} deg | Audio target ${formatDegrees(audioTarget)}`
    : `Motor heading ${formatDegrees(motorDirection)} | FOV ${CAMERA_FOV_DEG} deg`;
}

function drawRadarGrid(cx, cy, radius) {
  radarCtx.strokeStyle = "#36404a";
  radarCtx.lineWidth = 1;
  for (const scale of [0.33, 0.66, 1]) {
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, radius * scale, 0, Math.PI * 2);
    radarCtx.stroke();
  }

  for (const deg of [0, 45, 90, 135, 180, 225, 270, 315]) {
    const end = directionPoint(cx, cy, deg, radius);
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.lineTo(end.x, end.y);
    radarCtx.stroke();
  }

  drawCompassLabels(cx, cy, radius);
}

function drawFov(cx, cy, radius, tracking) {
  const center = motorDirectionDeg(tracking || {});
  if (typeof center !== "number") {
    return;
  }
  const normalizedCenter = radarDisplayDegrees(center);
  const start = bearingToCanvasRadians(normalizedCenter + CAMERA_FOV_HALF_DEG);
  const end = bearingToCanvasRadians(normalizedCenter - CAMERA_FOV_HALF_DEG);
  radarCtx.fillStyle = "rgba(83, 167, 255, 0.14)";
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.arc(cx, cy, radius, start, end, false);
  radarCtx.closePath();
  radarCtx.fill();
  radarCtx.strokeStyle = "rgba(83, 167, 255, 0.55)";
  radarCtx.lineWidth = 2;
  radarCtx.stroke();

  const centerLine = directionPoint(cx, cy, normalizedCenter, radius * MOTOR_RING_SCALE);
  radarCtx.strokeStyle = "rgba(83, 167, 255, 0.85)";
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.lineTo(centerLine.x, centerLine.y);
  radarCtx.stroke();
}

function drawAudioBearing(targetDegrees, confidence, radius) {
  if (typeof targetDegrees !== "number") {
    return;
  }
  const cx = radar.width / 2;
  const cy = radar.height / 2;
  const alpha = typeof confidence === "number" ? 0.25 + Math.max(0, Math.min(1, confidence)) * 0.75 : 0.65;
  const displayTarget = radarDisplayDegrees(targetDegrees);
  const point = directionPoint(cx, cy, displayTarget, radius * 0.82);

  radarCtx.save();
  radarCtx.globalAlpha = alpha;
  radarCtx.strokeStyle = "#40d8d0";
  radarCtx.fillStyle = "#40d8d0";
  radarCtx.lineWidth = 4;
  radarCtx.setLineDash([8, 6]);
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.lineTo(point.x, point.y);
  radarCtx.stroke();
  radarCtx.setLineDash([]);

  const headA = directionPoint(point.x, point.y, displayTarget + 145, 12);
  const headB = directionPoint(point.x, point.y, displayTarget - 145, 12);
  radarCtx.beginPath();
  radarCtx.moveTo(point.x, point.y);
  radarCtx.lineTo(headA.x, headA.y);
  radarCtx.lineTo(headB.x, headB.y);
  radarCtx.closePath();
  radarCtx.fill();
  radarCtx.restore();
}

function drawRadarLegend(audioVisible) {
  if (!audioVisible) {
    return;
  }
  radarCtx.save();
  radarCtx.font = "11px system-ui";
  radarCtx.textAlign = "left";
  radarCtx.fillStyle = "#40d8d0";
  radarCtx.fillText("Audio search target", 16, radar.height - 16);
  radarCtx.restore();
}

function drawMotorMarker(degrees, lockState, length, laserHit = false) {
  const cx = radar.width / 2;
  const cy = radar.height / 2;
  const point = directionPoint(cx, cy, radarDisplayDegrees(degrees), length);
  const activeHit = laserHit || isHitFlashActive();
  const locked = lockState === "LOCKED" || lockState === "FIRING" || activeHit;
  const color = activeHit ? "#ff3b3b" : locked ? "#ff5f5f" : "#f0c84b";

  radarCtx.fillStyle = color;
  radarCtx.strokeStyle = color;
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.lineTo(point.x, point.y);
  radarCtx.stroke();
  radarCtx.beginPath();
  radarCtx.arc(point.x, point.y, 7, 0, Math.PI * 2);
  radarCtx.fill();

  if (locked) {
    const pulse = activeHit ? 16 + (Date.now() % 700) / 18 : 10 + (Date.now() % 1000) / 100;
    radarCtx.strokeStyle = activeHit ? "rgba(255, 59, 59, 0.92)" : "rgba(255, 95, 95, 0.75)";
    radarCtx.lineWidth = activeHit ? 4 : 2;
    radarCtx.beginPath();
    radarCtx.arc(point.x, point.y, pulse, 0, Math.PI * 2);
    radarCtx.stroke();

    if (activeHit) {
      radarCtx.font = "bold 16px system-ui";
      radarCtx.textAlign = "center";
      radarCtx.fillStyle = "#ffb2a8";
      radarCtx.fillText("HIT", point.x, Math.max(20, point.y - pulse - 10));
      radarCtx.textAlign = "start";
    }
  }
}

function directionPoint(cx, cy, degrees, length) {
  const radians = (normalizeDegrees(degrees) * Math.PI) / 180;
  return {
    x: cx - Math.sin(radians) * length,
    y: cy - Math.cos(radians) * length,
  };
}

function bearingToCanvasRadians(degrees) {
  return (270 - normalizeDegrees(degrees)) * (Math.PI / 180);
}

function radarDisplayDegrees(degrees) {
  return normalizeDegrees(360 - degrees);
}

function drawRadarCenter(cx, cy, activeHit = false) {
  radarCtx.fillStyle = activeHit ? "#ff3b3b" : "#eef3f6";
  radarCtx.beginPath();
  radarCtx.arc(cx, cy, activeHit ? 6 : 4, 0, Math.PI * 2);
  radarCtx.fill();
  if (activeHit) {
    const pulse = 22 + (Date.now() % 650) / 10;
    radarCtx.strokeStyle = "rgba(255, 59, 59, 0.55)";
    radarCtx.lineWidth = 3;
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, pulse, 0, Math.PI * 2);
    radarCtx.stroke();
  }
}

function drawCompassLabels(cx, cy, radius) {
  radarCtx.fillStyle = "#9fb0bd";
  radarCtx.font = "12px system-ui";
  radarCtx.textAlign = "center";
  radarCtx.fillText("N 0", cx, cy - radius - 6);
  radarCtx.fillText("S 180", cx, cy + radius + 16);
  radarCtx.textAlign = "left";
  radarCtx.fillText("W 90", cx - radius + 8, cy - 8);
  radarCtx.textAlign = "right";
  radarCtx.fillText("E 270", cx + radius - 8, cy - 8);
  radarCtx.textAlign = "start";
}

function drawGraphs(history) {
  const width = graphs.width;
  const height = graphs.height;
  graphCtx.clearRect(0, 0, width, height);
  graphCtx.fillStyle = "#111820";
  graphCtx.fillRect(0, 0, width, height);

  graphCtx.strokeStyle = "#36404a";
  graphCtx.lineWidth = 1;
  for (let i = 1; i < 4; i += 1) {
    const y = (height / 4) * i;
    graphCtx.beginPath();
    graphCtx.moveTo(0, y);
    graphCtx.lineTo(width, y);
    graphCtx.stroke();
  }

  const recent = history.slice(-300);
  GRAPH_SERIES.forEach((series) => {
    drawSeries(recent, series.key, series.min, series.max, series.color);
  });
}

function drawSeries(samples, key, min, max, color) {
  if (samples.length < 2) {
    return;
  }
  graphCtx.strokeStyle = color;
  graphCtx.lineWidth = 2;
  graphCtx.beginPath();
  let started = false;
  samples.forEach((sample, index) => {
    const value = sample[key];
    if (typeof value !== "number") {
      return;
    }
    const x = (graphs.width * index) / Math.max(1, samples.length - 1);
    const normalized = Math.max(0, Math.min(1, (value - min) / (max - min)));
    const y = graphs.height - normalized * graphs.height;
    if (!started) {
      graphCtx.moveTo(x, y);
      started = true;
    } else {
      graphCtx.lineTo(x, y);
    }
  });
  graphCtx.stroke();
}

function drawEvents(items) {
  events.innerHTML = items
    .slice(-24)
    .reverse()
    .map((item, index) => {
      const ts = new Date(item.timestamp * 1000).toLocaleTimeString();
      const category = eventCategory(item.message);
      const fresh = index === 0 ? " fresh" : "";
      const hit = isHitEvent(item) ? " hit-event" : "";
      return `<div class="event ${item.level} ${category}${fresh}${hit}"><span>${ts}</span><span>${category}</span><span>${item.message}</span></div>`;
    })
    .join("");
}

function updateHitVisualization(snapshot) {
  const tracking = snapshot.tracking || {};
  const laser = tracking.laser || {};
  const latest = latestHistorySample(snapshot);
  const hitCount = snapshot.hit_count ?? latest.hit_count ?? 0;
  const laserHit = Boolean(laser.hit_detected);
  const countIncreased = lastHitCount !== null && hitCount > lastHitCount;
  const laserRising = laserHit && !lastLaserHit;

  if (countIncreased || laserRising) {
    triggerHitFlash(snapshot, tracking, hitCount);
  }

  lastHitCount = hitCount;
  lastLaserHit = laserHit;
  renderHitOverlay();
}

function triggerHitFlash(snapshot, tracking, hitCount) {
  hitFlashUntil = Date.now() + 2600;
  lastHitDetail = hitDetail(snapshot, tracking, hitCount);
  renderHitOverlay();
}

function renderHitOverlay() {
  if (!hitOverlay) {
    return;
  }
  const active = isHitFlashActive();
  hitOverlay.classList.toggle("active", active);
  if (hitOverlayDetail) {
    hitOverlayDetail.textContent = lastHitDetail;
  }
}

function isHitFlashActive() {
  return Date.now() < hitFlashUntil;
}

function hitDetail(snapshot, tracking, hitCount) {
  const parts = [];
  parts.push(`Hits ${hitCount}`);
  if (tracking.state) {
    parts.push(String(tracking.state).toUpperCase());
  }
  if (typeof tracking.confidence === "number") {
    parts.push(`conf ${tracking.confidence.toFixed(2)}`);
  }
  if (tracking.error && typeof tracking.error.x_px === "number" && typeof tracking.error.y_px === "number") {
    parts.push(`err ${Math.hypot(tracking.error.x_px, tracking.error.y_px).toFixed(0)} px`);
  }
  if (typeof tracking.frame_id === "number") {
    parts.push(`frame ${tracking.frame_id}`);
  }
  return parts.join(" | ");
}

function renderLegend() {
  graphLegend.innerHTML = GRAPH_SERIES.map(
    (series) =>
      `<span class="legend-item"><span class="legend-swatch" style="background:${series.color}"></span>${series.label}</span>`
  ).join("");
}

function latestHistorySample(snapshot) {
  const history = snapshot.history || [];
  return history.length ? history[history.length - 1] : {};
}

function sampleSignature(items) {
  if (!items.length) {
    return "0";
  }
  const last = items[items.length - 1];
  const timestamp = last && typeof last.timestamp !== "undefined" ? last.timestamp : "";
  const count = last && typeof last.hit_count !== "undefined" ? last.hit_count : "";
  const frame = last && typeof last.frame_id !== "undefined" ? last.frame_id : "";
  const message = last && typeof last.message !== "undefined" ? last.message : "";
  return `${items.length}:${timestamp}:${count}:${frame}:${message}`;
}

function renderStatusRow(row) {
  return `<dt>${row.label}</dt><dd>${row.value}</dd>`;
}

function renderStatusMeter(row) {
  const width = Number.isFinite(row.width) ? Math.max(0, Math.min(100, row.width)) : 0;
  return `
    <div class="status-meter">
      <div class="meter-label"><span>${row.label}</span><strong>${row.displayValue || row.value}</strong></div>
      <div class="meter-track">
        <div class="meter-fill" style="width:${width}%; background:${row.color}"></div>
      </div>
    </div>
  `;
}

function renderKeyStat(item) {
  return `<div class="key-stat ${item.className}"><span>${item.label}</span><strong>${item.value}</strong></div>`;
}

function renderStateStep(step, state) {
  const active = stateStepActive(step, state) ? `active ${stateClass(state)}` : "";
  return `<span class="state-step ${active}">${step}</span>`;
}

function renderNodes(snapshot, tracking) {
  const ultraPs = tracking.ultra_ps || tracking.ultraPs || tracking.ultraps;
  const nodes = [
    { label: "Tello", online: Boolean(snapshot.tello && snapshot.tello.connected) },
    { label: "Jetson", online: snapshot.jetson_status === "CONNECTED" },
    { label: "Ultra96", online: Boolean(ultraPs) && snapshot.jetson_status === "CONNECTED" },
    { label: "RPi", online: true },
  ];
  return nodes
    .map((node) => `<span class="node ${node.online ? "online" : "offline"}"><i></i>${node.label}</span>`)
    .join("");
}

function statusText(label, value) {
  return { label, value };
}

function statusMeter(label, value, min, max, suffix, color, displayValue = null) {
  const numeric = typeof value === "number" ? value : null;
  const formattedValue = numeric === null ? "-" : formatValue(numeric, suffix);
  const width = numeric === null ? 0 : ((numeric - min) / (max - min)) * 100;
  return {
    label,
    value: formattedValue,
    displayValue,
    width,
    color,
  };
}

function keyStat(label, value, className) {
  return { label, value, className };
}

function numberOr(...values) {
  return values.find((value) => typeof value === "number") ?? null;
}

function trackingErrorPx(tracking, latest) {
  if (typeof latest.tracking_error_px === "number") {
    return latest.tracking_error_px;
  }
  const error = tracking.error || {};
  if (typeof error.x_px === "number" && typeof error.y_px === "number") {
    return Math.hypot(error.x_px, error.y_px);
  }
  return null;
}

function displayState(tracking, laser) {
  const raw = String(tracking.state || "").toUpperCase();
  const fire = laser.fire || {};
  if (laser.hit_detected || laser.fired || fire.active || raw.includes("FIR")) {
    return "FIRING";
  }
  if (raw.includes("LOCK")) {
    return "LOCKED";
  }
  if (raw.includes("TRACK") || raw.includes("DETECT")) {
    return "TRACKING";
  }
  if (raw.includes("SEARCH") || raw.includes("SCAN") || raw.includes("LOST")) {
    return "SEARCHING";
  }
  return "IDLE";
}

function visionState(tracking) {
  const raw = String(tracking.state || "").toUpperCase();
  const found = tracking.target_found === true;
  if (raw.includes("LOCK")) {
    return { label: "LOCKED", className: "success" };
  }
  if (found && (raw.includes("TRACK") || raw.includes("DETECT"))) {
    return { label: raw.includes("DETECT") ? "DETECTED" : "TRACKING", className: "track" };
  }
  if (found) {
    return { label: "TARGET FOUND", className: "track" };
  }
  return { label: raw.includes("SCAN") ? "SCANNING" : "NO TARGET", className: "search" };
}

function audioAssistState(tracking) {
  const audio = tracking.audio || {};
  const enabled = audio.enabled === true;
  const active = audio.fallback_active === true;
  const status = typeof audio.status === "string" && audio.status ? audio.status : "";
  const confidence = formatRatio(typeof audio.confidence === "number" ? audio.confidence : null);
  if (!enabled) {
    return { label: "OFF", className: "muted", detail: status || "disabled" };
  }
  if (active) {
    const sector = audio.sector || "-";
    const delta = angleDeltaDeg(audioTargetDeg(audio), motorDirectionDeg(tracking));
    return { label: "ACTIVE", className: "warn", detail: status || `${sector} ${formatDegrees(audioTargetDeg(audio))} err ${formatSignedDegrees(delta)} conf ${confidence}` };
  }
  return { label: "VISION", className: "muted", detail: status || "hidden" };
}

function fireState(laser) {
  const fire = laser.fire || {};
  const rawResult = typeof fire.result === "string" && fire.result ? fire.result : "idle";
  const result = rawResult.toUpperCase();
  if (laser.hit_detected || result === "HIT") {
    return { label: "HIT", className: "danger hit-pulse", detail: result };
  }
  if (laser.fired || fire.active) {
    return { label: "ACTIVE", className: "warn", detail: result === "IDLE" ? "FIRE ACTIVE" : result };
  }
  if (result === "MISS") {
    return { label: "MISS", className: "warn", detail: result };
  }
  return { label: result, className: "muted", detail: result };
}

function telemetryState(snapshot) {
  const status = String(snapshot.jetson_status || "DISCONNECTED").toUpperCase();
  const ageSec = snapshot.last_received_age;
  const score = signalQuality(snapshot);
  const detail = typeof ageSec === "number" ? `${ageSec.toFixed(2)}s ago` : "-";
  if (status === "CONNECTED") {
    return { label: "LIVE", badge: `JETSON LIVE ${detail}`, detail, score };
  }
  if (status === "STALE") {
    return { label: "STALE", badge: `JETSON STALE ${detail}`, detail, score };
  }
  return { label: "DISCONNECTED", badge: "JETSON OFF", detail, score };
}

function stateClass(state) {
  if (state === "FIRING") return "danger";
  if (state === "LOCKED") return "success";
  if (state === "TRACKING") return "track";
  if (state === "SEARCHING") return "search";
  return "muted";
}

function stateStepActive(step, state) {
  const normalized =
    state === "SEARCHING" ? "SEARCH" : state === "TRACKING" ? "TRACK" : state === "LOCKED" ? "LOCK" : state === "FIRING" ? "FIRE" : state;
  return step === normalized;
}

function signalQuality(snapshot) {
  const ageSec = snapshot.last_received_age;
  if (typeof ageSec !== "number") {
    return 0;
  }
  return Math.max(0, Math.min(100, 100 - ageSec * 50));
}

function bboxAreaRatio(tracking) {
  const bbox = tracking.bbox || {};
  const frame = tracking.frame || {};
  if (typeof bbox.w !== "number" || typeof bbox.h !== "number" || typeof frame.width !== "number" || typeof frame.height !== "number") {
    return null;
  }
  return Math.max(0, Math.min(1, (bbox.w * bbox.h) / (frame.width * frame.height)));
}

function shouldShowAudioBearing(tracking) {
  const audio = tracking.audio || {};
  if (audio.fallback_active !== true) {
    return false;
  }
  return typeof audioTargetDeg(audio) === "number";
}

function audioTargetDeg(audio) {
  if (typeof audio.target_motor_deg === "number") {
    return normalizeDegrees(audio.target_motor_deg);
  }
  return null;
}

function angleDeltaDeg(target, current) {
  if (typeof target !== "number" || typeof current !== "number") {
    return null;
  }
  return ((normalizeDegrees(target) - normalizeDegrees(current) + 540) % 360) - 180;
}

function motorDirectionDeg(tracking) {
  const ultraPs = tracking.ultra_ps || tracking.ultraPs || tracking.ultraps || {};
  const tickHeading = motorDirectionFromTick(tracking, ultraPs);
  if (typeof tickHeading === "number") {
    return rememberMotorDirection(tickHeading);
  }
  if (typeof ultraPs.motor_deg === "number") {
    return rememberMotorDirection(ultraPs.motor_deg);
  }
  const fallback = tracking.ptz && tracking.ptz.pan_deg;
  return typeof fallback === "number" ? rememberMotorDirection(fallback) : lastMotorDirection;
}

function motorDirectionFromTick(tracking, ultraPs) {
  const ptz = tracking.ptz || {};
  const panTick = typeof ultraPs.pan_tick === "number" ? ultraPs.pan_tick : ptz.pan_cmd;
  if (typeof panTick !== "number") {
    return null;
  }
  if (tracking.target_found === false && isDefaultPanTick(panTick) && typeof lastMotorDirection === "number") {
    return null;
  }
  const frontPan = typeof ultraPs.front_pan === "number" ? ultraPs.front_pan : DEFAULT_FRONT_PAN_TICK;
  return normalizeDegrees(((frontPan - panTick) * 360 * PAN_DIRECTION) / PAN_TICK_RANGE);
}

function rememberMotorDirection(value) {
  const normalized = normalizeDegrees(value);
  lastMotorDirection = normalized;
  return normalized;
}

function isDefaultPanTick(value) {
  return Math.abs(value - DEFAULT_FRONT_PAN_TICK) <= 2;
}

function normalizeDegrees(value) {
  return ((Number(value) % 360) + 360) % 360;
}

function formatDegrees(value) {
  if (typeof value !== "number") {
    return "-";
  }
  return `${normalizeDegrees(value).toFixed(0).padStart(3, "0")} deg`;
}

function formatSignedDegrees(value) {
  if (typeof value !== "number") {
    return "-";
  }
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(0)} deg`;
}

function formatRatio(value) {
  if (typeof value !== "number") {
    return "-";
  }
  return `${(value * 100).toFixed(1)}%`;
}

function formatValue(value, suffix = "") {
  if (typeof value !== "number") {
    return "-";
  }
  return `${value.toFixed(Number.isInteger(value) ? 0 : 1)}${suffix}`;
}

function yesNo(value) {
  if (value === true) return "yes";
  if (value === false) return "no";
  return "-";
}

function age(value) {
  return typeof value === "number" ? `${value.toFixed(2)} sec` : "-";
}

function eventCategory(message) {
  const text = String(message || "").toUpperCase();
  if (text.includes("ERROR") || text.includes("DISCONNECTED")) return "ERROR";
  if (text.includes("WARN") || text.includes("TIMEOUT") || text.includes("LOST")) return "WARNING";
  if (text.includes("AUDIO")) return "AUDIO";
  if (text.includes("VISION") || text.includes("TARGET")) return "VISION";
  if (text.includes("MOTOR")) return "MOTOR";
  if (text.includes("LASER") || text.includes("HIT")) return "LASER";
  return "SYSTEM";
}

function isHitEvent(item) {
  const text = String((item && item.message) || "").toUpperCase();
  return text.includes("HIT");
}

function tickClock() {
  const clock = new Date().toLocaleTimeString();
  liveClock.textContent = `LIVE ${clock}`;
  renderHitOverlay();
  if (lastSnapshot && isHitFlashActive()) {
    drawRadar(lastSnapshot.tracking);
  }
}

connect();
