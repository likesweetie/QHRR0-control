const $ = (id) => document.getElementById(id);

let latestState = null;
let ws = null;
let dashboardConfig = null;

const fmt = {
  fixed(value, digits = 1, fallback = "-") {
    return Number.isFinite(value) ? value.toFixed(digits) : fallback;
  },
  age(value) {
    if (value === null || value === undefined) return "never";
    if (value < 1) return `${(value * 1000).toFixed(0)} ms`;
    return `${value.toFixed(2)} s`;
  },
  vector(values) {
    if (!Array.isArray(values)) return "-";
    return values.map((value) => this.fixed(value, 3)).join(", ");
  },
  maybe(value, digits = 2) {
    if (value === null || value === undefined) return "-";
    if (typeof value === "number") return value.toFixed(digits);
    return String(value);
  },
};

function badgeClass(status) {
  if (status === "online" || status === "connected") return "badge ok";
  if (status === "timeout" || status === "disconnected") return "badge danger";
  if (status === "stale") return "badge warn";
  return "badge muted";
}

function dotClass(status) {
  if (status === "online") return "status-dot online";
  if (status === "timeout") return "status-dot timeout";
  if (status === "stale") return "status-dot stale";
  return "status-dot";
}

function setText(id, value) {
  $(id).textContent = value;
}

function render(state) {
  latestState = state;
  renderTopbar(state);
  renderBus(state);
  renderImu(state);
  renderMjcfModel(state.motors || [], state.imu ? state.imu.quat_xyzw : null);
  renderNodes(state.nodes || []);
  renderMotors(state.motors || [], state.safety || {});
  renderEnabledMotorCards(state.motors || []);
  renderFrames(state.recent_frames || []);
  syncControls(state);
}

function hexCanId(value) {
  if (typeof value === "number") return `0x${value.toString(16).toUpperCase()}`;
  return String(value);
}

function normalizePayload(value) {
  return String(value || "").replaceAll(" ", "").toUpperCase();
}

function renderTopbar(state) {
  $("ifaceBadge").textContent = state.can.iface;
  $("socketBadge").textContent = state.can.socket_status;
  $("socketBadge").className = badgeClass(state.can.socket_status);
  $("txBadge").textContent = state.safety.tx_enabled ? "TX ENABLED" : "TX LOCKED";
  $("txBadge").className = state.safety.tx_enabled ? "badge warn" : "badge danger";
}

function renderBus(state) {
  const load = Math.max(0, state.can.load_percent || 0);
  const width = Math.min(load, 100);
  setText("loadPercent", `${fmt.fixed(load, 1)}%`);
  setText("rxRate", `${fmt.fixed(state.can.rx_rate, 1)} fps`);
  setText("txRate", `${fmt.fixed(state.can.tx_rate, 1)} fps`);
  setText("kbps", `${fmt.fixed(state.can.estimated_kbps, 1)} kbps`);
  setText("totals", `${state.can.total_rx} / ${state.can.total_tx}`);
  $("socketError").textContent = state.can.socket_error || "";

  const bar = $("loadBar");
  bar.style.width = `${width}%`;
  bar.className = "progress-fill";
  if (load >= 75) bar.classList.add("danger");
  else if (load >= 45) bar.classList.add("warn");
}

function renderImu(state) {
  const imu = state.imu || {};
  const quatAge = imu.quat_age_s === null || imu.quat_age_s === undefined ? Infinity : imu.quat_age_s;
  const gyroAge = imu.gyro_age_s === null || imu.gyro_age_s === undefined ? Infinity : imu.gyro_age_s;
  const maxAge = Math.max(quatAge, gyroAge);
  const fresh = Number.isFinite(maxAge) && maxAge <= 0.25;
  $("imuFreshBadge").textContent = fresh ? "online" : maxAge === Infinity ? "never" : "timeout";
  $("imuFreshBadge").className = fresh ? "badge ok" : maxAge === Infinity ? "badge muted" : "badge danger";
  setText("quatValue", fmt.vector(imu.quat_xyzw));
  setText("gravityValue", fmt.vector(imu.projected_gravity_b));
  setText("gyroValue", fmt.vector(imu.angular_velocity_rad_s));
  renderRobotAttitude(imu.quat_xyzw);
  setText("imuReq", imu.req_count || 0);
  setText("imuQuat", imu.quat_count || 0);
  setText("imuGyro", imu.gyro_count || 0);
  setText("imuAge", fmt.age(maxAge === Infinity ? null : maxAge));
}

function normalizeQuat(quat) {
  if (!Array.isArray(quat) || quat.length < 4) return null;
  const [x, y, z, w] = quat.map(Number);
  const norm = Math.hypot(x, y, z, w);
  if (!Number.isFinite(norm) || norm <= 1e-9) return null;
  return { x: x / norm, y: y / norm, z: z / norm, w: w / norm };
}

function quatToEulerDeg(quat) {
  const sinrCosp = 2 * (quat.w * quat.x + quat.y * quat.z);
  const cosrCosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * Math.PI / 2
    : Math.asin(sinp);

  const sinyCosp = 2 * (quat.w * quat.z + quat.x * quat.y);
  const cosyCosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  const toDeg = 180 / Math.PI;
  return { roll: roll * toDeg, pitch: pitch * toDeg, yaw: yaw * toDeg };
}

function quatToCssMatrix3d(quat) {
  const { x, y, z, w } = quat;
  const xx = x * x;
  const yy = y * y;
  const zz = z * z;
  const xy = x * y;
  const xz = x * z;
  const yz = y * z;
  const wx = w * x;
  const wy = w * y;
  const wz = w * z;

  const m11 = 1 - 2 * (yy + zz);
  const m12 = 2 * (xy - wz);
  const m13 = 2 * (xz + wy);
  const m21 = 2 * (xy + wz);
  const m22 = 1 - 2 * (xx + zz);
  const m23 = 2 * (yz - wx);
  const m31 = 2 * (xz - wy);
  const m32 = 2 * (yz + wx);
  const m33 = 1 - 2 * (xx + yy);

  return [
    m11, m21, m31, 0,
    m12, m22, m32, 0,
    m13, m23, m33, 0,
    0, 0, 0, 1,
  ].map((value) => value.toFixed(6)).join(",");
}

function renderRobotAttitude(quatValues) {
  const body = $("robotAttitudeBody");
  const quat = normalizeQuat(quatValues);
  if (!quat) {
    body.style.transform = "translate(-50%, -50%)";
    setText("rollValue", "-");
    setText("pitchValue", "-");
    setText("yawValue", "-");
    return;
  }

  const euler = quatToEulerDeg(quat);
  body.style.transform = `translate(-50%, -50%) matrix3d(${quatToCssMatrix3d(quat)})`;
  setText("rollValue", `${euler.roll.toFixed(1)} deg`);
  setText("pitchValue", `${euler.pitch.toFixed(1)} deg`);
  setText("yawValue", `${euler.yaw.toFixed(1)} deg`);
}

const mjcfModel = {
  baseSize: [0.22, 0.16, 0.08],
  hipOffset: [-0.058, 0.0082, 0],
  thighOffset: [0, 0.089, -0.35],
  calfOffset: [0, 0, -0.33],
};

function motorAngleByName(motors, name) {
  const motor = motors.find((item) => item.name === name);
  return motor && Number.isFinite(motor.position_rad) ? motor.position_rad : 0;
}

function vecAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function rotateX(point, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [point[0], point[1] * c - point[2] * s, point[1] * s + point[2] * c];
}

function rotateY(point, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [point[0] * c + point[2] * s, point[1], -point[0] * s + point[2] * c];
}

function quatRotatePoint(point, quat) {
  if (!quat) return point;
  const { x, y, z, w } = quat;
  const uv = [
    y * point[2] - z * point[1],
    z * point[0] - x * point[2],
    x * point[1] - y * point[0],
  ];
  const uuv = [
    y * uv[2] - z * uv[1],
    z * uv[0] - x * uv[2],
    x * uv[1] - y * uv[0],
  ];
  return [
    point[0] + 2 * (w * uv[0] + uuv[0]),
    point[1] + 2 * (w * uv[1] + uuv[1]),
    point[2] + 2 * (w * uv[2] + uuv[2]),
  ];
}

function projectMjcf(point, centerX, centerY, scale) {
  const view = rotateX(rotateY(point, -0.68), -0.34);
  return {
    x: centerX + (view[0] - view[1] * 0.28) * scale,
    y: centerY - (view[2] + view[1] * 0.26) * scale,
    depth: view[1],
  };
}

function drawLink(ctx, a, b, width, color) {
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "rgba(10, 21, 48, 0.18)";
  ctx.lineWidth = width + 8;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y + 6);
  ctx.lineTo(b.x, b.y + 6);
  ctx.stroke();

  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
  ctx.restore();
}

function drawJoint(ctx, point, radius, color) {
  ctx.save();
  ctx.fillStyle = "white";
  ctx.strokeStyle = color;
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawBaseBox(ctx, center, quat, canvasCenterX, canvasCenterY, scale) {
  const [sx, sy, sz] = mjcfModel.baseSize.map((value) => value * 0.5);
  const corners = [
    [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
    [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
  ].map((point) => projectMjcf(vecAdd(center, quatRotatePoint(point, quat)), canvasCenterX, canvasCenterY, scale));
  const faces = [
    [0, 1, 2, 3, "#d9f3e1"],
    [4, 5, 6, 7, "#dcecfa"],
    [1, 2, 6, 5, "#227c9d"],
    [0, 3, 7, 4, "#5645d4"],
  ];

  faces
    .map((face) => ({
      face,
      depth: face.slice(0, 4).reduce((sum, index) => sum + corners[index].depth, 0) / 4,
    }))
    .sort((a, b) => a.depth - b.depth)
    .forEach(({ face }) => {
      ctx.save();
      ctx.fillStyle = face[4];
      ctx.strokeStyle = "rgba(10, 21, 48, 0.22)";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(corners[face[0]].x, corners[face[0]].y);
      face.slice(1, 4).forEach((index) => ctx.lineTo(corners[index].x, corners[index].y));
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    });
}

function drawGround(ctx, width, height) {
  ctx.save();
  ctx.strokeStyle = "rgba(55, 53, 47, 0.12)";
  ctx.lineWidth = 1;
  for (let i = -5; i <= 5; i += 1) {
    ctx.beginPath();
    ctx.moveTo(width * 0.18, height * 0.76 + i * 12);
    ctx.lineTo(width * 0.82, height * 0.76 + i * 12);
    ctx.stroke();
  }
  ctx.restore();
}

function renderMjcfModel(motors, quatValues) {
  const canvas = $("mjcfCanvas");
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, rect.width || canvas.clientWidth || 720);
  const height = Math.max(220, rect.height || canvas.clientHeight || 360);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const hipRoll = motorAngleByName(motors, "RL_hip_roll");
  const hipPitch = motorAngleByName(motors, "RL_hip_pitch");
  const kneePitch = motorAngleByName(motors, "RL_knee_pitch");
  const baseQuat = normalizeQuat(quatValues);
  const base = [0, 0, 0.1];
  const hip = vecAdd(base, quatRotatePoint([0, 0, 0], baseQuat));
  const thigh = vecAdd(base, quatRotatePoint(rotateX(mjcfModel.hipOffset, hipRoll), baseQuat));
  const calfLocal = vecAdd(rotateX(mjcfModel.hipOffset, hipRoll), rotateX(rotateY(mjcfModel.thighOffset, -hipPitch), hipRoll));
  const footLocal = vecAdd(calfLocal, rotateX(rotateY(mjcfModel.calfOffset, -(hipPitch + kneePitch)), hipRoll));
  const calf = vecAdd(base, quatRotatePoint(calfLocal, baseQuat));
  const foot = vecAdd(base, quatRotatePoint(footLocal, baseQuat));

  const scale = Math.min(width * 1.25, height * 2.05);
  const centerX = width * 0.5;
  const centerY = height * 0.47;
  const points = {
    hip: projectMjcf(hip, centerX, centerY, scale),
    thigh: projectMjcf(thigh, centerX, centerY, scale),
    calf: projectMjcf(calf, centerX, centerY, scale),
    foot: projectMjcf(foot, centerX, centerY, scale),
  };

  const links = [
    { a: points.hip, b: points.thigh, width: 20, color: "#5645d4", depth: (points.hip.depth + points.thigh.depth) / 2 },
    { a: points.thigh, b: points.calf, width: 18, color: "#227c9d", depth: (points.thigh.depth + points.calf.depth) / 2 },
    { a: points.calf, b: points.foot, width: 15, color: "#2f4858", depth: (points.calf.depth + points.foot.depth) / 2 },
  ];

  const gradient = ctx.createLinearGradient(0, 0, 0, height);
  gradient.addColorStop(0, "#dcecfa");
  gradient.addColorStop(0.66, "#fafaf9");
  gradient.addColorStop(1, "#f6f5f4");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);
  drawGround(ctx, width, height);
  drawBaseBox(ctx, base, baseQuat, centerX, centerY, scale);
  links.sort((a, b) => a.depth - b.depth).forEach((link) => drawLink(ctx, link.a, link.b, link.width, link.color));
  drawJoint(ctx, points.hip, 8, "#5645d4");
  drawJoint(ctx, points.thigh, 9, "#227c9d");
  drawJoint(ctx, points.calf, 9, "#2f4858");
  drawJoint(ctx, points.foot, 7, "#37352f");

  setText("mjcfHipRoll", `${hipRoll.toFixed(3)} rad`);
  setText("mjcfHipPitch", `${hipPitch.toFixed(3)} rad`);
  setText("mjcfKneePitch", `${kneePitch.toFixed(3)} rad`);
}

function renderNodes(nodes) {
  const rows = nodes.map((node) => `
    <tr>
      <td>${escapeHtml(node.name)}</td>
      <td><code>${node.can_id}</code></td>
      <td>${escapeHtml(node.role)}</td>
      <td>${fmt.fixed(node.heartbeat_hz, 1)}</td>
      <td>${fmt.age(node.last_seen_s)}</td>
      <td><span class="${dotClass(node.status)}">${node.status}</span></td>
    </tr>
  `);
  $("nodeRows").innerHTML = rows.join("");
}

function renderMotors(motors, safety) {
  const motorGated = !safety.tx_enabled || !safety.allow_motor_commands;
  const seen = new Set();
  const tbody = $("motorRows");

  motors.forEach((motor) => {
    const key = String(motor.motor_id);
    seen.add(key);

    let row = tbody.querySelector(`tr[data-motor-id="${key}"]`);
    if (!row) {
      row = document.createElement("tr");
      row.dataset.motorId = key;
      row.innerHTML = `
        <td data-field="motor_id"></td>
        <td data-field="name"></td>
        <td><code data-field="can_id"></code></td>
        <td><span data-field="last_kind"></span></td>
        <td data-field="age"></td>
        <td data-field="temperature"></td>
        <td data-field="iq"></td>
        <td data-field="speed"></td>
        <td data-field="position"></td>
        <td><code data-field="raw"></code></td>
        <td>
          <div class="row-actions">
            <button class="button secondary compact" data-motor-action="enter" data-motor-id="${key}">Enable</button>
            <button class="button secondary compact" data-motor-action="zero" data-motor-id="${key}">Zero set</button>
            <button class="button secondary compact" data-motor-action="mit-poll" data-motor-id="${key}">MIT Poll</button>
          </div>
        </td>
      `;
      tbody.appendChild(row);
    }

    row.querySelector('[data-field="motor_id"]').textContent = motor.motor_id;
    row.querySelector('[data-field="name"]').textContent = motor.name || `Motor ${motor.motor_id}`;
    row.querySelector('[data-field="can_id"]').textContent = motor.can_id;
    const stateCell = row.querySelector('[data-field="last_kind"]');
    stateCell.className = dotClass(motor.status);
    stateCell.textContent = motor.last_kind;
    row.querySelector('[data-field="age"]').textContent = fmt.age(motor.age_s);
    row.querySelector('[data-field="temperature"]').textContent = fmt.maybe(motor.temperature_c, 0);
    row.querySelector('[data-field="iq"]').textContent = fmt.maybe(motor.iq_a_approx, 2);
    row.querySelector('[data-field="speed"]').textContent = fmt.maybe(motor.speed_dps, 0);
    row.querySelector('[data-field="position"]').textContent = fmt.maybe(motor.position_rad, 3);
    row.querySelector('[data-field="raw"]').textContent = motor.raw || "";

    row.querySelectorAll("[data-motor-action]").forEach((button) => {
      if (button.dataset.motorAction === "mit-poll") {
        button.textContent = motor.mit_polling ? "Stop Poll" : "MIT Poll";
        button.classList.toggle("danger", motor.mit_polling);
        button.classList.toggle("secondary", !motor.mit_polling);
      }
      button.classList.toggle("gated", motorGated);
      button.title = motorGated ? "Requires TX unlock and allow_motor_commands=true" : "";
    });
  });

  tbody.querySelectorAll("tr[data-motor-id]").forEach((row) => {
    if (!seen.has(row.dataset.motorId)) {
      row.remove();
    }
  });
}

function renderEnabledMotorCards(motors) {
  const enabled = motors
    .filter((motor) => motor.enabled_hint === true)
    .sort((a, b) => a.motor_id - b.motor_id)
    .slice(0, 3);

  const container = $("enabledMotorCards");
  if (!enabled.length) {
    container.innerHTML = "";
    container.hidden = true;
    return;
  }

  container.hidden = false;
  container.innerHTML = enabled.map((motor) => {
    const positionRad = Number.isFinite(motor.position_rad) ? motor.position_rad : 0;
    const positionDeg = positionRad * 180 / Math.PI;
    const dialDeg = positionDeg - 90;
    const hasPosition = Number.isFinite(motor.position_rad);
    return `
      <article class="enabled-motor-card">
        <div class="enabled-motor-copy">
          <span class="badge ok">Motor ${motor.motor_id}</span>
          <strong>${escapeHtml(motor.name || motor.can_id)}</strong>
          <span>${escapeHtml(motor.last_kind)}</span>
        </div>
        <div class="angle-dial ${hasPosition ? "" : "muted"}" style="--angle:${dialDeg.toFixed(2)}deg">
          <span class="dial-arrow"></span>
          <span class="dial-center"></span>
        </div>
        <div class="enabled-motor-metrics">
          <div><span>Angle</span><strong>${hasPosition ? `${fmt.maybe(motor.position_rad, 3)} rad` : "-"}</strong></div>
          <div><span>Deg</span><strong>${hasPosition ? `${positionDeg.toFixed(1)} deg` : "-"}</strong></div>
          <div><span>Speed</span><strong>${fmt.maybe(motor.speed_dps, 0)}</strong></div>
        </div>
      </article>
    `;
  }).join("");
}

function renderFrames(frames) {
  if (!frames.length) {
    $("recentFrames").innerHTML = '<div class="frame-item"><strong>No frames</strong><p>Waiting for CAN traffic</p></div>';
    return;
  }
  $("recentFrames").innerHTML = frames.map((frame) => `
    <div class="frame-item">
      <strong><span>${frame.can_id}</span><span>${fmt.age(frame.age_s)}</span></strong>
      <p>DLC ${frame.dlc} · count ${frame.count}</p>
      <p><code>${escapeHtml(frame.data || "")}</code></p>
    </div>
  `).join("");
}

function syncControls(state) {
  $("txToggle").textContent = state.safety.tx_enabled ? "Lock TX" : "Unlock TX";
  $("txToggle").className = state.safety.tx_enabled ? "button secondary" : "button danger";
  $("pollToggle").textContent = state.controls.imu_polling ? "Stop Poll" : "Start Poll";
  $("pollHz").value = state.controls.imu_poll_hz || $("pollHz").value;
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.detail || response.statusText);
  }
  return data;
}

async function loadDashboardConfig() {
  const response = await fetch("/api/config");
  dashboardConfig = await response.json();
  populateTransmitIds(dashboardConfig);
}

function populateTransmitIds(config) {
  const select = $("rawCanId");
  const entries = (config.dashboard && Array.isArray(config.dashboard.transmit_ids))
    ? config.dashboard.transmit_ids
    : [];

  select.innerHTML = "";

  if (!entries.length) {
    const option = document.createElement("option");
    option.value = "0x221";
    option.textContent = "E2Box request (0x221)";
    option.dataset.payload = "03";
    select.appendChild(option);
    $("rawPayload").value = "03";
    return;
  }

  entries.forEach((entry, index) => {
    const canId = hexCanId(entry.can_id);
    const label = entry.label || canId;
    const option = document.createElement("option");
    option.value = canId;
    option.textContent = `${label} (${canId})`;
    option.dataset.payload = normalizePayload(entry.payload);
    select.appendChild(option);

    if (index === 0 && option.dataset.payload) {
      $("rawPayload").value = option.dataset.payload;
    }
  });
}

function showMessage(text, isError = false) {
  const el = $("commandMessage");
  el.textContent = text;
  el.style.color = isError ? "var(--danger)" : "var(--slate)";
}

async function sendMotorAction(button) {
  if (!button || button.dataset.busy === "true") return;

  button.dataset.busy = "true";
  try {
    const motorId = button.dataset.motorId;
    const action = button.dataset.motorAction;
    if (action === "mit-poll") {
      const motors = latestState && Array.isArray(latestState.motors) ? latestState.motors : [];
      const motor = motors.find((item) => String(item.motor_id) === String(motorId));
      if (motor && motor.mit_polling) {
        await postJson(`/api/motor/${motorId}/mit-poll/stop`);
        showMessage(`Motor ${motorId} MIT polling stopped`);
      } else {
        const confirmed = window.confirm(`Start MIT polling for motor ${motorId}?`);
        if (!confirmed) return;
        await postJson(`/api/motor/${motorId}/mit-poll/start`, { confirmed });
        showMessage(`Motor ${motorId} MIT polling started`);
      }
      return;
    }

    await postJson(`/api/motor/${motorId}/${action}`, action === "zero" ? { offset_count: 0 } : {});
    showMessage(`Motor ${motorId} ${action === "enter" ? "enable" : "zero set"} sent`);
  } catch (error) {
    showMessage(error.message, true);
  } finally {
    button.dataset.busy = "false";
  }
}

function connectWs() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${scheme}://${window.location.host}/ws/state`);
  ws.onmessage = (event) => render(JSON.parse(event.data));
  ws.onclose = () => {
    $("socketBadge").textContent = "ws disconnected";
    $("socketBadge").className = "badge danger";
    window.setTimeout(connectWs, 1000);
  };
}

function bindControls() {
  $("rawCanId").addEventListener("change", () => {
    const option = $("rawCanId").selectedOptions[0];
    if (option && option.dataset.payload) {
      $("rawPayload").value = option.dataset.payload;
    }
  });

  $("txToggle").addEventListener("click", async () => {
    try {
      if (latestState && latestState.safety && latestState.safety.tx_enabled) {
        await postJson("/api/tx/lock");
        showMessage("TX locked");
        return;
      }
      await postJson("/api/tx/unlock");
      showMessage("TX enabled");
    } catch (error) {
      showMessage(error.message, true);
    }
  });

  $("pollToggle").addEventListener("click", async () => {
    try {
      const hz = Number($("pollHz").value);
      await postJson("/api/imu/poll/hz", { hz });
      if (latestState && latestState.controls && latestState.controls.imu_polling) {
        await postJson("/api/imu/poll/stop");
        showMessage("IMU polling stopped");
      } else {
        await postJson("/api/imu/poll/start");
        showMessage("IMU polling started");
      }
    } catch (error) {
      showMessage(error.message, true);
    }
  });

  $("rawForm").addEventListener("submit", async (event) => {
    event.preventDefault();
    try {
      const response = await postJson("/api/can/send", {
        can_id: $("rawCanId").value,
        data: $("rawPayload").value,
      });
      showMessage("Frame sent");
    } catch (error) {
      showMessage(error.message, true);
    }
  });

  $("motorRows").addEventListener("pointerdown", (event) => {
    const button = event.target.closest("[data-motor-action]");
    if (!button) return;
    event.preventDefault();
    sendMotorAction(button);
  });

  $("motorRows").addEventListener("click", (event) => {
    const button = event.target.closest("[data-motor-action]");
    if (!button || event.detail !== 0) return;
    sendMotorAction(button);
  });
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

loadDashboardConfig()
  .catch((error) => showMessage(error.message, true))
  .finally(() => {
    bindControls();
    connectWs();
  });
