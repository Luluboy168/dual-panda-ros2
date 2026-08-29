// Copyright 2026 The multipanda_ros2 Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

// Panda URDF position limits, display fallback only when no fence is known
// (the real fence arrives with a Motion-mode gains upload in Stage 2).
const PANDA_LIMITS = [
  [-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698],
  [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973],
];

const el = (id) => document.getElementById(id);

const state = {
  token: null,
  claimedAt: 0,
  heartbeatTimer: null,
  lastFrame: null,
  lastServerTime: '',
  localError: null,
  pollTimer: null,
};

// ---------------------------------------------------------------- operator

async function api(method, path, body) {
  const headers = {};
  if (state.token) headers['X-Operator-Token'] = state.token;
  if (body !== undefined) headers['Content-Type'] = 'application/json; charset=utf-8';
  let response;
  try {
    response = await fetch(path, {
      method,
      headers,
      body: body === undefined ? undefined : JSON.stringify(body),
    });
  } catch (error) {
    throw { ok: false, error: 'transport_error', detail: 'server unreachable' };
  }
  let payload;
  try {
    payload = await response.json();
  } catch (error) {
    throw {
      ok: false, error: 'transport_error',
      detail: `HTTP ${response.status} without a JSON body`,
    };
  }
  if (!payload.ok) throw payload;
  return payload;
}

async function claimLock() {
  try {
    const result = await api('POST', '/api/operator/claim');
    state.token = result.token;
    state.claimedAt = Date.now();
    el('operator').textContent = 'operator: you hold control';
    el('operator').classList.remove('held');
    if (state.heartbeatTimer) clearInterval(state.heartbeatTimer);
    state.heartbeatTimer = setInterval(heartbeat, 5000);
    setControlsEnabled(true);
  } catch (error) {
    state.token = null;
    setControlsEnabled(false);
    el('operator').classList.add('held');
    el('operator').textContent = error.error === 'operator_lock_held'
      ? 'operator: another operator holds control (read-only)'
      : 'operator: server unreachable — retrying';
    setTimeout(claimLock, 5000);
  }
}

async function heartbeat() {
  if (!state.token) return;
  try {
    await api('POST', '/api/operator/heartbeat');
  } catch (error) {
    state.token = null;
    setControlsEnabled(false);
    claimLock();
  }
}

function setControlsEnabled(enabled) {
  el('start').disabled = !enabled;
  el('stop').disabled = !enabled;
  syncMotionControls();
  // The dynamically built Control card is rebuilt/refreshed by render();
  // gate whatever is on screen right now too.
  for (const button of el('control-body').querySelectorAll('button')) {
    button.disabled = !enabled || button.disabled;
  }
}

// ---------------------------------------------------------------- commands

function selectedMode() {
  return document.querySelector('input[name="mode"]:checked').value;
}

function syncMotionControls() {
  const usable = selectedMode() === 'motion' && !!state.token;
  el('controller').disabled = !usable;
  el('gains').disabled = !usable;
  el('gains-file').disabled = !usable;
  el('gains-upload').disabled = !usable;
}

async function refreshGainsList(selectSha) {
  try {
    const result = await api('GET', '/api/gains');
    const select = el('gains');
    select.innerHTML = '';
    if (!result.gains.length) {
      const option = document.createElement('option');
      option.value = '';
      option.textContent = 'upload a config first';
      select.appendChild(option);
      return;
    }
    for (const entry of result.gains) {
      const option = document.createElement('option');
      option.value = entry.config_sha256;
      option.textContent = `${entry.controller_name.replace('dual_arm_joint_', '')}`
        + ` · ${entry.arms.join('+')} · ${entry.config_sha256.slice(0, 10)}`;
      select.appendChild(option);
    }
    if (selectSha) select.value = selectSha;
  } catch (error) { /* list stays as-is */ }
}

async function uploadGains() {
  const file = el('gains-file').files[0];
  if (!file) {
    state.localError = 'gains_invalid: choose a YAML file first';
    return;
  }
  const arms = document.querySelector('input[name="arms"]:checked').value;
  const controller = el('controller').value;
  try {
    const response = await fetch(
      `/api/gains?controller_name=${encodeURIComponent(controller)}`
      + `&arms=${encodeURIComponent(arms)}`,
      {
        method: 'POST',
        headers: {
          'X-Operator-Token': state.token || '',
          'Content-Type': 'application/x-yaml',
        },
        body: await file.arrayBuffer(),
      });
    const payload = await response.json();
    if (!payload.ok) throw payload;
    state.localError = null;
    await refreshGainsList(payload.config_sha256);
  } catch (error) {
    state.localError = `${error.error || 'transport_error'}: ${
      error.detail || 'the upload did not complete'}`;
  }
}

async function startSession() {
  const arms = document.querySelector('input[name="arms"]:checked').value;
  const mode = selectedMode();
  const body = { arms, mode };
  if (mode === 'motion') {
    body.controller_name = el('controller').value;
    body.gains_sha256 = el('gains').value;
  }
  try {
    await api('POST', '/api/session/start', body);
    state.localError = null;
  } catch (error) {
    // Held in state (not written straight to the DOM) so the 5 Hz render
    // cannot wipe the refusal before the operator reads it.
    state.localError = `${error.error}: ${error.detail}`;
  }
}

async function stopSession() {
  try {
    await api('POST', '/api/session/stop');
    state.localError = null;
  } catch (error) {
    state.localError = `${error.error}: ${error.detail}`;
  }
}

// ---------------------------------------------------------------- rendering

function render(frame) {
  // Frames must move forward: a slow poll response arriving after a newer
  // SSE frame must not repaint stale state (RFC 3339 sorts lexically).
  if (frame.server_time && frame.server_time < state.lastServerTime) return;
  state.lastServerTime = frame.server_time || state.lastServerTime;
  state.lastFrame = frame;
  const session = frame.session;
  el('session-state').textContent = session.state;
  el('session-id').textContent = session.session_id || '—';
  el('session-uptime').textContent =
    session.uptime_s == null ? '—' : `${session.uptime_s.toFixed(0)} s`;
  const frameError = session.last_error
    ? `${session.last_error.code}: ${session.last_error.detail}` : null;
  // A session-ending error outranks a stale local refusal: the operator
  // must see why the session died, not why an earlier click was refused.
  const frameWins = frameError
    && (session.state === 'fault' || session.state === 'stopped');
  el('last-error').textContent =
    (frameWins ? frameError : state.localError || frameError) || '—';

  const recording = frame.recording;
  el('recording-status').textContent = recording.active
    ? `${recording.name} (segment ${recording.sequence})`
    : 'not recording';

  const preflight = frame.preflight;
  el('preflight-status').textContent = preflight.overall
    ? `${preflight.overall}${preflight.blocking ? '' : ' (advisory)'}` : '—';

  const operator = frame.operator;
  if (!state.token && operator.locked) {
    el('operator').classList.add('held');
    el('operator').textContent =
      `operator: another operator holds control (expires in ${
        Math.max(0, operator.expires_in_s).toFixed(0)} s)`;
  } else if (state.token && !operator.locked
             && Date.now() - state.claimedAt > 3000) {
    // The server says nobody holds the lock but we think we do: our token
    // silently expired. Stop pretending and reclaim.
    state.token = null;
    setControlsEnabled(false);
    el('operator').textContent = 'operator: control lost — reclaiming';
    claimLock();
  }

  renderArms(frame.arms);
  renderControl(frame);
}

// ------------------------------------------------------------- control card

async function toggleEnable(armId, enabled) {
  try {
    await api('POST', `/api/arm/${armId}/enable`, { enabled });
    state.localError = null;
  } catch (error) {
    state.localError = `${error.error}: ${error.detail}`;
  }
}

async function jog(armId, jointIndex, direction, row, button) {
  // One press, one step (§6.13): the button stays disabled for the round
  // trip, so keyboard auto-repeat or mashing cannot become a teleop stream.
  if (button) button.disabled = true;
  try {
    const result = await api('POST', `/api/arm/${armId}/jog`,
      { joint_index: jointIndex, direction });
    state.localError = null;
    if (result.clamped[jointIndex] && row) {
      row.classList.add('clamped');
      setTimeout(() => row.classList.remove('clamped'), 600);
    }
  } catch (error) {
    state.localError = `${error.error}: ${error.detail}`;
  } finally {
    if (button) button.disabled = !state.token;
  }
}

async function recover(armId, container) {
  try {
    const result = await api('POST', `/api/arm/${armId}/recover`);
    state.localError = null;
    const steps = result.steps
      .map((s) => `${s.step}: ${s.ok ? 'ok' : 'FAILED'}`
        + `${s.detail ? ` (${s.detail})` : ''}`)
      .join(' · ');
    if (container) {
      container.textContent = `${armId}: ${steps} — if the fault has cleared, `
        + 'the jog grid returns; press Enable then';
    }
  } catch (error) {
    state.localError = `${error.error}: ${error.detail}`;
  }
}

function controlSignature(frame) {
  const session = frame.session;
  return [session.session_id, session.state, session.mode,
    session.controller_name, String(!!state.token),
    frame.fault.recoverable,
    frame.fault.reasons.map((r) => `${r.code}@${r.arm_id}`).join(','),
    Object.values(frame.arms).map((a) => a.motion.enabled).join(',')].join('|');
}

function renderControl(frame) {
  const session = frame.session;
  const card = el('control-card');
  const visible = session.mode === 'motion'
    && (session.state === 'running' || session.state === 'fault');
  card.hidden = !visible;
  if (!visible) {
    state.controlSignature = null;
    el('control-body').innerHTML = '';
    return;
  }
  const signature = controlSignature(frame);
  if (state.controlSignature !== signature) {
    state.controlSignature = signature;
    buildControlBody(frame);
  }
  updateControlBody(frame);
}

function buildControlBody(frame) {
  const session = frame.session;
  const body = el('control-body');
  body.innerHTML = '';
  state.controlRefs = {};
  if (session.state === 'fault') {
    body.appendChild(buildFaultPanel(frame));
    return;
  }
  for (const [armId, arm] of Object.entries(frame.arms)) {
    if (!arm.motion.available) {
      const note = document.createElement('p');
      note.className = 'hold-note';
      note.textContent = 'Hold: arms held at activation pose. '
        + 'This controller has no enable or jog surface.';
      body.appendChild(note);
      return;
    }
    body.appendChild(buildArmPanel(armId, arm));
  }
}

function buildArmPanel(armId, arm) {
  const panel = document.createElement('div');
  panel.className = 'control-arm';
  const head = document.createElement('header');
  const title = document.createElement('h3');
  title.textContent = armId;
  head.appendChild(title);
  const toggle = document.createElement('button');
  toggle.className = `enable-toggle${arm.motion.enabled ? ' on' : ''}`;
  toggle.textContent = arm.motion.enabled ? 'Enabled — click to disable'
    : 'Enable';
  toggle.disabled = !state.token;
  toggle.addEventListener('click',
    () => toggleEnable(armId, !arm.motion.enabled));
  head.appendChild(toggle);
  panel.appendChild(head);
  const refs = [];
  for (let joint = 0; joint < 7; joint += 1) {
    const row = document.createElement('div');
    row.className = 'jog-row';
    const label = document.createElement('span');
    label.className = 'name';
    label.textContent = `J${joint + 1}`;
    row.appendChild(label);
    const minus = document.createElement('button');
    minus.className = 'jog-btn';
    minus.textContent = '−';
    minus.disabled = !arm.motion.enabled || !state.token;
    minus.addEventListener('click', () => jog(armId, joint, -1, row, minus));
    row.appendChild(minus);
    const bar = document.createElement('div');
    bar.className = 'bar';
    const marker = document.createElement('div');
    marker.className = 'marker';
    bar.appendChild(marker);
    row.appendChild(bar);
    const plus = document.createElement('button');
    plus.className = 'jog-btn';
    plus.textContent = '+';
    plus.disabled = !arm.motion.enabled || !state.token;
    plus.addEventListener('click', () => jog(armId, joint, 1, row, plus));
    row.appendChild(plus);
    const target = document.createElement('span');
    target.className = 'target';
    row.appendChild(target);
    panel.appendChild(row);
    refs.push({ marker, target });
  }
  state.controlRefs[armId] = refs;
  return panel;
}

function buildFaultPanel(frame) {
  const panel = document.createElement('div');
  panel.className = 'control-arm';
  const note = document.createElement('p');
  note.className = 'fault-note';
  const limp = frame.fault.reasons.some((r) => r.code === 'controller_deactivated');
  note.innerHTML = '<strong>Faulted.</strong> Release the physical stop '
    + '(the pilot’s E-stop / enabling device) first. Then press Recover. '
    + 'Recovery returns the arm to state-only reading — it never resumes '
    + 'motion. You must press Enable again afterwards.'
    + (limp ? ' The controller tripped its effort envelope: the arms dropped '
      + 'to gravity compensation (limp), not a hold.' : '');
  panel.appendChild(note);
  const reasons = document.createElement('p');
  reasons.className = 'fault-note';
  reasons.textContent = frame.fault.reasons
    .map((r) => `${r.code}${r.arm_id ? ` (${r.arm_id})` : ''}: ${r.detail}`)
    .join(' · ');
  panel.appendChild(reasons);
  const stepsOut = document.createElement('p');
  stepsOut.className = 'recover-steps';
  if (frame.fault.recoverable) {
    // One Recover button per faulted arm: error recovery is a strictly
    // per-arm service, and a dual session's fault may be on either arm.
    const faultedArms = [...new Set(frame.fault.reasons
      .map((r) => r.arm_id).filter(Boolean))];
    const targets = faultedArms.length ? faultedArms : frame.session.arm_ids;
    for (const armId of targets) {
      const button = document.createElement('button');
      button.className = 'recover-button';
      button.textContent = targets.length > 1 ? `Recover ${armId}` : 'Recover';
      button.disabled = !state.token;
      button.addEventListener('click', () => recover(armId, stepsOut));
      panel.appendChild(button);
    }
  } else {
    const hint = document.createElement('p');
    hint.className = 'fault-note';
    hint.textContent = 'This fault is not recoverable from the page: '
      + 'stop and restart the session.';
    panel.appendChild(hint);
  }
  panel.appendChild(stepsOut);
  return panel;
}

function updateControlBody(frame) {
  for (const [armId, arm] of Object.entries(frame.arms)) {
    const refs = (state.controlRefs || {})[armId];
    if (!refs || !arm.motion.available) continue;
    const lower = arm.motion.fence_lower;
    const upper = arm.motion.fence_upper;
    for (let joint = 0; joint < 7; joint += 1) {
      const target = arm.motion.target ? arm.motion.target[joint] : null;
      const ref = refs[joint];
      ref.target.textContent = target == null ? '—' : `${target.toFixed(3)} rad`;
      if (target != null && lower && upper && upper[joint] > lower[joint]) {
        const fraction = Math.min(1, Math.max(0,
          (target - lower[joint]) / (upper[joint] - lower[joint])));
        ref.marker.style.left = `calc(${(fraction * 100).toFixed(1)}% - 1px)`;
      }
    }
  }
}

function renderArms(arms) {
  const container = el('arm-tiles');
  const armIds = Object.keys(arms);
  if (armIds.length === 0) {
    container.innerHTML =
      '<p class="placeholder">No session. Start one to see arm health.</p>';
    return;
  }
  container.innerHTML = '';
  for (const armId of armIds) {
    container.appendChild(renderArmTile(arms[armId]));
  }
}

function renderArmTile(arm) {
  const tile = document.createElement('div');
  tile.className = `tile ${arm.status}`;

  const title = document.createElement('h3');
  title.textContent = `${arm.arm_id} — ${arm.status}`;
  tile.appendChild(title);

  const line = document.createElement('div');
  line.className = 'line';
  line.textContent = arm.status_line;
  tile.appendChild(line);

  arm.joint_names.forEach((name, index) => {
    tile.appendChild(renderJointRow(arm, name, index));
  });

  const meta = document.createElement('div');
  meta.className = 'meta';
  const parts = [];
  if (arm.robot_state.available) {
    const ccsr = arm.robot_state.control_command_success_rate;
    parts.push(`ccsr ${ccsr == null ? 'n/a' : ccsr.toFixed(3)}`);
    parts.push(`mode ${arm.robot_state.robot_mode_label}`);
    if (arm.robot_state.current_errors.length) {
      parts.push(`errors: ${arm.robot_state.current_errors.join(', ')}`);
    }
  } else {
    parts.push('simulated — no Franka health data');
  }
  parts.push(arm.positions_stale ? 'joints STALE' :
    `joints ${(arm.positions_age_s ?? 0).toFixed(2)} s old`);
  meta.textContent = parts.join(' · ');
  tile.appendChild(meta);
  return tile;
}

function renderJointRow(arm, name, index) {
  const row = document.createElement('div');
  row.className = 'joint';

  const label = document.createElement('span');
  label.className = 'name';
  label.textContent = `J${index + 1}`;
  row.appendChild(label);

  const bar = document.createElement('div');
  bar.className = `bar${arm.positions_stale ? ' stale' : ''}`;
  const marker = document.createElement('div');
  marker.className = 'marker';
  const position = arm.positions[index];
  const motion = arm.motion || {};
  const low = motion.fence_lower ? motion.fence_lower[index] : PANDA_LIMITS[index][0];
  const high = motion.fence_upper ? motion.fence_upper[index] : PANDA_LIMITS[index][1];
  if (position != null && high > low) {
    const fraction = Math.min(1, Math.max(0, (position - low) / (high - low)));
    marker.style.left = `calc(${(fraction * 100).toFixed(1)}% - 1px)`;
  } else {
    marker.style.left = '0';
  }
  bar.appendChild(marker);
  row.appendChild(bar);

  const value = document.createElement('span');
  value.className = 'value';
  value.textContent = position == null ? '—' : `${position.toFixed(3)} rad`;
  row.appendChild(value);
  return row;
}

// ---------------------------------------------------------------- stream

function connectStream() {
  const source = new EventSource('/api/state/stream');
  source.addEventListener('state', (event) => {
    let frame;
    try {
      frame = JSON.parse(event.data);
    } catch (error) {
      el('link-state').textContent = 'stream: received an unparseable frame';
      return;
    }
    el('link-state').textContent = 'stream: live';
    render(frame);
  });
  source.onerror = () => {
    el('link-state').textContent = 'stream: reconnecting… (polling fallback)';
    if (!state.pollTimer) {
      state.pollTimer = setInterval(async () => {
        try {
          const result = await api('GET', '/api/state');
          render(result.state);
        } catch (error) { /* keep trying */ }
      }, 1000);
    }
  };
  source.onopen = () => {
    if (state.pollTimer) {
      clearInterval(state.pollTimer);
      state.pollTimer = null;
    }
  };
}

// ---------------------------------------------------------------- boot

async function boot() {
  try {
    const caps = await api('GET', '/api/capabilities');
    el('server-version').textContent =
      `${caps.server_version} · domain ${caps.ros_domain_id} · ${caps.transport}`;
  } catch (error) { /* footer stays empty */ }
  await claimLock();
  connectStream();
  el('start').addEventListener('click', startSession);
  el('stop').addEventListener('click', stopSession);
  el('gains-upload').addEventListener('click', uploadGains);
  for (const radio of document.querySelectorAll('input[name="mode"]')) {
    radio.addEventListener('change', syncMotionControls);
  }
  syncMotionControls();
  refreshGainsList();
  window.addEventListener('pagehide', () => {
    // Free the lock on reload/close so the returning page need not wait
    // out the 15 s TTL; keepalive lets the request outlive the page.
    if (state.token) {
      fetch('/api/operator/release', {
        method: 'POST',
        headers: { 'X-Operator-Token': state.token },
        keepalive: true,
      }).catch(() => {});
    }
  });
}

boot();
