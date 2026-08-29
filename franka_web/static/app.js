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
}

// ---------------------------------------------------------------- commands

async function startSession() {
  const arms = document.querySelector('input[name="arms"]:checked').value;
  const mode = document.querySelector('input[name="mode"]:checked').value;
  try {
    await api('POST', '/api/session/start', { arms, mode });
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
  el('last-error').textContent = state.localError || frameError || '—';

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
    parts.push(`ccsr ${arm.robot_state.control_command_success_rate.toFixed(3)}`);
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
