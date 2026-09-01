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

// Panda URDF position limits.  These are a display-only fallback when a
// Watch session has no reviewed joint-angle fence selected; they are never a
// substitute for an uploaded impedance fence.  Reviewed Hold configs do not
// contain a joint-angle fence and continue to use this display-only view.
const PANDA_LIMITS = [
  [-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698],
  [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973],
];

const IMPEDANCE_CONTROLLER = 'dual_arm_joint_impedance_controller';
const HOLD_CONTROLLER = 'dual_arm_joint_hold_controller';
const SETTLING_NOTICE = 'Torque control is active; startup settling verification '
  + 'is in progress. Enable and Jog are unavailable.';

const el = (id) => document.getElementById(id);

const state = {
  token: null,
  claimedAt: 0,
  heartbeatTimer: null,
  lastFrame: null,
  lastServerTime: '',
  localError: null,
  pollTimer: null,
  gainsEntries: [],
  recoveryPending: false,
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

function selectedArms() {
  return document.querySelector('input[name="arms"]:checked').value;
}

function selectedArmIds() {
  const arms = selectedArms();
  return arms === 'both' ? ['panda1', 'panda2'] : [arms];
}

function sameArmIds(left, right) {
  return Array.isArray(left) && Array.isArray(right)
    && left.length === right.length
    && left.every((armId, index) => armId === right[index]);
}

function rebuildGainsList(selectSha) {
  const select = el('gains');
  const mode = selectedMode();
  const previous = selectSha === undefined ? select.value : selectSha;
  const controller = el('controller').value;
  const arms = selectedArmIds();
  const matches = state.gainsEntries.filter((entry) =>
    entry.controller_name === controller && sameArmIds(entry.arms, arms));

  select.innerHTML = '';
  const blank = document.createElement('option');
  blank.value = '';
  if (mode === 'watch') {
    blank.textContent = 'No fence preview — show Panda limits';
  } else if (mode === 'motion') {
    blank.textContent = matches.length
      ? 'Select a validated config — required'
      : 'Upload a matching config — required';
  } else {
    blank.textContent = 'No config used in Simulate';
  }
  select.appendChild(blank);
  for (const entry of matches) {
    const option = document.createElement('option');
    option.value = entry.config_sha256;
    option.textContent = `${entry.controller_name.replace('dual_arm_joint_', '')}`
      + ` · ${entry.arms.join('+')} · ${entry.config_sha256.slice(0, 10)}`;
    select.appendChild(option);
  }
  // Never silently select the newest upload. Preserve only an explicit
  // selection that still matches the current mode/controller/arm tuple.
  if (previous && matches.some((entry) => entry.config_sha256 === previous)) {
    select.value = previous;
  }
}

function syncMotionControls() {
  const mode = selectedMode();
  const configMode = mode === 'watch' || mode === 'motion';
  const usable = configMode && !!state.token;
  const holdOption = [...el('controller').options]
    .find((option) => option.value === HOLD_CONTROLLER);
  if (holdOption) {
    holdOption.disabled = mode === 'watch';
    holdOption.hidden = mode === 'watch';
  }
  if (mode === 'watch') el('controller').value = IMPEDANCE_CONTROLLER;
  el('controller').disabled = !usable;
  el('gains').disabled = !usable;
  el('gains-file').disabled = !usable;
  el('gains-upload').disabled = !usable;
  rebuildGainsList();
}

async function refreshGainsList(selectSha) {
  try {
    const result = await api('GET', '/api/gains');
    state.gainsEntries = Array.isArray(result.gains) ? result.gains : [];
    rebuildGainsList(selectSha);
  } catch (error) { /* list stays as-is */ }
}

async function uploadGains() {
  const file = el('gains-file').files[0];
  if (!file) {
    state.localError = 'gains_invalid: choose a YAML file first';
    return;
  }
  const arms = selectedArms();
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
  if (mode === 'motion' || (mode === 'watch' && el('gains').value)) {
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
  el('session-state').textContent = sessionStateLabel(session);
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

  renderPreviewStatus(frame);

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

  renderArms(frame.arms, session);
  renderControl(frame);
}

function sessionStateLabel(session) {
  return session.state === 'settling' ? 'settling (torque active)' : session.state;
}

function renderPreviewStatus(frame) {
  const session = frame.session;
  const output = el('preview-status');
  if (session.mode === 'watch' && session.gains_sha256) {
    const arms = frame.arms && typeof frame.arms === 'object'
      && !Array.isArray(frame.arms) ? frame.arms : {};
    if (session.state === 'stopped' && Object.keys(arms).length === 0) {
      output.textContent = `Read-only joint-angle fence · SHA-256 ${session.gains_sha256}`
        + ' · Unverified (no active Watch arm data)';
      output.className = 'unverified';
      return;
    }
    if (!hasExactArmSet(session.arm_ids, arms)) {
      output.textContent = `Read-only joint-angle fence · SHA-256 ${session.gains_sha256}`
        + ' · Unverified (Watch arm set does not match the session)';
      output.className = 'unverified';
      return;
    }
    const verdicts = session.arm_ids.map((armId) => {
      const arm = arms[armId];
      const result = evaluateReviewedArmFence(arm);
      if (result.reason === 'server-verdict-inconsistent') {
        return {
          status: 'unverified',
          text: `${armId} Unverified (server verdict inconsistent with q/L/U)`,
        };
      }
      if (result.reason === 'server-verdict-unavailable') {
        return {
          status: 'unverified',
          text: `${armId} Unverified (server verdict missing or non-boolean)`,
        };
      }
      return {
        status: result.status,
        text: `${armId} ${result.status === 'inside' ? 'Inside'
          : result.status === 'outside' ? 'OUTSIDE' : 'Unverified'}`,
      };
    });
    if (verdicts.length === 0) {
      output.textContent = `Read-only joint-angle fence · SHA-256 ${session.gains_sha256}`
        + ' · Unverified (no active Watch arm data)';
      output.className = 'unverified';
      return;
    }
    output.textContent = `Read-only joint-angle fence · SHA-256 ${session.gains_sha256}`
      + ` · ${verdicts.map((verdict) => verdict.text).join(' · ')}`;
    output.className = verdicts.some((verdict) => verdict.status === 'outside')
      ? 'outside'
      : verdicts.some((verdict) => verdict.status === 'unverified')
        ? 'unverified' : 'inside';
    return;
  }
  if (session.mode === 'watch') {
    output.textContent = 'None — joint bars use Panda limits for display only';
    output.className = '';
    return;
  }
  if (session.mode === 'motion' && session.gains_sha256
      && session.controller_name === IMPEDANCE_CONTROLLER) {
    output.textContent = `Motion joint fence ${session.gains_sha256.slice(0, 12)}`;
    output.className = '';
    return;
  }
  if (session.mode === 'motion' && session.gains_sha256
      && session.controller_name === HOLD_CONTROLLER) {
    output.textContent = `Reviewed Hold config ${session.gains_sha256.slice(0, 12)}`
      + ' · no joint-angle fence — joint bars use Panda limits for display only';
    output.className = '';
    return;
  }
  if (session.mode === 'motion' && session.gains_sha256) {
    output.textContent = `Reviewed controller config ${session.gains_sha256.slice(0, 12)}`
      + ' · no joint-angle fence preview';
    output.className = '';
    return;
  }
  output.textContent = '—';
  output.className = '';
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

function formatRecoverySteps(steps) {
  return (steps || []).map((step) => {
    const subject = step.arm_id || step.controller || '';
    const phase = step.phase ? `/${step.phase}` : '';
    return `${step.step}${phase}${subject ? ` [${subject}]` : ''}: `
      + `${step.ok ? 'ok' : 'FAILED'}`
      + `${step.detail ? ` (${step.detail})` : ''}`;
  }).join(' · ');
}

async function recover(container, button) {
  state.recoveryPending = true;
  if (button) button.disabled = true;
  let succeeded = false;
  try {
    const result = await api('POST', '/api/session/recover');
    succeeded = true;
    state.localError = null;
    if (container) {
      container.textContent = `${formatRecoverySteps(result.steps)} — `
        + 'all controller-side enables are off; if the fault clears, '
        + 'Enable is a new authorization.';
    }
  } catch (error) {
    state.localError = `${error.error}: ${error.detail}`;
    if (container && error.steps) {
      container.textContent = formatRecoverySteps(error.steps);
    }
  } finally {
    state.recoveryPending = false;
    // A successful command has already restored the stack, but the next SSE
    // frame performs fault -> running. Keep this one-shot button closed across
    // that small gap so a second recovery cannot be queued against old UI.
    if (button) button.disabled = succeeded || !state.token;
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
  const visible = (session.mode === 'motion'
      && (session.state === 'running' || session.state === 'settling'))
    || ((session.mode === 'motion' || session.mode === 'watch')
      && session.state === 'fault');
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
  if (session.state === 'settling') {
    const note = document.createElement('p');
    note.className = 'settling-note';
    note.textContent = SETTLING_NOTICE;
    body.appendChild(note);
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
  const controllerInactive = frame.fault.reasons
    .some((r) => r.code === 'controller_deactivated');
  const isHold = frame.session.controller_name === HOLD_CONTROLLER;
  if (frame.fault.recoverable) {
    note.innerHTML = '<strong>Faulted.</strong> Release the physical stop '
      + '(the pilot’s E-stop / enabling device) first. Recover restores every '
      + 'arm, the hardware, and all state/model broadcasters.'
      + (frame.session.mode === 'motion'
        ? ' The impedance controller is restored last with every controller-side '
          + 'enable confirmed off; Enable afterwards is a new authorization.'
        : ' Watch recovery activates no motion controller.')
      + (controllerInactive ? ' The motion controller is inactive; recovery keeps it inactive '
        + 'until the final controller-restore step.' : '');
  } else if (isHold) {
    note.innerHTML = '<strong>Faulted.</strong> Hold cannot be recovered in place: '
      + 'activating it immediately engages measured-pose effort control. Stop '
      + 'and restart the session.';
  } else {
    note.innerHTML = '<strong>Faulted.</strong> This fault requires stopping and '
      + 'restarting the session.';
  }
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
    const button = document.createElement('button');
    button.className = 'recover-button';
    button.textContent = frame.session.arm_ids.length > 1
      ? 'Recover full session (both arms)' : 'Recover full session';
    button.disabled = !state.token || state.recoveryPending;
    button.addEventListener('click', () => recover(stepsOut, button));
    panel.appendChild(button);
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

function renderArms(arms, session) {
  const container = el('arm-tiles');
  const armIds = Object.keys(arms);
  if (armIds.length === 0) {
    container.innerHTML =
      '<p class="placeholder">No session. Start one to see arm health.</p>';
    return;
  }
  container.innerHTML = '';
  const useReviewedFence = usesReviewedJointFence(session);
  for (const armId of armIds) {
    container.appendChild(renderArmTile(
      arms[armId], useReviewedFence, session && session.mode));
  }
}

function renderArmTile(arm, useReviewedFence, sessionMode) {
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
    tile.appendChild(renderJointRow(arm, name, index, useReviewedFence));
  });

  const meta = document.createElement('div');
  meta.className = 'meta';
  const parts = [];
  if (arm.robot_state.available) {
    const ccsr = arm.robot_state.control_command_success_rate;
    const ccsrText = `ccsr ${ccsr == null ? 'n/a' : ccsr.toFixed(3)}`;
    parts.push(sessionMode === 'watch'
      ? `${ccsrText} (state-only; command-quality gate not applied)`
      : ccsrText);
    parts.push(`mode ${arm.robot_state.robot_mode_label}`);
    if (arm.robot_state.current_errors.length) {
      parts.push(`errors: ${arm.robot_state.current_errors.join(', ')}`);
    }
  } else {
    parts.push('simulated — no Franka health data');
  }
  parts.push(arm.positions_stale === false
    ? `joints ${(arm.positions_age_s ?? 0).toFixed(2)} s old`
    : 'joints STALE');
  meta.textContent = parts.join(' · ');
  tile.appendChild(meta);
  return tile;
}

function isFiniteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

function validPositionArray(positions) {
  return Array.isArray(positions)
    && positions.length === PANDA_LIMITS.length
    && positions.every(isFiniteNumber);
}

function validFenceArrays(motion) {
  return Array.isArray(motion.fence_lower)
    && Array.isArray(motion.fence_upper)
    && motion.fence_lower.length === PANDA_LIMITS.length
    && motion.fence_upper.length === PANDA_LIMITS.length
    && motion.fence_lower.every(isFiniteNumber)
    && motion.fence_upper.every(isFiniteNumber)
    && motion.fence_lower.every((low, index) => low < motion.fence_upper[index]);
}

function hasExactArmSet(expectedArmIds, arms) {
  if (!Array.isArray(expectedArmIds) || arms == null
      || typeof arms !== 'object' || Array.isArray(arms)
      || new Set(expectedArmIds).size !== expectedArmIds.length) {
    return false;
  }
  const actualArmIds = Object.keys(arms);
  return actualArmIds.length === expectedArmIds.length
    && expectedArmIds.every((armId) => typeof armId === 'string'
      && Object.prototype.hasOwnProperty.call(arms, armId));
}

function usesReviewedJointFence(session) {
  return !!session && !!session.gains_sha256
    && (session.mode === 'watch'
      || (session.mode === 'motion'
        && session.controller_name === IMPEDANCE_CONTROLLER));
}

function evaluateReviewedArmFence(arm) {
  if (!arm || typeof arm !== 'object') {
    return { status: 'unverified', reason: 'arm-unavailable' };
  }
  const motion = arm.motion || {};
  if (!validPositionArray(arm.positions)) {
    return { status: 'unverified', reason: 'positions-incomplete' };
  }
  if (!validFenceArrays(motion)) {
    return { status: 'unverified', reason: 'fence-unavailable' };
  }
  if (arm.positions_stale !== false) {
    return { status: 'unverified', reason: 'positions-unverified' };
  }

  const clientInside = arm.positions.every((position, index) =>
    position >= motion.fence_lower[index] && position <= motion.fence_upper[index]);
  if (motion.pose_inside_fence !== true && motion.pose_inside_fence !== false) {
    return { status: 'unverified', reason: 'server-verdict-unavailable' };
  }
  if (motion.pose_inside_fence !== clientInside) {
    return { status: 'unverified', reason: 'server-verdict-inconsistent' };
  }
  return { status: clientInside ? 'inside' : 'outside', reason: null };
}

function evaluateJointFence(arm, index, useReviewedFence) {
  const motion = useReviewedFence ? (arm.motion || {}) : {};
  const lowerArray = motion.fence_lower;
  const upperArray = motion.fence_upper;
  const reviewedFence = validFenceArrays(motion);
  const completePositions = validPositionArray(arm.positions);
  const rawPosition = completePositions ? arm.positions[index] : null;
  const position = isFiniteNumber(rawPosition) ? rawPosition : null;

  // Once the server says a reviewed preview/config is selected, missing or
  // malformed bounds must fail closed as Unverified.  Panda limits are only
  // the explicit bare-Watch, Simulate, or reviewed-Hold display fallback;
  // silently substituting them here would make the summary and joint rows
  // disagree about which fence the operator is inspecting.
  if (useReviewedFence && !reviewedFence) {
    return {
      source: 'reviewed', status: 'unverified', position,
      low: null, high: null, signedMargin: null,
      reason: 'fence-unavailable',
    };
  }

  const low = reviewedFence ? lowerArray[index] : PANDA_LIMITS[index][0];
  const high = reviewedFence ? upperArray[index] : PANDA_LIMITS[index][1];
  if (!reviewedFence) {
    return {
      source: 'panda-display', status: 'display-only', position,
      low, high, signedMargin: null, reason: null,
    };
  }
  const armResult = evaluateReviewedArmFence(arm);
  if (armResult.status === 'unverified' || position == null) {
    return {
      source: 'reviewed', status: 'unverified', position,
      low, high, signedMargin: null,
      reason: position == null ? 'positions-incomplete' : armResult.reason,
    };
  }

  const signedMargin = Math.min(position - low, high - position);
  return {
    source: 'reviewed',
    status: position >= low && position <= high ? 'inside' : 'outside',
    position,
    low,
    high,
    signedMargin,
    reason: null,
  };
}

function formatRad(value) {
  return isFiniteNumber(value) ? value.toFixed(6) : '—';
}

function fenceDetailText(result) {
  const { low, high, signedMargin } = result;
  if (result.source === 'panda-display') {
    return `Panda limits — display only · L ${formatRad(low)}`
      + ` · U ${formatRad(high)}`;
  }
  if (low == null || high == null) {
    return 'Unverified · reviewed fence bounds unavailable'
      + ' · nearest signed margin —';
  }
  if (result.reason === 'server-verdict-inconsistent') {
    return 'Unverified · server fence verdict is inconsistent with q/L/U'
      + ' · nearest signed margin —';
  }
  if (result.reason === 'server-verdict-unavailable') {
    return 'Unverified · server fence verdict is missing or non-boolean'
      + ' · nearest signed margin —';
  }
  if (result.status === 'unverified') {
    return `Unverified · L ${formatRad(low)} · U ${formatRad(high)}`
      + ' · nearest signed margin —';
  }
  return `${result.status === 'inside' ? 'Inside' : 'OUTSIDE'}`
    + ` · L ${formatRad(low)} · U ${formatRad(high)}`
    + ` · nearest signed margin ${signedMargin >= 0 ? '+' : ''}`
    + `${formatRad(signedMargin)} rad`;
}

function renderJointRow(arm, name, index, useReviewedFence) {
  const row = document.createElement('div');
  row.className = 'joint';

  const label = document.createElement('span');
  label.className = 'name';
  label.textContent = `J${index + 1}`;
  row.appendChild(label);

  const bar = document.createElement('div');
  bar.className = `bar${arm.positions_stale === false ? '' : ' stale'}`;
  const marker = document.createElement('div');
  marker.className = 'marker';
  const result = evaluateJointFence(arm, index, useReviewedFence);
  const { position, low, high } = result;
  row.dataset.fenceSource = result.source;
  row.dataset.fenceStatus = result.status;
  if (result.status === 'inside' || result.status === 'outside'
      || result.status === 'unverified') {
    row.classList.add(result.status);
  }
  if (position != null && low != null && high != null && high > low) {
    const fraction = Math.min(1, Math.max(0, (position - low) / (high - low)));
    marker.style.left = `calc(${(fraction * 100).toFixed(1)}% - 1px)`;
  } else {
    marker.style.left = '0';
  }
  bar.appendChild(marker);
  row.appendChild(bar);

  const readout = document.createElement('span');
  readout.className = 'joint-readout';
  const value = document.createElement('span');
  value.className = 'value';
  value.textContent = position == null ? 'q —' : `q ${formatRad(position)} rad`;
  readout.appendChild(value);
  const detail = document.createElement('span');
  detail.className = 'fence-detail';
  detail.textContent = fenceDetailText(result);
  readout.appendChild(detail);
  row.appendChild(readout);
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
  for (const radio of document.querySelectorAll('input[name="arms"]')) {
    radio.addEventListener('change', syncMotionControls);
  }
  el('controller').addEventListener('change', () => rebuildGainsList());
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

if (typeof module === 'object' && module.exports) {
  module.exports = {
    evaluateJointFence,
    fenceDetailText,
    renderControl,
    renderPreviewStatus,
    sessionStateLabel,
    usesReviewedJointFence,
  };
} else {
  boot();
}
