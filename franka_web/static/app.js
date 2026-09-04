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

(function () {
'use strict';

/* ------------------------------------------------- constants and state --- */

var DEFAULTS = {                     // used only until /api/capabilities answers
  heartbeat_ms: 5000, log_ring_lines: 500, joint_count: 7, poll_ms: 1000
};
// How far frame.logs.last_seq may run ahead of what this page has rendered
// before a backfill is worth doing. The production queue depth is 64, and the
// frame pump also caps its drain at the newest 16 events per tick, so falling
// tens of lines behind during a launch burst is normal, not a fault.
var LOG_GAP_TOLERANCE = 64;
var RESYNC_DEBOUNCE_MS = 2000;
var NOTICE_MS = 8000;
var COPIED_MS = 1200;
var CLAMP_FLASH_MS = 600;
var RAD_TO_DEG = 180 / Math.PI;
// How long the Recover control may stay in its pending state on the strength
// of the request alone. The server installs the recovery checklist as the
// FIRST thing it does once it takes the command, so silence past this is
// silence: the command is still queued behind a long operation, or the answer
// is lost. Live finding V2L-7: a second Recover press produced no
// 'recovery started' line server-side at all, and the page sat on
// "Recovering" indefinitely. Generous enough to cover a busy supervisor
// finishing a stop ladder, short enough that the operator is told.
var RECOVER_PENDING_MS = 12000;

var ui = {                           // survives every rebuild; never read from the DOM
  logOpen: false, logFollow: true, tmplOpen: {}, takeoverOpen: false,
  infoOpen: false, notice: null, noticeUntil: 0, pending: {}, copied: {},
  clamped: {}, selArms: 'both', selMode: 'motion',
  stageSignature: null, badgeSignature: null,
  // Ghost visibility and divergence are per tab and die with the tab: a
  // scratchpad restored beside a robot that has since moved is worse than no
  // scratchpad, so nothing here is persisted anywhere.
  ghostShown: {}, ghostDiffers: {}, ghostSegSignature: null,
  // Wall-clock deadline for the Recover pending state, and whether a recover
  // request is still unanswered. Both exist so the pending state can only
  // outlive the request while the SERVER says a recovery is running.
  recoverUntil: 0, recoverInFlight: false
};
var net = {
  caps: null, config: null, configTried: false, token: null, claimId: null,
  frame: null, lastServerTime: '', lastUptime: null,
  lastSeq: 0, warnCount: 0, errorCount: 0, dropped: 0,
  // live: null until the transport has said anything. false only after a real
  // stream error, so the shell does not claim 'reconnecting' before it has
  // ever connected.
  source: null, pollTimer: null, live: null,
  heartbeatTimer: null, resyncTimer: null, resyncing: false, restarting: false,
  lastSessionId: null
};
// Cached element references for the current stage structure. Rebuilt only
// when the structure signature changes (see render()).
var dom = {kind: null, steps: {}, arms: {}, recFinal: null, profileFor: null};

// Everything the 3D panel needs. The panel is a PANEL, not a mode: nothing
// here is keyed on session.mode, and the panel is never created or destroyed
// by a stage rebuild.
var scene = {
  info: null,            // the last scene description from the server
  handle: null,          // the mounted module handle, or null
  threePromise: null, modulePromise: null, mounting: false, failed: null,
  fetching: false,
  open: false,           // panel expanded
  present: {},           // armId -> in this session
  copy: {},              // armIndex -> the last server-computed copy payload
  verdict: {},           // armIndex -> the last verdict for that arm
  // armIndex -> the full-precision `positions` of that same solve. Apply
  // echoes the SERVER's own numbers for the pose on screen back to it; the
  // page never authors a joint vector, and the server re-validates them
  // anyway, so this is defence in depth rather than a delegation of trust.
  solved: {},
  applyNote: {},         // armId -> the pinned sentence of the last refusal
  // PER ARM, all three of them. A ghost's sentence, its verdict and its
  // copied snippet each describe one arm's pose; holding any of them in a
  // single slot is what made the panel answer for panda1 while the operator
  // was working on panda2.
  moduleNote: {},        // armIndex -> a sentence the 3D module authored
  solveNote: null,       // the IK-timeout sentence (panel-wide: the service)
  copiedText: {},        // armId -> the snippet last put on the clipboard
  failedKind: null,      // 'drawing' | 'assets' — which sentence the panel owes
  // The rows one re-check is answering for, and the ghosts still worth
  // asking. A re-check that never comes back solved must not leave the rows
  // it was going to refresh describing the cell as it was.
  recheckRows: [], recheckQueue: [],
  // Every solve that goes out is numbered, and `recheckAt` is the number the
  // last re-check round opened at. An answer to a question asked BEFORE that
  // describes the cell as it was before the change, so it refreshes nothing
  // the round is waiting on.
  solveSeq: 0, recheckAt: 0,
  rateNoticeSince: 0,
  webgl2: null
};

//: The source segment's three labels. `ghost` is `jog`'s sibling: both are
//: computed and streamed by the server, and only `external` hands the topic
//: to the operator's own node.
var SOURCE_LABELS = {jog: 'Jog', external: 'External', ghost: 'Ghost'};

var SCENE_NO_SESSION =
  'Start a session to see the arms. The measured cell is drawn from your '
  + 'workspace model.';
var SCENE_NO_IK =
  'Pose editing needs the IK service. Start it with '
  + '"ros2 launch franka_ik franka_ik.launch.py".';
var SCENE_IK_TIMEOUT = 'The IK service did not answer. Check that it is still running.';
var SCENE_NO_WEBGL =
  'This browser cannot draw the 3D scene (WebGL2 is required). Everything else '
  + 'on this page works normally.';
var SCENE_NO_ASSETS =
  'The 3D model files did not load. Reload the page; if it keeps failing, check '
  + 'the server log.';
var SCENE_CATCHING_UP = 'The console is catching up with your drag.';
var SCENE_NOT_RECHECKED =
  'The cell was not checked again after that change, so the lines above it '
  + 'were cleared. Move a ghost to ask again.';
//: How long refusals must persist before the panel says anything at all. A
//: throttled drag is not a refused action, so it never reaches the notice row.
var SCENE_RATE_QUIET_MS = 1500;
//: The panel's colour tokens, and the module's palette keys they feed.
var SCENE_PALETTE_KEYS = {
  '--scene-bg': 'sceneBg', '--scene-grid': 'grid', '--scene-grid-major': 'gridMajor',
  '--scene-stale': 'stale', '--cell-line': 'cellLine', '--cell-floor': 'cellFloor',
  '--ghost-1': 'ghost1', '--ghost-2': 'ghost2', '--ghost-collide': 'ghostCollide',
  '--ghost-unchecked': 'ghostUnchecked', '--handle': 'handle',
  '--handle-active': 'handleActive', '--handle-refused': 'handleRefused',
  '--ring': 'ring', '--ring-active': 'ringActive',
  '--axis-x': 'axisX', '--axis-y': 'axisY', '--axis-z': 'axisZ'
};
var SCENE_NARROW = '(max-width: 1020px)';

/* --------------------------------------------------------------- helpers --- */

// h('div', {class: 'jrow', dataset: {arm: 'panda1'}}, [child, 'text'])
function h(tag, attrs, kids) {
  var node = document.createElement(tag);
  if (attrs) {
    for (var key in attrs) {
      if (!Object.prototype.hasOwnProperty.call(attrs, key)) continue;
      var value = attrs[key];
      if (key === 'class') node.className = value;
      else if (key === 'text') node.textContent = value;          // never markup
      else if (key === 'dataset') { for (var d in value) node.dataset[d] = value[d]; }
      else if (value === true) node.setAttribute(key, '');
      else if (value !== false && value != null) node.setAttribute(key, String(value));
    }
  }
  (kids || []).forEach(function (kid) {
    if (kid == null) return;
    node.appendChild(typeof kid === 'string' ? document.createTextNode(kid) : kid);
  });
  return node;
}

function el(id) { return document.getElementById(id); }

function tickSvg() {                     // the checklist tick, cloned per use
  return el('tickTmpl').content.firstElementChild.cloneNode(true);
}

function deg(rad) { return rad == null ? null : rad * RAD_TO_DEG; }

function fmtDeg(rad, digits) {
  var value = deg(rad);
  return value == null || !isFinite(value)
    ? '—'
    : value.toFixed(digits == null ? 2 : digits) + '°';
}

function fmtRate(hz) {
  if (hz == null) return '—';
  return (Math.abs(hz - Math.round(hz)) < 0.05 ? String(Math.round(hz)) : hz.toFixed(1)) + ' Hz';
}

function fmtNum(value) {
  return value == null || !isFinite(value) ? '—' : String(value);
}

function pad2(n) { return (n < 10 ? '0' : '') + n; }

function fmtLogTime(t) {
  var ms = Date.parse(t);
  if (isNaN(ms)) return String(t);
  var d = new Date(ms);
  return pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds())
    + '.' + ('00' + d.getMilliseconds()).slice(-3);
}

function fmtClock(t) {
  var ms = Date.parse(t);
  if (isNaN(ms)) return '—';
  var d = new Date(ms);
  return pad2(d.getHours()) + ':' + pad2(d.getMinutes());
}

function parse(text) {
  try { return JSON.parse(text); } catch (error) { return null; }
}

function armIds(frame) {
  return (frame && frame.session && frame.session.arm_ids) || [];
}

function armOf(frame, armId) {
  return (frame && frame.arms && frame.arms[armId]) || null;
}

function motionOf(frame, armId) {
  var arm = armOf(frame, armId);
  return (arm && arm.motion) || {};
}

function gripperOf(frame, armId) {
  var arm = armOf(frame, armId);
  return (arm && arm.gripper) || {};
}

function jointCount(arm) {
  if (arm && arm.joint_names && arm.joint_names.length) return arm.joint_names.length;
  if (arm && arm.positions && arm.positions.length) return arm.positions.length;
  return 0;
}

/* ------------------------------------------------------------- transport --- */

function api(method, path, body) {
  var headers = {};
  if (net.token) headers['X-Operator-Token'] = net.token;
  if (body !== undefined) headers['Content-Type'] = 'application/json; charset=utf-8';
  var response;
  return fetch(path, {
    method: method,
    headers: headers,
    body: body === undefined ? undefined : JSON.stringify(body)
  }).then(function (result) {
    response = result;
    return response.json().catch(function () {
      throw {ok: false, error: 'transport_error',
             detail: 'HTTP ' + response.status + ' without a JSON body'};
    });
  }, function () {
    throw {ok: false, error: 'transport_error', detail: 'server unreachable'};
  }).then(function (payload) {
    if (!payload.ok) throw payload;                 // the failure envelope
    return payload;
  });
}

/* ----------------------------------------------------------- notices --- */

function notice(text) { ui.notice = text; ui.noticeUntil = Date.now() + NOTICE_MS; }

function noticeFromError(error) {
  notice(error && error.error ? error.error + ': ' + (error.detail || '')
                              : 'the request did not complete');
}

function syncNotice() {
  var bar = el('noticeBar');
  var live = ui.notice != null && Date.now() < ui.noticeUntil;
  bar.hidden = !live;
  el('noticeText').textContent = live ? ui.notice : '';
}

/* --------------------------------------------------------- operator lock --- */

function lockIsMine(frame) {
  return !!(frame && frame.operator && frame.operator.locked
            && frame.operator.claim_id === net.claimId);
}

function lockIsElsewhere(frame) {
  return !!(frame && frame.operator && frame.operator.locked
            && frame.operator.claim_id !== net.claimId);
}

function claim() {                                   // POST /api/operator/claim
  return api('POST', '/api/operator/claim').then(function (result) {
    net.token = result.token;
    net.claimId = result.claim_id;
    startHeartbeat();
    return result;
  });
}

function takeover() {                                // POST /api/operator/takeover
  return api('POST', '/api/operator/takeover').then(function (result) {
    net.token = result.token;
    net.claimId = result.claim_id;
    startHeartbeat();
    return result;
  });
}

function startHeartbeat() {
  if (net.heartbeatTimer) clearInterval(net.heartbeatTimer);
  var period = (net.caps && net.caps.operator_heartbeat_interval_s * 1000)
    || DEFAULTS.heartbeat_ms;
  net.heartbeatTimer = setInterval(function () {
    if (!net.token) return;
    api('POST', '/api/operator/heartbeat').catch(function () {
      net.token = null; net.claimId = null;            // TTL lapsed or server restarted
      clearInterval(net.heartbeatTimer); net.heartbeatTimer = null;
      render();                                        // badge falls back to 'nobody'
    });
  }, period);
}

// Every mutating action funnels through here. Claim happens on the FIRST such
// action and never on load.
function withLock(run) {
  if (net.token) return run().catch(function (error) {
    if (error && error.error === 'operator_token_invalid') {
      net.token = null; net.claimId = null;
      return claim().then(run);                        // exactly one retry
    }
    throw error;
  });
  return claim().then(run).catch(function (error) {
    if (error && error.error === 'operator_lock_held') {
      ui.takeoverOpen = true;                          // offer Take over, never force it
      notice('Another program holds control. Use Take over to command the arms.');
      render();
      return null;
    }
    throw error;
  });
}

function releaseOnUnload() {
  if (!net.token) return;
  var token = net.token;
  net.token = null;
  try {
    fetch('/api/operator/release',
          {method: 'POST', keepalive: true, headers: {'X-Operator-Token': token}});
  } catch (error) { /* the 15 s TTL is the backstop */ }
}

/* ---------------------------------------------------------------- actions --- */

function runAction(key, run, keepPending) {
  // Disabling a focused control for the duration of its request moves focus to
  // <body>; re-enabling it does not bring focus back. Put it back so a
  // keyboard operator can press the same control again — jog especially, and
  // the enable switch, which is this page's only instant-disable affordance.
  var wasFocused = document.activeElement;
  function restoreFocus() {
    if (!wasFocused || wasFocused === document.body) return;
    if (document.activeElement !== document.body) return;   // focus moved on
    if (!document.body.contains(wasFocused) || wasFocused.disabled) return;
    wasFocused.focus();
  }
  if (key) {
    if (ui.pending[key]) return;
    ui.pending[key] = true;
    render();
  }
  withLock(run).then(function () {
    if (key && !keepPending) delete ui.pending[key];
    render();
    restoreFocus();
  }, function (error) {
    if (key) delete ui.pending[key];
    noticeFromError(error);
    render();
    restoreFocus();
  });
}

function sessionLocked(frame) {
  return !!(frame && frame.session && frame.session.state !== 'stopped');
}

// The execCommand path is not a formality. The server binds every interface
// and the daily journey is "open the page from a laptop on the lab network by
// the computer's IP" — which is not a secure context, so navigator.clipboard
// is undefined exactly where the console is actually used.
function writeClipboard(text, key, button) {
  if (!text) return;
  function done() {
    ui.copied[key] = Date.now() + COPIED_MS;
    if (button) button.textContent = 'Copied';
    setTimeout(function () { delete ui.copied[key]; render(); }, COPIED_MS);
  }
  function fallback() {
    try {
      var area = document.createElement('textarea');
      area.value = text;
      area.style.position = 'fixed';
      area.style.opacity = '0';
      document.body.appendChild(area);
      area.select();
      document.execCommand('copy');
      document.body.removeChild(area);
      done();
    } catch (error) { /* nothing else to try */ }
  }
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).then(done, fallback);
  } else {
    fallback();
  }
}

function copyText(key, button) {
  var parts = String(key).split(':');
  var motion = motionOf(net.frame, parts[1]);
  var text = parts[0] === 'topic' ? motion.command_topic : motion.command_template;
  writeClipboard(text, key, button);
}

/* ------------------------------------------------------------ 3D panel --- */

function sceneArmIds() {
  return (scene.info && Array.isArray(scene.info.arms)) ? scene.info.arms : [];
}

function armIndexOf(armId) {
  return Number(String(armId).slice(-1));
}

function sessionHasArms() {
  return armIds(net.frame).length > 0;
}

function supportsWebgl2() {
  if (scene.webgl2 !== null) return scene.webgl2;
  try {
    var probe = document.createElement('canvas').getContext('webgl2');
    scene.webgl2 = !!probe;
    if (probe && probe.getExtension) {
      var lose = probe.getExtension('WEBGL_lose_context');
      if (lose) lose.loseContext();
    }
  } catch (error) { scene.webgl2 = false; }
  return scene.webgl2;
}

function currentTheme() {
  var explicit = document.documentElement.dataset.theme;
  if (explicit === 'dark' || explicit === 'light') return explicit;
  return (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches)
    ? 'dark' : 'light';
}

// The stylesheet owns every colour the 3D view draws. A value that is not a
// resolved colour is dropped rather than handed on, so the module falls back
// per key instead of being given a string it cannot parse.
function readScenePalette() {
  var computed = getComputedStyle(document.documentElement);
  var out = {};
  for (var token in SCENE_PALETTE_KEYS) {
    if (!Object.prototype.hasOwnProperty.call(SCENE_PALETTE_KEYS, token)) continue;
    var value = String(computed.getPropertyValue(token) || '').trim();
    if (/^(#[0-9a-fA-F]{3,8}|rgba?\(|hsla?\()/.test(value)) {
      out[SCENE_PALETTE_KEYS[token]] = value;
    }
  }
  return out;
}

function fetchScene() {
  if (scene.fetching) return Promise.resolve();
  scene.fetching = true;
  return api('GET', '/api/scene').then(function (result) {
    scene.fetching = false;
    scene.info = result;
    if (scene.handle) {
      scene.handle.setCell(result.cell);
      scene.handle.setEnabled(result.ghost_available === true && sessionHasArms());
    }
    syncScenePanel();
    ensureScene();
  }, function () {
    scene.fetching = false;
    scene.info = null;
    syncScenePanel();
  });
}

function fetchSceneIfNeeded() {
  if (scene.info || scene.fetching) return;
  fetchScene();
}

// Lazily, and only once. A phone that never opens the panel downloads none of
// the 3D payload at all, and a bootstrap failure is a PANEL failure: the rest
// of the console is untouched.
function loadThree() {
  if (scene.threePromise) return scene.threePromise;
  scene.threePromise = new Promise(function (resolve, reject) {
    var tag = document.createElement('script');
    tag.src = '/ghost/vendor/three.r111.min.js';
    tag.onload = function () { resolve(window.THREE); };
    tag.onerror = function () { reject(new Error('the 3D library did not load')); };
    document.head.appendChild(tag);
  });
  return scene.threePromise;
}

function ensureScene() {
  if (scene.handle || scene.mounting || scene.failed || !scene.open || !scene.info) return;
  if (!scene.info.assets) {
    // A designed state, not an accident: the generated model tree is absent,
    // which is ordinary on a workspace that has not been built.
    scene.failed = new Error('assets absent');
    sceneFallbackText('assets');
    return;
  }
  if (!supportsWebgl2()) {
    scene.failed = new Error('webgl2 absent');
    sceneFallbackText('drawing');
    return;
  }
  scene.mounting = true;
  loadThree().then(function () {
    return scene.modulePromise || (scene.modulePromise = import('/ghost/ghost.js'));
  }).then(function (module) {
    return module.mount(el('sceneView'), {
      urdfUrl: scene.info.assets.urdf_url,
      manifestUrl: scene.info.assets.manifest_url,
      assetBase: scene.info.assets.asset_base,
      arms: sceneArmIds().map(function (armId) {
        return {armIndex: armIndexOf(armId), armId: armId};
      }),
      initialArm: armIndexOf(sceneArmIds()[0]),
      cell: scene.info.cell,
      theme: currentTheme(),
      onSolveRequest: sceneSolve,
      onGhostChanged: onGhostChanged
    });
  }).then(function (handle) {
    scene.handle = handle;
    scene.mounting = false;
    handle.setTheme(currentTheme(), readScenePalette());
    handle.setEnabled(scene.info.ghost_available === true && sessionHasArms());
    if (net.frame) syncScene(net.frame);
    syncScenePanel();
  }, function (error) {
    scene.mounting = false;
    scene.failed = error;
    sceneFallbackText('assets');
  });
}

function onGhostChanged(armIndex, positions7, differs) {
  ui.ghostDiffers['panda' + armIndex] = differs === true;
  syncScenePanel();
}

/* --- the solve route: the only place in this file that names an endpoint --- */

function sceneSolve(request) {
  if (request.kind === 'status') {
    // Not a request at all: the 3D module telling the panel something about
    // the gesture in progress. It carries no endpoint and goes nowhere.
    if (request.drawing === false) {
      // The drawing context went away. That is a PANEL failure and it gets the
      // panel's own sentence; nothing else on the page is affected.
      scene.failed = new Error('drawing context lost');
      sceneFallbackText('drawing');
      return Promise.resolve({ok: true});
    }
    if (request.armIndex != null) scene.moduleNote[request.armIndex] = request.text || null;
    if (request.verdict === 'pending') {
      scene.verdict[request.armIndex] = {status: 'pending', reason: null};
      if (scene.handle) scene.handle.setVerdict(request.armIndex, {status: 'pending'});
    }
    syncScenePanel();
    return Promise.resolve({ok: true});
  }
  var path = request.kind === 'redundancy' ? '/api/ghost/redundancy' : '/api/ghost/solve';
  var armId = 'panda' + request.armIndex;
  var body = {arm_id: armId, seed: request.seed, target: request.target};
  if (request.kind === 'redundancy') {
    body.samples = request.samples || 25;
  } else {
    body.redundancy = request.redundancy;
    body.scene = sceneVector();
  }
  scene.solveSeq += 1;
  var issued = scene.solveSeq;
  return api('POST', path, body).then(function (result) {
    if (request.kind === 'solve') {
      if (request.recheck === true) absorbRecheck(result);
      // Only an answer to a question asked after the round opened has
      // rewritten the rows the round is waiting on. An ordinary solve that
      // was already on the wire when a ghost was hidden answers about the
      // cell that ghost was still in, and the re-check still owes the rows.
      else absorbSolve(request.armIndex, result, issued > scene.recheckAt);
    }
    scene.rateNoticeSince = 0;
    return result;
  }, function (error) {
    if (error && error.error === 'ghost_unavailable') {
      // "Not running" and "did not answer" are different states with different
      // one-line fixes, and one code with one detail cannot carry both.
      scene.solveNote = error.ik_state === 'timeout' ? SCENE_IK_TIMEOUT : null;
      fetchScene();
    }
    if (error && error.error === 'ghost_rate_limited') {
      if (!scene.rateNoticeSince) scene.rateNoticeSince = Date.now();
      syncScenePanel();
      throw {retryAfterMs: Number(error.retry_after_ms) || 40};
    }
    // A re-check that never got an answer refreshed nothing, so the rows it
    // was going to speak for are handled the same way an unsolved one is.
    if (request.kind === 'solve' && request.recheck === true) askNextRecheck();
    syncScenePanel();
    throw error;
  });
}

// What the user SEES, per arm: the ghost pose where a ghost is shown, the
// interpolated measured pose otherwise. An arm that is not in the session has
// no entry at all — omitted, never sent as a null.
function sceneVector() {
  var out = {};
  sceneArmIds().forEach(function (armId) {
    if (scene.present[armId] !== true) return;
    var pose = scene.handle.getRenderedPose(armIndexOf(armId));
    if (pose) out[armId] = pose;
  });
  return out;
}

function absorbSolve(armIndex, result, discharges) {
  if (result.solved === true) {
    scene.solveNote = null;
    scene.moduleNote[armIndex] = null;
    // THIS arm's pose moved, so the snippet on screen no longer describes it.
    // A snippet that outlives its pose is the one failure Copy must not have
    // — and the neighbour's snippet still describes the neighbour, so it
    // stays.
    scene.copiedText['panda' + armIndex] = null;
    scene.copy[armIndex] = result.copy || null;
    scene.solved[armIndex] = result.positions || null;
    absorbSceneVerdict(result.verdict || null, discharges === true);
  } else {
    scene.moduleNote[armIndex] = result.solve_reason || scene.moduleNote[armIndex];
  }
  syncScenePanel();
}

// A re-check answers one question -- is what is drawn still allowed -- and
// answers it for every ghost on screen at once. It carries no new pose, so it
// must not touch a Copy payload, a snippet or a note: those describe poses
// that did not move, and only the verdicts did.
function absorbRecheck(result) {
  if (result && result.solved === true && result.verdict) {
    absorbSceneVerdict(result.verdict, true);
    syncScenePanel();
    return;
  }
  // The re-check came back unsolved, so nothing on screen was refreshed and
  // every line still describes the cell as it was BEFORE the change. Ask the
  // next shown ghost; when none is left, the rows go blank rather than go on
  // asserting a cell that is gone. A blank row is honest, a stale one is not.
  askNextRecheck();
}

// The cell changed with no gesture behind it -- a ghost hidden, a ghost reset
// -- so every sentence on screen now describes a cell that is gone. One
// re-check of one shown ghost re-asks about the WHOLE cell and so refreshes
// every row; asking each arm separately would be the same answer twice. The
// others are kept as fallbacks, not asked: a re-check target is the FK of a
// pose that may itself sit past a joint limit, which the solver may refuse.
function recheckScene() {
  if (!scene.handle) return;
  var shown = shownGhostArms();
  if (!shown.length) return;
  scene.recheckRows = shown.slice();
  scene.recheckQueue = shown.slice();
  scene.recheckAt = scene.solveSeq;
  askNextRecheck();
}

// Ask the next ghost that can be asked; clear what could not be refreshed.
function askNextRecheck() {
  var queue = scene.recheckQueue || [];
  while (queue.length) {
    if (scene.handle && scene.handle.recheckVerdict(armIndexOf(queue.shift()))) {
      return;
    }
  }
  var rows = scene.recheckRows || [];
  scene.recheckRows = [];
  if (!rows.length) return;
  rows.forEach(function (armId) { blankVerdict(armIndexOf(armId)); });
  scene.solveNote = SCENE_NOT_RECHECKED;
  syncScenePanel();
}

// A row nothing could check: it reads blank and tints neutral exactly as a
// row nobody has asked about, because that is what it is. The flag rides on
// the row itself rather than beside it, so it cannot outlive the row -- the
// next answer overwrites both at once -- and it is what keeps Copy shut. The
// pose is still the operator's and still on screen, but nothing has cleared
// it against the cell as it is now, and an uncleared pose is not handed on.
function blankVerdict(index) {
  scene.verdict[index] = {status: null, notChecked: true};
  if (scene.handle) scene.handle.setVerdict(index, null);
}

// One arm's share of a whole-cell verdict, in the same shape the server sends
// -- so the line, the tint, Copy and Apply all read one arm's verdict and
// nothing else's.
//
// THE DEFECT THIS EXISTS FOR. The check is asked about the whole cell and
// answers once, and `reason` is the worst thing it found ANYWHERE. Writing
// that sentence into the row of whichever arm happened to be dragged put
// "Panda 2 joint 4 is 4.0° past its limit." above Panda 1's degrees, and left
// it there until Panda 1 was dragged again. An arm's row must say what is
// wrong with THAT ARM, and must be rewritten by every check.
//
// The attribution is the SERVER'S, read out of `verdict.arms`, and this file
// makes none of its own. It cannot: the itemised `contacts` list is bounded
// for the wire, so an arm whose only contact sorts past the bound is absent
// from it and is not thereby clear. Deciding "no mention, therefore clear"
// off a list that may be partial is how a refused arm came to read green.
function verdictForArm(verdict, armId) {
  if (!verdict) return null;
  if (verdict.status !== 'collision') return verdict;
  var mine = verdict.arms ? verdict.arms[armId] : null;
  // No attribution to read: keep the whole-cell verdict rather than invent
  // one. Reading worse than before is a bug; reading clear when something is
  // not is a lie, and this fails towards the bug.
  if (!mine || typeof mine !== 'object') return verdict;
  // Clear is read POSITIVELY, off the word itself. Reading everything that is
  // not the string 'collision' as clear is the same "absence means clear"
  // reasoning this round removed everywhere else: a status this page does not
  // recognise says nothing about this arm, so the whole cell's answer stands.
  if (mine.status === 'clear') {
    return {status: 'clear', min_clearance: verdict.min_clearance,
            offending_links: [], reason: null, reason_code: null,
            checker: verdict.checker};
  }
  if (mine.status !== 'collision') return verdict;
  return {
    status: 'collision', min_clearance: verdict.min_clearance,
    offending_links: mine.offending_links || [],
    reason: mine.reason || verdict.reason,
    reason_code: verdict.reason_code, checker: verdict.checker
  };
}

// EVERY shown ghost's verdict, from ONE whole-cell answer. Called on every
// solve, because every solve re-checks the whole cell: a row refreshed only
// when its own arm is dragged is a row that can show a neighbour's fault long
// after the neighbour moved clear. A ghost that is not on screen gets
// nothing — there is no pose of its on the page for a sentence to be about.
//
// A REFUSAL IS NEVER UN-RENDERED. The checker looks at every arm in the cell,
// drawn as a ghost or standing where it is measured, so a whole-cell refusal
// can name only an arm with no ghost on screen — which leaves the sentence
// with no row to go in, and every row that IS on screen reading clear about a
// cell that was refused. When that happens the shown rows carry the
// whole-cell sentence instead: it is about a neighbour they are not drawing,
// but it is the answer, and Copy and Apply stay shut on it.
function absorbSceneVerdict(verdict, discharges) {
  // A whole-cell answer to a question asked AFTER the round opened just
  // rewrote every row, which is what the round was for. One asked before it
  // did not: it describes the cell as it was, so the round still stands and
  // the re-check it is waiting on may still have to blank the rows.
  if (discharges) {
    scene.recheckRows = [];
    scene.recheckQueue = [];
  }
  var shown = shownGhostArms();
  var refused = verdict && verdict.status === 'collision';
  var mineOf = {};
  var named = false;
  shown.forEach(function (armId) {
    mineOf[armId] = verdictForArm(verdict, armId);
    if (mineOf[armId] && mineOf[armId].status === 'collision') named = true;
  });
  sceneArmIds().forEach(function (armId) {
    var index = armIndexOf(armId);
    var mine = shown.indexOf(armId) >= 0 ? mineOf[armId] : null;
    if (mine && refused && !named) {
      // No link of THIS arm is at fault, so nothing of it is tinted; the
      // sentence is the whole cell's, which is whose fault it actually is.
      mine = {status: 'collision', min_clearance: verdict.min_clearance,
              offending_links: [], reason: verdict.reason,
              reason_code: verdict.reason_code, checker: verdict.checker};
    }
    scene.verdict[index] = mine;
    if (scene.handle) scene.handle.setVerdict(index, mine);
  });
}

// A refused Apply must be legible in two places at once: pinned under the
// button, and tinted in the scene. Both render the SERVER's sentence, with
// textContent only.
function absorbApplyRefusal(armId, error) {
  if (!error || typeof error.detail !== 'string') throw error;
  if (error.error !== 'apply_refused' && error.error !== 'apply_unavailable'
      && error.error !== 'apply_in_progress') {
    throw error;
  }
  scene.applyNote[armId] = error.detail;
  var links = Array.isArray(error.offending_links) ? error.offending_links : [];
  if (scene.handle && links.length) {
    // The ghost POSE may be clear while the PATH to it is not, so this tint
    // says something slightly stronger than the truth. The sentence carries
    // the distinction ("on the way there"), and reusing the existing token
    // keeps this build out of the scene module entirely.
    scene.handle.setVerdict(armIndexOf(armId), {
      status: 'collision', reason: error.detail, offending_links: links});
  }
  throw error;
}

/* ------------------------------------------------------- the frame feed --- */

function syncScene(frame) {
  syncScenePanel();
  if (!scene.handle || !frame) return;
  var hz = net.caps && net.caps.state_frame_hz;
  var period = (typeof hz === 'number' && hz > 0) ? 1000 / hz : 1000 / 5;
  var arrival = performance.now();
  var live = armIds(frame);
  var known = sceneArmIds().length ? sceneArmIds() : live;
  known.forEach(function (armId) {
    var index = armIndexOf(armId);
    var present = live.indexOf(armId) >= 0;
    var appeared = present && scene.present[armId] !== true;
    scene.present[armId] = present;
    if (!present) { scene.handle.setMeasured(index, null, {}); return; }
    var arm = armOf(frame, armId) || {};
    var positions = Array.isArray(arm.positions) ? arm.positions : null;
    scene.handle.setStale(index, arm.positions_stale === true);
    scene.handle.setMeasured(index, positions, {
      arrivalMs: arrival, framePeriodMs: period,
      snap: appeared || net.restarting === true || arm.positions_stale === true
    });
  });
}

/* --------------------------------------------------------- panel driver --- */

function setSceneOpen(open) {
  // No scroll correction, deliberately: this panel is in normal flow, so
  // expanding it grows the document below the fold rather than under a fixed
  // overlay — and the drawer's own correction measures a reservation this
  // panel must never change.
  scene.open = open;
  document.body.classList.toggle('scene-open', open);
  el('sceneBody').hidden = !open;
  // The ghost controls belong to the open panel. Hiding them takes them out of
  // the tab order too, so a collapsed bar is one stop, not five.
  el('sceneToolbar').hidden = !open;
  el('sceneBar').setAttribute('aria-expanded', open ? 'true' : 'false');
  if (open) { fetchSceneIfNeeded(); ensureScene(); }
  syncScenePanel();
}

// The panel's own failure sentence. `kind` is 'drawing' when the browser
// cannot draw at all — no WebGL2, or a context that went away under us — and
// 'assets' when the model files are absent or would not load.
function sceneFallbackText(kind) {
  if (kind) scene.failedKind = kind;
  var view = el('sceneView');
  if (!view) return;
  view.replaceChildren(h('p', {class: 'scene-fallback', text: sceneFallbackSentence()}));
  syncScenePanel();
}

function sceneFallbackSentence() {
  if (scene.failedKind === 'drawing' || !supportsWebgl2()) return SCENE_NO_WEBGL;
  return SCENE_NO_ASSETS;
}

function sceneNoteFor() {
  if (scene.failed) return sceneFallbackSentence();
  if (!scene.info) return null;
  if (!sessionHasArms()) return SCENE_NO_SESSION;
  if (scene.info.ghost_available !== true) return SCENE_NO_IK;
  if (scene.solveNote) return scene.solveNote;
  if (scene.rateNoticeSince && Date.now() - scene.rateNoticeSince > SCENE_RATE_QUIET_MS) {
    return SCENE_CATCHING_UP;
  }
  // A sentence the 3D module authored is about ONE ghost, so it is not here:
  // it is rendered inside that arm's own readout block, beside that arm's
  // verdict, where it can be read against the pose it describes.
  //
  // Rows the server authors. They are rendered verbatim and this file holds no
  // copy of any of them.
  if (scene.info.cell_source === 'unavailable') return scene.info.cell_note || null;
  if (scene.info.checker && scene.info.checker.note) return scene.info.checker.note;
  return null;
}

// A collapsed bar that says only "Scene" hides the one thing a phone user
// needs to decide whether opening it is worth the download.
function sceneStatusText() {
  if (!scene.info) return '';
  var live = armIds(net.frame);
  if (live.length === 0) return 'no session';
  if (scene.info.ghost_available !== true) return 'IK offline';
  var ghosts = live.some(function (armId) { return ui.ghostShown[armId] === true; });
  return live.length + (live.length === 1 ? ' arm live' : ' arms live')
    + (scene.info.cell ? ' · cell drawn' : ' · no cell')
    + (ghosts ? ' · ghost active' : '');
}

// Every arm whose ghost is drawn right now. There is no "the" ghost arm:
// both may be shown, both may differ from reality, and each one owns its own
// Reset, its own Copy, its own verdict and its own degrees.
function shownGhostArms() {
  return sceneArmIds().filter(function (armId) {
    return ui.ghostShown[armId] === true && scene.present[armId] === true;
  });
}

// The toolbar's per-arm controls and the foot's per-arm readouts are built
// from one arm list, under one signature, so the two can never disagree about
// which arms exist.
function buildGhostControls() {
  var live = armIds(net.frame);
  var signature = live.join(',');
  if (ui.ghostSegSignature === signature) return;
  ui.ghostSegSignature = signature;

  var seg = el('ghostSeg');
  seg.replaceChildren.apply(seg, live.map(function (armId) {
    return h('button', {
      type: 'button', class: 'btn btn-xs ghost-toggle',
      'aria-pressed': ui.ghostShown[armId] === true ? 'true' : 'false',
      dataset: {act: 'ghost-show', arm: armId},
      text: 'Ghost ' + armId
    });
  }));

  var tools = el('ghostArmTools');
  var toolNodes = [];
  live.forEach(function (armId) {
    toolNodes.push(h('button', {
      type: 'button', class: 'btn btn-xs',
      dataset: {act: 'ghost-reset', role: 'reset', arm: armId},
      text: 'Reset ghost — ' + armId
    }));
    toolNodes.push(h('button', {
      type: 'button', class: 'btn btn-xs',
      dataset: {act: 'ghost-copy', role: 'copy', arm: armId},
      text: 'Copy pose — ' + armId
    }));
    toolNodes.push(h('span', {
      class: 'scene-toast', dataset: {role: 'toast', arm: armId}, text: 'Copied'
    }));
  });
  tools.replaceChildren.apply(tools, toolNodes);

  var readouts = el('sceneReadouts');
  readouts.replaceChildren.apply(readouts, live.map(function (armId) {
    return h('div', {class: 'scene-arm', dataset: {arm: armId}}, [
      h('p', {class: 'scene-verdict', role: 'status',
              dataset: {role: 'verdict', arm: armId}}),
      h('p', {class: 'scene-degrees mono', dataset: {role: 'degrees', arm: armId}}),
      h('p', {class: 'scene-note', dataset: {role: 'armnote', arm: armId}}),
      h('pre', {class: 'scene-pre', dataset: {role: 'snippet', arm: armId}})
    ]);
  }));
}

// One arm's readout and one arm's two buttons. Called once per live arm, so
// the two-ghost case is the one-ghost case twice and cannot drift from it.
function syncGhostArm(armId, editable) {
  var index = armIndexOf(armId);
  var shown = ui.ghostShown[armId] === true && scene.present[armId] === true;
  var armState = armOf(net.frame, armId) || {};
  var verdict = scene.verdict[index] || null;
  var copy = scene.copy[index] || null;
  var status = verdict && verdict.status ? verdict.status : null;
  // A row that was blanked because nothing could check it: no status, and no
  // Copy either, because nothing has cleared the pose it is sitting over.
  var notChecked = !!(verdict && verdict.notChecked === true);
  var differs = shown && ui.ghostDiffers[armId] === true;
  var copied = ui.copied['ghost:' + armId];
  var pick = function (role) {
    return document.querySelector(
      '[data-role="' + role + '"][data-arm="' + armId + '"]');
  };

  var resetButton = pick('reset');
  resetButton.hidden = !editable || !shown;
  resetButton.disabled = !editable || !shown || armState.positions_stale === true;

  var copyButton = pick('copy');
  copyButton.hidden = !editable || !differs || !copy;
  copyButton.disabled = status === 'collision' || status === 'pending'
    || notChecked;
  if (!copied) copyButton.textContent = 'Copy pose — ' + armId;
  pick('toast').hidden = !copied;

  var verdictNode = pick('verdict');
  verdictNode.className = 'scene-verdict' + (status ? ' ' + status : '');
  verdictNode.textContent = shown ? verdictText(status, verdict) : '';

  var degrees = pick('degrees');
  var showDegrees = differs && copy && Array.isArray(copy.joints_deg);
  degrees.hidden = !showDegrees;
  degrees.textContent = showDegrees
    ? armId + '  ' + copy.joints_deg.map(function (value) { return value + '°'; }).join(', ')
    : '';

  var armNote = pick('armnote');
  var armNoteText = shown ? (scene.moduleNote[index] || null) : null;
  armNote.hidden = !armNoteText;
  armNote.textContent = armNoteText || '';

  // What was put on the clipboard, shown where it was taken from. The snippet
  // is the server's byte for byte — the panel neither assembles nor edits it,
  // so what the user reads here is exactly what they will paste.
  var snippet = pick('snippet');
  var text = scene.copiedText[armId];
  var showSnippet = Boolean(copied && text);
  snippet.hidden = !showSnippet;
  snippet.textContent = showSnippet ? text : '';
}

function syncScenePanel() {
  var bar = el('sceneBar');
  if (!bar) return;
  el('sceneSub').textContent = sceneStatusText();
  if (!scene.open) return;

  buildGhostControls();
  var live = armIds(net.frame);
  var editable = !!scene.handle && !scene.failed
    && scene.info && scene.info.ghost_available === true && live.length > 0;

  Array.prototype.forEach.call(el('ghostSeg').children, function (button) {
    var id = button.dataset.arm;
    button.setAttribute('aria-pressed', ui.ghostShown[id] === true ? 'true' : 'false');
    button.disabled = !editable;
  });

  live.forEach(function (armId) { syncGhostArm(armId, editable); });

  // The gizmo's instructions belong to a gizmo that is on screen. No ghost
  // shown, no handles drawn, nothing to explain — and the line goes with the
  // rest of the toolbar when the panel closes, because it lives inside it.
  el('sceneHint').hidden = !editable || shownGhostArms().length === 0;

  var note = el('sceneNote');
  var noteText = sceneNoteFor();
  note.hidden = !noteText;
  note.textContent = noteText || '';
}

function verdictText(status, verdict) {
  if (!status) return '';
  if (status === 'clear') return 'Clear of everything in the cell model.';
  if (status === 'pending') return 'Checking this pose…';
  // "Not checked" must never be readable as "checked and fine", so it says so
  // in words as well as in the ghost's neutral tint — two independent cues.
  if (status === 'unchecked') {
    return (verdict && verdict.reason) || 'Not checked: this pose was not compared with the cell model.';
  }
  // Every other sentence is the server's, rendered exactly as it arrived.
  return (verdict && verdict.reason) || '';
}

var ACT = {
  arms: function (node) {
    if (sessionLocked(net.frame)) return;
    ui.selArms = node.dataset.val;
    render();
  },
  mode: function (node) {
    if (sessionLocked(net.frame)) return;
    ui.selMode = node.dataset.val;
    render();
  },
  start: function () {
    if (sessionLocked(net.frame)) return;
    runAction('start', function () {
      return api('POST', '/api/session/start', {arms: ui.selArms, mode: ui.selMode});
    });
  },
  stop: function () {
    runAction('stop', function () { return api('POST', '/api/session/stop'); });
  },
  enable: function (node) {
    var armId = node.dataset.arm;
    var current = motionOf(net.frame, armId).enabled === true;
    runAction('enable:' + armId, function () {
      return api('POST', '/api/arm/' + armId + '/enable', {enabled: !current});
    });
  },
  source: function (node) {
    var armId = node.dataset.arm;
    var value = node.dataset.val;
    if (motionOf(net.frame, armId).source === value) return;      // already selected
    runAction('source:' + armId, function () {
      return api('POST', '/api/arm/' + armId + '/source', {source: value});
    });
  },
  gripper: function (node) {
    var armId = node.dataset.arm;
    var value = node.dataset.val;                       // 'open' | 'close'
    runAction('gripper:' + armId, function () {
      return api('POST', '/api/arm/' + armId + '/gripper', {action: value});
    });
  },
  apply: function (node) {
    var armId = node.dataset.arm;
    var index = armIndexOf(armId);
    var positions = scene.solved[index];
    // The SERVER's own numbers for the pose on screen, echoed back. The page
    // never authors a joint vector for Apply, and the server re-validates
    // and re-checks them anyway.
    if (!Array.isArray(positions)) return;
    scene.applyNote[armId] = null;
    runAction('apply:' + armId, function () {
      return api('POST', '/api/arm/' + armId + '/apply',
                 {action: 'start', positions: positions})
        .catch(function (error) { return absorbApplyRefusal(armId, error); });
    });
  },
  'apply-cancel': function (node) {
    var armId = node.dataset.arm;
    // Deliberately NOT through runAction's pending key: a stop must not be
    // disabled while it is in flight, and a second press is free.
    withLock(function () {
      return api('POST', '/api/arm/' + armId + '/apply', {action: 'cancel'});
    }).then(render, function (error) { noticeFromError(error); render(); });
  },
  jog: function (node) {
    var armId = node.dataset.arm;
    var index = Number(node.dataset.j);
    var direction = Number(node.dataset.dir);
    runAction('jog:' + armId + ':' + index + ':' + direction, function () {
      return api('POST', '/api/arm/' + armId + '/jog',
                 {joint_index: index, direction: direction})
        .then(function (result) {
          if (result.clamped && result.clamped[index]) {
            var flashKey = armId + ':' + index;
            ui.clamped[flashKey] = Date.now() + CLAMP_FLASH_MS;
            setTimeout(function () { delete ui.clamped[flashKey]; render(); },
                       CLAMP_FLASH_MS);
          }
          return result;
        });
    });
  },
  recover: function () {
    // One shot, but a BOUNDED one. The control stays disabled while the
    // request is unanswered (up to RECOVER_PENDING_MS) and for as long after
    // that as the FRAME shows a recovery actually running; a refusal, a
    // failure or silence releases it and says so in the notice bar. It is
    // never disabled on the strength of this click alone (V2L-7).
    if (ui.pending.recover === true) return;
    ui.recoverUntil = Date.now() + RECOVER_PENDING_MS;
    ui.recoverInFlight = true;
    runAction('recover', function () {
      return api('POST', '/api/session/recover').then(function (result) {
        ui.recoverInFlight = false;
        // Answered. From here the pending state lives on server evidence
        // only: recoveryInProgress(), re-checked on every frame and tick.
        ui.recoverUntil = 0;
        return result;
      }, function (error) {
        ui.recoverInFlight = false;
        ui.recoverUntil = 0;
        throw error;                  // runAction renders it and re-enables
      });
    }, true);
  },
  reclaim: function () {
    if (ui.pending.reclaim) return;
    ui.pending.reclaim = true;
    render();
    claim().then(function () {
      delete ui.pending.reclaim;
      render();
    }, function (error) {
      delete ui.pending.reclaim;
      if (error && error.error === 'operator_lock_held') {
        ui.takeoverOpen = true;
        notice('Another program holds control. Use Take over to command the arms.');
      } else {
        noticeFromError(error);
      }
      render();
    });
  },
  restart: function () {
    runAction('restart', function () { return api('POST', '/api/session/stop'); });
  },
  'takeover-open': function () { ui.takeoverOpen = true; render(); },
  'takeover-no': function () { ui.takeoverOpen = false; render(); },
  'takeover-yes': function () {
    ui.takeoverOpen = false;
    if (ui.pending.takeover) return;
    ui.pending.takeover = true;
    render();
    takeover().then(function () {
      delete ui.pending.takeover;
      render();
    }, function (error) {
      delete ui.pending.takeover;
      noticeFromError(error);
      render();
    });
  },
  copy: function (node) { copyText(node.dataset.copy, node); },
  'scene-toggle': function () { setSceneOpen(!scene.open); },
  'ghost-show': function (node) {
    var armId = node.dataset.arm;
    var next = ui.ghostShown[armId] !== true;
    ui.ghostShown[armId] = next;
    scene.copiedText[armId] = null;
    if (!next) {
      ui.ghostDiffers[armId] = false;
      // Its ghost has left the screen, so nothing on the page is describing
      // it and nothing of its may be left behind on another arm's row.
      scene.verdict[armIndexOf(armId)] = null;
      if (scene.handle) scene.handle.setVerdict(armIndexOf(armId), null);
    }
    if (scene.handle) scene.handle.setGhostVisible(armIndexOf(armId), next);
    if (next && scene.handle) scene.handle.selectArm(armIndexOf(armId));
    syncScenePanel();
    // A ghost taken off the screen changed the cell without a drag: what the
    // remaining ghost was last told is about a cell that no longer exists.
    if (!next) recheckScene();
  },
  // Both of these read the arm off the control that was pressed. There is no
  // lookup of "the ghost arm" anywhere in this file any more: that lookup
  // returned the first shown ghost, which meant panda2's controls did not
  // exist and panda1's answered for both.
  'ghost-reset': function (node) {
    var armId = node.dataset.arm;
    var index = armIndexOf(armId);
    if (!scene.handle || shownGhostArms().indexOf(armId) < 0) return;
    scene.handle.syncGhostToMeasured(index);
    ui.ghostDiffers[armId] = false;
    scene.copy[index] = null;
    scene.verdict[index] = null;
    scene.solved[index] = null;
    scene.applyNote[armId] = null;
    scene.handle.setVerdict(index, null);
    scene.moduleNote[index] = null;
    scene.copiedText[armId] = null;
    syncScenePanel();
    // The ghost moved back onto the arm, so the cell is not the one the
    // verdicts on screen were computed for -- this arm's row and its
    // neighbour's alike. Ask once, for the cell as it is now.
    recheckScene();
  },
  'ghost-copy': function (node) {
    var armId = node.dataset.arm;
    var copy = scene.copy[armIndexOf(armId)];
    if (!copy || !copy.snippet) return;
    // Byte for byte, exactly as the server built it. Nothing is appended and
    // nothing is trimmed: the snippet is where the claim gets believed.
    scene.copiedText[armId] = copy.snippet;
    writeClipboard(copy.snippet, 'ghost:' + armId, node);
    syncScenePanel();
  },
  info: function () { ui.infoOpen = !ui.infoOpen; render(); },
  'log-toggle': function () { setLogOpen(!ui.logOpen); },
  'log-view': function () {
    setLogOpen(true);
    var list = el('logList');
    ui.logFollow = true;
    list.scrollTop = list.scrollHeight;
    list.focus();
  },
  'notice-dismiss': function () { ui.notice = null; ui.noticeUntil = 0; render(); }
};

function wireDelegatedClicks() {
  document.addEventListener('click', function (event) {
    var target = event.target;
    if (!target || !target.closest) return;
    var node = target.closest('[data-act]');
    if (node && ACT[node.dataset.act]) {
      if (ui.infoOpen && node.dataset.act !== 'info') ui.infoOpen = false;
      ACT[node.dataset.act](node);
      return;
    }
    var changed = false;
    if (ui.infoOpen && !target.closest('#infoPop')) { ui.infoOpen = false; changed = true; }
    if (ui.takeoverOpen && !target.closest('.op-badge')) {
      ui.takeoverOpen = false;
      changed = true;
    }
    if (changed) render();
  });
}

/* ------------------------------------------------------------ log drawer --- */

function ringLimit() {
  return (net.caps && net.caps.log_ring_lines) || DEFAULTS.log_ring_lines;
}

function appendLogLine(line, atFront) {
  var list = el('logList');
  var row = h('div', {class: 'log-line lvl-' + line.level}, [
    h('span', {class: 'lt', text: fmtLogTime(line.t)}),
    h('span', {class: 'll', text: String(line.level).toUpperCase()}),
    h('span', {class: 'ln', text: '[' + line.node + ']'}),
    h('span', {class: 'lm', text: line.message})          // textContent — never markup
  ]);
  if (atFront) list.insertBefore(row, list.firstChild); else list.appendChild(row);
  while (list.children.length > ringLimit()) list.removeChild(list.firstChild);
  if (ui.logOpen && ui.logFollow && !atFront) list.scrollTop = list.scrollHeight;
}

function appendGapNote(count) {
  var list = el('logList');
  var row = h('div', {class: 'log-note',
                      text: count + " earlier lines are not in this page's history."});
  list.insertBefore(row, list.firstChild);
}

function syncLogBadge() {
  var badge = el('logBadge');
  if (net.errorCount > 0) {
    badge.hidden = false;
    badge.className = 'log-badge err';
    badge.textContent = net.errorCount + (net.errorCount === 1 ? ' error' : ' errors');
  } else if (net.warnCount > 0) {
    badge.hidden = false;
    badge.className = 'log-badge warn';
    badge.textContent = '⚠ ' + net.warnCount
      + (net.warnCount === 1 ? ' warning' : ' warnings');
  } else {
    badge.hidden = true;
    badge.textContent = '';
  }
}

function setLogOpen(open) {
  ui.logOpen = open;
  var list = el('logList');
  // The dock is position:fixed, so the page has to RESERVE the drawer's
  // height (.log-open) and then take up the slack, or the expanded drawer
  // simply overlays whatever is at the bottom of the viewport — on a phone
  // that is the Start/Stop row. Scrolling by exactly the reservation's growth
  // moves everything that was visible clear of the drawer, and the same
  // arithmetic in reverse puts it back when the drawer closes. The document
  // grows by the same amount, so this scroll can never be clamped short.
  var reserved = reservedHeight();
  document.body.classList.toggle('log-open', open);
  el('logBody').hidden = !open;
  el('logBar').setAttribute('aria-expanded', open ? 'true' : 'false');
  var growth = reservedHeight() - reserved;
  if (growth) window.scrollBy(0, growth);
  if (open) list.scrollTop = list.scrollHeight;
}

function reservedHeight() {
  var value = parseFloat(getComputedStyle(document.body).paddingBottom);
  return isFinite(value) ? value : 0;
}

function onLogEvent(line) {
  if (!line || typeof line.seq !== 'number') return;
  net.lastSeq = Math.max(net.lastSeq, line.seq);
  net.warnCount = line.warn_count;          // ASSIGN, never increment
  net.errorCount = line.error_count;
  if (line.level === 'debug') { syncLogBadge(); return; }   // belt and braces
  appendLogLine(line, false);
  syncLogBadge();
}

function applyBacklog(payload) {
  if (!payload) return;
  (payload.lines || []).forEach(function (line) {
    if (!line || typeof line.seq !== 'number') return;
    // The stream can deliver a line between this request going out and its
    // answer coming back; without this the connect-time backfill renders those
    // few lines a second time.
    if (line.seq <= net.lastSeq) return;
    net.lastSeq = line.seq;
    if (line.level === 'debug') return;     // never rendered, but the seq still counts
    appendLogLine(line, false);
  });
  if (typeof payload.warn_count === 'number') net.warnCount = payload.warn_count;
  if (typeof payload.error_count === 'number') net.errorCount = payload.error_count;
  if (typeof payload.dropped === 'number' && payload.dropped > net.dropped) {
    appendGapNote(payload.dropped - net.dropped);
    net.dropped = payload.dropped;
  }
  syncLogBadge();
}

function wireLogList() {
  var list = el('logList');
  list.addEventListener('mouseenter', function () { ui.logFollow = false; });
  list.addEventListener('mouseleave', function () {
    ui.logFollow = true;
    list.scrollTop = list.scrollHeight;
  });
  list.addEventListener('scroll', function () {
    ui.logFollow = (list.scrollHeight - list.scrollTop - list.clientHeight) < 8;
  });
}

/* --------------------------------------------------------------- chrome --- */

var CHIP_LABEL = {                       // session.state -> visible chip text
  stopped: 'idle', preflight: 'preflight', starting: 'starting',
  settling: 'settling', running: 'running', fault: 'fault', stopping: 'stopping'
};

// The hint line is persistent: it must never be blank. With no frame there is
// no server sentence to show, so the shell says what to do about that — the
// one state where the console cannot reach its server.
var SHELL_HINT = 'Waiting for the server — check that franka_web_server is running.';

function paintIdleShell() {
  var chip = el('stateChip');
  chip.textContent = 'idle';
  chip.className = 'chip chip-idle';
  el('linkChip').hidden = net.live !== false;    // 'reconnecting' is legible here too
  el('simChip').hidden = true;
  el('recChip').hidden = true;
  el('hintText').textContent = SHELL_HINT;
  var advisory = el('advisoryLine');
  advisory.textContent = '';
  advisory.hidden = true;
  if (ui.badgeSignature !== 'shell') {
    ui.badgeSignature = 'shell';
    el('opBadge').replaceChildren(
      h('span', {class: 'op-k', text: 'Control'}),
      h('span', {}, [
        h('strong', {text: 'nobody'}),
        ' — taken automatically when you act'
      ])
    );
  }
  if (dom.kind !== 'shell') {
    dom.kind = 'shell';
    dom.arms = {};
    dom.steps = {};
    el('stage').replaceChildren(
      placeholderCard('No live data. Configure a session on the left and press Start.', null)
    );
  }
}

function syncChrome(frame) {
  if (!frame) { paintIdleShell(); return; }
  var session = frame.session;
  var chip = el('stateChip');
  chip.textContent = CHIP_LABEL[session.state] || session.state;
  var chipClass = session.state === 'stopped'
    ? (session.session_id === null ? 'chip-idle' : 'chip-stopped')
    : 'chip-' + session.state;
  chip.className = 'chip ' + chipClass;

  el('simChip').hidden = !(session.mode === 'simulate' && session.state !== 'stopped');

  var recording = frame.recording || {};
  el('recChip').hidden = !(recording.active === true && recording.disabled !== true);

  el('linkChip').hidden = net.live !== false;

  var advisory = el('advisoryLine');
  var advisoryText = typeof session.advisory === 'string' ? session.advisory : '';
  advisory.textContent = advisoryText;
  advisory.hidden = advisoryText === '';

  syncBadge(frame);
}

function syncBadge(frame) {
  var operator = frame.operator || {};
  var mine = lockIsMine(frame);
  var signature = [String(operator.locked), String(operator.claim_id), String(mine),
                   String(ui.takeoverOpen)].join('|');
  if (signature === ui.badgeSignature) {
    var timeNode = el('opTime');
    if (timeNode) timeNode.textContent = sinceMinutes(operator.since);
    return;
  }
  ui.badgeSignature = signature;
  var badge = el('opBadge');
  var kids = [h('span', {class: 'op-k', text: 'Control'})];
  if (mine) {
    kids.push(h('span', {}, [
      h('strong', {text: 'this page'}),
      ' · ',
      h('span', {class: 'mono', id: 'opTime', text: sinceMinutes(operator.since)})
    ]));
  } else if (operator.locked) {
    kids.push(h('span', {}, [
      h('strong', {text: 'another program'}),
      ' · since ',
      h('span', {class: 'mono', text: fmtClock(operator.since)})
    ]));
    kids.push(h('button', {type: 'button', class: 'btn btn-xs',
                           dataset: {act: 'takeover-open'}, text: 'Take over'}));
    if (ui.takeoverOpen) {
      kids.push(h('div', {class: 'pop'}, [
        'Taking over resets every enable.',
        h('div', {class: 'poprow'}, [
          h('button', {type: 'button', class: 'btn btn-xs btn-primary',
                       dataset: {act: 'takeover-yes'}, text: 'Take over'}),
          h('button', {type: 'button', class: 'btn btn-xs',
                       dataset: {act: 'takeover-no'}, text: 'Cancel'})
        ])
      ]));
    }
  } else {
    kids.push(h('span', {}, [
      h('strong', {text: 'nobody'}),
      ' — taken automatically when you act'
    ]));
  }
  badge.replaceChildren.apply(badge, kids);
}

function sinceMinutes(since) {
  var ms = Date.parse(since);
  if (isNaN(ms)) return 'just now';
  var minutes = Math.floor((Date.now() - ms) / 60000);
  return minutes < 1 ? 'just now' : minutes + ' min';
}

/* --------------------------------------------------------- session card --- */

function syncSession(frame) {
  syncProfile();
  el('infoPop').hidden = !ui.infoOpen;
  var infoBtn = document.querySelector('.info-btn');
  if (infoBtn) infoBtn.setAttribute('aria-expanded', ui.infoOpen ? 'true' : 'false');
  if (!frame) return;

  var session = frame.session;
  var locked = session.state !== 'stopped';
  var arms = locked ? session.arms : ui.selArms;
  var mode = locked ? session.mode : ui.selMode;

  var seg = el('armSeg');
  seg.classList.toggle('seg-locked', locked);
  Array.prototype.forEach.call(seg.querySelectorAll('.seg-btn'), function (button) {
    button.classList.toggle('sel', button.dataset.val === arms);
    button.disabled = locked;
  });
  Array.prototype.forEach.call(
    document.querySelectorAll('#modeField [data-act="mode"]'), function (button) {
      var selected = button.dataset.val === mode;
      button.classList.toggle('sel', selected);
      button.setAttribute('aria-pressed', selected ? 'true' : 'false');
      button.classList.toggle('mode-locked', locked);
      button.disabled = locked;
    });
  el('btnStart').disabled = locked || ui.pending.start === true;
  el('btnStop').disabled = !locked || ui.pending.stop === true;
}

function syncProfile() {
  var text = el('profileText');
  var pop = el('infoPop');
  if (!net.config) {
    // Three states, not two: the request has not been answered yet, it failed,
    // or it succeeded. The first frame is always painted before /api/config
    // returns, so 'unavailable' before the attempt settles would report a
    // failure that has not happened.
    if (!net.configTried) {
      if (dom.profileFor !== 'reading') {
        dom.profileFor = 'reading';
        text.textContent = 'Profile: reading…';
        pop.replaceChildren();
      }
      return;
    }
    if (dom.profileFor !== 'none') {
      dom.profileFor = 'none';
      text.textContent = 'Profile: unavailable';
      pop.replaceChildren(document.createTextNode(
        'The effective configuration could not be read from this server.'));
    }
    return;
  }
  if (dom.profileFor === net.config) return;
  dom.profileFor = net.config;

  var profiles = net.config.profiles || {};
  var fromConfig = false;
  var ids = [];
  for (var armId in profiles) {
    if (!Object.prototype.hasOwnProperty.call(profiles, armId)) continue;
    ids.push(armId);
    if (profiles[armId] && profiles[armId].source === 'config') fromConfig = true;
  }
  ids.sort();
  text.textContent = 'Profile: ' + (fromConfig ? 'from your config file' : 'default')
    + ' — stiffness, speed and torque limits';

  var kids = [h('div', {class: 'popline',
                        text: 'Read from ' + net.config.config_path})];
  kids.push(h('div', {class: 'popline',
                      text: net.config.config_present
                        ? 'This console only reads it — editing happens in the file.'
                        : 'No file yet — the built-in defaults are in use.'}));
  ids.forEach(function (armId) {
    var profile = profiles[armId] || {};
    var speed = profile.max_target_velocity_rad_s
      && profile.max_target_velocity_rad_s.length
      ? deg(profile.max_target_velocity_rad_s[0])
      : null;
    kids.push(h('div', {class: 'popline mono', text:
      armId
      + ' · stiffness ' + (profile.k_gains || []).map(fmtNum).join('/')
      + ' · torque ≤ ' + (profile.max_effort_nm || []).map(fmtNum).join('/')
      + ' N·m · speed ≤ '
      + (speed == null || !isFinite(speed) ? '—' : speed.toFixed(2)) + ' °/s'}));
  });
  pop.replaceChildren.apply(pop, kids);
}

/* -------------------------------------------------------- stage builders --- */

function stepIds(steps) {
  return (steps || []).map(function (step) { return step.id; }).join('>');
}

function stageSignature(frame) {
  var session = frame.session;
  var parts = [session.state, session.mode, (session.arm_ids || []).join('+'),
               session.session_id, frame.fault.active ? frame.fault.cause : '-',
               frame.fault.active ? frame.fault.action : '-',
               stepIds(session.steps), String(lockIsElsewhere(frame)),
               session.last_error ? session.last_error.code : '-',
               (frame.recording || {}).disabled === true ? 'norec' : '-',
               // Which STAGE is built flips on this, and the step ids alone
               // cannot carry it: a finished recovery checklist has the same
               // ids as a running one.
               String(recoveryInProgress(frame)),
               session.recording_sealed === true ? 'sealed' : '-'];
  (session.arm_ids || []).forEach(function (armId) {
    var motion = (frame.arms[armId] || {}).motion || {};
    // available and source change the STRUCTURE this arm's column is built
    // from. motion.enabled does not: patchControl() carries every visual
    // consequence of it, so it stays out of the signature — including it made
    // each toggle rebuild both columns and drop focus and disclosure state.
    parts.push(armId + ':' + String(motion.available) + ':' + String(motion.source)
               + ':' + String((frame.arms[armId] || {}).gripper
                              ? frame.arms[armId].gripper.configured : false));
  });
  return parts.join('|');
}

function isRecoverySteps(steps) {
  return !!(steps && steps.length && typeof steps[0].id === 'string'
            && steps[0].id.indexOf('reconnect:') === 0);
}

// SERVER EVIDENCE ONLY. Reads the frame and nothing else — no `ui` state, no
// memory of a click — because "Recovering" is a claim about what the SERVER
// is doing. It holds when the session is faulted, the checklist the server
// published is a recovery checklist, and at least one of its steps is still
// unfinished (running) or failed (finished, and the operator must read where
// it broke). A checklist whose every step is `done` while the session is
// still faulted is a FINISHED recovery — evidence of a past attempt, not a
// live one — and is exactly what made the page claim "Recovering" forever
// after a Recover the server never started (V2L-7).
function recoveryInProgress(frame) {
  var session = frame && frame.session;
  if (!session || session.state !== 'fault') return false;
  if (!isRecoverySteps(session.steps)) return false;
  return session.steps.some(function (step) {
    return step.status === 'pending' || step.status === 'active'
      || step.status === 'failed';
  });
}

// Resolve the Recover control's pending state against the bounded request and
// the frame, in that order. Called from every frame and from the 1 Hz tick, so
// a request that never answers and a server that never starts a recovery both
// end in a released control and a notice rather than a stuck "Recovering".
function resolveRecoverPending() {
  if (ui.pending.recover !== true) return;
  if (recoveryInProgress(net.frame)) return;            // the server says so
  if (ui.recoverUntil && Date.now() < ui.recoverUntil) return;   // still asking
  delete ui.pending.recover;
  ui.recoverUntil = 0;
  if (ui.recoverInFlight) {
    ui.recoverInFlight = false;
    notice('The server has not started a recovery. Check the log, then '
           + 'press Recover again.');
  }
}

function placeholderCard(text, detail) {
  var kids = [text];
  if (detail) kids.push(h('span', {class: 'placeholder-detail', text: detail}));
  return h('div', {class: 'card placeholder'}, kids);
}

function checklistTitle(mode) {
  if (mode === 'watch') return 'Starting watch session';
  if (mode === 'simulate') return 'Starting simulated session';
  return 'Starting motion session';
}

function stepItem(step, key) {
  var icon = h('span', {class: 'vicon'});
  var label = h('span', {class: 'vlabel', text: step.label});
  var duration = h('span', {class: 'vdur mono'});
  var attrs = {class: 'vitem', dataset: {}};
  attrs.dataset[key] = step.id;
  var item = h('li', attrs, [icon, label, duration]);
  return {li: item, icon: icon, label: label, dur: duration, detail: null,
          status: null, iconFor: null};
}

function buildChecklist(frame, key, title) {
  var session = frame.session;
  var list = h('ol', {class: 'vlist'}, []);
  dom.steps = {};
  (session.steps || []).forEach(function (step) {
    var entry = stepItem(step, key);
    dom.steps[step.id] = entry;
    list.appendChild(entry.li);
  });
  var card = h('section', {class: 'card checklist'}, [
    h('h2', {class: 'card-title', text: title}),
    list
  ]);
  return card;
}

function faultCard(frame) {
  var fault = frame.fault;
  var card = h('section', {class: 'card banner-fault', role: 'alert'}, [
    h('p', {class: 'banner-text', text: fault.headline})
  ]);
  if (fault.steps && fault.steps.length) {
    card.appendChild(h('ol', {class: 'banner-steps'}, fault.steps.map(function (step) {
      return h('li', {text: step});                    // textContent, never markup
    })));
  }
  var buttons = h('div', {class: 'bannerbtns'}, []);
  var primary = {recover: ['Recover', 'recover'], reclaim: ['Reclaim', 'reclaim'],
                 restart: ['Stop session', 'restart']}[fault.action];
  if (primary) {
    var button = h('button', {type: 'button', class: 'btn btn-primary',
                              dataset: {act: primary[1]}, text: primary[0]});
    dom.faultPrimary = button;
    dom.faultPrimaryKey = primary[1];
    buttons.appendChild(button);
  } else {
    dom.faultPrimary = null;
    dom.faultPrimaryKey = null;
  }
  buttons.appendChild(h('button', {type: 'button', class: 'linkbtn',
                                   dataset: {act: 'log-view'}, text: 'View logs'}));
  card.appendChild(buttons);
  return card;
}

// The joint scale, in radians: the frame's fence when it carries one, else the
// server's own configured profile bounds, else nothing at all.
function jointScale(frame, armId, count) {
  var motion = motionOf(frame, armId);
  var pair = usableBounds(motion.fence_lower, motion.fence_upper, count);
  if (pair) return pair;
  var profiles = (net.config && net.config.profiles) || {};
  var profile = profiles[armId];
  if (profile) {
    pair = usableBounds(profile.position_lower_rad, profile.position_upper_rad, count);
    if (pair) return pair;
  }
  return null;
}

function usableBounds(lower, upper, count) {
  if (!lower || !upper || lower.length !== count || upper.length !== count) return null;
  for (var i = 0; i < count; i += 1) {
    if (typeof lower[i] !== 'number' || typeof upper[i] !== 'number') return null;
    if (!isFinite(lower[i]) || !isFinite(upper[i])) return null;
    if (!(lower[i] < upper[i])) return null;
  }
  return {lower: lower, upper: upper};
}

function pct(value, low, high) {
  var fraction = (value - low) / (high - low);
  if (!isFinite(fraction)) return 0;
  return Math.max(0, Math.min(100, fraction * 100));
}

function buildTile(frame, armId) {
  var arm = armOf(frame, armId) || {};
  var count = jointCount(arm);
  var refs = {joints: [], armId: armId};

  var name = h('span', {class: 'tile-name', text: armId});
  var head = h('div', {class: 'tile-head'}, [
    h('div', {}, [name,
                  frame.session.mode === 'simulate'
                    ? h('span', {class: 'tile-sub', text: 'simulated hardware'})
                    : null]),
    null
  ]);
  refs.pillLabel = h('span', {});
  refs.pill = h('span', {class: 'pill pill-unknown'}, [h('i', {}), refs.pillLabel]);
  head.appendChild(refs.pill);

  refs.succ = h('span', {class: 'mono'});
  refs.meter = h('span', {});
  var rate = h('div', {class: 'raterow tile-rate'}, [
    h('span', {class: 'fieldlabel', text: 'Command success'}),
    h('span', {class: 'succ'}, [refs.succ, h('span', {class: 'meter'}, [refs.meter])])
  ]);

  var list = h('div', {class: 'jlist'}, []);
  for (var i = 0; i < count; i += 1) {
    var zero = h('i', {class: 'jzero'});
    var thumb = h('i', {class: 'jthumb'});
    var ghost = h('i', {class: 'jthumb jthumb-target'});
    var track = h('div', {class: 'jtrack'}, [zero, thumb, ghost]);
    var value = h('span', {class: 'jval mono'});
    var row = h('div', {class: 'jrow'}, [
      h('span', {class: 'jname', text: 'J' + (i + 1)}), track, value
    ]);
    refs.joints.push({row: row, track: track, zero: zero, thumb: thumb,
                      ghost: ghost, value: value});
    list.appendChild(row);
  }

  refs.status = h('div', {class: 'tile-status'});
  var kids = [head, rate, list];
  // Only when the server says this arm HAS a gripper. In Simulate that is
  // always false, so the arm card is byte-for-byte what it was before this
  // row existed.
  if (gripperOf(frame, armId).configured === true) {
    refs.gripper = buildGripperRow(armId);
    kids.push(refs.gripper.row);
  }
  kids.push(refs.status);
  refs.card = h('section', {class: 'card tile'}, kids);
  return refs;
}

function buildGripperRow(armId) {
  var refs = {};
  refs.open = h('button', {type: 'button', class: 'gbtn',
                           dataset: {act: 'gripper', arm: armId, val: 'open'},
                           text: 'Open'});
  refs.close = h('button', {type: 'button', class: 'gbtn',
                            dataset: {act: 'gripper', arm: armId, val: 'close'},
                            text: 'Close'});
  refs.width = h('span', {class: 'gwidth mono'});
  refs.pillLabel = h('span', {});
  refs.pill = h('span', {class: 'pill pill-unknown'}, [h('i', {}), refs.pillLabel]);
  refs.row = h('div', {class: 'grow'}, [
    h('span', {class: 'fieldlabel', text: 'Gripper'}),
    h('div', {class: 'gbtns'}, [refs.open, refs.close]),
    refs.width, refs.pill
  ]);
  return refs;
}

function buildJogPanel(frame, armId, count) {
  var step = jogStepDeg();
  var rows = h('div', {class: 'jogrows'}, []);
  var buttons = [];
  for (var i = 0; i < count; i += 1) {
    var minus = jogButton(armId, i, -1, step);
    var plus = jogButton(armId, i, 1, step);
    buttons.push(minus, plus);
    rows.appendChild(h('div', {class: 'jogrow'}, [
      h('span', {class: 'jname', text: 'J' + (i + 1)}), minus, plus
    ]));
  }
  var note = step == null
    ? 'Joint limits enforced'
    : step.toFixed(1) + '° per press · joint limits enforced';
  var panel = h('div', {class: 'jog'}, [
    h('div', {class: 'jog-note', text: note}),
    rows
  ]);
  return {panel: panel, buttons: buttons};
}

function jogStepDeg() {
  return net.caps && typeof net.caps.jog_step_rad === 'number'
    ? net.caps.jog_step_rad * RAD_TO_DEG
    : null;
}

function jogButton(armId, index, direction, step) {
  var word = direction > 0 ? 'plus' : 'minus';
  var label = armId + ' J' + (index + 1) + ' ' + word
    + (step == null ? '' : ' ' + step.toFixed(1) + ' degrees');
  return h('button', {
    type: 'button', class: 'jogbtn', 'aria-label': label,
    dataset: {act: 'jog', arm: armId, j: String(index), dir: String(direction)},
    text: direction > 0 ? '+' : '−'
  });
}

function buildExternalPanel(frame, armId) {
  var refs = {};
  refs.topic = h('code', {});
  refs.topicCopy = h('button', {type: 'button', class: 'copybtn',
                                dataset: {act: 'copy', copy: 'topic:' + armId},
                                text: 'Copy'});
  var topicBlock = h('div', {}, [
    h('div', {class: 'fieldlabel ext-label', text: 'Command topic'}),
    h('div', {class: 'codeline'}, [refs.topic, refs.topicCopy])
  ]);

  refs.template = h('pre', {});
  refs.tmplCopy = h('button', {type: 'button', class: 'copybtn',
                               dataset: {act: 'copy', copy: 'tmpl:' + armId},
                               text: 'Copy template'});
  var details = h('details', {class: 'tmpl'}, [
    h('summary', {text: 'Message template — trajectory_msgs/JointTrajectory'}),
    h('div', {class: 'prewrap'}, [refs.template]),
    refs.tmplCopy
  ]);
  details.open = ui.tmplOpen[armId] === true;
  details.addEventListener('toggle', function () {
    ui.tmplOpen[armId] = details.open;
  });

  refs.notReady = h('div', {class: 'jog-note', text:
    "No fresh pose yet — the template's positions are placeholders."});
  refs.rate = h('span', {class: 'rate mono'});
  var rateRow = h('div', {class: 'raterow'}, [
    h('span', {class: 'fieldlabel', text: 'Incoming rate'}),
    refs.rate
  ]);
  refs.panel = h('div', {class: 'ext'}, [topicBlock, details, refs.notReady, rateRow]);
  return refs;
}

// The Apply panel: the pose the operator drew, one button, and the promise
// the button makes. It lives on the ARM CARD, which is the only place a
// motion control may live -- the scene panel gains nothing.
function buildApplyPanel(armId) {
  var refs = {};
  refs.degrees = h('div', {class: 'apply-degrees mono'});
  refs.button = h('button', {type: 'button', class: 'applybtn',
                             dataset: {act: 'apply', arm: armId},
                             text: 'Apply — ' + armId});
  refs.cancel = h('button', {type: 'button', class: 'applybtn cancel',
                             dataset: {act: 'apply-cancel', arm: armId},
                             text: 'Cancel'});
  refs.reason = h('div', {class: 'apply-reason'});
  refs.bar = h('i', {});
  refs.meter = h('div', {class: 'apply-meter'}, [refs.bar]);
  refs.progressText = h('span', {class: 'apply-pct mono'});
  refs.progressRow = h('div', {class: 'apply-progress'}, [
    h('span', {class: 'fieldlabel', text: 'Applying'}),
    refs.progressText
  ]);
  refs.goal = h('div', {class: 'apply-degrees mono'});
  refs.promise = h('div', {class: 'jog-note', text:
    'Moves this arm along a straight line in joint space to the pose you '
    + 'drew. The whole line is checked before anything is sent. The hand does '
    + 'not travel in a straight line through space.'});
  refs.scope = h('div', {class: 'jog-note', text:
    'This check looks at the path this arm will command. It does not watch or '
    + 'limit anything else in the cell.'});
  refs.panel = h('div', {class: 'apply'}, [
    refs.progressRow, refs.meter, refs.goal,
    refs.degrees, refs.button, refs.cancel, refs.reason,
    refs.promise, refs.scope
  ]);
  return refs;
}

// Every one of the eleven conditions, evaluated in one place and returned as
// a verdict the patcher renders. Rows 1-6 and 11 DISABLE the button with a
// reason under it; rows 7-10 HIDE it, exactly as the scene's Copy button is
// hidden until the ghost differs -- there is nothing to apply, so an
// affordance would be a lie.
function applyVerdict(frame, armId) {
  var motion = motionOf(frame, armId);
  var apply = motion.apply || {};
  var index = armIndexOf(armId);
  if (apply.state === 'travelling') return {mode: 'travelling'};
  // 7-10: nothing to apply. Hidden, not disabled.
  if (ui.ghostShown[armId] !== true) return {mode: 'hidden'};
  if (ui.ghostDiffers[armId] !== true) return {mode: 'hidden'};
  if (!Array.isArray(scene.solved[index]) || !scene.copy[index]) {
    return {mode: 'hidden'};
  }
  var verdict = scene.verdict[index];
  if (!verdict || verdict.status !== 'clear') return {mode: 'hidden'};
  // 1-6 and 11: there is something to apply, and something is stopping it.
  if (motion.available !== true) {
    return {mode: 'blocked', reason: 'This arm has no command surface yet.'};
  }
  if (lockIsElsewhere(frame)) {
    return {mode: 'blocked', reason: 'Another program holds control.'};
  }
  if (motion.enabled !== true) {
    return {mode: 'blocked', reason: 'Enable this arm before applying a pose.'};
  }
  if (motion.source !== 'ghost') {
    return {mode: 'blocked', reason: 'Switch the source to Ghost first.'};
  }
  // The server's own sentence, rendered verbatim: the checker's words have
  // one author, and this page holds no copy of any of them.
  if (apply.note) return {mode: 'blocked', reason: apply.note};
  var busy = travellingArm(frame);
  if (busy) {
    return {mode: 'blocked', reason: busy + ' is travelling. Wait for it to '
            + 'arrive, or cancel it, then apply this one.'};
  }
  if (ui.pending['apply:' + armId] === true) {
    return {mode: 'blocked', reason: null};
  }
  return {mode: 'ready'};
}

function travellingArm(frame) {
  var found = null;
  armIds(frame).forEach(function (armId) {
    var apply = motionOf(frame, armId).apply || {};
    if (apply.state === 'travelling') found = armId;
  });
  return found;
}

function degreeLine(values) {
  return (values || []).map(function (value) {
    return (typeof value === 'number' && isFinite(value))
      ? value.toFixed(1) + '°' : '—';
  }).join(', ');
}

function patchApplyPanel(frame, armId, refs, elsewhere) {
  var apply = (motionOf(frame, armId).apply) || {};
  var verdict = applyVerdict(frame, armId);
  var travelling = verdict.mode === 'travelling';
  var index = armIndexOf(armId);
  var copy = scene.copy[index];

  refs.progressRow.hidden = !travelling;
  refs.meter.hidden = !travelling;
  refs.goal.hidden = !travelling;
  if (travelling) {
    var fraction = typeof apply.fraction === 'number' ? apply.fraction : 0;
    refs.bar.style.width = (Math.max(0, Math.min(1, fraction)) * 100).toFixed(1) + '%';
    var left = typeof apply.seconds_remaining === 'number'
      ? '  ~' + Math.max(0, Math.round(apply.seconds_remaining)) + ' s left' : '';
    refs.progressText.textContent =
      Math.round(fraction * 100) + '%' + left;
    // Degrees for display only: a transform of a frame value, never a
    // template this page authored.
    refs.goal.textContent = 'Goal  ' + degreeLine(
      (apply.goal || []).map(function (value) { return value * RAD_TO_DEG; }));
  }

  // Cancel is NEVER disabled while a travel runs and this page holds the
  // lock. A stop control that can be greyed out is not a stop control, and it
  // is idempotent by design, so a double press is free.
  refs.cancel.hidden = !travelling;
  refs.cancel.disabled = elsewhere;

  var showDegrees = !travelling && copy && Array.isArray(copy.joints_deg);
  refs.degrees.hidden = !showDegrees;
  refs.degrees.textContent = showDegrees
    ? 'Ghost pose  ' + degreeLine(copy.joints_deg) : '';

  refs.button.hidden = travelling || verdict.mode === 'hidden';
  refs.button.disabled = verdict.mode !== 'ready';

  // A refusal is PINNED under the button until the next solve or the next
  // Apply: a notice that fades is not a witness.
  var reason = travelling ? null : (verdict.reason || scene.applyNote[armId] || null);
  refs.reason.hidden = !reason;
  refs.reason.textContent = reason || '';
  refs.promise.hidden = travelling;
  refs.scope.hidden = travelling;
}

function buildControl(frame, armId) {
  var arm = armOf(frame, armId) || {};
  var motion = arm.motion || {};
  var count = jointCount(arm);
  var refs = {armId: armId};

  refs.switch = h('button', {type: 'button', class: 'switch', role: 'switch',
                             'aria-checked': 'false',
                             dataset: {act: 'enable', arm: armId}}, [h('i', {})]);
  refs.lockSub = h('span', {class: 'ctrl-sub', text: 'another program holds control'});
  refs.serviceSub = h('span', {class: 'ctrl-advisory',
                               text: 'enable service not reachable'});
  var enrow = h('div', {class: 'enrow'}, [
    refs.switch,
    h('div', {class: 'enlabel'}, [
      h('strong', {text: 'Enable'}),
      ' — allows commands to move this arm',
      refs.lockSub,
      refs.serviceSub
    ])
  ]);

  var kids = [h('h2', {class: 'card-title', text: 'Control — ' + armId}), enrow];

  if (motion.source !== null && motion.source !== undefined) {
    refs.srcButtons = ['jog', 'external', 'ghost'].map(function (value) {
      return h('button', {type: 'button', class: 'seg-btn',
                          dataset: {act: 'source', arm: armId, val: value},
                          text: SOURCE_LABELS[value]});
    });
    refs.srcRow = h('div', {class: 'srcrow'}, [
      h('span', {class: 'fieldlabel', text: 'Source'}),
      h('div', {class: 'seg seg-sm'}, refs.srcButtons)
    ]);
    kids.push(refs.srcRow);
  }

  if (motion.source === 'external') {
    refs.ext = buildExternalPanel(frame, armId);
    kids.push(refs.ext.panel);
  } else if (motion.source === 'ghost') {
    refs.apply = buildApplyPanel(armId);
    kids.push(refs.apply.panel);
  } else {
    var jog = buildJogPanel(frame, armId, count);
    refs.jogButtons = jog.buttons;
    kids.push(jog.panel);
  }

  refs.card = h('section', {class: 'card ctrl'}, kids);
  return refs;
}

function buildStage(frame) {
  if (!frame) return;
  var stage = el('stage');
  var session = frame.session;
  dom.steps = {};
  dom.arms = {};
  dom.recFinal = null;
  dom.faultPrimary = null;
  dom.faultPrimaryKey = null;

  if (session.state === 'stopped') {
    dom.kind = 'placeholder';
    if (session.session_id === null) {
      stage.replaceChildren(placeholderCard(
        'No live data. Configure a session on the left and press Start.', null));
      return;
    }
    // Keyed on whether a recording actually SEALED — the same evidence the
    // server's hint line branches on — and not on recording.disabled, which
    // answers the different question "is recording switched off in the
    // config?". A start refused at preflight adopts no recorder at all, so it
    // saves nothing while recording stays enabled; this card told that
    // operator "The recording was saved." (live finding V2L-2).
    var text = frame.session.recording_sealed === true
      ? 'Session ended. The recording was saved.'
      : 'Session ended.';
    var detail = session.last_error
      ? session.last_error.code + ': ' + (session.last_error.detail || '')
      : null;
    stage.replaceChildren(placeholderCard(text, detail));
    return;
  }

  if (session.state === 'preflight' || session.state === 'starting'
      || session.state === 'settling') {
    dom.kind = 'checklist';
    stage.replaceChildren(
      buildChecklist(frame, 'vid', checklistTitle(session.mode)));
    return;
  }

  if (recoveryInProgress(frame)) {
    dom.kind = 'recover';
    var card = buildChecklist(frame, 'rid', 'Recovering');
    dom.recFinal = h('div', {class: 'vfinal',
                             text: 'All enables are off — re-enable to continue.'});
    dom.recFinal.hidden = true;
    card.appendChild(dom.recFinal);
    stage.replaceChildren(card);
    return;
  }

  dom.kind = 'arms';
  var kids = [];
  if (session.state === 'fault') kids.push(faultCard(frame));
  if (session.state === 'running' && session.mode === 'watch') {
    kids.push(h('div', {class: 'card watchnote', text:
      'Arm is free — it can be moved by hand. '
      + 'Motion is impossible in this mode.'}));
  }
  var grid = h('div', {class: 'armgrid'}, []);
  armIds(frame).forEach(function (armId) {
    var column = h('div', {class: 'armcol'}, []);
    var tile = buildTile(frame, armId);
    var entry = {tile: tile, control: null};
    column.appendChild(tile.card);
    if (session.state === 'running' && motionOf(frame, armId).available === true) {
      entry.control = buildControl(frame, armId);
      column.appendChild(entry.control.card);
    }
    dom.arms[armId] = entry;
    grid.appendChild(column);
  });
  kids.push(grid);
  stage.replaceChildren.apply(stage, kids);
}

/* ---------------------------------------------------------- stage patches --- */

function patchStage(frame) {
  if (dom.kind === 'checklist' || dom.kind === 'recover') {
    patchSteps(frame);
    return;
  }
  if (dom.kind === 'arms') patchArms(frame);
  if (dom.kind === 'arms' && dom.faultPrimaryKey && dom.faultPrimary) {
    dom.faultPrimary.disabled = ui.pending[dom.faultPrimaryKey] === true;
  }
}

function patchSteps(frame) {
  var steps = (frame.session && frame.session.steps) || [];
  var allDone = steps.length > 0;
  steps.forEach(function (step) {
    var entry = dom.steps[step.id];
    if (!entry) return;
    if (step.status !== 'done') allDone = false;
    if (entry.status !== step.status) {
      entry.status = step.status;
      entry.li.className = 'vitem'
        + (step.status === 'active' ? ' active'
          : step.status === 'done' ? ' done'
            : step.status === 'failed' ? ' failed' : '');
      if (step.status === 'done') {
        entry.icon.replaceChildren(tickSvg());
      } else if (step.status === 'failed') {
        entry.icon.replaceChildren(document.createTextNode('!'));
      } else {
        entry.icon.replaceChildren();
      }
    }
    var duration = step.duration_s == null ? '' : step.duration_s.toFixed(1) + ' s';
    if (entry.dur.textContent !== duration) entry.dur.textContent = duration;
    if (step.detail) {
      if (!entry.detail) {
        entry.detail = h('span', {class: 'vdetail mono'});
        entry.li.appendChild(entry.detail);
      }
      if (entry.detail.textContent !== step.detail) entry.detail.textContent = step.detail;
    }
  });
  if (dom.recFinal) dom.recFinal.hidden = !allDone;
}

function patchArms(frame) {
  var session = frame.session;
  var elsewhere = lockIsElsewhere(frame);
  armIds(frame).forEach(function (armId) {
    var entry = dom.arms[armId];
    if (!entry) return;
    patchTile(frame, armId, entry.tile);
    if (entry.control) patchControl(frame, armId, entry.control, elsewhere, session);
  });
}

function pillFor(arm, motion) {
  var status = arm.status;
  if (status === 'ok' && motion.enabled === true && motion.source === 'external'
      && motion.external_rate_hz != null && motion.external_rate_hz < 10) {
    return ['pill-warning', 'waiting'];
  }
  if (status === 'ok') return ['pill-ok', 'ok'];
  if (status === 'warn') return ['pill-warning', 'warn'];
  if (status === 'error') return ['pill-fault', 'error'];
  return ['pill-unknown', status ? String(status) : 'unknown'];
}

function patchTile(frame, armId, refs) {
  var arm = armOf(frame, armId) || {};
  var motion = arm.motion || {};
  var down = arm.positions_stale === true;
  var cardClass = 'card tile' + (down ? ' down' : '');
  if (refs.card.className !== cardClass) refs.card.className = cardClass;

  var pill = pillFor(arm, motion);
  var pillClass = 'pill ' + pill[0];
  if (refs.pill.className !== pillClass) refs.pill.className = pillClass;
  if (refs.pillLabel.textContent !== pill[1]) refs.pillLabel.textContent = pill[1];

  var robotState = arm.robot_state || {};
  var rate = robotState.available === false ? null : robotState.control_command_success_rate;
  var succ = rate == null || !isFinite(rate) ? '—' : rate.toFixed(2);
  if (refs.succ.textContent !== succ) refs.succ.textContent = succ;
  refs.meter.style.width = (rate == null || !isFinite(rate)
    ? 0
    : Math.max(0, Math.min(100, rate * 100))).toFixed(0) + '%';

  var count = refs.joints.length;
  var scale = jointScale(frame, armId, count);
  var positions = arm.positions || [];
  var targets = motion.target || [];
  refs.joints.forEach(function (joint, i) {
    var value = positions[i];
    joint.value.textContent = fmtDeg(value);
    var trackClass = 'jtrack' + (scale ? '' : ' jtrack-noscale');
    if (joint.track.className !== trackClass) joint.track.className = trackClass;
    var rowClass = 'jrow'
      + (ui.clamped[armId + ':' + i] > Date.now() ? ' clamped' : '');
    if (joint.row.className !== rowClass) joint.row.className = rowClass;
    if (!scale) {
      joint.zero.hidden = true;
      joint.ghost.hidden = true;
      return;
    }
    var low = scale.lower[i];
    var high = scale.upper[i];
    if (low < 0 && high > 0) {
      joint.zero.hidden = false;
      joint.zero.style.left = pct(0, low, high).toFixed(2) + '%';
    } else {
      joint.zero.hidden = true;
    }
    if (typeof value === 'number' && isFinite(value)) {
      joint.thumb.hidden = false;
      joint.thumb.style.left = pct(value, low, high).toFixed(2) + '%';
    } else {
      joint.thumb.hidden = true;
    }
    var target = targets[i];
    if (typeof target === 'number' && isFinite(target)
        && typeof value === 'number' && Math.abs(target - value) > 0.001) {
      joint.ghost.hidden = false;
      joint.ghost.style.left = pct(target, low, high).toFixed(2) + '%';
    } else {
      joint.ghost.hidden = true;
    }
  });

  var statusLine = typeof arm.status_line === 'string' ? arm.status_line : '';
  if (refs.status.textContent !== statusLine) refs.status.textContent = statusLine;

  if (refs.gripper) patchGripper(frame, armId, refs.gripper);
}

function patchGripper(frame, armId, refs) {
  var g = gripperOf(frame, armId);
  var busy = g.busy === true;
  var elsewhere = lockIsElsewhere(frame);
  var pending = ui.pending['gripper:' + armId] === true;
  // The first three conditions are exactly the three the jog buttons use,
  // plus this page's own pending flag. The fourth is Watch: "observe only,
  // arm free", and moving fingers is motion.
  //
  // THE WATCH GATE IS PAGE-ONLY, DELIBERATELY. The API does not refuse a
  // gripper command in Watch: the gripper is a standing node commandable
  // from ROS in every mode, so a server-side mode gate would refuse this
  // button while the identical motion stayed one `ros2 action send_goal`
  // away — a fence with no fence-post. Disabled buttons here are an
  // affordance against an accidental click, not a boundary. Simulate is a
  // DIFFERENT rule and needs no branch at all: there the server sends
  // configured:false and buildTile never builds this row.
  var watching = frame.session && frame.session.mode === 'watch';
  var blocked = g.available !== true || busy || elsewhere || pending || watching;
  refs.open.disabled = blocked;
  refs.close.disabled = blocked;
  var width = typeof g.width_mm === 'number' && isFinite(g.width_mm)
    ? g.width_mm.toFixed(1) + ' mm' : '—';
  if (refs.width.textContent !== width) refs.width.textContent = width;
  var cls = 'pill ' + ({ok: 'pill-ok', warn: 'pill-warning',
                        error: 'pill-fault'}[g.level] || 'pill-unknown');
  if (refs.pill.className !== cls) refs.pill.className = cls;
  // textContent only, never markup: a driver can print anything.
  var line = typeof g.status_line === 'string' ? g.status_line : '';
  if (refs.pillLabel.textContent !== line) refs.pillLabel.textContent = line;
}

function patchControl(frame, armId, refs, elsewhere, session) {
  var motion = motionOf(frame, armId);
  var enabled = motion.enabled === true;
  var enablePending = ui.pending['enable:' + armId] === true;
  var switchClass = 'switch' + (enabled ? ' on' : '');
  if (refs['switch'].className !== switchClass) refs['switch'].className = switchClass;
  refs['switch'].setAttribute('aria-checked', enabled ? 'true' : 'false');
  // The enable control is ALWAYS pressable while this page holds the lock.
  // enable_service_available is advisory only; it never gates this control,
  // because the switch is the product's only instant-disable affordance.
  refs['switch'].disabled = elsewhere || enablePending;
  refs.lockSub.hidden = !elsewhere;
  refs.serviceSub.hidden = motion.enable_service_available !== false;

  if (refs.srcButtons) {
    var sourcePending = ui.pending['source:' + armId] === true;
    refs.srcRow.className = 'srcrow' + (enabled ? '' : ' muted');
    refs.srcButtons.forEach(function (button) {
      button.classList.toggle('sel', button.dataset.val === motion.source);
      // 'elsewhere' for the same reason as the switch and the jog buttons: a
      // press that cannot succeed must not look pressable on a locked card.
      button.disabled = sourcePending || elsewhere;
    });
  }

  if (refs.jogButtons) {
    refs.jogButtons.forEach(function (button) {
      var key = 'jog:' + armId + ':' + button.dataset.j + ':' + button.dataset.dir;
      button.disabled = !enabled || elsewhere || ui.pending[key] === true;
    });
  }

  if (refs.apply) patchApplyPanel(frame, armId, refs.apply, elsewhere);

  if (refs.ext) {
    var ext = refs.ext;
    var topic = motion.command_topic || '';
    if (ext.topic.textContent !== topic) ext.topic.textContent = topic;
    var template = motion.command_template || '';
    if (ext.template.textContent !== template) ext.template.textContent = template;
    ext.notReady.hidden = motion.command_template_ready !== false;
    var rateText = fmtRate(motion.external_rate_hz);
    if (ext.rate.textContent !== rateText) ext.rate.textContent = rateText;
    ext.rate.classList.toggle('on', motion.external_rate_hz != null
      && motion.external_rate_hz >= 10);
    ext.topicCopy.textContent = ui.copied['topic:' + armId] ? 'Copied' : 'Copy';
    ext.tmplCopy.textContent = ui.copied['tmpl:' + armId] ? 'Copied' : 'Copy template';
  }
}

/* ------------------------------------------------- frames and reconnects --- */

function onFrame(frame) {
  if (!frame || frame.schema_version !== 5) {
    notice('This page is out of date — reload it.');
    render();
    return;
  }

  // 1. Monotonic guard FIRST. A stale frame updates NOTHING.
  //    The mark is a WALL CLOCK, so it is corroborated with server_uptime_s,
  //    which is monotonic within a run. The race this guard exists for — a
  //    1 Hz poll answer losing to a newer stream frame — carries an older
  //    uptime as well, so it is still dropped. A backward wall-clock step on
  //    the server (chrony makestep, timedatectl, a corrected RTC) does not
  //    stop uptime advancing, so the page no longer wedges for the size of
  //    the step; and a restart regresses uptime, which must reach step 2
  //    rather than be swallowed here.
  var uptime = frame.server_uptime_s;
  var tracked = net.lastUptime != null && typeof uptime === 'number';
  var restarted = tracked && uptime + 1 < net.lastUptime;
  var fresher = tracked && uptime > net.lastUptime;
  if (frame.server_time && frame.server_time < net.lastServerTime
      && !restarted && !fresher) {
    return;
  }

  // 2. Restart: a server_uptime_s REGRESSION, and nothing else.
  if (restarted) onServerRestart();          // clears the wall-clock mark too
  net.lastServerTime = frame.server_time || net.lastServerTime;
  net.lastUptime = uptime;

  // 3. Backfill: last_seq running AHEAD of us by more than one queue depth.
  if (frame.logs && frame.logs.last_seq - net.lastSeq > LOG_GAP_TOLERANCE) {
    scheduleResync();                       // debounced
  }
  if (frame.logs) {                         // badge is correct even with no log event
    net.warnCount = frame.logs.warn_count;
    net.errorCount = frame.logs.error_count;
    syncLogBadge();
  }

  // 4. New session id (no restart): drop per-session view state, keep the rest.
  var sessionId = frame.session.session_id;
  if (sessionId && net.lastSessionId && sessionId !== net.lastSessionId) {
    ui.pending = {}; ui.copied = {}; ui.tmplOpen = {};
    ui.recoverUntil = 0; ui.recoverInFlight = false;
  }
  net.lastSessionId = sessionId;

  net.frame = frame;
  // The one-shot Recover press stays disabled until the session leaves
  // 'fault' — and, WITHIN fault, only while the server's own frame shows a
  // recovery running or the request is still unanswered inside its bound.
  if (frame.session.state !== 'fault') {
    delete ui.pending.recover;
    ui.recoverUntil = 0;
    ui.recoverInFlight = false;
  } else {
    resolveRecoverPending();
  }
  render();
}

function onServerRestart() {
  if (net.restarting) return;                            // idempotent: never boot twice
  net.restarting = true;
  net.token = null; net.claimId = null;                  // the old token is meaningless
  if (net.heartbeatTimer) { clearInterval(net.heartbeatTimer); net.heartbeatTimer = null; }
  net.lastSeq = 0; net.warnCount = 0; net.errorCount = 0; net.dropped = 0;
  // The new run's clock is unrelated to the old run's: keeping the mark would
  // drop every frame from a server whose wall clock now reads earlier.
  net.lastServerTime = ''; net.lastUptime = null;
  el('logList').replaceChildren();      // seq restarts at 1; old lines are another run
  ui.pending = {}; ui.takeoverOpen = false;
  ui.recoverUntil = 0; ui.recoverInFlight = false;
  net.caps = null; net.config = null; net.configTried = false;
  net.lastSessionId = null;
  dom.profileFor = null;
  // The 3D module survives a restart — the model it drew is the same one — but
  // every point-in-time fact about the new run has to be asked for again.
  scene.info = null; scene.present = {}; scene.solveNote = null;
  scene.moduleNote = {}; scene.copiedText = {}; scene.rateNoticeSince = 0;
  scene.recheckRows = []; scene.recheckQueue = [];
  fetchScene();
  syncLogBadge();
  bootMetadata().then(function () { net.restarting = false; },
                      function () { net.restarting = false; });
  notice('The server restarted. This page reconnected; '
    + 'any session it was running is gone.');
}

function resync() {
  net.resyncing = true;
  return api('GET', '/api/state').then(function (result) { onFrame(result.state); })
    .then(function () { return api('GET', '/api/logs?since=' + net.lastSeq); })
    .then(applyBacklog)
    .catch(function () { /* the next tick tries again */ })
    .then(function () { net.resyncing = false; });
}

// At most one repair per 2 s, and never a second one while the first is still
// in flight: until its backfill lands every frame still shows the same gap,
// and scheduling on those would queue a redundant repair behind it.
function scheduleResync() {
  if (net.resyncTimer || net.resyncing) return;
  net.resyncTimer = setTimeout(function () {
    net.resyncTimer = null;
    resync();
  }, RESYNC_DEBOUNCE_MS);
}

function startPolling() {
  if (net.pollTimer) return;
  net.pollTimer = setInterval(function () {
    api('GET', '/api/state').then(function (result) { onFrame(result.state); })
      .catch(function () { /* the next tick tries again */ });
    api('GET', '/api/logs?since=' + net.lastSeq).then(applyBacklog)
      .catch(function () { /* the next tick tries again */ });
  }, DEFAULTS.poll_ms);
}

function stopPolling() {
  if (!net.pollTimer) return;
  clearInterval(net.pollTimer);
  net.pollTimer = null;
}

function connect() {
  if (!window.EventSource) { startPolling(); return; }
  var source = new EventSource('/api/state/stream');
  net.source = source;
  source.addEventListener('state', function (event) { onFrame(parse(event.data)); });
  source.addEventListener('log', function (event) { onLogEvent(parse(event.data)); });
  source.addEventListener('ping', function () { /* liveness only */ });
  source.onopen = function () {
    net.live = true;
    stopPolling();
    resync();                       // one /api/state + one /api/logs backfill
    fetchScene();                   // point-in-time facts; no polling loop
  };
  source.onerror = function () {
    net.live = false;
    render();                       // #linkChip appears; the last frame stays on screen
    startPolling();                 // EventSource retries on its own
  };
}

/* ------------------------------------------------------- render and boot --- */

function syncHint(frame) {
  var node = el('hintText');
  // The frame's sentence verbatim, never composed; the shell sentence only
  // when there is no frame at all, so the two paths agree and the line is
  // never left blank under its NEXT label.
  var text = frame ? (typeof frame.hint === 'string' ? frame.hint : '') : SHELL_HINT;
  if (node.textContent !== text) node.textContent = text;
}

function render() {
  var frame = net.frame;
  syncChrome(frame); syncSession(frame); syncHint(frame); syncNotice();
  // A fault in the 3D panel must never be able to take this function down with
  // it: render() is the whole console, arm cards and hint line included.
  try { syncScene(frame); }
  catch (error) { scene.failed = error; sceneFallbackText(); }
  var signature = frame ? stageSignature(frame) : 'empty';
  if (signature !== ui.stageSignature) {
    ui.stageSignature = signature;
    if (frame) buildStage(frame);
  }
  if (frame) patchStage(frame);
}

function tick() {
  if (ui.notice != null && Date.now() >= ui.noticeUntil) {
    ui.notice = null;
    syncNotice();
  }
  // A recover request that never answers, on a page that stops receiving
  // frames, must still release its control. This is the only path that runs
  // without a frame, so the pending state can never outlive its bound.
  if (ui.pending.recover === true) {
    resolveRecoverPending();
    if (ui.pending.recover !== true) render();
  }
  var timeNode = el('opTime');
  if (timeNode && net.frame && net.frame.operator) {
    timeNode.textContent = sinceMinutes(net.frame.operator.since);
  }
}

function bootMetadata() {                    // returns a promise
  var caps = api('GET', '/api/capabilities').then(function (result) {
    net.caps = result;
    el('brandSub').textContent = 'dual-Panda cell · ' + result.server_version;
    // The first frame can beat this response, and the jog step size is baked
    // into the jog panel at build time. Force one rebuild so the note and the
    // per-button aria-labels carry the server's real step rather than none.
    ui.stageSignature = null;
    render();
  }).catch(function () { /* DEFAULTS carry the page */ });
  var config = api('GET', '/api/config').then(function (result) {
    net.configTried = true;
    net.config = result;
    render();
  }).catch(function () {
    net.configTried = true;       // only now may the profile line say 'unavailable'
    render();
  });
  return Promise.all([caps, config]);
}

function wireSceneTheme() {
  if (!window.matchMedia) return;
  var query = window.matchMedia('(prefers-color-scheme: dark)');
  var onChange = function () {
    if (scene.handle) scene.handle.setTheme(currentTheme(), readScenePalette());
  };
  if (query.addEventListener) query.addEventListener('change', onChange);
  else if (query.addListener) query.addListener(onChange);
}

function boot() {
  wireDelegatedClicks();
  wireLogList();
  wireSceneTheme();
  window.addEventListener('pagehide', releaseOnUnload);
  window.addEventListener('beforeunload', releaseOnUnload);
  setInterval(tick, 1000);        // badge relative time + notice expiry only
  bootMetadata();
  connect();
  render();                       // paint the empty shell immediately
  // Wide screens open the panel beside the arm cards; a narrow one keeps the
  // slim bar, so a phone never flashes an empty 320 px panel and downloads
  // nothing until someone asks for it.
  setSceneOpen(!(window.matchMedia && window.matchMedia(SCENE_NARROW).matches));
  fetchSceneIfNeeded();
}

boot();
})();
