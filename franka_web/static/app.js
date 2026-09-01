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

var ui = {                           // survives every rebuild; never read from the DOM
  logOpen: false, logFollow: true, tmplOpen: {}, takeoverOpen: false,
  infoOpen: false, notice: null, noticeUntil: 0, pending: {}, copied: {},
  clamped: {}, selArms: 'both', selMode: 'motion',
  stageSignature: null, badgeSignature: null
};
var net = {
  caps: null, config: null, token: null, claimId: null,
  frame: null, lastServerTime: '', lastUptime: null,
  lastSeq: 0, warnCount: 0, errorCount: 0, dropped: 0,
  source: null, pollTimer: null, live: false,
  heartbeatTimer: null, resyncTimer: null, resyncing: false, restarting: false,
  lastSessionId: null
};
// Cached element references for the current stage structure. Rebuilt only
// when the structure signature changes (see render()).
var dom = {kind: null, steps: {}, arms: {}, recFinal: null, profileFor: null};

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
  if (key) {
    if (ui.pending[key]) return;
    ui.pending[key] = true;
    render();
  }
  withLock(run).then(function () {
    if (key && !keepPending) delete ui.pending[key];
    render();
  }, function (error) {
    if (key) delete ui.pending[key];
    noticeFromError(error);
    render();
  });
}

function sessionLocked(frame) {
  return !!(frame && frame.session && frame.session.state !== 'stopped');
}

function copyText(key, button) {
  var parts = String(key).split(':');
  var motion = motionOf(net.frame, parts[1]);
  var text = parts[0] === 'topic' ? motion.command_topic : motion.command_template;
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
    // One shot: the button stays disabled through the fault -> running
    // transition, which onFrame clears when the session leaves 'fault'.
    runAction('recover', function () {
      return api('POST', '/api/session/recover');
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
  el('logBody').hidden = !open;
  el('logBar').setAttribute('aria-expanded', open ? 'true' : 'false');
  if (open) list.scrollTop = list.scrollHeight;
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

function paintIdleShell() {
  var chip = el('stateChip');
  chip.textContent = 'idle';
  chip.className = 'chip chip-idle';
  el('linkChip').hidden = true;
  el('simChip').hidden = true;
  el('recChip').hidden = true;
  el('hintText').textContent = '';
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
               (frame.recording || {}).disabled === true ? 'norec' : '-'];
  (session.arm_ids || []).forEach(function (armId) {
    var motion = (frame.arms[armId] || {}).motion || {};
    parts.push(armId + ':' + String(motion.available) + ':' + String(motion.source)
               + ':' + String(motion.enabled));
  });
  return parts.join('|');
}

function isRecoverySteps(steps) {
  return !!(steps && steps.length && typeof steps[0].id === 'string'
            && steps[0].id.indexOf('reconnect:') === 0);
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
  refs.card = h('section', {class: 'card tile'}, [head, rate, list, refs.status]);
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
    refs.srcButtons = ['jog', 'external'].map(function (value) {
      return h('button', {type: 'button', class: 'seg-btn',
                          dataset: {act: 'source', arm: armId, val: value},
                          text: value === 'jog' ? 'Jog' : 'External'});
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
    var recording = frame.recording || {};
    var text = recording.disabled === true
      ? 'Session ended.'
      : 'Session ended. The recording was saved.';
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

  if (session.state === 'fault' && isRecoverySteps(session.steps)) {
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
      button.disabled = sourcePending;
    });
  }

  if (refs.jogButtons) {
    refs.jogButtons.forEach(function (button) {
      var key = 'jog:' + armId + ':' + button.dataset.j + ':' + button.dataset.dir;
      button.disabled = !enabled || elsewhere || ui.pending[key] === true;
    });
  }

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
  if (!frame || frame.schema_version !== 3) {
    notice('This page is out of date — reload it.');
    render();
    return;
  }

  // 1. Monotonic guard FIRST. A stale frame updates NOTHING.
  if (frame.server_time && frame.server_time < net.lastServerTime) return;
  net.lastServerTime = frame.server_time || net.lastServerTime;

  // 2. Restart: a server_uptime_s REGRESSION, and nothing else.
  if (net.lastUptime != null && frame.server_uptime_s + 1 < net.lastUptime) {
    onServerRestart();
  }
  net.lastUptime = frame.server_uptime_s;

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
  }
  net.lastSessionId = sessionId;

  // The one-shot Recover press stays disabled until the session leaves 'fault'.
  if (frame.session.state !== 'fault') delete ui.pending.recover;

  net.frame = frame;
  render();
}

function onServerRestart() {
  if (net.restarting) return;                            // idempotent: never boot twice
  net.restarting = true;
  net.token = null; net.claimId = null;                  // the old token is meaningless
  if (net.heartbeatTimer) { clearInterval(net.heartbeatTimer); net.heartbeatTimer = null; }
  net.lastSeq = 0; net.warnCount = 0; net.errorCount = 0; net.dropped = 0;
  el('logList').replaceChildren();      // seq restarts at 1; old lines are another run
  ui.pending = {}; ui.takeoverOpen = false;
  net.caps = null; net.config = null;
  net.lastSessionId = null;
  dom.profileFor = null;
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
  var text = frame && typeof frame.hint === 'string' ? frame.hint : '';
  if (node.textContent !== text) node.textContent = text;   // verbatim, never composed
}

function render() {
  var frame = net.frame;
  syncChrome(frame); syncSession(frame); syncHint(frame); syncNotice();
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
    net.config = result;
    render();
  }).catch(function () { /* the profile line shows 'unavailable' */ });
  return Promise.all([caps, config]);
}

function boot() {
  wireDelegatedClicks();
  wireLogList();
  window.addEventListener('pagehide', releaseOnUnload);
  window.addEventListener('beforeunload', releaseOnUnload);
  setInterval(tick, 1000);        // badge relative time + notice expiry only
  bootMetadata();
  connect();
  render();                       // paint the empty shell immediately
}

boot();
})();
