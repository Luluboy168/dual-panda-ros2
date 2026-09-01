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

const assert = require('node:assert/strict');
const test = require('node:test');

class FakeElement {
  constructor(tagName) {
    this.tagName = tagName.toUpperCase();
    this.children = [];
    this.hidden = false;
    this.className = '';
    this.textContent = '';
    this.listeners = [];
  }

  appendChild(child) {
    this.children.push(child);
    return child;
  }

  addEventListener(type, handler) {
    this.listeners.push({ type, handler });
  }

  set innerHTML(value) {
    assert.equal(value, '');
    this.children = [];
  }

  get innerHTML() {
    return '';
  }
}

const controlCard = new FakeElement('section');
const controlBody = new FakeElement('div');

global.document = {
  createElement(tagName) {
    return new FakeElement(tagName);
  },
  getElementById(id) {
    if (id === 'control-card') return controlCard;
    if (id === 'control-body') return controlBody;
    throw new Error(`unexpected element id ${id}`);
  },
};

const { renderControl, sessionStateLabel } = require('../static/app.js');

function descendants(element) {
  return element.children.flatMap((child) => [child, ...descendants(child)]);
}

function frame(state) {
  return {
    session: {
      session_id: 'offline-test',
      state,
      mode: 'motion',
      controller_name: 'dual_arm_joint_impedance_controller',
    },
    fault: { recoverable: false, reasons: [] },
    arms: {
      panda1: { motion: { available: true, enabled: false } },
      panda2: { motion: { available: true, enabled: false } },
    },
  };
}

test('settling is visibly torque-active and exposes no enable or jog surface', () => {
  const staleEnable = new FakeElement('button');
  staleEnable.addEventListener('click', () => {});
  controlBody.appendChild(staleEnable);
  renderControl(frame('settling'));

  assert.equal(controlCard.hidden, false);
  assert.equal(controlBody.children.length, 1);
  assert.equal(controlBody.children[0].tagName, 'P');
  assert.equal(controlBody.children[0].className, 'settling-note');
  assert.equal(
    controlBody.children[0].textContent,
    'Torque control is active; startup settling verification is in progress. '
      + 'Enable and Jog are unavailable.');
  assert.equal(descendants(controlBody).some((node) => node.tagName === 'BUTTON'), false);
  assert.equal(descendants(controlBody).some((node) => node.listeners.length > 0), false);
  assert.equal(sessionStateLabel(frame('settling').session), 'settling (torque active)');

  renderControl(frame('settling'));
  assert.equal(controlBody.children.length, 1);
  assert.equal(controlBody.children[0].textContent,
    'Torque control is active; startup settling verification is in progress. '
      + 'Enable and Jog are unavailable.');
});

test('non-settling state labels remain unchanged', () => {
  for (const state of ['starting', 'running', 'fault', 'stopped']) {
    assert.equal(sessionStateLabel(frame(state).session), state);
  }
});

test('running controls are removed while settling and rebuilt afterwards', () => {
  renderControl(frame('running'));
  let controls = descendants(controlBody);
  assert.equal(controls.filter((node) => node.tagName === 'BUTTON').length, 30);
  assert.equal(controls.filter((node) => node.listeners.length > 0).length, 30);

  renderControl(frame('settling'));
  controls = descendants(controlBody);
  assert.equal(controls.some((node) => node.tagName === 'BUTTON'), false);
  assert.equal(controls.some((node) => node.listeners.length > 0), false);

  renderControl(frame('running'));
  controls = descendants(controlBody);
  assert.equal(controls.filter((node) => node.tagName === 'BUTTON').length, 30);
  assert.equal(controls.filter((node) => node.listeners.length > 0).length, 30);
});
