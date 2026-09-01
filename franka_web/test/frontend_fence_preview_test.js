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

const previewOutput = { textContent: '', className: '' };
global.document = {
  getElementById(id) {
    assert.equal(id, 'preview-status');
    return previewOutput;
  },
};

const {
  evaluateJointFence,
  fenceDetailText,
  renderPreviewStatus,
  usesReviewedJointFence,
} = require('../static/app.js');

const LOWER = [-1.0, -1.1, -1.2, -2.0, -1.4, 0.1, -1.6];
const UPPER = [1.0, 1.1, 1.2, -0.1, 1.4, 2.5, 1.6];
const INSIDE = [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0];

function makeArm(overrides = {}) {
  const defaultMotion = {
    fence_lower: [...LOWER],
    fence_upper: [...UPPER],
    pose_inside_fence: true,
  };
  const motion = Object.prototype.hasOwnProperty.call(overrides, 'motion')
    ? overrides.motion : defaultMotion;
  return {
    positions: [...INSIDE],
    positions_stale: false,
    ...overrides,
    motion,
  };
}

function makeFrame(armIds, arms, sessionOverrides = {}) {
  return {
    session: {
      state: 'running',
      mode: 'watch',
      gains_sha256: 'a'.repeat(64),
      arm_ids: armIds,
      ...sessionOverrides,
    },
    arms,
  };
}

function preview(frame) {
  previewOutput.textContent = '';
  previewOutput.className = '';
  renderPreviewStatus(frame);
  return { text: previewOutput.textContent, className: previewOutput.className };
}

test('only positions_stale exactly false is fresh in rows and summary', () => {
  for (const positionsStale of [undefined, null, true, 0]) {
    const arm = makeArm({ positions_stale: positionsStale });
    for (let joint = 0; joint < 7; joint += 1) {
      const result = evaluateJointFence(arm, joint, true);
      assert.equal(result.status, 'unverified');
      assert.equal(result.signedMargin, null);
    }
    const result = preview(makeFrame(['panda1'], { panda1: arm }));
    assert.equal(result.className, 'unverified');
    assert.match(result.text, /panda1 Unverified$/);
  }

  const fresh = makeArm({ positions_stale: false });
  assert.equal(evaluateJointFence(fresh, 0, true).status, 'inside');
  assert.equal(preview(makeFrame(['panda1'], { panda1: fresh })).className, 'inside');
});

test('aggregate Inside requires the exact expected Watch arm set', () => {
  const panda1 = makeArm();
  const panda2 = makeArm();

  for (const frame of [
    makeFrame(['panda1', 'panda2'], { panda1 }),
    makeFrame(['panda1'], { panda1, panda2 }),
    makeFrame(['panda1', 'panda2'], { panda1, panda2, panda3: makeArm() }),
  ]) {
    const result = preview(frame);
    assert.equal(result.className, 'unverified');
    assert.match(result.text, /Unverified \(Watch arm set does not match the session\)$/);
  }

  const single = preview(makeFrame(['panda1'], { panda1 }));
  assert.equal(single.className, 'inside');
  assert.match(single.text, /panda1 Inside$/);

  const dual = preview(makeFrame(['panda1', 'panda2'], { panda2, panda1 }));
  assert.equal(dual.className, 'inside');
  assert.match(dual.text, /panda1 Inside · panda2 Inside$/);
});

test('a short positions array makes every reviewed joint row unverified', () => {
  const arm = makeArm({ positions: [INSIDE[0]] });
  for (let joint = 0; joint < 7; joint += 1) {
    const result = evaluateJointFence(arm, joint, true);
    assert.equal(result.status, 'unverified');
    assert.equal(result.position, null);
    assert.equal(result.signedMargin, null);
  }
  assert.equal(
    preview(makeFrame(['panda1'], { panda1: arm })).className,
    'unverified');
});

test('short or absent reviewed fence arrays never fall back per joint', () => {
  const malformedMotions = [
    { fence_lower: LOWER.slice(0, 1), fence_upper: [...UPPER] },
    { fence_lower: [...LOWER], fence_upper: UPPER.slice(0, 1) },
    { fence_lower: null, fence_upper: null },
    {},
  ];
  for (const motion of malformedMotions) {
    const arm = makeArm({ motion });
    for (let joint = 0; joint < 7; joint += 1) {
      const result = evaluateJointFence(arm, joint, true);
      assert.equal(result.source, 'reviewed');
      assert.equal(result.status, 'unverified');
      assert.equal(result.low, null);
      assert.equal(result.high, null);
      assert.equal(result.signedMargin, null);
    }
  }
});

test('a consistent true verdict renders complete inside rows and summary', () => {
  const arm = makeArm();
  for (let joint = 0; joint < 7; joint += 1) {
    const result = evaluateJointFence(arm, joint, true);
    assert.equal(result.status, 'inside');
    assert.equal(Number.isFinite(result.signedMargin), true);
  }
  const summary = preview(makeFrame(['panda1'], { panda1: arm }));
  assert.equal(summary.className, 'inside');
  assert.match(summary.text, /panda1 Inside$/);
});

test('a contradictory true verdict with outside q invalidates every result', () => {
  const outsidePositions = [...INSIDE];
  outsidePositions[0] = UPPER[0] + 0.01;
  const arm = makeArm({
    positions: outsidePositions,
    motion: {
      fence_lower: [...LOWER],
      fence_upper: [...UPPER],
      pose_inside_fence: true,
    },
  });
  for (let joint = 0; joint < 7; joint += 1) {
    const result = evaluateJointFence(arm, joint, true);
    assert.equal(result.status, 'unverified');
    assert.equal(result.reason, 'server-verdict-inconsistent');
    assert.equal(result.signedMargin, null);
    assert.match(fenceDetailText(result), /server fence verdict is inconsistent with q\/L\/U/);
  }
  const summary = preview(makeFrame(['panda1'], { panda1: arm }));
  assert.equal(summary.className, 'unverified');
  assert.match(summary.text, /server verdict inconsistent with q\/L\/U/);
});

test('a missing or non-boolean verdict keeps inside q Unverified', () => {
  for (const serverVerdict of [null, undefined, 'true']) {
    const arm = makeArm({
      motion: {
        fence_lower: [...LOWER],
        fence_upper: [...UPPER],
        pose_inside_fence: serverVerdict,
      },
    });
    for (let joint = 0; joint < 7; joint += 1) {
      const result = evaluateJointFence(arm, joint, true);
      assert.equal(result.status, 'unverified');
      assert.equal(result.reason, 'server-verdict-unavailable');
      assert.equal(result.signedMargin, null);
      assert.match(fenceDetailText(result), /server fence verdict is missing or non-boolean/);
    }
    const summary = preview(makeFrame(['panda1'], { panda1: arm }));
    assert.equal(summary.className, 'unverified');
    assert.match(summary.text, /server verdict missing or non-boolean/);
  }
});

test('a consistent false verdict preserves ordinary outside display', () => {
  const outsidePositions = [...INSIDE];
  outsidePositions[0] = UPPER[0] + 0.01;
  const arm = makeArm({
    positions: outsidePositions,
    motion: {
      fence_lower: [...LOWER],
      fence_upper: [...UPPER],
      pose_inside_fence: false,
    },
  });
  const outside = evaluateJointFence(arm, 0, true);
  assert.equal(outside.status, 'outside');
  assert.ok(outside.signedMargin < 0);
  for (let joint = 1; joint < 7; joint += 1) {
    assert.equal(evaluateJointFence(arm, joint, true).status, 'inside');
  }
  const summary = preview(makeFrame(['panda1'], { panda1: arm }));
  assert.equal(summary.className, 'outside');
  assert.match(summary.text, /panda1 OUTSIDE$/);
});

test('a verified OUTSIDE arm keeps red priority over an Unverified sibling', () => {
  const outsidePositions = [...INSIDE];
  outsidePositions[0] = UPPER[0] + 0.01;
  const outside = makeArm({
    positions: outsidePositions,
    motion: {
      fence_lower: [...LOWER],
      fence_upper: [...UPPER],
      pose_inside_fence: false,
    },
  });
  const unverified = makeArm({
    motion: {
      fence_lower: [...LOWER],
      fence_upper: [...UPPER],
      pose_inside_fence: null,
    },
  });
  const summary = preview(makeFrame(
    ['panda1', 'panda2'], { panda1: outside, panda2: unverified }));
  assert.equal(summary.className, 'outside');
  assert.match(summary.text, /panda1 OUTSIDE/);
  assert.match(summary.text, /panda2 Unverified \(server verdict missing or non-boolean\)$/);
});

test('positions exactly on lower or upper bounds remain inclusively Inside', () => {
  const boundaryPositions = LOWER.map((low, index) =>
    index % 2 === 0 ? low : UPPER[index]);
  const arm = makeArm({ positions: boundaryPositions });
  for (let joint = 0; joint < 7; joint += 1) {
    const result = evaluateJointFence(arm, joint, true);
    assert.equal(result.status, 'inside');
    assert.equal(result.signedMargin, 0);
  }
  const summary = preview(makeFrame(['panda1'], { panda1: arm }));
  assert.equal(summary.className, 'inside');
  assert.match(summary.text, /panda1 Inside$/);
});

test('Motion Hold is a reviewed config without a joint-angle fence', () => {
  const arm = makeArm({
    motion: { fence_lower: null, fence_upper: null, pose_inside_fence: null },
  });
  const frame = makeFrame(['panda1'], { panda1: arm }, {
    mode: 'motion',
    controller_name: 'dual_arm_joint_hold_controller',
  });
  const summary = preview(frame);
  assert.equal(summary.className, '');
  assert.match(summary.text, /^Reviewed Hold config /);
  assert.match(summary.text, /no joint-angle fence/);
  assert.match(summary.text, /Panda limits for display only$/);
  assert.equal(usesReviewedJointFence(frame.session), false);

  const row = evaluateJointFence(arm, 0, usesReviewedJointFence(frame.session));
  assert.equal(row.source, 'panda-display');
  assert.equal(row.status, 'display-only');

  assert.equal(usesReviewedJointFence(makeFrame([], {}).session), true);
  assert.equal(usesReviewedJointFence({
    mode: 'motion',
    gains_sha256: 'b'.repeat(64),
    controller_name: 'dual_arm_joint_impedance_controller',
  }), true);
});

test('empty stopped-Watch arm data remains Unverified', () => {
  const result = preview(makeFrame(['panda1', 'panda2'], {}, { state: 'stopped' }));
  assert.equal(result.className, 'unverified');
  assert.match(result.text, /Unverified \(no active Watch arm data\)$/);
});
