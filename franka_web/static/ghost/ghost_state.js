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

// Provenance: the Session C renderer prototype, moved unchanged at the ghost merge.
//
// The per-arm measured/ghost joint store. Salvaged from the prototype's
// per-joint drag controller; everything arc-, panel- and keyboard-shaped is
// gone with the per-joint editing it served.

const JOINT_COUNT = 7;

//: Any joint step larger than this in one frame is a teleport, not motion, so
//: it is snapped rather than smoothed. Smoothing a jump invents motion that is
//: not happening, which is the one failure the interpolation rule prevents.
const TELEPORT_STEP_RAD = 0.35;

//: The ghost "differs from reality" test, and therefore what shows and hides
//: the Copy affordance.
const DIFFERS_RAD = 1e-4;

export function armIdForIndex(armIndex) {
  if (armIndex !== 1 && armIndex !== 2) {
    throw new RangeError("armIndex must be 1 or 2");
  }
  return `panda${armIndex}`;
}

export function requireJointIndex(jointIndex) {
  if (!Number.isInteger(jointIndex) || jointIndex < 0 || jointIndex >= JOINT_COUNT) {
    throw new RangeError("jointIndex must be an integer from 0 through 6");
  }
}

export function cloneSeven(values, name) {
  if (!Array.isArray(values) || values.length !== JOINT_COUNT
      || values.some((value) => !Number.isFinite(value))) {
    throw new TypeError(`${name} must contain exactly seven finite numbers`);
  }
  return [...values];
}

/** Register a listener in a registry that dispose() can drain to zero. */
export function addTrackedListener(registry, target, type, listener, options) {
  target.addEventListener(type, listener, options);
  registry.push({target, type, listener, options});
}

/** Remove every listener a registry holds. */
export function removeTrackedListeners(registry) {
  while (registry.length > 0) {
    const entry = registry.pop();
    entry.target.removeEventListener(entry.type, entry.listener, entry.options);
  }
}

function setNodeAngle(graph, name, value) {
  const node = graph.jointNodes.get(name);
  if (!node) {
    return;
  }
  node.userData.value = value;
  node.quaternion.setFromAxisAngle(node.userData.axis, value);
}

/**
 * Build the per-arm measured/ghost store for one mount.
 *
 * `render` is the scene's requestRender; `onChange` is called with
 * (armIndex, ghost7, differsFromMeasured) whenever a ghost pose moves.
 */
export function createGhostState({
  three,
  model,
  solidGraph,
  ghostGraph,
  armIndices = [1, 2],
  initialArm = 1,
  render,
  onChange = null,
}) {
  if (!model || !Array.isArray(model.joints)) {
    throw new TypeError("model must be a parsed URDF model");
  }
  armIdForIndex(initialArm);
  if (!Array.isArray(armIndices) || armIndices.length < 1 || armIndices.length > 2
      || new Set(armIndices).size !== armIndices.length
      || armIndices.some((armIndex) => armIndex !== 1 && armIndex !== 2)
      || !armIndices.includes(initialArm)) {
    throw new TypeError("armIndices must contain one or two unique configured arms");
  }

  const states = new Map();
  const listeners = [];
  let selected = initialArm;
  let disposed = false;

  for (const armIndex of armIndices) {
    const armId = armIdForIndex(armIndex);
    const joints = Array.from({length: JOINT_COUNT}, (_unused, index) => {
      const name = `${armId}_joint${index + 1}`;
      const joint = model.joints.find((candidate) => candidate.name === name);
      if (!joint || !joint.limit) {
        throw new Error(`model is missing limited revolute joint ${name}`);
      }
      return joint;
    });
    states.set(armIndex, {
      armIndex,
      armId,
      joints,
      urdfLower: joints.map((joint) => joint.limit.lower),
      urdfUpper: joints.map((joint) => joint.limit.upper),
      fenceLower: joints.map((joint) => joint.limit.lower),
      fenceUpper: joints.map((joint) => joint.limit.upper),
      fenceSource: "urdf",
      previous: new Array(JOINT_COUNT).fill(0),
      target: new Array(JOINT_COUNT).fill(Number.NaN),
      displayed: new Array(JOINT_COUNT).fill(0),
      arrivalMs: 0,
      framePeriodMs: 0,
      settled: true,
      ghost: new Array(JOINT_COUNT).fill(Number.NaN),
      ghostVisible: false,
      present: false,
      stale: false,
      verdict: null,
    });
  }

  function stateFor(armIndex) {
    armIdForIndex(armIndex);
    const state = states.get(armIndex);
    if (!state) {
      throw new RangeError("armIndex is not configured for this mount");
    }
    return state;
  }

  /**
   * The only clamp: every ghost value passes through it.
   *
   * It clamps to the URDF limits and nothing else. There is deliberately no
   * session fence here: the ghost commands nothing, so the jog fence is not
   * its business, and borrowing it would imply a motion relationship that does
   * not exist.
   */
  function clampJoint(armIndex, jointIndex, requested) {
    requireJointIndex(jointIndex);
    if (!Number.isFinite(requested)) {
      throw new TypeError("joint value must be finite");
    }
    const state = stateFor(armIndex);
    return Math.min(
      state.fenceUpper[jointIndex],
      Math.max(state.fenceLower[jointIndex], requested),
    );
  }

  function writeSolid(state) {
    state.joints.forEach((joint, jointIndex) => {
      setNodeAngle(solidGraph, joint.name, state.displayed[jointIndex]);
    });
  }

  function writeGhost(state) {
    state.joints.forEach((joint, jointIndex) => {
      const value = state.ghost[jointIndex];
      if (Number.isFinite(value)) {
        setNodeAngle(ghostGraph, joint.name, value);
      }
    });
  }

  function notifyChange(armIndex) {
    if (typeof onChange !== "function") {
      return;
    }
    const state = stateFor(armIndex);
    if (!state.ghost.every(Number.isFinite)) {
      return;
    }
    onChange(armIndex, [...state.ghost], differsFromMeasured(armIndex));
  }

  /**
   * Take one measured frame for one arm.
   *
   * `positions7 === null` means the arm is NOT in the session: it is hidden,
   * never drawn at a fake pose. Inside the array the rule is ELEMENT-WISE: a
   * non-finite element (null included) HOLDS that joint's previous value and
   * is never passed on to three.js or to forward kinematics. Missing joints
   * are routine — the state frame reports null for any joint name absent from
   * the JointState — and this call runs inside the console's whole render
   * pass, so a throw here would take far more than the scene down.
   */
  function setMeasured(armIndex, positions7, meta) {
    const state = stateFor(armIndex);
    if (positions7 === null || positions7 === undefined) {
      setPresent(armIndex, false);
      return;
    }
    if (!Array.isArray(positions7)) {
      throw new TypeError("positions must be an array of seven values or null");
    }
    const options = meta || {};
    const held = state.target.map(
      (value, index) => (Number.isFinite(value) ? value : state.displayed[index]),
    );
    const next = held.slice();
    for (let index = 0; index < JOINT_COUNT; index += 1) {
      const value = positions7[index];
      if (Number.isFinite(value)) {
        next[index] = value;
      }
    }

    const period = options.framePeriodMs;
    const usable = Number.isFinite(period) && period > 0;
    const teleport = next.some(
      (value, index) => Math.abs(value - held[index]) > TELEPORT_STEP_RAD,
    );
    const snap = options.snap === true || state.stale === true || teleport || !usable;

    state.previous = snap ? next.slice() : state.displayed.slice();
    state.target = next;
    state.framePeriodMs = usable ? period : state.framePeriodMs;
    state.arrivalMs = Number.isFinite(options.arrivalMs) ? options.arrivalMs : performance.now();
    state.settled = snap;
    if (snap) {
      state.displayed = next.slice();
    }
    writeSolid(state);

    if (!state.ghost.every(Number.isFinite) && next.every(Number.isFinite)) {
      // A ghost that has never been posed starts at reality, so the first
      // solve of a drag is seeded from the real arm.
      state.ghost = next.map(
        (value, index) => clampJoint(armIndex, index, value),
      );
      writeGhost(state);
    }
    setPresent(armIndex, true);
    if (typeof render === "function") {
      render();
    }
  }

  /** Advance every arm's interpolation to `nowMs`. Solid arms only. */
  function applyInterpolation(nowMs) {
    let moved = false;
    for (const state of states.values()) {
      if (state.settled || !state.present || state.stale) {
        continue;
      }
      const period = state.framePeriodMs;
      if (!Number.isFinite(period) || period <= 0) {
        state.displayed = state.target.slice();
        state.settled = true;
        writeSolid(state);
        moved = true;
        continue;
      }
      const elapsed = nowMs - state.arrivalMs;
      const alpha = Math.min(1, Math.max(0, elapsed / period));
      state.displayed = state.previous.map(
        (value, index) => value + alpha * (state.target[index] - value),
      );
      // Past two frame periods the newest frame is simply late; hold the
      // target rather than keep extrapolating a pose nobody sent.
      if (elapsed >= 2 * period || alpha >= 1) {
        state.displayed = state.target.slice();
        state.settled = true;
      }
      writeSolid(state);
      moved = true;
    }
    if (moved && typeof render === "function") {
      render();
    }
    return moved;
  }

  function measuredNow(armIndex) {
    return [...stateFor(armIndex).displayed];
  }

  function setStale(armIndex, isStale) {
    const state = stateFor(armIndex);
    state.stale = Boolean(isStale);
    if (state.stale) {
      state.displayed = state.target.map(
        (value, index) => (Number.isFinite(value) ? value : state.displayed[index]),
      );
      state.settled = true;
      writeSolid(state);
    }
  }

  function setGhost(armIndex, positions7) {
    const state = stateFor(armIndex);
    const values = cloneSeven(positions7, "positions");
    state.ghost = values.map((value, index) => clampJoint(armIndex, index, value));
    writeGhost(state);
    if (typeof render === "function") {
      render();
    }
    notifyChange(armIndex);
    return [...state.ghost];
  }

  function getGhost(armIndex) {
    return [...stateFor(armIndex).ghost];
  }

  function syncGhostToMeasured(armIndex) {
    const state = stateFor(armIndex);
    if (!state.displayed.every(Number.isFinite)) {
      return null;
    }
    return setGhost(armIndex, state.displayed);
  }

  function setGhostVisible(armIndex, visible) {
    if (typeof visible !== "boolean") {
      throw new TypeError("ghost visibility must be a boolean");
    }
    stateFor(armIndex).ghostVisible = visible;
    return visible;
  }

  function isGhostVisible(armIndex) {
    const state = stateFor(armIndex);
    return state.ghostVisible === true && state.present === true
      && state.ghost.every(Number.isFinite);
  }

  function selectArm(armIndex) {
    stateFor(armIndex);
    selected = armIndex;
    return selected;
  }

  function selectedArm() {
    return selected;
  }

  function setPresent(armIndex, isPresent) {
    const state = stateFor(armIndex);
    state.present = Boolean(isPresent);
    if (!state.present) {
      state.settled = true;
    }
    return state.present;
  }

  function isPresent(armIndex) {
    return stateFor(armIndex).present === true;
  }

  function differsFromMeasured(armIndex) {
    const state = stateFor(armIndex);
    if (!state.ghost.every(Number.isFinite)) {
      return false;
    }
    return state.ghost.some(
      (value, index) => Math.abs(value - state.displayed[index]) > DIFFERS_RAD,
    );
  }

  function setVerdict(armIndex, verdict) {
    stateFor(armIndex).verdict = verdict || null;
  }

  function getVerdict(armIndex) {
    return stateFor(armIndex).verdict;
  }

  /**
   * Narrow the editing limits. Kept from the prototype for one reason: the
   * ghost commands nothing, so it never borrows the session's jog fence, and
   * this build only ever calls it with the URDF limits it already has.
   */
  function setFence(armIndex, fence) {
    if (!fence || typeof fence !== "object") {
      throw new TypeError("fence must be an object");
    }
    const lower = cloneSeven(fence.lower, "fence.lower");
    const upper = cloneSeven(fence.upper, "fence.upper");
    const state = stateFor(armIndex);
    lower.forEach((value, index) => {
      if (value < state.urdfLower[index] || upper[index] > state.urdfUpper[index]) {
        throw new RangeError(`fence for joint ${index + 1} may only narrow the URDF limits`);
      }
      if (value > upper[index]) {
        throw new RangeError(`fence lower exceeds upper for joint ${index + 1}`);
      }
    });
    state.fenceLower = lower;
    state.fenceUpper = upper;
    state.fenceSource = "urdf";
  }

  function jointLimits(armIndex) {
    const state = stateFor(armIndex);
    return {lower: [...state.fenceLower], upper: [...state.fenceUpper]};
  }

  function jointMap(armIndex, positions7) {
    const state = stateFor(armIndex);
    const map = {};
    state.joints.forEach((joint, index) => {
      map[joint.name] = positions7[index];
    });
    return map;
  }

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    removeTrackedListeners(listeners);
    states.clear();
  }

  return {
    setMeasured,
    measuredNow,
    applyInterpolation,
    setStale,
    setGhost,
    getGhost,
    syncGhostToMeasured,
    setGhostVisible,
    isGhostVisible,
    selectArm,
    selectedArm,
    setPresent,
    isPresent,
    differsFromMeasured,
    setVerdict,
    getVerdict,
    setFence,
    clampJoint,
    jointLimits,
    jointMap,
    dispose,
    get armIndices() {
      return [...armIndices];
    },
    get listenerCount() {
      return listeners.length;
    },
  };
}
