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

// The only interaction module: the flange grab handle, the drag plane, the
// three rotation rings, the elbow ring, and the request discipline that keeps
// a drag inside one solve at a time.
//
// TWO INVARIANTS THIS FILE EXISTS TO KEEP:
//  1. The ghost pose is only ever set from a solved response, from a reset,
//     from the driver, or from the elbow ring's table lerp. It is NEVER set
//     from a raw cursor position, and never moved to a pose the solver did
//     not return.
//  2. The verdict and the copyable pose always describe what is rendered.
//     While the ghost is ahead of the last answer -- which is every moment of
//     an elbow drag -- the verdict reads "pending" and copying is refused.

import {
  axisAngleMatrix,
  forwardKinematics,
  invertRigidMatrix,
  multiplyMatrices,
  quaternionFromMatrix,
  translationFromMatrix,
  translationMatrix,
} from "./kinematics.js";
import {addTrackedListener, removeTrackedListeners} from "./ghost_state.js";

//: A move smaller than this in position, and this in rotation, is not worth a
//: request: it is below what the operator can see and below the solver's own
//: position tolerance.
const MIN_MOVE_M = 0.0005;
const MIN_ROTATION_RAD = 0.0017;

//: Screen-space pick sizes. A fingertip is not a mouse cursor, so the
//: invisible pick proxies are sized in CSS pixels and recomputed whenever the
//: camera moves; the visible handle keeps a small fixed world size.
const PICK_PX_COARSE = 22;
const PICK_PX_FINE = 12;

const HANDLE_RADIUS_M = 0.016;
const TRIAD_LENGTH_M = 0.055;
const RING_SEGMENTS = 96;
const RING_TUBE_M = 0.006;

//: The rotation rings orbit the flange at a CONSTANT SCREEN radius, which is
//: the gizmo idiom the operator already knows from Isaac Sim and every DCC
//: tool: the handles stay the same size to the hand no matter how far the
//: camera is. That also makes the pick band a fixed fraction of the radius,
//: so the annulus geometry is built once and never rebuilt.
const ROTATE_RADIUS_PX = 54;
//: A ring seen edge-on projects to a line: its plane is nearly parallel to
//: the view ray, so the ray/plane intersection runs off to infinity and the
//: smallest cursor twitch would spin the hand. Such a grab is declined
//: outright; the operator orbits a little and the ring is there again.
const ROTATE_EDGE_ON_MIN = 0.12;

//: The three rings, in world axes. Each carries the two in-plane vectors the
//: drag angle is measured from, with u x v = axis, so a point dragged round
//: the ring turns the hand the same way the cursor went.
//
// WORLD axes, not the hand's own. The drag plane above is world-horizontal
// for the same reason: on a table-top cell a world axis means the same thing
// from every orbit angle, and a hand-local ring would make one gesture mean
// three different things depending on where the wrist happened to be.
const ROTATE_AXES = [
  {key: "axisX", axis: [1, 0, 0], u: [0, 1, 0], v: [0, 0, 1]},
  {key: "axisY", axis: [0, 1, 0], u: [0, 0, 1], v: [1, 0, 0]},
  {key: "axisZ", axis: [0, 0, 1], u: [1, 0, 0], v: [0, 1, 0]},
];

//: The elbow table's three acceptance tests.
const TABLE_MIN_ROWS = 5;
//: Every row of the table is a solve against the SAME flange target, so the
//: flange is the point the solver pins -- to its own 1e-4 m position
//: tolerance. Two millimetres is that tolerance plus margin. Measuring this at
//: any unpinned link would fail on every real arm, always.
const FLANGE_SPREAD_M = 0.002;
const PSI_MONOTONE_EPSILON = 1e-6;
const REDUNDANCY_SAMPLES = 25;

const RING_FALLBACK_TEXT = "The spare rotation is mapped directly here; "
  + "the elbow may not follow your cursor exactly.";

function distance3(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function quaternionAngle(a, b) {
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, dot));
}

/** Fold an angle difference into (-pi, pi]. */
function wrapAngle(value) {
  let angle = value;
  while (angle > Math.PI) {
    angle -= 2 * Math.PI;
  }
  while (angle <= -Math.PI) {
    angle += 2 * Math.PI;
  }
  return angle;
}

function unwrapSeries(values) {
  const out = [values[0]];
  for (let index = 1; index < values.length; index += 1) {
    let value = values[index];
    while (value - out[index - 1] > Math.PI) {
      value -= 2 * Math.PI;
    }
    while (value - out[index - 1] < -Math.PI) {
      value += 2 * Math.PI;
    }
    out.push(value);
  }
  return out;
}

export function createHandDrag({
  three,
  canvas,
  camera,
  model,
  ghostGraph,
  ghostState,
  orbitControls,
  render,
  onSolveRequest,
  onStatus,
  palette,
}) {
  const listeners = [];
  const raycaster = new three.Raycaster();
  const pointerNdc = new three.Vector2();
  const worldPoint = new three.Vector3();
  const dragPlane = new three.Plane();

  let colours = Object.assign({}, palette);
  let enabled = true;
  let disposed = false;
  let drag = null;

  // Request discipline. At most one solve in flight; a pointer move while one
  // is outstanding REPLACES a single pending target rather than queueing.
  let inFlight = false;
  let pending = null;
  let rafId = null;
  let backoffUntil = 0;
  let backoffTimer = null;
  let lastSent = null;
  let solveCount = 0;

  // The renderer root frame is the URDF root; the solver wants the target in
  // the arm's own base frame. Both base frames are fixed joints, so this is
  // computed exactly once.
  const zeroPose = forwardKinematics(model, {});
  const invLink0 = {};
  for (const armIndex of ghostState.armIndices) {
    invLink0[armIndex] = invertRigidMatrix(zeroPose.links[`panda${armIndex}_link0`]);
  }

  const overlay = new three.Group();
  overlay.name = "ghost_overlay";
  ghostGraph.root.parent.add(overlay);

  // A pick proxy must stay pickable while drawing nothing: an invisible object
  // is skipped by the raycaster outright, so these are visible objects with
  // colour writes switched off.
  const pickMaterial = new three.MeshBasicMaterial({
    colorWrite: false, depthWrite: false, depthTest: false,
    transparent: true, opacity: 0, side: three.DoubleSide,
  });

  const parts = new Map();
  const pickTargets = [];

  // One unit circle and one unit pick annulus, shared by every rotation ring
  // on both arms. The rings are the same size on screen always, so the band
  // is a fixed fraction of the radius and neither geometry ever changes.
  const circlePoints = [];
  for (let step = 0; step <= RING_SEGMENTS; step += 1) {
    const angle = (step / RING_SEGMENTS) * 2 * Math.PI;
    circlePoints.push(Math.cos(angle), Math.sin(angle), 0);
  }
  const rotateLineGeometry = new three.BufferGeometry();
  rotateLineGeometry.setAttribute(
    "position", new three.BufferAttribute(new Float32Array(circlePoints), 3),
  );
  const rotateBand = (pickPixels() * 0.5) / ROTATE_RADIUS_PX;
  const rotatePickGeometry = new three.RingBufferGeometry(
    1 - rotateBand, 1 + rotateBand, 64, 1,
  );

  function buildRotateRings(armIndex, group) {
    return ROTATE_AXES.map((spec, axisIndex) => {
      const material = new three.LineBasicMaterial({
        color: new three.Color(colours[spec.key] || "#8494A3"), depthTest: false,
      });
      const ringGroup = new three.Group();
      ringGroup.name = `rotate_ring_${armIndex}_${axisIndex}`;
      ringGroup.quaternion.setFromRotationMatrix(new three.Matrix4().makeBasis(
        new three.Vector3(...spec.u),
        new three.Vector3(...spec.v),
        new three.Vector3(...spec.axis),
      ));
      group.add(ringGroup);

      const line = new three.Line(rotateLineGeometry, material);
      line.renderOrder = 31;
      ringGroup.add(line);

      const pick = new three.Mesh(rotatePickGeometry, pickMaterial);
      pick.name = `rotate_pick_${armIndex}_${axisIndex}`;
      pick.userData.pickKind = "rotate";
      pick.userData.armIndex = armIndex;
      pick.userData.axisIndex = axisIndex;
      ringGroup.add(pick);
      pickTargets.push(pick);

      return {
        axisIndex,
        key: spec.key,
        axis: new three.Vector3(...spec.axis),
        u: new three.Vector3(...spec.u),
        v: new three.Vector3(...spec.v),
        group: ringGroup, line, pick, material,
      };
    });
  }

  function buildParts(armIndex) {
    // Materials are per arm so a refusal on one hand cannot tint the other.
    const handleMaterial = new three.MeshBasicMaterial({
      color: new three.Color(colours.handle || "#2557C7"), depthTest: false,
    });
    const triadMaterial = new three.LineBasicMaterial({
      color: new three.Color(colours.handle || "#2557C7"), depthTest: false,
    });
    const ringMaterial = new three.LineBasicMaterial({
      color: new three.Color(colours.ring || "#8494A3"), depthTest: false,
    });
    const group = new three.Group();
    group.name = `hand_handle_${armIndex}`;
    group.userData.armIndex = armIndex;
    group.visible = false;
    overlay.add(group);

    const knob = new three.Mesh(
      new three.SphereBufferGeometry(HANDLE_RADIUS_M, 16, 12), handleMaterial,
    );
    knob.renderOrder = 30;
    group.add(knob);

    const triadPoints = [];
    [[TRIAD_LENGTH_M, 0, 0], [0, TRIAD_LENGTH_M, 0], [0, 0, TRIAD_LENGTH_M]]
      .forEach((axis) => {
        triadPoints.push(0, 0, 0, axis[0], axis[1], axis[2]);
      });
    const triadGeometry = new three.BufferGeometry();
    triadGeometry.setAttribute(
      "position", new three.BufferAttribute(new Float32Array(triadPoints), 3),
    );
    const triad = new three.LineSegments(triadGeometry, triadMaterial);
    triad.renderOrder = 30;
    group.add(triad);

    const pick = new three.Mesh(
      new three.SphereBufferGeometry(1, 12, 8), pickMaterial,
    );
    pick.name = `hand_pick_${armIndex}`;
    pick.userData.pickKind = "hand";
    pick.userData.armIndex = armIndex;
    group.add(pick);
    pickTargets.push(pick);

    const ringGroup = new three.Group();
    ringGroup.name = `elbow_ring_${armIndex}`;
    ringGroup.visible = false;
    overlay.add(ringGroup);

    const ringPoints = [];
    for (let step = 0; step <= RING_SEGMENTS; step += 1) {
      const angle = (step / RING_SEGMENTS) * 2 * Math.PI;
      ringPoints.push(Math.cos(angle), Math.sin(angle), 0);
    }
    const ringGeometry = new three.BufferGeometry();
    ringGeometry.setAttribute(
      "position", new three.BufferAttribute(new Float32Array(ringPoints), 3),
    );
    const ring = new three.Line(ringGeometry, ringMaterial);
    ring.renderOrder = 29;
    ringGroup.add(ring);

    const ringPick = new three.Mesh(
      new three.RingBufferGeometry(1 - RING_TUBE_M, 1 + RING_TUBE_M, 48, 1),
      pickMaterial,
    );
    ringPick.name = `elbow_pick_${armIndex}`;
    ringPick.userData.pickKind = "ring";
    ringPick.userData.armIndex = armIndex;
    ringGroup.add(ringPick);
    pickTargets.push(ringPick);

    parts.set(armIndex, {
      group, knob, triad, pick, ringGroup, ring, ringPick,
      rotate: buildRotateRings(armIndex, group),
      handleMaterial, triadMaterial, ringMaterial,
      band: RING_TUBE_M,
      target: null,             // the FROZEN target orientation for this arm
      active: false,
      refused: false,
    });
  }

  for (const armIndex of ghostState.armIndices) {
    buildParts(armIndex);
  }

  /* ------------------------------------------------------------ geometry --- */

  function ghostLinks(armIndex, positions7) {
    return forwardKinematics(model, ghostState.jointMap(armIndex, positions7)).links;
  }

  function flangeMatrix(armIndex, positions7) {
    return ghostLinks(armIndex, positions7)[`panda${armIndex}_link8`];
  }

  /** Capture the target orientation the hand keeps for the whole session. */
  function captureTarget(armIndex) {
    const part = parts.get(armIndex);
    if (!part) {
      return;
    }
    const ghost = ghostState.getGhost(armIndex);
    if (!ghost.every(Number.isFinite)) {
      part.target = null;
      return;
    }
    const matrix = flangeMatrix(armIndex, ghost);
    // The hand's authored orientation. A hand drag carries it unchanged and a
    // rotation ring rewrites it -- on a SOLVED answer only, never from the
    // cursor -- so this is the pose the operator drew, not a pose the solver
    // reported back. Showing a ghost and Reset are the two moments it is
    // taken from reality again, and the only two.
    part.target = {
      orientation: quaternionFromMatrix(matrix),
      rotation: [
        matrix[0], matrix[1], matrix[2], 0,
        matrix[4], matrix[5], matrix[6], 0,
        matrix[8], matrix[9], matrix[10], 0,
        0, 0, 0, 1,
      ],
    };
  }

  /** Express a world-frame hand pose in the arm's own base frame. */
  function targetInArmBase(armIndex, worldPosition, rotationMatrix) {
    const worldTarget = multiplyMatrices(
      translationMatrix(worldPosition), rotationMatrix,
    );
    const local = multiplyMatrices(invLink0[armIndex], worldTarget);
    return {
      position: translationFromMatrix(local),
      orientation: quaternionFromMatrix(local),
    };
  }

  function worldPerPixel(worldPosition) {
    const bounds = canvas.getBoundingClientRect();
    const height = Math.max(1, bounds.height || canvas.clientHeight || 1);
    const distance = camera.position.distanceTo(
      new three.Vector3(worldPosition[0], worldPosition[1], worldPosition[2]),
    );
    return (2 * distance * Math.tan((camera.fov * Math.PI) / 360)) / height;
  }

  function pickPixels() {
    const coarse = typeof matchMedia === "function"
      && matchMedia("(pointer: coarse)").matches;
    return coarse ? PICK_PX_COARSE : PICK_PX_FINE;
  }

  /** Reposition and re-scale every handle and ring for the current poses. */
  function refresh() {
    if (disposed) {
      return;
    }
    for (const armIndex of ghostState.armIndices) {
      const part = parts.get(armIndex);
      const editable = enabled && ghostState.isGhostVisible(armIndex);
      part.group.visible = editable;
      part.ringGroup.visible = editable;
      if (!editable) {
        continue;
      }
      const ghost = ghostState.getGhost(armIndex);
      const links = ghostLinks(armIndex, ghost);
      const flange = translationFromMatrix(links[`panda${armIndex}_link8`]);
      const shoulder = translationFromMatrix(links[`panda${armIndex}_link1`]);
      const elbow = translationFromMatrix(links[`panda${armIndex}_link4`]);

      part.group.position.set(flange[0], flange[1], flange[2]);
      const perPixel = worldPerPixel(flange);
      part.pick.scale.setScalar(Math.max(HANDLE_RADIUS_M, perPixel * pickPixels() * 0.5));
      placeRotateRings(part, armIndex, perPixel);

      const basis = ringBasis(shoulder, flange, elbow);
      if (basis) {
        placeRing(part, basis);
      } else {
        part.ringGroup.visible = false;
      }
    }
    if (typeof render === "function") {
      render();
    }
  }

  /**
   * Size the three rotation rings, and decide which of them are on screen.
   *
   * During a rotation the other two rings go away. Three concentric circles
   * around a hand that is turning is a picture nobody can read, and every
   * gizmo the operator has used does the same thing.
   */
  function placeRotateRings(part, armIndex, perPixel) {
    const radius = perPixel * ROTATE_RADIUS_PX;
    const soloing = drag && drag.kind === "rotate";
    for (const entry of part.rotate) {
      entry.group.scale.setScalar(radius);
      entry.group.visible = !soloing
        || (drag.armIndex === armIndex && drag.axisIndex === entry.axisIndex);
    }
  }

  /** The ring's own frame: the shoulder-to-FLANGE axis, and the elbow on it. */
  function ringBasis(shoulder, flange, elbow) {
    const axis = new three.Vector3(
      flange[0] - shoulder[0], flange[1] - shoulder[1], flange[2] - shoulder[2],
    );
    if (axis.length() < 1e-6) {
      return null;
    }
    axis.normalize();
    const toElbow = new three.Vector3(
      elbow[0] - shoulder[0], elbow[1] - shoulder[1], elbow[2] - shoulder[2],
    );
    const along = toElbow.dot(axis);
    const radial = toElbow.clone().addScaledVector(axis, -along);
    const radius = radial.length();
    if (radius < 1e-4) {
      return null;
    }
    const u = radial.clone().normalize();
    const v = axis.clone().cross(u).normalize();
    const centre = new three.Vector3(shoulder[0], shoulder[1], shoulder[2])
      .addScaledVector(axis, along);
    return {axis, u, v, centre, radius, shoulder};
  }

  function placeRing(part, basis) {
    const matrix = new three.Matrix4().makeBasis(basis.u, basis.v, basis.axis);
    part.ringGroup.quaternion.setFromRotationMatrix(matrix);
    part.ringGroup.position.copy(basis.centre);
    part.ringGroup.scale.setScalar(basis.radius);
    // The pick annulus is a band around the drawn ring whose WIDTH is a screen
    // measurement, so a fingertip lands on it at any zoom. The group's scale is
    // the ring radius, so the band is expressed as a fraction of it, and the
    // geometry is only rebuilt when that fraction moves materially.
    const band = Math.max(
      RING_TUBE_M,
      (worldPerPixel([basis.centre.x, basis.centre.y, basis.centre.z])
        * pickPixels() * 0.5) / basis.radius,
    );
    if (Math.abs(band - part.band) > 0.1 * part.band) {
      part.ringPick.geometry.dispose();
      part.ringPick.geometry = new three.RingBufferGeometry(
        Math.max(1e-3, 1 - band), 1 + band, 48, 1,
      );
      part.band = band;
    }
  }

  function psiOf(basis, elbow) {
    const toElbow = new three.Vector3(
      elbow[0] - basis.shoulder[0],
      elbow[1] - basis.shoulder[1],
      elbow[2] - basis.shoulder[2],
    );
    toElbow.addScaledVector(basis.axis, -toElbow.dot(basis.axis));
    return Math.atan2(toElbow.dot(basis.v), toElbow.dot(basis.u));
  }

  /* ------------------------------------------------- request discipline --- */

  function status(armIndex, text, verdict) {
    if (typeof onStatus === "function") {
      onStatus({armIndex, text: text || null, verdict: verdict || null});
    }
  }

  function requestSolve(target) {
    pending = target;                                  // replaces, never queues
    if (rafId !== null) {
      return;
    }
    rafId = requestAnimationFrame(flush);
  }

  function flush() {
    rafId = null;
    if (disposed || inFlight || !pending) {
      return;
    }
    const wait = backoffUntil - performance.now();
    if (wait > 0) {
      // Arm ONE timer and re-enter with whatever is newest then. Returning
      // here without rescheduling strands the final target of a drag: the
      // only other rescheduler is the promise tail, which fires no further
      // frame callback once the pointer stops moving -- and a pointer that
      // stops during the backoff window is the ordinary end of a drag.
      if (backoffTimer === null) {
        backoffTimer = setTimeout(function () {
          backoffTimer = null;
          if (pending) {
            requestSolve(pending);
          }
        }, wait);
      }
      return;
    }
    const next = pending;
    pending = null;
    if (lastSent
        && lastSent.armIndex === next.armIndex
        && distance3(next.target.position, lastSent.target.position) < MIN_MOVE_M
        && quaternionAngle(next.target.orientation, lastSent.target.orientation)
           < MIN_ROTATION_RAD) {
      return;
    }
    lastSent = next;
    inFlight = true;
    solveCount += 1;
    Promise.resolve(onSolveRequest({
      kind: "solve",
      armIndex: next.armIndex,
      seed: ghostState.getGhost(next.armIndex),
      target: next.target,
      redundancy: next.redundancy || {mode: "from_seed"},
    })).then(
      (response) => onSolved(next, response),
      (error) => onSolveFailed(next, error),
    ).then(function () {
      inFlight = false;
      if (pending) {
        requestSolve(pending);
      }
    });
  }

  function onSolved(request, response) {
    if (disposed || !response) {
      return;
    }
    const armIndex = request.armIndex;
    if (response.solved === true && Array.isArray(response.positions)) {
      if (request.rotation) {
        // An ACCEPTED rotation becomes the hand's authored orientation, so
        // every later hand drag carries it. The requested rotation is what is
        // kept, not one re-derived from the solution: the solver answers to a
        // tolerance, and re-deriving would let the hand drift a little on
        // every solve of a long drag.
        adoptRotation(armIndex, request.rotation);
      }
      ghostState.setGhost(armIndex, response.positions);
      handleTint(armIndex, false);
      status(armIndex, null, null);
      refresh();
      return;
    }
    // A refusal keeps the ghost exactly where it is and says why. The ghost
    // never moves to a pose the solver did not return.
    handleTint(armIndex, true);
    status(armIndex, response.solve_reason || null, null);
  }

  function onSolveFailed(request, error) {
    if (disposed) {
      return;
    }
    if (error && Number.isFinite(error.retryAfterMs)) {
      backoffUntil = performance.now() + error.retryAfterMs;
      // The dropped INTERMEDIATE targets stay dropped -- a drag's last
      // position is the only one that matters -- but the refused one is the
      // newest there is until the pointer moves again, and a pointer that
      // stops during the backoff window is the ordinary end of a drag.
      pending = pending || request;
      lastSent = null;
      requestSolve(pending);
      return;
    }
    handleTint(request.armIndex, true);
  }

  function adoptRotation(armIndex, rotation) {
    const part = parts.get(armIndex);
    if (part) {
      part.target = {orientation: quaternionFromMatrix(rotation), rotation};
    }
  }

  function handleTint(armIndex, refused) {
    const part = parts.get(armIndex);
    if (!part) {
      return;
    }
    part.refused = refused === true;
    applyHandleColours();
  }

  function applyHandleColours() {
    for (const part of parts.values()) {
      const key = part.refused ? "handleRefused"
        : part.active ? "handleActive" : "handle";
      const colour = colours[key] || colours.handle || "#2557C7";
      part.handleMaterial.color.set(colour);
      part.triadMaterial.color.set(colour);
      part.ringMaterial.color.set(
        (drag && drag.kind === "ring" && drag.armIndex === part.group.userData.armIndex
          ? colours.ringActive : colours.ring) || colours.ring || "#8494A3",
      );
      // An axis colour is an IDENTITY, not a severity: the rings keep theirs
      // through a refusal, and the knob is what turns red. A ring that went
      // red on a refused pose would be unreadable beside the collision tint.
      part.rotate.forEach((entry) => {
        entry.material.color.set(colours[entry.key] || colours.ring || "#8494A3");
      });
    }
    if (typeof render === "function") {
      render();
    }
  }

  /* ------------------------------------------------------- elbow ring ------ */

  async function buildTable(armIndex, basis) {
    const part = parts.get(armIndex);
    const seed = ghostState.getGhost(armIndex);
    const request = {
      kind: "redundancy",
      armIndex,
      seed,
      target: baseTarget(armIndex, seed, part),
      samples: REDUNDANCY_SAMPLES,
    };
    let response;
    try {
      response = await onSolveRequest(request);
    } catch (error) {
      if (error && Number.isFinite(error.retryAfterMs)) {
        // ONE retry, and only one. A hand drag holds the shared budget near
        // empty, and "place the hand, then adjust the elbow" is the ordinary
        // way of working -- so without this the feature would be lost in
        // exactly the workflow it was built for.
        await new Promise((resolve) => setTimeout(resolve, error.retryAfterMs));
        try {
          response = await onSolveRequest(request);
        } catch (_second) {
          return null;
        }
      } else {
        return null;
      }
    }
    return acceptTable(armIndex, basis, response);
  }

  /** The three acceptance tests. Any failure means the ring cannot be honest. */
  function acceptTable(armIndex, basis, response) {
    const rows = (response && Array.isArray(response.table) ? response.table : [])
      .filter((entry) => entry && Number.isFinite(entry.q7)
        && Array.isArray(entry.positions) && entry.positions.length === 7
        && entry.positions.every(Number.isFinite))
      .map((entry) => {
        const links = ghostLinks(armIndex, entry.positions);
        const elbow = translationFromMatrix(links[`panda${armIndex}_link4`]);
        const flange = translationFromMatrix(links[`panda${armIndex}_link8`]);
        return {q7: entry.q7, positions: [...entry.positions], elbow, flange};
      })
      .sort((a, b) => a.q7 - b.q7);

    if (rows.length < TABLE_MIN_ROWS) {
      return null;
    }
    const psiByQ7 = unwrapSeries(rows.map((row) => psiOf(basis, row.elbow)));
    let rising = 0;
    let falling = 0;
    for (let index = 1; index < psiByQ7.length; index += 1) {
      const step = psiByQ7[index] - psiByQ7[index - 1];
      if (step > PSI_MONOTONE_EPSILON) {
        rising += 1;
      } else if (step < -PSI_MONOTONE_EPSILON) {
        falling += 1;
      }
    }
    if (rising > 0 && falling > 0) {
      return null;
    }
    let spread = 0;
    for (let index = 1; index < rows.length; index += 1) {
      spread = Math.max(spread, distance3(rows[0].flange, rows[index].flange));
    }
    if (spread >= FLANGE_SPREAD_M) {
      return null;
    }
    rows.forEach((row, index) => {
      row.psi = psiByQ7[index];
    });
    const sorted = [...rows].sort((a, b) => a.psi - b.psi);
    return {rows: sorted, spread};
  }

  function lerpTable(table, psi) {
    const rows = table.rows;
    const wanted = Math.min(
      rows[rows.length - 1].psi, Math.max(rows[0].psi, psi),
    );
    let low = 0;
    let high = rows.length - 1;
    while (high - low > 1) {
      const mid = (low + high) >> 1;
      if (rows[mid].psi <= wanted) {
        low = mid;
      } else {
        high = mid;
      }
    }
    const span = rows[high].psi - rows[low].psi;
    const alpha = span > 1e-9 ? (wanted - rows[low].psi) / span : 0;
    return {
      positions: rows[low].positions.map(
        (value, index) => value + alpha * (rows[high].positions[index] - value),
      ),
      q7: rows[low].q7 + alpha * (rows[high].q7 - rows[low].q7),
    };
  }

  function baseTarget(armIndex, positions7, part) {
    const flange = translationFromMatrix(flangeMatrix(armIndex, positions7));
    const rotation = part && part.target
      ? part.target.rotation
      : (() => {
        captureTarget(armIndex);
        return parts.get(armIndex).target.rotation;
      })();
    return targetInArmBase(armIndex, flange, rotation);
  }

  /* --------------------------------------------------------- interaction --- */

  function pointerRay(event) {
    const bounds = canvas.getBoundingClientRect();
    pointerNdc.set(
      2 * (event.clientX - bounds.left) / bounds.width - 1,
      1 - 2 * (event.clientY - bounds.top) / bounds.height,
    );
    raycaster.setFromCamera(pointerNdc, camera);
    return raycaster.ray;
  }

  function planeHit(event, plane) {
    pointerRay(event);
    return raycaster.ray.intersectPlane(plane, worldPoint) ? worldPoint.clone() : null;
  }

  function beginHandDrag(event, armIndex) {
    const ghost = ghostState.getGhost(armIndex);
    if (!ghost.every(Number.isFinite)) {
      return false;
    }
    const part = parts.get(armIndex);
    if (!part.target) {
      captureTarget(armIndex);
    }
    const flange = translationFromMatrix(flangeMatrix(armIndex, ghost));
    const origin = new three.Vector3(flange[0], flange[1], flange[2]);
    // The world-horizontal plane through the hand's current height, which is
    // predictable and axis-true above a table. Shift swaps to the vertical
    // line through the hand.
    const normal = event.shiftKey
      ? new three.Vector3().subVectors(camera.position, origin)
          .setComponent(2, 0).normalize()
      : new three.Vector3(0, 0, 1);
    if (normal.lengthSq() < 1e-9) {
      normal.set(0, 0, 1);
    }
    dragPlane.setFromNormalAndCoplanarPoint(normal, origin);
    const plane = dragPlane.clone();
    const hit = planeHit(event, plane);
    if (!hit) {
      return false;
    }
    drag = {
      kind: "hand",
      pointerId: event.pointerId,
      armIndex,
      plane,
      vertical: event.shiftKey === true,
      origin,
      grabOffset: origin.clone().sub(hit),
    };
    part.active = true;
    applyHandleColours();
    return true;
  }

  /* ----------------------------------------------------- rotation rings --- */

  /** Where a world point sits on one ring, as an angle in its own plane. */
  function angleOn(entry, centre, point) {
    const offset = point.clone().sub(centre);
    return Math.atan2(offset.dot(entry.v), offset.dot(entry.u));
  }

  function beginRotateDrag(event, armIndex, axisIndex) {
    const ghost = ghostState.getGhost(armIndex);
    if (!ghost.every(Number.isFinite)) {
      return false;
    }
    const part = parts.get(armIndex);
    if (!part.target) {
      captureTarget(armIndex);
    }
    const entry = part.rotate[axisIndex];
    if (Math.abs(pointerRay(event).direction.dot(entry.axis)) < ROTATE_EDGE_ON_MIN) {
      return false;
    }
    const flange = translationFromMatrix(flangeMatrix(armIndex, ghost));
    const centre = new three.Vector3(flange[0], flange[1], flange[2]);
    const plane = new three.Plane().setFromNormalAndCoplanarPoint(entry.axis, centre);
    const hit = planeHit(event, plane);
    if (!hit) {
      return false;
    }
    drag = {
      kind: "rotate",
      pointerId: event.pointerId,
      armIndex,
      axisIndex,
      entry,
      plane,
      centre,
      // The FROZEN starting orientation. Every frame of the gesture is that
      // one turned by the total angle so far, never the previous frame turned
      // again, so a dropped or refused frame cannot accumulate error.
      startRotation: part.target.rotation,
      lastAngle: 0,
      total: 0,
    };
    drag.lastAngle = angleOn(entry, centre, hit);
    part.active = true;
    applyHandleColours();
    refresh();
    return true;
  }

  function moveRotate(event) {
    const hit = planeHit(event, drag.plane);
    if (!hit) {
      return;
    }
    // Accumulated, not wrapped: a half turn is an ordinary gesture and the
    // arc the cursor travels must keep meaning the same thing past 180
    // degrees.
    const angle = angleOn(drag.entry, drag.centre, hit);
    drag.total += wrapAngle(angle - drag.lastAngle);
    drag.lastAngle = angle;
    const rotation = multiplyMatrices(
      axisAngleMatrix([drag.entry.axis.x, drag.entry.axis.y, drag.entry.axis.z],
                      drag.total),
      drag.startRotation,
    );
    // The hand TURNS; it does not travel. The target position is the flange
    // the gesture started on, so the ring is a pure rotation about the point
    // the operator grabbed around.
    requestSolve({
      armIndex: drag.armIndex,
      target: targetInArmBase(
        drag.armIndex, [drag.centre.x, drag.centre.y, drag.centre.z], rotation,
      ),
      rotation,
    });
  }

  function beginRingDrag(event, armIndex) {
    const ghost = ghostState.getGhost(armIndex);
    if (!ghost.every(Number.isFinite)) {
      return false;
    }
    const links = ghostLinks(armIndex, ghost);
    const basis = ringBasis(
      translationFromMatrix(links[`panda${armIndex}_link1`]),
      translationFromMatrix(links[`panda${armIndex}_link8`]),
      translationFromMatrix(links[`panda${armIndex}_link4`]),
    );
    if (!basis) {
      return false;
    }
    const plane = new three.Plane().setFromNormalAndCoplanarPoint(
      basis.axis, basis.centre,
    );
    const hit = planeHit(event, plane);
    if (!hit) {
      return false;
    }
    drag = {
      kind: "ring",
      pointerId: event.pointerId,
      armIndex,
      basis,
      plane,
      table: null,
      fallback: false,
      startPsi: psiOf(basis, translationFromMatrix(links[`panda${armIndex}_link4`])),
      grabPsi: null,
      startQ7: ghost[6],
      lastQ7: ghost[6],
      lastPositions: [...ghost],
      ready: false,
    };
    drag.grabPsi = Math.atan2(
      hit.clone().sub(basis.centre).dot(basis.v),
      hit.clone().sub(basis.centre).dot(basis.u),
    );
    // The ghost is about to run ahead of the last answer, so the verdict is
    // "pending" and copying is refused until one reconciling solve lands.
    status(armIndex, null, "pending");
    parts.get(armIndex).active = true;
    applyHandleColours();

    const active = drag;
    buildTable(armIndex, basis).then(function (table) {
      if (disposed || drag !== active) {
        return;
      }
      active.table = table;
      active.fallback = table === null;
      active.ready = true;
      if (active.fallback) {
        status(armIndex, RING_FALLBACK_TEXT, "pending");
      }
    });
    return true;
  }

  function moveRing(event) {
    const hit = planeHit(event, drag.plane);
    if (!hit || !drag.ready) {
      return;
    }
    const offset = hit.clone().sub(drag.basis.centre);
    const psi = Math.atan2(offset.dot(drag.basis.v), offset.dot(drag.basis.u));
    const delta = wrapAngle(psi - drag.grabPsi);
    if (drag.fallback) {
      // Option (A): the ring maps straight onto the spare rotation. The elbow
      // will not track the cursor exactly, and the panel says so.
      const limits = ghostState.jointLimits(drag.armIndex);
      const q7 = Math.min(
        limits.upper[6], Math.max(limits.lower[6], drag.startQ7 + delta),
      );
      const positions = [...drag.lastPositions];
      positions[6] = q7;
      drag.lastQ7 = q7;
      drag.lastPositions = ghostState.setGhost(drag.armIndex, positions);
      refresh();
      return;
    }
    const solution = lerpTable(drag.table, drag.startPsi + delta);
    drag.lastQ7 = solution.q7;
    drag.lastPositions = ghostState.setGhost(drag.armIndex, solution.positions);
    refresh();
  }

  function finishRingDrag(active) {
    const armIndex = active.armIndex;
    const part = parts.get(armIndex);
    // Exactly ONE reconciling solve, at the end of the gesture. A failed one
    // leaves the ghost where the lerp put it: discarding a whole end-to-end
    // gesture because its final reconciliation missed is an undo nobody asked
    // for. The verdict simply stays "pending" until the next successful solve.
    Promise.resolve(onSolveRequest({
      kind: "solve",
      armIndex,
      seed: [...active.lastPositions],
      target: baseTarget(armIndex, active.lastPositions, part),
      redundancy: {mode: "fixed", value: active.lastQ7},
    })).then(function (response) {
      if (disposed) {
        return;
      }
      if (response && response.solved === true && Array.isArray(response.positions)) {
        ghostState.setGhost(armIndex, response.positions);
        status(armIndex, active.fallback ? RING_FALLBACK_TEXT : null, null);
        refresh();
        return;
      }
      status(armIndex, (response && response.solve_reason) || null, "pending");
    }, function () {
      if (!disposed) {
        status(armIndex, null, "pending");
      }
    });
  }

  function onPointerDown(event) {
    if (!enabled || drag || event.button !== 0) {
      return;
    }
    pointerRay(event);
    const hits = raycaster.intersectObjects(pickTargets.filter(isVisible), false);
    const hit = hits[0];
    if (!hit) {
      return;
    }
    const armIndex = hit.object.userData.armIndex;
    const kind = hit.object.userData.pickKind;
    const started = kind === "ring" ? beginRingDrag(event, armIndex)
      : kind === "rotate"
        ? beginRotateDrag(event, armIndex, hit.object.userData.axisIndex)
        : beginHandDrag(event, armIndex);
    if (!started) {
      return;
    }
    ghostState.selectArm(armIndex);
    event.preventDefault();
    event.stopImmediatePropagation();
    orbitControls.cancel();
    orbitControls.setEnabled(false);
    canvas.classList.add("ghost-dragging");
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch (_error) {
      // Synthetic PointerEvents do not become active pointers in Chromium.
    }
  }

  function onPointerMove(event) {
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }
    event.preventDefault();
    if (drag.kind === "ring") {
      moveRing(event);
      return;
    }
    if (drag.kind === "rotate") {
      moveRotate(event);
      return;
    }
    const hit = planeHit(event, drag.plane);
    if (!hit) {
      return;
    }
    const wanted = hit.clone().add(drag.grabOffset);
    if (drag.vertical) {
      // Shift constrains the drag to the vertical LINE through the hand: the
      // plane merely carries the cursor, and only its height is taken.
      wanted.x = drag.origin.x;
      wanted.y = drag.origin.y;
    }
    const part = parts.get(drag.armIndex);
    requestSolve({
      armIndex: drag.armIndex,
      target: targetInArmBase(
        drag.armIndex, [wanted.x, wanted.y, wanted.z], part.target.rotation,
      ),
    });
  }

  function onPointerUp(event) {
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }
    const active = drag;
    drag = null;
    const part = parts.get(active.armIndex);
    if (part) {
      part.active = false;
    }
    try {
      if (canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
    } catch (_error) {
      // See the synthetic PointerEvent note in onPointerDown().
    }
    canvas.classList.remove("ghost-dragging");
    orbitControls.setEnabled(enabled);
    applyHandleColours();
    if (active.kind === "ring") {
      finishRingDrag(active);
    }
    if (active.kind === "rotate") {
      // Nothing to reconcile: every frame of a rotation WAS a solve, so the
      // ghost already stands on an answer. This only brings the two rings
      // that stepped aside back onto the screen.
      refresh();
    }
  }

  function isVisible(object) {
    let cursor = object;
    while (cursor) {
      if (!cursor.visible) {
        return false;
      }
      cursor = cursor.parent;
    }
    return true;
  }

  addTrackedListener(listeners, canvas, "pointerdown", onPointerDown);
  addTrackedListener(listeners, canvas, "pointermove", onPointerMove);
  addTrackedListener(listeners, canvas, "pointerup", onPointerUp);
  addTrackedListener(listeners, canvas, "pointercancel", onPointerUp);

  function setEnabled(nextEnabled) {
    if (typeof nextEnabled !== "boolean") {
      throw new TypeError("enabled must be a boolean");
    }
    enabled = nextEnabled;
    if (!enabled && drag) {
      const active = drag;
      drag = null;
      const part = parts.get(active.armIndex);
      if (part) {
        part.active = false;
      }
      canvas.classList.remove("ghost-dragging");
    }
    orbitControls.setEnabled(true);
    refresh();
  }

  function setPalette(next) {
    colours = Object.assign({}, next);
    applyHandleColours();
  }

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    drag = null;
    removeTrackedListeners(listeners);
    if (rafId !== null) {
      cancelAnimationFrame(rafId);
      rafId = null;
    }
    if (backoffTimer !== null) {
      clearTimeout(backoffTimer);
      backoffTimer = null;
    }
    for (const part of parts.values()) {
      part.knob.geometry.dispose();
      part.triad.geometry.dispose();
      part.pick.geometry.dispose();
      part.ring.geometry.dispose();
      part.ringPick.geometry.dispose();
      part.handleMaterial.dispose();
      part.triadMaterial.dispose();
      part.ringMaterial.dispose();
      // The two rotation geometries are shared across every ring on both
      // arms, so they are disposed once below, not here.
      part.rotate.forEach((entry) => entry.material.dispose());
      overlay.remove(part.group);
      overlay.remove(part.ringGroup);
    }
    parts.clear();
    pickTargets.length = 0;
    if (overlay.parent) {
      overlay.parent.remove(overlay);
    }
    rotateLineGeometry.dispose();
    rotatePickGeometry.dispose();
    pickMaterial.dispose();
  }

  refresh();

  return {
    setEnabled,
    refresh,
    setPalette,
    captureTarget,
    dispose,
    get listenerCount() {
      return listeners.length;
    },
    testing: {
      parts,
      pickTargets,
      targetInArmBase,
      acceptTable,
      lerpTable,
      ringBasis,
      psiOf,
      rotateAxes: ROTATE_AXES,
      rotateRadiusPx: ROTATE_RADIUS_PX,
      targetRotation(armIndex) {
        const part = parts.get(armIndex);
        return part && part.target ? [...part.target.rotation] : null;
      },
      fallbackText: RING_FALLBACK_TEXT,
      get solveCount() {
        return solveCount;
      },
      get dragging() {
        return drag ? drag.kind : null;
      },
      get backoffTimerArmed() {
        return backoffTimer !== null;
      },
      get inFlight() {
        return inFlight;
      },
    },
  };
}
