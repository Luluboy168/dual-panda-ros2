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

import {mountSolidScene} from "../../../static/ghost/scene.js";
import {createGhostState} from "../../../static/ghost/ghost_state.js";
import {createHandDrag} from "../../../static/ghost/hand_drag.js";
import {
  axisAngleMatrix,
  forwardKinematics,
  invertRigidMatrix,
  multiplyMatrices,
  quaternionFromMatrix,
  translationFromMatrix,
  translationMatrix,
} from "../../../static/ghost/kinematics.js";
import {ASSET_BASE, loadModel} from "./urdf.js";

const CELL = {
  id: "work_area", frame: "cell",
  x_min: -0.35, x_max: 0.9, y_min: -1.0, y_max: 1.0, z_min: 0.0, z_max: 2.0,
};
const HOME = [0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4];

function makeContainer(width, height) {
  const container = document.createElement("div");
  container.style.width = `${width}px`;
  container.style.height = `${height}px`;
  document.body.append(container);
  return container;
}

function screenOf(three, world, camera, canvas) {
  const bounds = canvas.getBoundingClientRect();
  const projected = new three.Vector3(world[0], world[1], world[2]).project(camera);
  return {
    x: bounds.left + ((projected.x + 1) / 2) * bounds.width,
    y: bounds.top + ((1 - projected.y) / 2) * bounds.height,
  };
}

function pointer(type, canvas, at, extra) {
  const event = new PointerEvent(type, Object.assign({
    bubbles: true, cancelable: true, pointerId: 7, isPrimary: true,
    button: 0, buttons: 1, clientX: at.x, clientY: at.y,
  }, extra || {}));
  canvas.dispatchEvent(event);
  return event;
}

/**
 * The world-space radius the mesh actually draws.
 *
 * Read from the geometry's own bounding sphere times the world scale, so it
 * is blind to HOW the size was arrived at: a fat geometry drawn at scale 1
 * and a unit geometry drawn at scale r are the same answer here. That is what
 * lets one case span both the defect and its fix.
 */
function worldRadiusOf(three, mesh) {
  mesh.geometry.computeBoundingSphere();
  mesh.updateWorldMatrix(true, false);
  const scale = new three.Vector3().setFromMatrixScale(mesh.matrixWorld);
  return mesh.geometry.boundingSphere.radius
    * Math.max(scale.x, scale.y, scale.z);
}

/** The angle between two unit quaternions, sign-insensitive. */
function quaternionAngle(a, b) {
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, dot));
}

function flangeOf(model, armIndex, positions) {
  const map = {};
  positions.forEach((value, index) => {
    map[`panda${armIndex}_joint${index + 1}`] = value;
  });
  const links = forwardKinematics(model, map).links;
  return {
    shoulder: translationFromMatrix(links[`panda${armIndex}_link1`]),
    elbow: translationFromMatrix(links[`panda${armIndex}_link4`]),
    wrist: translationFromMatrix(links[`panda${armIndex}_link6`]),
    flange: translationFromMatrix(links[`panda${armIndex}_link8`]),
  };
}

/**
 * A seed whose FLANGE sits directly above the shoulder.
 *
 * That makes the shoulder-to-flange axis the joint-1 axis exactly, so sweeping
 * joint 1 is a true self-motion: the elbow swings all the way round, the wrist
 * (`link6`, which sits 0.088 m off the joint-7 axis) travels a long way, and
 * the flange origin does not move at all. It is the real geometry the elbow
 * ring rests on, built without an IK solver.
 */
function alignedSeed(model, armIndex) {
  const base = [0, 0, 0, -1.5, 0, 1.0, 0];
  const offsetAt = (q2) => {
    const positions = [...base];
    positions[1] = q2;
    const {shoulder, flange} = flangeOf(model, armIndex, positions);
    return Math.hypot(flange[0] - shoulder[0], flange[1] - shoulder[1]);
  };
  let best = null;
  for (let step = 0; step <= 200; step += 1) {
    const q2 = -1.7628 + (3.5256 * step) / 200;
    const value = offsetAt(q2);
    if (best === null || value < best.value) {
      best = {q2, value};
    }
  }
  // Refine by golden-section around the coarse minimum.
  let low = best.q2 - 0.02;
  let high = best.q2 + 0.02;
  for (let step = 0; step < 60; step += 1) {
    const midLow = low + (high - low) / 3;
    const midHigh = high - (high - low) / 3;
    if (offsetAt(midLow) < offsetAt(midHigh)) {
      high = midHigh;
    } else {
      low = midLow;
    }
  }
  const positions = [...base];
  positions[1] = (low + high) / 2;
  return {positions, offset: offsetAt(positions[1])};
}

/** A redundancy table that sweeps that self-motion, exactly. */
function selfMotionTable(model, armIndex, seed, {rows = 25, span = 4.0} = {}) {
  const table = [];
  for (let index = 0; index < rows; index += 1) {
    const turn = -span / 2 + (span * index) / (rows - 1);
    const positions = [...seed];
    positions[0] = turn;
    positions[6] = turn;                 // q7 indexes the sweep, monotonically
    table.push({q7: turn, positions});
  }
  return {ok: true, arm_id: `panda${armIndex}`, samples: rows, table};
}

export async function runDragCases(context) {
  const {test, assert, assertEqual, assertNear, assertArrayNear, settle} = context;
  const three = globalThis.THREE;
  const {model, manifest} = await loadModel();

  const container = makeContainer(900, 640);
  const scene = await mountSolidScene(container, {
    model, manifest, assetBase: ASSET_BASE,
    assetFetch: (url) => window.fetch(url),
    cell: CELL,
  });

  const requests = [];
  const statuses = [];
  let responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});

  const ghostState = createGhostState({
    three, model,
    solidGraph: scene.solidGraph, ghostGraph: scene.ghostGraph,
    armIndices: [1, 2], initialArm: 1, render: scene.requestRender,
  });
  const handDrag = createHandDrag({
    three, canvas: scene.canvas, camera: scene.camera, model,
    ghostGraph: scene.ghostGraph, ghostState,
    orbitControls: scene.orbitControls, render: scene.requestRender,
    palette: scene.palette,
    onSolveRequest(request) {
      requests.push(request);
      return responder(request);
    },
    onStatus(update) {
      statuses.push(update);
    },
  });

  ghostState.setMeasured(1, HOME, {arrivalMs: performance.now(), framePeriodMs: 200, snap: true});
  ghostState.setMeasured(2, HOME, {arrivalMs: performance.now(), framePeriodMs: 200, snap: true});
  ghostState.setGhostVisible(1, true);
  ghostState.syncGhostToMeasured(1);
  handDrag.captureTarget(1);
  scene.setGhostRootVisible(1, true);
  handDrag.refresh();
  await settle();

  await test("the drag target is expressed in the arm's own base frame", () => {
    const ghost = ghostState.getGhost(1);
    const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
    const flange = links.panda1_link8;
    const rotation = [
      flange[0], flange[1], flange[2], 0,
      flange[4], flange[5], flange[6], 0,
      flange[8], flange[9], flange[10], 0,
      0, 0, 0, 1,
    ];
    const world = translationFromMatrix(flange);
    const local = handDrag.testing.targetInArmBase(1, world, rotation);
    // Composing the base back on must reproduce the pose the renderer drew.
    const rebuilt = multiplyMatrices(
      links.panda1_link0,
      multiplyMatrices(translationMatrix(local.position), rotation),
    );
    const expected = multiplyMatrices(
      links.panda1_link0,
      multiplyMatrices(
        invertRigidMatrix(links.panda1_link0),
        multiplyMatrices(translationMatrix(world), rotation),
      ),
    );
    assertArrayNear(translationFromMatrix(rebuilt), translationFromMatrix(expected), 1e-9,
      "base-frame target does not reproduce the rendered flange");
  });

  await test("differsFromMeasured turns over at exactly 1e-4 rad", () => {
    ghostState.syncGhostToMeasured(1);
    assertEqual(ghostState.differsFromMeasured(1), false, "a synced ghost must not differ");
    for (let joint = 0; joint < 7; joint += 1) {
      const below = ghostState.measuredNow(1);
      below[joint] += 1e-4 - 1e-6;
      ghostState.setGhost(1, below);
      assertEqual(ghostState.differsFromMeasured(1), false,
        `joint ${joint + 1} at 1e-4 - 1e-6 must not count as different`);
      const above = ghostState.measuredNow(1);
      above[joint] += 1e-4 + 1e-6;
      ghostState.setGhost(1, above);
      assertEqual(ghostState.differsFromMeasured(1), true,
        `joint ${joint + 1} at 1e-4 + 1e-6 must count as different`);
      ghostState.syncGhostToMeasured(1);
    }
  });

  await test("a hand drag emits one solve per frame, seeded from the GHOST", async () => {
    requests.length = 0;
    responder = (request) => Promise.resolve({
      ok: true, solved: true,
      positions: ghostState.getGhost(request.armIndex).map((v, i) => v + (i === 0 ? 0.01 : 0)),
      verdict: {status: "clear", offending_links: []},
      copy: {joints_deg: [], joints_rad: [], snippet: "x"},
    });
    const ghost = ghostState.getGhost(1);
    const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
    const flange = translationFromMatrix(links.panda1_link8);
    scene.frameCamera();
    handDrag.refresh();
    await settle();
    const at = screenOf(three, flange, scene.camera, scene.canvas);
    pointer("pointerdown", scene.canvas, at);
    assertEqual(handDrag.testing.dragging, "hand", "pointerdown on the handle started no drag");
    const seedBefore = ghostState.getGhost(1);
    pointer("pointermove", scene.canvas, {x: at.x + 40, y: at.y + 10});
    pointer("pointermove", scene.canvas, {x: at.x + 60, y: at.y + 14});
    // Two moves inside one frame must COALESCE into one request, and the second
    // must replace the first rather than queue behind it.
    await settle(3);
    assertEqual(requests.length, 1, `expected one coalesced solve, got ${requests.length}`);
    assertEqual(requests[0].kind, "solve", "the hand drag sent the wrong request kind");
    assertArrayNear(requests[0].seed, seedBefore, 1e-12,
      "the solve was seeded from something other than the current ghost pose");
    assert(Array.isArray(requests[0].target.position)
      && requests[0].target.position.length === 3, "no base-frame position was sent");
    assert(Array.isArray(requests[0].target.orientation)
      && requests[0].target.orientation.length === 4, "no target orientation was sent");
    assertEqual(requests[0].redundancy.mode, "from_seed", "a hand drag must seed its redundancy");
    pointer("pointerup", scene.canvas, {x: at.x + 60, y: at.y + 14});
    await settle(2);
  });

  await test("a move under half a millimetre sends nothing at all", async () => {
    requests.length = 0;
    const ghost = ghostState.getGhost(1);
    const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
    const flange = translationFromMatrix(links.panda1_link8);
    handDrag.refresh();
    await settle();
    const at = screenOf(three, flange, scene.camera, scene.canvas);
    pointer("pointerdown", scene.canvas, at);
    pointer("pointermove", scene.canvas, {x: at.x + 30, y: at.y});
    await settle(3);
    const afterFirst = requests.length;
    assert(afterFirst >= 1, "the first real move sent nothing");
    // The same place again: below both thresholds, so nothing new is worth a
    // round trip.
    pointer("pointermove", scene.canvas, {x: at.x + 30, y: at.y});
    await settle(3);
    assertEqual(requests.length, afterFirst, "a sub-millimetre move was still sent");
    pointer("pointerup", scene.canvas, {x: at.x + 30, y: at.y});
    await settle(2);
  });

  await test("a refusal leaves the ghost exactly where it was", async () => {
    requests.length = 0;
    const before = ghostState.getGhost(1);
    responder = () => Promise.resolve({
      ok: true, solved: false, positions: null, verdict: null,
      solve_reason: "That point is outside this arm's reach.",
    });
    const links = forwardKinematics(model, ghostState.jointMap(1, before)).links;
    const at = screenOf(three, translationFromMatrix(links.panda1_link8),
      scene.camera, scene.canvas);
    pointer("pointerdown", scene.canvas, at);
    pointer("pointermove", scene.canvas, {x: at.x + 90, y: at.y + 30});
    await settle(4);
    assertArrayNear(ghostState.getGhost(1), before, 1e-12,
      "the ghost moved to a pose the solver did not return");
    assert(statuses.some((entry) => entry.text
      && entry.text.indexOf("outside this arm's reach") >= 0),
      "the refusal sentence never reached the panel");
    pointer("pointerup", scene.canvas, {x: at.x + 90, y: at.y + 30});
    await settle(2);
  });

  await test("a throttled drag backs off, drops the middle, and resumes with the newest",
    async () => {
      requests.length = 0;
      let refusals = 0;
      // Long enough that a slow software-GL frame cannot outrun the window:
      // with a short one, three animation frames on a loaded machine can take
      // longer than the backoff and the test measures its own slowness.
      const backoffMs = 2000;
      responder = (request) => {
        if (request.kind === "solve" && refusals < 1) {
          refusals += 1;
          return Promise.reject({retryAfterMs: backoffMs});
        }
        return Promise.resolve({
          ok: true, solved: true, positions: ghostState.getGhost(request.armIndex),
          verdict: {status: "clear", offending_links: []},
          copy: {joints_deg: [], joints_rad: [], snippet: "x"},
        });
      };
      const ghost = ghostState.getGhost(1);
      const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
      const at = screenOf(three, translationFromMatrix(links.panda1_link8),
        scene.camera, scene.canvas);
      try {
        pointer("pointerdown", scene.canvas, at);
        pointer("pointermove", scene.canvas, {x: at.x + 50, y: at.y + 5});
        await settle(3);
        assertEqual(requests.length, 1, "the first solve did not go out");
        // The pointer now STOPS -- the ordinary end of a drag, and the case the
        // un-timed backoff stranded forever.
        await new Promise((resolve) => setTimeout(resolve, 300));
        assertEqual(requests.length, 1, "a request went out during the backoff window");
        assert(handDrag.testing.backoffTimerArmed, "no backoff timer was armed");
        await new Promise((resolve) => setTimeout(resolve, backoffMs));
        await settle(3);
        assertEqual(requests.length, 2,
          `the final target was never resent (got ${requests.length} requests)`);
      } finally {
        // Always lift the pointer: a drag left active would refuse every
        // pointerdown after it, turning one failure into a cascade of them.
        pointer("pointerup", scene.canvas, {x: at.x + 50, y: at.y + 5});
        await settle(2);
      }
    });

  /* ------------------------------ the rotation gizmo (orientation) ------- */

  await test("the hand carries three world-axis rotation rings, both arms", () => {
    const specs = handDrag.testing.worldAxes;
    assertEqual(JSON.stringify(specs.map((spec) => spec.key)),
      JSON.stringify(["axisX", "axisY", "axisZ"]),
      "the gizmo does not offer exactly one ring per world axis");
    // u x v = axis on every ring. That is what makes the drag follow the
    // cursor: the angle is measured from u towards v, and the hand is turned
    // about the axis by that same angle in the same sense.
    specs.forEach((spec) => {
      const cross = new three.Vector3(...spec.u).cross(new three.Vector3(...spec.v));
      assertArrayNear([cross.x, cross.y, cross.z], spec.axis, 1e-12,
        `${spec.key} measures its angle the wrong way round its own axis`);
    });
    const proxies = handDrag.testing.pickTargets
      .filter((object) => object.userData.pickKind === "rotate");
    assertEqual(proxies.length, 6,
      "each of the two arms must contribute three rotation pick proxies");
    assertEqual(handDrag.testing.parts.get(2).rotate.length, 3,
      "the second arm's hand has no rotation rings of its own");
  });

  /**
   * The radius the mesh subtends ON SCREEN, in CSS pixels.
   *
   * Run through the camera's own projection -- the centre, and a point one
   * world radius away along the camera's right vector, both projected -- so
   * it measures what the projection will do rather than what a pixel read
   * happens to catch after antialiasing has had its way with a small dot.
   */
  function projectedRadiusPx(mesh) {
    const radius = worldRadiusOf(three, mesh);
    scene.camera.updateMatrixWorld();
    const centre = new three.Vector3().setFromMatrixPosition(mesh.matrixWorld);
    const right = new three.Vector3()
      .setFromMatrixColumn(scene.camera.matrixWorld, 0).normalize();
    const edge = centre.clone().addScaledVector(right, radius);
    const at = screenOf(three, [centre.x, centre.y, centre.z],
      scene.camera, scene.canvas);
    const rim = screenOf(three, [edge.x, edge.y, edge.z],
      scene.camera, scene.canvas);
    return Math.hypot(rim.x - at.x, rim.y - at.y);
  }

  /** A screen point inside the canvas that no handle can possibly occupy. */
  function offGizmo() {
    const bounds = scene.canvas.getBoundingClientRect();
    return {x: bounds.left + 3, y: bounds.top + 3};
  }

  /** Put the ghost at HOME and return the screen points for one ring drag. */
  async function grabRotateRing(axisIndex, turn) {
    handDrag.setEnabled(false);
    handDrag.setEnabled(true);
    ghostState.setGhost(1, HOME);
    handDrag.captureTarget(1);
    scene.frameCamera();
    handDrag.refresh();
    await settle(2);
    const part = handDrag.testing.parts.get(1);
    const entry = part.rotate[axisIndex];
    const centre = part.group.position.clone();
    const radius = entry.group.scale.x;
    const pointAt = (angle) => centre.clone()
      .addScaledVector(entry.u, radius * Math.cos(angle))
      .addScaledVector(entry.v, radius * Math.sin(angle));
    const screen = (point) => screenOf(
      three, [point.x, point.y, point.z], scene.camera, scene.canvas);
    // BETWEEN the axes, never on one. Any two rings cross exactly where an
    // axis pierces them, so a grab at angle 0 is a grab on two rings at once
    // and the nearer one wins -- which is not the ring this case named.
    const start = Math.PI / 4;
    return {entry, centre,
            at: screen(pointAt(start)), to: screen(pointAt(start + turn))};
  }

  await test("an axis ring turns the hand about that axis and does not move it",
    async () => {
      requests.length = 0;
      responder = (request) => Promise.resolve({
        ok: true, solved: true,
        // A distinguishable answer, so "the ghost is what the solver returned"
        // is a thing this case can actually see.
        positions: ghostState.getGhost(request.armIndex)
          .map((value, index) => value + (index === 6 ? 0.07 : 0)),
        verdict: {status: "clear", offending_links: []},
        copy: {joints_deg: [], joints_rad: [], snippet: "x"},
      });
      const turn = 0.5;
      const grip = await grabRotateRing(2, turn);          // the world-Z ring
      const startRotation = handDrag.testing.targetRotation(1);
      const flange = translationFromMatrix(
        forwardKinematics(model, ghostState.jointMap(1, ghostState.getGhost(1)))
          .links.panda1_link8);
      const before = handDrag.testing.targetInArmBase(1, flange, startRotation);

      pointer("pointerdown", scene.canvas, grip.at);
      assertEqual(handDrag.testing.dragging, "rotate",
        "pointerdown on the world-Z ring did not start a rotation");
      pointer("pointermove", scene.canvas, grip.to);
      await settle(3);

      const solve = requests[requests.length - 1];
      assertEqual(solve.kind, "solve", "the rotation sent the wrong request kind");
      assertEqual(solve.redundancy.mode, "from_seed",
        "a rotation must seed its redundancy like any other hand gesture");
      // The hand TURNS; it does not travel. Position identical, to the metre.
      assertArrayNear(solve.target.position, before.position, 1e-9,
        "the rotation moved the hand instead of turning it about itself");
      const expected = handDrag.testing.targetInArmBase(1, flange, multiplyMatrices(
        axisAngleMatrix([0, 0, 1], turn), startRotation,
      ));
      assertNear(quaternionAngle(solve.target.orientation, expected.orientation),
        0, 1e-6, "the quaternion sent is not the start orientation turned about world Z");
      assertNear(quaternionAngle(solve.target.orientation, before.orientation),
        turn, 1e-6, "the hand did not turn by the angle the cursor travelled");

      // The answer is adopted, and it is what the renderer is drawing.
      const ghost = ghostState.getGhost(1);
      assertNear(ghost[6], HOME[6] + 0.07, 1e-9,
        "the ghost is not standing on the pose the solver returned");
      assertNear(
        scene.ghostGraph.jointNodes.get("panda1_joint7").userData.value,
        ghost[6], 1e-9, "the renderer is drawing a different pose from the one adopted");

      pointer("pointerup", scene.canvas, grip.to);
      await settle(3);
      // ...and the turn is now the hand's own orientation, so the NEXT hand
      // drag carries it. This is the whole of what promoting orientation
      // authoring means: the frozen capture is no longer frozen for ever.
      assertNear(
        quaternionAngle(quaternionFromMatrix(handDrag.testing.targetRotation(1)),
          quaternionFromMatrix(startRotation)),
        turn, 1e-6, "an accepted turn did not become the hand's authored orientation");
    });

  await test("a hand drag after a turn carries the authored orientation", async () => {
    requests.length = 0;
    // Release anything an earlier failure left holding the canvas.
    handDrag.setEnabled(false);
    handDrag.setEnabled(true);
    const authored = handDrag.testing.targetRotation(1);
    responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});
    const ghost = ghostState.getGhost(1);
    const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
    scene.frameCamera();
    handDrag.refresh();
    await settle();
    const at = screenOf(three, translationFromMatrix(links.panda1_link8),
      scene.camera, scene.canvas);
    pointer("pointerdown", scene.canvas, at);
    assertEqual(handDrag.testing.dragging, "hand", "the handle grab was lost");
    pointer("pointermove", scene.canvas, {x: at.x + 45, y: at.y + 12});
    await settle(3);
    const solve = requests[requests.length - 1];
    const expected = handDrag.testing.targetInArmBase(
      1, translationFromMatrix(links.panda1_link8), authored,
    );
    assertNear(quaternionAngle(solve.target.orientation, expected.orientation), 0, 1e-6,
      "the hand drag threw the authored orientation away and sent the measured one");
    pointer("pointerup", scene.canvas, {x: at.x + 45, y: at.y + 12});
    await settle(2);
  });

  await test("a refused turn keeps the hand where it was and says why", async () => {
    requests.length = 0;
    statuses.length = 0;
    responder = () => Promise.resolve({
      ok: true, solved: false, positions: null, verdict: null,
      solve_reason: "Reaching that point would push a joint past its limit.",
    });
    const grip = await grabRotateRing(2, 0.4);
    const beforeGhost = ghostState.getGhost(1);
    const beforeRotation = handDrag.testing.targetRotation(1);
    pointer("pointerdown", scene.canvas, grip.at);
    pointer("pointermove", scene.canvas, grip.to);
    await settle(4);
    assertArrayNear(ghostState.getGhost(1), beforeGhost, 1e-12,
      "a refused turn moved the ghost to a pose the solver did not return");
    assertArrayNear(handDrag.testing.targetRotation(1), beforeRotation, 1e-12,
      "a refused turn was still adopted as the hand's authored orientation");
    assertEqual(handDrag.testing.parts.get(1).refused, true,
      "a refused turn did not tint the handle");
    assert(statuses.some((entry) => entry.text
      && entry.text.indexOf("past its limit") >= 0),
      "the endpoint's own refusal sentence never reached the panel");
    pointer("pointerup", scene.canvas, grip.to);
    await settle(3);
  });

  await test("the ring being dragged is the only one on screen", async () => {
    responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});
    const grip = await grabRotateRing(2, 0.3);
    const part = handDrag.testing.parts.get(1);
    assert(part.rotate.every((entry) => entry.group.visible),
      "all three rings must be offered before a gesture starts");
    pointer("pointerdown", scene.canvas, grip.at);
    await settle(2);
    assertEqual(part.rotate.map((entry) => entry.group.visible).join(","),
      "false,false,true",
      "three concentric circles round a turning hand is a picture nobody can read");
    pointer("pointerup", scene.canvas, grip.at);
    await settle(2);
    assert(part.rotate.every((entry) => entry.group.visible),
      "the other two rings never came back after the gesture");
    ghostState.setGhost(1, HOME);
    handDrag.captureTarget(1);
  });

  await test("the wrist knob keeps a minimum SCREEN size at table-wide zoom",
    async () => {
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      // The shipped default framing, scripted rather than inherited so the
      // number in this case is the number under test: frameCamera() puts the
      // eye about 4.5 m from the cell centre, which is the "see the whole
      // table" view the console opens in -- and the view a live check found
      // the handle missing from, as a dot a few pixels across.
      scene.orbitControls.frame([0.275, 0, 0.5], 4.5);
      handDrag.refresh();
      await settle(2);
      const part = handDrag.testing.parts.get(1);
      const floor = handDrag.testing.handleMinPx;
      const drawn = projectedRadiusPx(part.knob);
      assert(drawn >= floor - 1e-6,
        `the wrist knob draws ${drawn.toFixed(2)} px of radius at the default `
        + `table-wide view, under a floor of ${floor} px: at that size the `
        + "operator cannot find the handle, and the feature does not exist");
      // What the eye can find, the finger must be able to hit.
      assert(projectedRadiusPx(part.pick) >= drawn - 1e-6,
        "the pick proxy is smaller than the knob that is drawn");
      const farProportion = worldRadiusOf(three, part.triad)
        / worldRadiusOf(three, part.knob);
      // Close in it is a world object again, not a sticker pasted on screen.
      const flange = part.group.position.clone();
      scene.orbitControls.frame([flange.x, flange.y, flange.z], 0.45);
      handDrag.refresh();
      await settle(2);
      assertNear(worldRadiusOf(three, part.knob), 0.016, 1e-9,
        "close in, the knob must be its authored world size of 16 mm");
      // The whole handle grows together. A knob that outgrew its own triad
      // would swallow the three axes that say which way the hand faces.
      assertNear(farProportion,
        worldRadiusOf(three, part.triad) / worldRadiusOf(three, part.knob), 1e-6,
        "the triad does not keep its proportion to the knob across zoom");
      scene.frameCamera();
      handDrag.refresh();
      await settle(2);
    });

  await test("a press another gizmo lies in front of still goes to the hand",
    async () => {
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      // The console's own panel, to the pixel: 445x273 is what a 512x597
      // browser window leaves the scene, and 4.5 m is where frameCamera()
      // puts the eye. Neither number is decoration. The gizmos are sized in
      // SCREEN pixels while the arm is sized in metres, so how much of a
      // 26-pixel handle another gizmo's band can cover depends entirely on
      // how small the arm is drawn. At the harness's own 900x640 nothing
      // crowds the handle at all, and this case would prove nothing while
      // passing.
      container.style.width = "445px";
      container.style.height = "273px";
      scene.resize();
      try {
        scene.orbitControls.frame([0.275, 0, 0.5], 4.5);
        handDrag.refresh();
        await settle(2);
        const part = handDrag.testing.parts.get(1);
        const centre = part.group.position;
        const bounds = scene.canvas.getBoundingClientRect();
        const knob = screenOf(three, [centre.x, centre.y, centre.z],
          scene.camera, scene.canvas);
        const caster = new three.Raycaster();
        const drawn = (object) => {
          for (let node = object; node; node = node.parent) {
            if (!node.visible) {
              return false;
            }
          }
          return true;
        };
        const hitsAt = (at) => {
          caster.setFromCamera(new three.Vector2(
            ((at.x - bounds.left) / bounds.width) * 2 - 1,
            -(((at.y - bounds.top) / bounds.height) * 2 - 1)), scene.camera);
          return caster.intersectObjects(
            handDrag.testing.pickTargets.filter(drawn), false);
        };
        // State the defect as a search: a press the hand's own pick disc covers,
        // which some OTHER proxy also covers, nearer the camera. Sorting hits
        // nearest-first handed exactly that press to the other one -- the
        // gesture began, the canvas took the dragging class, and the ghost did
        // not move. If no such press can be produced the rule is untested, and
        // this case must say so rather than pass by finding nothing.
        let crowded = null;
        let nearer = null;
        let turns = 0;
        const search = (from) => {
          for (let radius = 1; radius <= 14 && crowded === null; radius += 1) {
            for (let step = 0; step < 36 && crowded === null; step += 1) {
              const angle = (step * Math.PI) / 18;
              const at = {
                x: from.x + radius * Math.cos(angle),
                y: from.y + radius * Math.sin(angle),
              };
              const hits = hitsAt(at);
              if (hits.length > 0
                && hits[0].object.userData.pickKind !== "hand"
                && hits.some((one) => one.object.userData.pickKind === "hand")) {
                crowded = at;
                nearer = hits[0].object.userData.pickKind;
              }
            }
          }
        };
        // None of the other gizmos is centred on the hand -- the elbow ring
        // circles the ELBOW, the rotation rings stand off the knob -- so
        // whether any of their bands crosses the handle is a question about the
        // viewing angle. Turn the view, a corner drag at a time, until one
        // does: the same thing the operator does before running into this.
        const corner = {x: bounds.left + 6, y: bounds.top + bounds.height - 6};
        search(knob);
        while (crowded === null && turns < 24) {
          pointer("pointerdown", scene.canvas, corner);
          pointer("pointermove", scene.canvas, {x: corner.x + 60, y: corner.y});
          pointer("pointerup", scene.canvas, {x: corner.x + 60, y: corner.y});
          turns += 1;
          handDrag.refresh();
          await settle(2);
          search(screenOf(three, [centre.x, centre.y, centre.z],
            scene.camera, scene.canvas));
        }
        assert(crowded !== null,
          `after ${turns} turns of the view, no press within 14 px of the handle `
          + "has another pick proxy in front of the hand at the console's own "
          + "panel size, so this case cannot test the rule it is named for");
        pointer("pointerdown", scene.canvas, crowded);
        await settle(2);
        assertEqual(handDrag.testing.dragging, "hand",
          `a press the hand's own target covers was answered by the ${nearer} `
          + "proxy, because that proxy happened to lie nearer the camera; that "
          + "is the gesture the operator reported as a ghost that cannot be "
          + "dragged");
        pointer("pointerup", scene.canvas, crowded);
        await settle(2);
        assertEqual(handDrag.testing.dragging, null,
          "the gesture outlived the pointer that started it");
      } finally {
        container.style.width = "900px";
        container.style.height = "640px";
        scene.resize();
        scene.frameCamera();
        handDrag.refresh();
        await settle(2);
      }
    });

  /* ================= the translate arrows: one axis at a time ============= */

  //: The arm's base frame is a fixed joint, so composing a base-frame target
  //: back into the world is one constant matrix. The arrows' whole claim is
  //: about WORLD coordinates, and this is what lets these cases read them
  //: straight off the wire instead of trusting the module's own helper.
  const armBase = forwardKinematics(model, {}).links.panda1_link0;
  const worldTargetOf = (request) => translationFromMatrix(
    multiplyMatrices(armBase, translationMatrix(request.target.position)));

  /**
   * Put the eye at a world offset from a point and look back at it.
   *
   * The orbit controller offers a target and a radius but no angle, and these
   * cases need NAMED angles -- including one that looks almost straight down
   * a world axis. Writing the camera is safe here because nothing drags the
   * canvas while a camera is placed this way, so the controller's own azimuth
   * and polar are never contradicted behind its back.
   */
  function eyeAt(at, offset) {
    scene.camera.position.set(at[0] + offset[0], at[1] + offset[1], at[2] + offset[2]);
    scene.camera.up.set(0, 0, 1);
    scene.camera.lookAt(new three.Vector3(at[0], at[1], at[2]));
    scene.camera.updateMatrixWorld(true);
  }

  function ghostFlange(armIndex) {
    return translationFromMatrix(forwardKinematics(
      model, ghostState.jointMap(armIndex, ghostState.getGhost(armIndex)),
    ).links[`panda${armIndex}_link8`]);
  }

  /** Every pick proxy a screen point touches, nearest first, and the ray. */
  function proxyHitsAt(at) {
    const bounds = scene.canvas.getBoundingClientRect();
    const caster = new three.Raycaster();
    caster.setFromCamera(new three.Vector2(
      ((at.x - bounds.left) / bounds.width) * 2 - 1,
      -(((at.y - bounds.top) / bounds.height) * 2 - 1)), scene.camera);
    const drawn = (object) => {
      for (let node = object; node; node = node.parent) {
        if (!node.visible) {
          return false;
        }
      }
      return true;
    };
    return {
      direction: caster.ray.direction.clone(),
      hits: caster.intersectObjects(
        handDrag.testing.pickTargets.filter(drawn), false),
    };
  }

  /**
   * Where one arrow can be grabbed, and which way the cursor must travel.
   *
   * Both are read off the SCENE: a point lying ON the axis at a fraction of
   * the arrow's own length, projected, and the projection of the axis itself.
   * No pixel here is guessed, so a case that lands on a neighbouring handle
   * is a case that fails rather than one that quietly tests something else.
   */
  function arrowGrip(armIndex, axisIndex, fraction) {
    const part = handDrag.testing.parts.get(armIndex);
    const entry = part.arrows[axisIndex];
    const origin = part.group.position.clone();
    const perPixel = entry.group.scale.x;
    const pointAt = (pixels) => {
      const world = origin.clone().addScaledVector(entry.axis, perPixel * pixels);
      return screenOf(three, [world.x, world.y, world.z], scene.camera, scene.canvas);
    };
    const knob = pointAt(0);
    const at = pointAt(handDrag.testing.arrowLengthPx * fraction);
    const tip = pointAt(handDrag.testing.arrowLengthPx);
    const span = Math.hypot(tip.x - knob.x, tip.y - knob.y);
    return {
      entry, origin, at, perPixel,
      along: span < 1e-6 ? {x: 1, y: 0}
        : {x: (tip.x - knob.x) / span, y: (tip.y - knob.y) / span},
      offKnobPx: Math.hypot(at.x - knob.x, at.y - knob.y),
    };
  }

  const solvedInPlace = (request) => Promise.resolve({
    ok: true, solved: true, positions: ghostState.getGhost(request.armIndex),
    verdict: {status: "clear", offending_links: []},
    copy: {joints_deg: [], joints_rad: [], snippet: "x"},
  });

  await test("the hand carries three world-axis translate arrows, both arms", () => {
    const part = handDrag.testing.parts.get(1);
    assertEqual(part.arrows.length, 3, "the hand offers no arrow per world axis");
    assertEqual(part.arrows.map((entry) => entry.key).join(","), "axisX,axisY,axisZ",
      "the arrows are not the same three world axes the rings are");
    part.arrows.forEach((entry, axisIndex) => {
      assertArrayNear([entry.axis.x, entry.axis.y, entry.axis.z],
        handDrag.testing.worldAxes[axisIndex].axis, 1e-12,
        `arrow ${entry.key} does not point along its own world axis`);
    });
    // The arrows carry the AXIS palette, not a severity token: an axis colour
    // says which axis this is, and a refusal is the knob's business.
    part.arrows.forEach((entry) => {
      assertEqual(entry.material.color.getHexString(),
        new three.Color(scene.palette[entry.key]).getHexString(),
        `arrow ${entry.key} is not drawn in its own axis colour`);
    });
    assertEqual(handDrag.testing.parts.get(2).arrows.length, 3,
      "the second arm's hand has no arrows of its own");
    assertEqual(handDrag.testing.pickTargets
      .filter((object) => object.userData.pickKind === "translate").length, 6,
      "each of the two arms must contribute three arrow pick proxies");
    // The arrows run out PAST the rings, so their heads are never buried in
    // one -- and so the shaft crosses two rings, which is the overlap the
    // pick-order case below is built on.
    assert(handDrag.testing.arrowLengthPx > handDrag.testing.rotateRadiusPx,
      "the arrows stop short of the rotation rings");
    assertEqual(handDrag.testing.pickOrder.join(">"),
      "hand>translate>rotate>ring",
      "the pick order is not hand, then arrow, then rotation ring, then elbow");
  });

  await test("each arrow moves the hand along its OWN world axis and no other",
    async () => {
      // Three framings of the same gesture. The last looks nearly along world
      // X, so the X arrow is foreshortened to a stub -- the arrow equivalent
      // of a ring seen edge-on, and the one place a projection onto an axis
      // can be expected to misbehave.
      const views = [
        {name: "three-quarters from above", offset: [1.35, 1.05, 0.95]},
        {name: "low from the far side", offset: [-0.55, 1.85, 0.30]},
        {name: "nearly along world X", offset: [1.879, 0.684, 0.0]},
      ];
      for (const view of views) {
        for (let axisIndex = 0; axisIndex < 3; axisIndex += 1) {
          responder = solvedInPlace;
          handDrag.setEnabled(false);
          handDrag.setEnabled(true);
          ghostState.setGhost(1, HOME);
          handDrag.captureTarget(1);
          eyeAt(ghostFlange(1), view.offset);
          handDrag.refresh();
          await settle(2);
          const key = handDrag.testing.worldAxes[axisIndex].key;
          const where = `${view.name}, ${key}`;
          const grip = arrowGrip(1, axisIndex, 0.8);
          assert(grip.offKnobPx > handDrag.testing.handPickPx * 0.5,
            `${where}: the press is ${grip.offKnobPx.toFixed(1)} px from the knob `
            + "centre, inside the hand's own footprint, so this case would be "
            + "measuring a plane drag and calling it an arrow");
          requests.length = 0;
          pointer("pointerdown", scene.canvas, grip.at);
          assertEqual(handDrag.testing.dragging, "translate",
            `${where}: a press on the arrow started no axis drag`);
          assertEqual(handDrag.testing.draggingAxis, axisIndex,
            `${where}: the press started a drag of a different axis`);
          const step = handDrag.testing.draggingStepM;
          assert(Number.isFinite(step) && step > 0,
            `${where}: the drag carries no per-event bound`);

          // 200 px in ten events, ACROSS the projected axis as well as along
          // it. A cursor that tracks the projection exactly is a cursor no
          // hand ever produces, and it would hide the very failure this case
          // exists for: the world X and Y axes both lie IN the drag plane, so
          // a plane drag pushed exactly along one of their projections stays
          // on that axis by accident and looks like an axis handle.
          const travel = 200;
          const skew = 0.45;
          const scale = travel / Math.hypot(1, skew) / 10;
          const path = (move) => ({
            x: grip.at.x + (grip.along.x - skew * grip.along.y) * scale * move,
            y: grip.at.y + (grip.along.y + skew * grip.along.x) * scale * move,
          });
          const seen = [];
          for (let move = 1; move <= 10; move += 1) {
            pointer("pointermove", scene.canvas, path(move));
            await settle(2);
            if (requests.length > 0) {
              seen.push(worldTargetOf(requests[requests.length - 1]));
            }
          }
          pointer("pointerup", scene.canvas, path(10));
          await settle(2);

          const start = [grip.origin.x, grip.origin.y, grip.origin.z];
          assert(seen.length > 0, `${where}: the drag asked for nothing at all`);
          seen.forEach((target, index) => {
            target.forEach((value, coordinate) => {
              assert(Number.isFinite(value),
                `${where}: target ${index} carries ${value}`);
              if (coordinate !== axisIndex) {
                assertNear(value, start[coordinate], 1e-6,
                  `${where}: dragging one arrow moved world coordinate `
                  + `${"xyz"[coordinate]} as well, which is the whole of what an `
                  + "axis handle promises not to do");
              }
            });
            // Well-behaved as well as true: no single event may carry the hand
            // further than the clamp allows, however steeply the axis is seen.
            const previous = index === 0 ? start[axisIndex] : seen[index - 1][axisIndex];
            assert(Math.abs(target[axisIndex] - previous) <= step + 1e-9,
              `${where}: one pointer event moved the hand `
              + `${Math.abs(target[axisIndex] - previous).toFixed(3)} m, past the `
              + `${step.toFixed(3)} m this drag is allowed to spend on one event`);
          });
          assert(Math.abs(seen[seen.length - 1][axisIndex] - start[axisIndex]) > 0.005,
            `${where}: a 200 px drag moved the hand less than five millimetres, `
            + "so the case proved only that nothing happens");
        }
      }
    });

  await test("the arrow's clamp is live, and its edge-on grab is refused",
    async () => {
      responder = solvedInPlace;
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      // Nearly along world X: the X arrow is a stub, so one 200 px event asks
      // for metres. The clamp is what answers, and it must answer EXACTLY.
      eyeAt(ghostFlange(1), [1.879, 0.684, 0.0]);
      handDrag.refresh();
      await settle(2);
      const grip = arrowGrip(1, 0, 0.8);
      requests.length = 0;
      pointer("pointerdown", scene.canvas, grip.at);
      assertEqual(handDrag.testing.dragging, "translate",
        "the steep-but-allowed grab was refused, so the clamp is untested");
      const step = handDrag.testing.draggingStepM;
      pointer("pointermove", scene.canvas, {
        x: grip.at.x + grip.along.x * 200, y: grip.at.y + grip.along.y * 200,
      });
      await settle(3);
      const target = worldTargetOf(requests[requests.length - 1]);
      assertNear(Math.abs(target[0] - grip.origin.x), step, 1e-9,
        "one event asked for metres and was not cut to the per-event bound; the "
        + "clamp is dead code");
      pointer("pointerup", scene.canvas, {
        x: grip.at.x + grip.along.x * 200, y: grip.at.y + grip.along.y * 200,
      });
      await settle(2);

      // ...and closer to end-on than that, the grab is declined outright, the
      // way a ring seen edge-on is. Swept rather than asserted at one angle:
      // the window where the press is outside the knob AND the axis is inside
      // the refusal is a real one, and if it cannot be produced this case says
      // so instead of passing.
      let refusedAt = null;
      for (let tenths = 60; tenths <= 240 && refusedAt === null; tenths += 5) {
        const angle = ((tenths / 10) * Math.PI) / 180;
        eyeAt(ghostFlange(1), [2 * Math.cos(angle), 2 * Math.sin(angle), 0]);
        handDrag.refresh();
        await settle(2);
        const steep = arrowGrip(1, 0, 0.95);
        if (steep.offKnobPx <= handDrag.testing.handPickPx * 0.5) {
          continue;                    // the press is the hand's, not an arrow's
        }
        const probe = proxyHitsAt(steep.at);
        // The X arrow, nearest, with nothing of the hand's under it: then the
        // pick order sends this press to that arrow whatever else it touches.
        if (probe.hits.length === 0
            || probe.hits[0].object.userData.pickKind !== "translate"
            || probe.hits[0].object.userData.axisIndex !== 0
            || probe.hits.some((one) => one.object.userData.pickKind === "hand")) {
          continue;
        }
        const axis = handDrag.testing.parts.get(1).arrows[0].axis;
        const along = Math.abs(probe.direction.dot(axis));
        if (Math.sqrt(1 - along * along) >= handDrag.testing.arrowEdgeOnMin) {
          continue;                    // the axis is still comfortably aimable
        }
        pointer("pointerdown", scene.canvas, steep.at);
        assertEqual(handDrag.testing.dragging, null,
          `at ${(tenths / 10).toFixed(1)} degrees off world X the arrow is inside `
          + "its own edge-on refusal, and a grab there must be declined rather "
          + "than amplify a one-pixel twitch into metres");
        pointer("pointerup", scene.canvas, steep.at);
        await settle(2);
        refusedAt = tenths / 10;
      }
      assert(refusedAt !== null,
        "no camera angle between 6 and 24 degrees off world X put an arrow press "
        + "outside the knob AND inside the edge-on refusal, so the refusal is "
        + "untested and this case must not pass");
      scene.frameCamera();
      handDrag.refresh();
      await settle(2);
    });

  await test("the hand keeps its own footprint, and an arrow beats a ring",
    async () => {
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});
      // The console's own panel, to the pixel, for the reason the handle case
      // gives: the gizmos are sized in SCREEN pixels and the arm in metres, so
      // how much of a 26 px handle another proxy can cover depends entirely on
      // how small the arm is drawn. At the harness's own 900x640 nothing
      // crowds anything and both halves of this case would pass vacuously.
      container.style.width = "445px";
      container.style.height = "273px";
      scene.resize();
      try {
        scene.orbitControls.frame([0.275, 0, 0.5], 4.5);
        handDrag.refresh();
        await settle(2);
        const part = handDrag.testing.parts.get(1);
        const knobAt = () => {
          const centre = part.group.position;
          return screenOf(three, [centre.x, centre.y, centre.z],
            scene.camera, scene.canvas);
        };

        // (a) A press the hand's own footprint covers, which an ARROW also
        // covers, nearer the camera. Nearest-first would hand it to the arrow.
        let inside = null;
        const findInside = () => {
          const knob = knobAt();
          for (let radius = 1; radius <= 12 && inside === null; radius += 1) {
            for (let step = 0; step < 36 && inside === null; step += 1) {
              const angle = (step * Math.PI) / 18;
              const at = {x: knob.x + radius * Math.cos(angle),
                          y: knob.y + radius * Math.sin(angle)};
              const kinds = proxyHitsAt(at).hits
                .map((one) => one.object.userData.pickKind);
              if (kinds[0] === "translate" && kinds.indexOf("hand") > 0) {
                inside = at;
              }
            }
          }
        };

        // (b) A press OUTSIDE that footprint which an arrow and a rotation
        // ring both cover, with the RING nearer the camera. The two meet by
        // construction: every axis pierces the two rings that contain it, at
        // the rings' own 54 px radius.
        let crossing = null;
        let crossingAxis = null;
        const findCrossing = () => {
          const knob = knobAt();
          for (let axisIndex = 0; axisIndex < 3 && crossing === null; axisIndex += 1) {
            const entry = part.arrows[axisIndex];
            const world = part.group.position.clone().addScaledVector(
              entry.axis, entry.group.scale.x * handDrag.testing.rotateRadiusPx);
            const centre = screenOf(three, [world.x, world.y, world.z],
              scene.camera, scene.canvas);
            for (let dx = -9; dx <= 9 && crossing === null; dx += 1) {
              for (let dy = -9; dy <= 9 && crossing === null; dy += 1) {
                const at = {x: centre.x + dx, y: centre.y + dy};
                if (Math.hypot(at.x - knob.x, at.y - knob.y)
                    <= handDrag.testing.handPickPx * 0.5) {
                  continue;
                }
                const probe = proxyHitsAt(at);
                const kinds = probe.hits.map((one) => one.object.userData.pickKind);
                if (kinds[0] !== "rotate" || kinds.indexOf("hand") >= 0) {
                  continue;
                }
                const arrow = probe.hits.find(
                  (one) => one.object.userData.pickKind === "translate");
                if (!arrow) {
                  continue;
                }
                const axis = part.arrows[arrow.object.userData.axisIndex].axis;
                const along = Math.abs(probe.direction.dot(axis));
                if (Math.sqrt(1 - along * along) < handDrag.testing.arrowEdgeOnMin) {
                  continue;      // that arrow is edge-on; its grab is refused
                }
                crossing = at;
                crossingAxis = arrow.object.userData.axisIndex;
              }
            }
          }
        };

        // Which proxy lies in front of which is a question about the viewing
        // angle, so turn the view a corner drag at a time until both presses
        // exist -- the same thing the operator does before running into this.
        const bounds = scene.canvas.getBoundingClientRect();
        const corner = {x: bounds.left + 6, y: bounds.top + bounds.height - 6};
        let turns = 0;
        findInside();
        findCrossing();
        while ((inside === null || crossing === null) && turns < 24) {
          pointer("pointerdown", scene.canvas, corner);
          pointer("pointermove", scene.canvas, {x: corner.x + 60, y: corner.y});
          pointer("pointerup", scene.canvas, {x: corner.x + 60, y: corner.y});
          turns += 1;
          handDrag.refresh();
          await settle(2);
          findInside();
          findCrossing();
        }
        assert(inside !== null,
          `after ${turns} turns of the view, no press inside the hand's footprint `
          + "at the console's own panel size has an arrow in front of it, so half "
          + "of this case cannot test the rule it is named for");
        assert(crossing !== null,
          `after ${turns} turns of the view, no press outside the hand's footprint `
          + "has a rotation ring in front of an arrow, so the other half cannot "
          + "test its rule either");

        pointer("pointerdown", scene.canvas, inside);
        assertEqual(handDrag.testing.dragging, "hand",
          "a press inside the hand's own 26 px footprint was answered by the "
          + "arrow lying in front of it; the knob must keep its footprint "
          + "against every handle that crosses it, arrows included");
        pointer("pointerup", scene.canvas, inside);
        await settle(2);

        pointer("pointerdown", scene.canvas, crossing);
        assertEqual(handDrag.testing.dragging, "translate",
          "a press on an arrow, outside the hand's footprint, was answered by "
          + "the rotation ring the arrow passes through; an arrow beats a ring "
          + "wherever the two overlap");
        assertEqual(handDrag.testing.draggingAxis, crossingAxis,
          "the press started an axis drag of the wrong axis");
        pointer("pointerup", scene.canvas, crossing);
        await settle(2);
        assertEqual(handDrag.testing.dragging, null,
          "the gesture outlived the pointer that started it");
      } finally {
        container.style.width = "900px";
        container.style.height = "640px";
        scene.resize();
        scene.frameCamera();
        handDrag.refresh();
        await settle(2);
      }
    });

  await test("Shift still lifts the hand straight up, with the arrows on screen",
    async () => {
      responder = solvedInPlace;
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      scene.frameCamera();
      handDrag.refresh();
      await settle(2);
      const part = handDrag.testing.parts.get(1);
      assert(part.arrows.every((entry) => entry.group.visible),
        "the arrows must be on screen for this to prove they changed nothing");
      const start = ghostFlange(1);
      const at = screenOf(three, start, scene.camera, scene.canvas);
      requests.length = 0;
      pointer("pointerdown", scene.canvas, at, {shiftKey: true});
      assertEqual(handDrag.testing.dragging, "hand",
        "a press on the knob went to an arrow; the knob keeps its footprint");
      pointer("pointermove", scene.canvas, {x: at.x + 30, y: at.y - 70},
        {shiftKey: true});
      await settle(3);
      assert(requests.length > 0, "the Shift drag asked for nothing");
      const target = worldTargetOf(requests[requests.length - 1]);
      assertNear(target[0], start[0], 1e-9,
        "Shift no longer holds the hand's x while it moves it up and down");
      assertNear(target[1], start[1], 1e-9,
        "Shift no longer holds the hand's y while it moves it up and down");
      assert(Math.abs(target[2] - start[2]) > 0.005,
        "Shift moved the hand less than five millimetres in z, so the vertical "
        + "drag has stopped being a vertical drag");
      pointer("pointerup", scene.canvas, {x: at.x + 30, y: at.y - 70});
      await settle(2);
      // ...and without Shift the same grab is the world-horizontal plane it
      // has always been: z held, x and y free.
      requests.length = 0;
      const flat = screenOf(three, ghostFlange(1), scene.camera, scene.canvas);
      const flatStart = ghostFlange(1);
      pointer("pointerdown", scene.canvas, flat);
      assertEqual(handDrag.testing.dragging, "hand", "the plain grab was lost");
      pointer("pointermove", scene.canvas, {x: flat.x + 60, y: flat.y + 20});
      await settle(3);
      const flatTarget = worldTargetOf(requests[requests.length - 1]);
      assertNear(flatTarget[2], flatStart[2], 1e-9,
        "the plane drag no longer keeps the hand at its own height");
      assert(Math.hypot(flatTarget[0] - flatStart[0], flatTarget[1] - flatStart[1])
        > 0.005, "the plane drag moved the hand nowhere");
      pointer("pointerup", scene.canvas, {x: flat.x + 60, y: flat.y + 20});
      await settle(2);
    });

  await test("every gesture takes the arrows off screen, and gives them back",
    async () => {
      responder = solvedInPlace;
      handDrag.setEnabled(false);
      handDrag.setEnabled(true);
      ghostState.setGhost(1, HOME);
      handDrag.captureTarget(1);
      scene.frameCamera();
      handDrag.refresh();
      await settle(2);
      const part = handDrag.testing.parts.get(1);
      assert(part.arrows.every((entry) => entry.group.visible),
        "a shown idle ghost must offer all three arrows; a handle that appears "
        + "only once you have grabbed it cannot be discovered");
      assert(part.arrows.every((entry) => entry.material.opacity > 0
        && entry.material.opacity < 1),
        "the idle arrows are not drawn faintly, the way the rings are");
      const at = screenOf(three, ghostFlange(1), scene.camera, scene.canvas);
      pointer("pointerdown", scene.canvas, at);
      await settle(2);
      assertEqual(handDrag.testing.dragging, "hand", "the knob grab was lost");
      assertEqual(part.arrows.map((entry) => entry.group.visible).join(","),
        "false,false,false",
        "the arrows stayed drawn under a gesture they are not part of, where "
        + "the captured pointer cannot reach them");
      pointer("pointerup", scene.canvas, at);
      await settle(2);
      assert(part.arrows.every((entry) => entry.group.visible),
        "the arrows never came back after the gesture that hid them");
      // ...and its own arrow stays while an arrow is being dragged.
      const grip = arrowGrip(1, 2, 0.8);
      pointer("pointerdown", scene.canvas, grip.at);
      assertEqual(handDrag.testing.dragging, "translate", "the arrow grab was lost");
      await settle(2);
      assertEqual(part.arrows.map((entry) => entry.group.visible).join(","),
        "false,false,true", "an axis drag must leave its own arrow on screen");
      assertEqual(part.arrows[2].material.opacity, 1,
        "the arrow being dragged is still drawn faintly");
      pointer("pointerup", scene.canvas, grip.at);
      await settle(2);
      assert(part.arrows.every((entry) => entry.group.visible),
        "the other two arrows never came back after the axis drag");
    });

  await test("a shown idle ghost offers its rings faintly, and lights the one "
    + "under the cursor", async () => {
    handDrag.setEnabled(false);
    handDrag.setEnabled(true);
    ghostState.setGhost(1, HOME);
    handDrag.captureTarget(1);
    scene.frameCamera();
    handDrag.refresh();
    await settle(2);
    const part = handDrag.testing.parts.get(1);
    // One accepted drag first, to clear the refusal an earlier case left
    // standing: a red knob is answering the collision tint, and this case is
    // about what answers the cursor. The solver hands the seed straight back,
    // so the ghost does not move while the tint clears.
    responder = (request) => Promise.resolve({
      ok: true, solved: true, positions: ghostState.getGhost(request.armIndex),
      verdict: {status: "clear", offending_links: []},
      copy: {joints_deg: [], joints_rad: [], snippet: "x"},
    });
    const centre = part.group.position;
    const onKnob = screenOf(three, [centre.x, centre.y, centre.z],
      scene.camera, scene.canvas);
    pointer("pointerdown", scene.canvas, onKnob);
    pointer("pointermove", scene.canvas, {x: onKnob.x + 40, y: onKnob.y + 12});
    await settle(4);
    pointer("pointerup", scene.canvas, {x: onKnob.x + 40, y: onKnob.y + 12});
    await settle(2);
    assertEqual(part.refused, false, "the refusal tint never cleared");
    responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});
    assertEqual(handDrag.testing.dragging, null,
      "this case must run with nothing held");
    part.rotate.forEach((entry) => {
      assert(entry.group.visible,
        `ring ${entry.key} is off screen while the ghost is shown and idle; `
        + "a gizmo that appears only once you have already grabbed it cannot "
        + "be discovered");
      assert(entry.material.opacity > 0,
        `ring ${entry.key} draws at zero opacity, which is not an affordance`);
      assert(entry.material.opacity < 1,
        `ring ${entry.key} is at full strength while idle; idle is a hint, `
        + "and full strength is reserved for the ring in play");
    });
    // Hover lights the ring under the cursor, and only that one.
    const grip = await grabRotateRing(2, 0);
    pointer("pointermove", scene.canvas, grip.at);
    await settle(2);
    assertEqual(handDrag.testing.dragging, null, "a hover must not start a drag");
    assertEqual(part.rotate.map((entry) => entry.material.opacity === 1).join(","),
      "false,false,true",
      "hovering the world-Z ring did not raise that ring, and only that ring");
    // ...and the knob answers a cursor of its own.
    const idle = part.handleMaterial.color.getHex();
    pointer("pointermove", scene.canvas, onKnob);
    await settle(2);
    assertEqual(handDrag.testing.hovering.kind, "hand",
      "the cursor on the knob was not read as the knob");
    assert(part.handleMaterial.color.getHex() !== idle,
      "the knob does not answer the cursor resting on it");
    // Off the gizmo entirely, everything returns to its idle strength.
    pointer("pointermove", scene.canvas, offGizmo());
    await settle(2);
    assertEqual(part.handleMaterial.color.getHex(), idle,
      "the knob stayed highlighted after the cursor left it");
    assert(part.rotate.every((entry) => entry.material.opacity < 1),
      "a ring stayed lit after the cursor left it");
  });

  const aligned = alignedSeed(model, 1);
  const alignedBasis = () => {
    const geometry = flangeOf(model, 1, aligned.positions);
    return handDrag.testing.ringBasis(
      geometry.shoulder, geometry.flange, geometry.elbow,
    );
  };

  await test("the probe pose really is a self-motion about the hand", () => {
    assert(aligned.offset < 1e-4,
      `the probe flange is ${aligned.offset.toFixed(6)} m off the shoulder axis; `
      + "the rest of the elbow cases rest on it being on the axis");
    const rows = selfMotionTable(model, 1, aligned.positions).table
      .map((entry) => flangeOf(model, 1, entry.positions));
    let wristSpread = 0;
    let flangeSpread = 0;
    let elbowSpread = 0;
    rows.forEach((row) => {
      const spread = (key) => Math.hypot(
        row[key][0] - rows[0][key][0], row[key][1] - rows[0][key][1],
        row[key][2] - rows[0][key][2],
      );
      wristSpread = Math.max(wristSpread, spread("wrist"));
      flangeSpread = Math.max(flangeSpread, spread("flange"));
      elbowSpread = Math.max(elbowSpread, spread("elbow"));
    });
    // This is the whole geometric argument, measured: the hand stays put while
    // the elbow AND the wrist travel a long way. A hand-stays-put test aimed at
    // link6 would reject this table by a factor of thirty-five, on every real
    // arm, always -- and with it the entire elbow-ring feature.
    assert(flangeSpread < 0.002,
      `the probe table must pin the flange; it moved ${flangeSpread.toFixed(6)} m`);
    assert(wristSpread > 0.1,
      `the probe table must sweep link6 a long way; it moved ${wristSpread.toFixed(4)} m`);
    assert(elbowSpread > 0.1,
      `the probe table must swing the elbow; it moved ${elbowSpread.toFixed(4)} m`);
  });

  await test("a table that pins the hand and turns psi is accepted", () => {
    const basis = alignedBasis();
    assert(basis, "no ring basis for the probe pose");
    const accepted = handDrag.testing.acceptTable(
      1, basis, selfMotionTable(model, 1, aligned.positions),
    );
    assert(accepted, "a physically real self-motion table was rejected");
    assertEqual(accepted.rows.length, 25, "the accepted table lost rows");
    assert(accepted.spread < 0.002, "the accepted table reported a moving flange");
  });

  await test("the elbow table is refused when any one acceptance test fails", () => {
    const basis = alignedBasis();

    // (a) too few solved entries.
    assertEqual(
      handDrag.testing.acceptTable(
        1, basis, selfMotionTable(model, 1, aligned.positions, {rows: 4}),
      ),
      null, "a four-row table was accepted",
    );

    // (b) a non-monotone psi column: the arm passes through a posture change,
    // and pretending otherwise would make the ghost jump.
    const bent = selfMotionTable(model, 1, aligned.positions);
    bent.table.forEach((entry, index) => {
      if (index > 12) {
        entry.positions = [...bent.table[24 - index].positions];
        entry.positions[6] = entry.q7;
      }
    });
    assertEqual(handDrag.testing.acceptTable(1, basis, bent), null,
      "a non-monotone table was accepted");

    // (c) a table whose FLANGE genuinely wanders: the ring would be lying
    // about the one thing it promises.
    const wandering = selfMotionTable(model, 1, aligned.positions);
    wandering.table.forEach((entry, index) => {
      entry.positions = [...entry.positions];
      entry.positions[3] += index * 0.002;         // bends the elbow open
    });
    assertEqual(handDrag.testing.acceptTable(1, basis, wandering), null,
      "a table whose flange moved more than 2 mm was accepted");
  });

  await test("the ring lerps between neighbours and clamps at the ends", () => {
    const basis = alignedBasis();
    const table = handDrag.testing.acceptTable(
      1, basis, selfMotionTable(model, 1, aligned.positions),
    );
    assert(table, "the probe table was rejected");
    const rows = table.rows;
    const low = handDrag.testing.lerpTable(table, rows[0].psi - 5);
    assertArrayNear(low.positions, rows[0].positions, 1e-9, "the low end did not clamp");
    const high = handDrag.testing.lerpTable(table, rows[rows.length - 1].psi + 5);
    assertArrayNear(high.positions, rows[rows.length - 1].positions, 1e-9,
      "the high end did not clamp");
    const mid = handDrag.testing.lerpTable(table, (rows[3].psi + rows[4].psi) / 2);
    rows[3].positions.forEach((value, index) => {
      const expected = (value + rows[4].positions[index]) / 2;
      assertNear(mid.positions[index], expected, 1e-9, `midpoint joint ${index + 1}`);
    });
  });

  // Every ring interaction below runs from the probe pose, where the elbow is
  // a long way off the shoulder-to-flange axis and the ring is therefore a
  // real, grabbable circle rather than a dot on top of the hand.
  async function grabRing() {
    // Release any gesture an earlier failure left holding the canvas: a live
    // drag refuses every pointerdown after it, so one failure would otherwise
    // read as a failure in every case that follows.
    handDrag.setEnabled(false);
    handDrag.setEnabled(true);
    ghostState.setGhost(1, aligned.positions);
    handDrag.captureTarget(1);
    scene.frameCamera();
    handDrag.refresh();
    await settle();
    const basis = alignedBasis();
    const onRing = basis.centre.clone().addScaledVector(basis.u, basis.radius);
    const at = screenOf(three, [onRing.x, onRing.y, onRing.z], scene.camera, scene.canvas);
    return {basis, at};
  }

  await test("an elbow drag reads pending, sends nothing, then reconciles once",
    async () => {
      requests.length = 0;
      statuses.length = 0;
      responder = (request) => {
        if (request.kind === "redundancy") {
          return Promise.resolve(selfMotionTable(model, request.armIndex, aligned.positions));
        }
        return Promise.resolve({
          ok: true, solved: true, positions: ghostState.getGhost(request.armIndex),
          verdict: {status: "clear", offending_links: []},
          copy: {joints_deg: [], joints_rad: [], snippet: "x"},
        });
      };
      const {at} = await grabRing();
      pointer("pointerdown", scene.canvas, at);
      assertEqual(handDrag.testing.dragging, "ring", "pointerdown on the ring started no drag");
      assert(statuses.some((entry) => entry.verdict === "pending"),
        "the ring drag never declared the verdict pending");
      await settle(4);
      assertEqual(requests.filter((r) => r.kind === "redundancy").length, 1,
        "the ring did not build its table with exactly one request");
      const duringStart = requests.length;
      for (let step = 1; step <= 6; step += 1) {
        pointer("pointermove", scene.canvas, {x: at.x + step * 6, y: at.y + step * 4});
        await settle(2);
      }
      assertEqual(requests.length, duringStart,
        `the ring drag sent ${requests.length - duringStart} requests mid-gesture; it must send none`);
      pointer("pointerup", scene.canvas, {x: at.x + 36, y: at.y + 24});
      await settle(4);
      const reconciling = requests.filter(
        (r) => r.kind === "solve" && r.redundancy && r.redundancy.mode === "fixed",
      );
      assertEqual(reconciling.length, 1,
        "pointer-up must issue exactly one reconciling solve at a fixed redundancy");
      assert(Number.isFinite(reconciling[0].redundancy.value),
        "the reconciling solve carried no redundancy value");
    });

  await test("a failed reconciliation leaves the ghost where the lerp put it", async () => {
    requests.length = 0;
    statuses.length = 0;
    responder = (request) => {
      if (request.kind === "redundancy") {
        return Promise.resolve(selfMotionTable(model, request.armIndex, aligned.positions));
      }
      return Promise.resolve({
        ok: true, solved: false, positions: null, verdict: null,
        solve_reason: "The solver could not settle on that point.",
      });
    };
    const {at} = await grabRing();
    pointer("pointerdown", scene.canvas, at);
    await settle(4);
    for (let step = 1; step <= 5; step += 1) {
      pointer("pointermove", scene.canvas, {x: at.x + step * 8, y: at.y + step * 5});
      await settle(2);
    }
    const lerped = ghostState.getGhost(1);
    pointer("pointerup", scene.canvas, {x: at.x + 40, y: at.y + 25});
    await settle(4);
    assertArrayNear(ghostState.getGhost(1), lerped, 1e-12,
      "a failed reconciliation snapped the ghost back and discarded the whole gesture");
    assert(statuses[statuses.length - 1].verdict === "pending",
      "the verdict left pending state after a failed reconciliation");
  });

  await test("a refused redundancy call retries once, then falls back to option (A)",
    async () => {
      requests.length = 0;
      statuses.length = 0;
      let redundancyCalls = 0;
      responder = (request) => {
        if (request.kind === "redundancy") {
          redundancyCalls += 1;
          return Promise.reject({retryAfterMs: 20});
        }
        return Promise.resolve({ok: true, solved: false, solve_reason: null});
      };
      const {at} = await grabRing();
      pointer("pointerdown", scene.canvas, at);
      await new Promise((resolve) => setTimeout(resolve, 120));
      await settle(3);
      assertEqual(redundancyCalls, 2,
        `the ring must retry a refused table exactly once; it made ${redundancyCalls} calls`);
      assert(statuses.some((entry) => entry.text === handDrag.testing.fallbackText),
        "the option (A) fallback sentence never reached the panel");
      pointer("pointerup", scene.canvas, {x: at.x + 10, y: at.y + 5});
      await settle(3);
    });

  await test("two fingers pinch and pan; one finger on the handle drags", async () => {
    // The pinch half is about the orbit controls alone, so the grab handle is
    // taken out of the picture first: otherwise a finger that happens to land
    // on the hand starts a drag and there is nothing left to measure.
    handDrag.setEnabled(false);
    const beforeRadius = scene.orbitControls.radius;
    const canvas = scene.canvas;
    const bounds = canvas.getBoundingClientRect();
    const mid = {x: bounds.left + bounds.width / 2, y: bounds.top + bounds.height / 2};
    canvas.dispatchEvent(new PointerEvent("pointerdown", {
      bubbles: true, cancelable: true, pointerId: 21, button: 0, buttons: 1,
      clientX: mid.x - 60, clientY: mid.y,
    }));
    canvas.dispatchEvent(new PointerEvent("pointerdown", {
      bubbles: true, cancelable: true, pointerId: 22, button: 0, buttons: 1,
      clientX: mid.x + 60, clientY: mid.y,
    }));
    canvas.dispatchEvent(new PointerEvent("pointermove", {
      bubbles: true, cancelable: true, pointerId: 21, clientX: mid.x - 120, clientY: mid.y,
    }));
    canvas.dispatchEvent(new PointerEvent("pointermove", {
      bubbles: true, cancelable: true, pointerId: 22, clientX: mid.x + 120, clientY: mid.y,
    }));
    assert(scene.orbitControls.radius < beforeRadius,
      "spreading two fingers did not zoom in");
    const beforeTarget = scene.orbitControls.target.clone();
    canvas.dispatchEvent(new PointerEvent("pointermove", {
      bubbles: true, cancelable: true, pointerId: 21, clientX: mid.x - 60, clientY: mid.y + 80,
    }));
    canvas.dispatchEvent(new PointerEvent("pointermove", {
      bubbles: true, cancelable: true, pointerId: 22, clientX: mid.x + 180, clientY: mid.y + 80,
    }));
    assert(scene.orbitControls.target.distanceTo(beforeTarget) > 1e-6,
      "moving both fingers together did not pan");
    canvas.dispatchEvent(new PointerEvent("pointerup", {
      bubbles: true, cancelable: true, pointerId: 21, clientX: mid.x - 60, clientY: mid.y + 80,
    }));
    canvas.dispatchEvent(new PointerEvent("pointerup", {
      bubbles: true, cancelable: true, pointerId: 22, clientX: mid.x + 180, clientY: mid.y + 80,
    }));
    assertEqual(scene.orbitControls.activePointerCount, 0, "a pointer was left captured");

    handDrag.setEnabled(true);
    scene.frameCamera();
    handDrag.refresh();
    await settle();
    const ghost = ghostState.getGhost(1);
    const links = forwardKinematics(model, ghostState.jointMap(1, ghost)).links;
    const at = screenOf(three, translationFromMatrix(links.panda1_link8),
      scene.camera, scene.canvas);
    responder = () => Promise.resolve({ok: true, solved: false, solve_reason: null});
    const event = pointer("pointerdown", scene.canvas, at);
    assertEqual(handDrag.testing.dragging, "hand",
      "a single pointer on the handle's pick proxy orbited instead of dragging");
    assert(event.defaultPrevented, "the handle grab did not claim the gesture");
    pointer("pointerup", scene.canvas, at);
    await settle(2);
  });

  await test("the ghost tints only the offending link, and neutrally when unchecked",
    async () => {
      const materials = scene.testing.ghostMaterials;
      scene.setGhostTint(1, {status: "clear", offending_links: []});
      await settle();
      const clearColours = new Map(
        [...materials.entries()].map(([key, list]) => [key, list[0].color.getHex()]),
      );
      scene.setGhostTint(1, {status: "collision", offending_links: ["panda1_link5"]});
      await settle();
      let changed = [];
      for (const [key, list] of materials.entries()) {
        if (list[0].color.getHex() !== clearColours.get(key)) {
          changed.push(list[0].userData.linkName);
        }
      }
      assertEqual(JSON.stringify(changed), JSON.stringify(["panda1_link5"]),
        `collision tinted ${JSON.stringify(changed)} instead of only the offending link`);

      scene.setGhostTint(1, {status: "unchecked", offending_links: []});
      await settle();
      const uncheckedArm1 = [...materials.entries()]
        .filter(([key]) => key.startsWith("1:"))
        .map(([, list]) => list[0].color.getHex());
      assertEqual(new Set(uncheckedArm1).size, 1,
        "an unchecked ghost must lose its per-link distinctions and read as one neutral");
      const collideHex = new three.Color(scene.palette.ghostCollide).getHex();
      const clearHex = clearColours.get([...materials.keys()].find((k) => k.startsWith("1:")));
      assert(uncheckedArm1[0] !== collideHex,
        "the unchecked treatment must not be the collision colour");
      assert(uncheckedArm1[0] !== clearHex,
        "the unchecked treatment must not be the clear colour");
      scene.setGhostTint(1, null);
      await settle();
    });

  await test("the clipboard falls back to execCommand outside a secure context", () => {
    // The daily journey is a laptop on the lab network reaching the console by
    // IP, which is not a secure context: navigator.clipboard is undefined
    // exactly where the console is actually used, so this path is the primary.
    const original = navigator.clipboard;
    let copied = null;
    const originalExec = document.execCommand;
    try {
      Object.defineProperty(navigator, "clipboard", {value: undefined, configurable: true});
      document.execCommand = function (command) {
        if (command === "copy") {
          copied = document.activeElement && document.activeElement.value;
        }
        return true;
      };
      const area = document.createElement("textarea");
      area.value = "ghost-pose-snippet";
      document.body.append(area);
      area.select();
      document.execCommand("copy");
      area.remove();
      assertEqual(copied, "ghost-pose-snippet", "the execCommand path copied nothing");
    } finally {
      document.execCommand = originalExec;
      Object.defineProperty(navigator, "clipboard", {value: original, configurable: true});
    }
  });

  handDrag.dispose();
  ghostState.dispose();
  scene.dispose();
  container.remove();

  await test("disposing the drag controller clears its listeners and its timer", () => {
    assertEqual(handDrag.listenerCount, 0, "the drag controller left listeners behind");
    assertEqual(handDrag.testing.backoffTimerArmed, false, "a backoff timer outlived dispose");
  });

  await runPanelCases(context);
}

/* ============================ the console's own panel ====================== */

function frame(overrides) {
  const arm = (armId, extra) => Object.assign({
    arm_id: armId,
    status: "ok",
    status_line: "ready",
    joint_names: Array.from({length: 7}, (_v, i) => `${armId}_joint${i + 1}`),
    positions: [...HOME],
    velocities: new Array(7).fill(0),
    efforts: new Array(7).fill(0),
    positions_age_s: 0.02,
    positions_stale: false,
    robot_state: {},
    diagnostic: {},
    motion: {
      available: true, enabled: false, target: null,
      fence_lower: null, fence_upper: null, max_target_velocity: null,
      pose_inside_fence: true, targets_published: 0, last_publish_age_s: null,
      enable_service_available: true, activation_delta_rad: null,
      activation_max_abs_delta_rad: null, activation_abs_velocity_rad_s: null,
      activation_max_abs_velocity_rad_s: null, activation_position_span_rad: null,
      activation_lower_margin_rad: null, activation_upper_margin_rad: null,
      source: "jog", external_rate_hz: null,
      command_topic: `/dual_arm_joint_impedance_controller/arm_1/joint_target`,
      command_template: "header:\n  stamp: now\n", command_template_ready: true,
    },
    gripper: {
      configured: false, available: false, width_mm: null, requested_width_mm: null,
      object: false, moving: false, activated: false, fault_code: null,
      fault_name: null, fault_class: null, status_line: "not configured",
      level: "unknown", speed_mm_s: null, force_n: null, port: null, busy: false,
    },
  }, extra || {});

  const armIds = (overrides && overrides.arm_ids) || ["panda1", "panda2"];
  const arms = {};
  armIds.forEach((armId) => {
    arms[armId] = arm(armId, (overrides && overrides.armExtras
      && overrides.armExtras[armId]) || {});
  });
  return Object.assign({
    schema_version: 5,
    server_time: new Date().toISOString(),
    server_uptime_s: 120 + Math.random(),
    session: {
      state: "running", session_id: "s-1", arms: "both", arm_ids: armIds,
      arm_mode: "dual", mode: "watch", started_at: new Date().toISOString(),
      uptime_s: 30, launch_running: true, last_error: null,
      recording_sealed: false, advisory: "", activation: {}, steps: [],
    },
    operator: {locked: false, claim_id: null, since: null, expires_in_s: null},
    preflight: {},
    recording: {active: false, name: null, sequence: 0, path: null,
                arm_mode: null, topics: [], disabled: true},
    controllers: {},
    hardware: {},
    fault: {active: false, since: null, reasons: [], recoverable: false,
            recover_hint: null, cause: null, arm_id: null, headline: null,
            steps: [], action: null},
    arms,
    hint: "Watching both arms.",
    logs: {warn_count: 0, error_count: 0, last_seq: 0},
  }, (overrides && overrides.frame) || {});
}

const SCENE_OK = {
  ok: true,
  assets: {
    manifest_url: "/ghost/assets/manifest.json",
    urdf_url: "/ghost/assets/model.urdf",
    asset_base: "/ghost/assets/",
    urdf_sha256: "0".repeat(64),
    total_bytes: 9155771,
  },
  cell: {id: "work_area", frame: "cell", x_min: -0.35, x_max: 0.9,
         y_min: -1.0, y_max: 1.0, z_min: 0.0, z_max: 2.0},
  cell_source: "cell_model",
  cell_note: null,
  model: {model_id: "hcis_dual_panda_cell", model_revision: 3, model_sha256: "0".repeat(64)},
  arms: ["panda1", "panda2"],
  ik: {available: true, arm_ids: ["panda1", "panda2"], tip_frame: "flange"},
  checker: {available: true, profile: "dual", interlock: "ok", note: null},
  ghost_available: true,
};

async function runPanelCases(context) {
  const {test, assert, assertEqual, settle} = context;

  // -- the console's own markup, unmodified -------------------------------
  const markup = await (await fetch("/index.html")).text();
  const parsed = new DOMParser().parseFromString(markup, "text/html");
  const fixture = document.createElement("div");
  fixture.id = "console-fixture";
  document.body.append(fixture);
  Array.from(parsed.body.children).forEach((node) => {
    if (node.tagName !== "SCRIPT") {
      fixture.append(document.importNode(node, true));
    }
  });
  // The console's stylesheet is not loaded here (these cases are about the
  // script, and a <link> in <head> is not part of the body markup), so the
  // scene's viewport has no height rule and a canvas at height:100% of an
  // auto-height parent re-measures itself every time the renderer resizes --
  // it grows without bound, taking the camera's aspect with it. Give the
  // viewport the size the stylesheet gives it and the fixture behaves like
  // the page.
  const view = fixture.querySelector("#sceneView");
  view.style.width = "760px";
  view.style.height = "520px";

  // -- the network, entirely under this file's control --------------------
  const routes = {scene: JSON.parse(JSON.stringify(SCENE_OK))};
  const posted = [];
  let currentFrame = frame();
  let solveResponse = null;
  const realFetch = window.fetch.bind(window);
  window.fetch = function (input, init) {
    const url = String(typeof input === "string" ? input : input.url);
    const path = url.replace(/^https?:\/\/[^/]+/, "").split("?")[0];
    const body = init && init.body ? JSON.parse(init.body) : null;
    if (path.indexOf("/api/") !== 0) {
      return realFetch(input, init);
    }
    let payload = {ok: true};
    if (path === "/api/scene") {
      payload = routes.scene;
    } else if (path === "/api/state") {
      payload = {ok: true, state: currentFrame};
    } else if (path === "/api/capabilities") {
      payload = {ok: true, server_version: "test", state_frame_hz: 5.0,
                 log_ring_lines: 500, joint_count: 7, heartbeat_ms: 5000};
    } else if (path === "/api/config") {
      payload = {ok: true, profile: null, source: "defaults"};
    } else if (path === "/api/logs") {
      payload = {ok: true, lines: [], dropped: 0};
    } else if (path === "/api/ghost/solve" || path === "/api/ghost/redundancy") {
      posted.push({path, body});
      // A function responder can answer per arm and per request, which is the
      // only way a two-ghost case can prove that each affordance carries its
      // OWN arm's payload rather than the first one's.
      payload = (typeof solveResponse === "function"
        ? solveResponse(body) : solveResponse)
        || {ok: true, solved: false, solve_reason: null};
    }
    // A responder may return a PROMISE, which holds that answer on the wire
    // for as long as the case wants. Ordering is the subject of some of the
    // cases below -- an answer that was already in flight when the operator
    // changed the cell is a different thing from one asked for afterwards --
    // and ordering cannot be written down with a stub that always answers at
    // once.
    return Promise.resolve(payload).then((answer) => new Response(
      JSON.stringify(answer), {
        status: answer.ok === false ? 503 : 200,
        headers: {"Content-Type": "application/json"},
      }));
  };

  const streams = [];
  window.EventSource = function () {
    const handlers = {};
    this.addEventListener = (type, handler) => {
      handlers[type] = handler;
    };
    this.close = () => {};
    streams.push({
      emit: (type, data) => {
        if (handlers[type]) {
          handlers[type]({data: JSON.stringify(data)});
        }
      },
      open: () => {
        if (this.onopen) {
          this.onopen();
        }
      },
      source: this,
    });
  };

  let scrollCount = 0;
  const realScrollBy = window.scrollBy.bind(window);
  window.scrollBy = function (...args) {
    scrollCount += 1;
    return realScrollBy(...args);
  };

  await new Promise((resolve, reject) => {
    const tag = document.createElement("script");
    tag.src = "/app.js";
    tag.onload = resolve;
    tag.onerror = () => reject(new Error("the console script did not load"));
    document.head.append(tag);
  });
  await settle();
  const stream = streams[0];
  assert(stream, "the console opened no state stream");
  const emit = (next) => {
    currentFrame = next;
    stream.emit("state", next);
  };
  // /api/scene is point-in-time and deliberately has NO polling loop, so a
  // changed scene description reaches the page exactly as it does in life:
  // on the stream reconnecting.
  const reconnect = async () => {
    stream.open();
    await settle(4);
  };

  // Every ghost affordance is addressed BY ARM, never by a fixed id: that is
  // the whole of the defect these cases exist for. `role` is the part
  // (reset / copy / toast / verdict / degrees / armnote / snippet).
  const ghostControl = (role, armId) => document.querySelector(
    `[data-role="${role}"][data-arm="${armId}"]`);
  const shownCopyButtons = () => Array.from(
    document.querySelectorAll('[data-act="ghost-copy"]')).filter((node) => !node.hidden);

  const bar = document.getElementById("sceneBar");
  const body = document.getElementById("sceneBody");
  const note = () => document.getElementById("sceneNote").textContent;
  const noteHidden = () => document.getElementById("sceneNote").hidden;

  // The scene mounts asynchronously — three.js, then the model, then the mesh
  // set — and on a loaded machine that outruns a fixed number of frames. Wait
  // for the thing being tested rather than for a guessed number of frames.
  async function waitFor(predicate, what, frames = 240) {
    for (let step = 0; step < frames; step += 1) {
      if (predicate()) {
        return true;
      }
      await settle(1);
    }
    throw new Error(`timed out waiting for ${what}`);
  }

  // Sweep the canvas until a pointerdown lands on the ghost's grab handle,
  // then drag. Returns the point it grabbed at.
  async function grabHandle(canvas, by) {
    const bounds = canvas.getBoundingClientRect();
    const middle = {x: bounds.left + bounds.width / 2, y: bounds.top + bounds.height / 2};
    for (let pass = 0; pass < 3; pass += 1) {
      for (let step = 0; step < 63; step += 1) {
        const at = {
          x: middle.x + ((step % 9) - 4) * 24,
          y: middle.y + (Math.floor(step / 9) - 3) * 24,
        };
        const down = pointer("pointerdown", canvas, at);
        if (!down.defaultPrevented) {
          pointer("pointerup", canvas, at);
          continue;
        }
        pointer("pointermove", canvas, {x: at.x + by.x, y: at.y + by.y});
        await settle(4);
        pointer("pointerup", canvas, {x: at.x + by.x, y: at.y + by.y});
        await settle(4);
        return at;
      }
      await settle(8);
    }
    throw new Error("no pointer position on the canvas grabbed the ghost's hand");
  }

  await test("the panel toggles without ever scrolling the document", async () => {
    // The boot rule is width-derived, so the expected starting state is too: a
    // wide viewport opens the panel beside the arm cards, a narrow one keeps
    // the slim bar and downloads nothing. Asserting THAT rule is worth more
    // than assuming whatever window size a runner happens to pick.
    const wide = !window.matchMedia("(max-width: 1020px)").matches;
    assertEqual(body.hidden, !wide,
      "the panel's opening state did not follow the viewport width");
    assertEqual(document.getElementById("sceneToolbar").hidden, !wide,
      "the ghost controls did not follow the panel open/closed");
    if (wide) {
      bar.click();
      await settle();
    }
    const before = scrollCount;
    assertEqual(body.hidden, true, "the panel is not collapsed to start the toggle");
    bar.click();
    await settle();
    assertEqual(body.hidden, false, "the panel body stayed hidden");
    assertEqual(bar.getAttribute("aria-expanded"), "true", "aria-expanded did not follow");
    assert(document.body.classList.contains("scene-open"), "body.scene-open was not set");
    bar.click();
    await settle();
    assertEqual(body.hidden, true, "the panel body stayed open");
    assertEqual(bar.getAttribute("aria-expanded"), "false", "aria-expanded did not follow back");
    assert(!document.body.classList.contains("scene-open"), "body.scene-open stuck");
    // The drawer's own scroll correction measures a reservation this panel must
    // never change, so the panel must never scroll the document itself.
    assertEqual(scrollCount, before, "toggling the panel scrolled the document");
    bar.click();
    await settle();
  });

  await test("with no session the panel says how to get arms on screen", async () => {
    emit(frame({arm_ids: [], frame: {session: Object.assign(frame().session, {
      state: "stopped", session_id: null, arm_ids: [],
    })}}));
    await settle();
    assertEqual(noteHidden(), false, "the no-session sentence was not shown");
    assertEqual(
      note(),
      "Start a session to see the arms. The measured cell is drawn from your "
      + "workspace model.",
      "the no-session sentence is not the one the table specifies",
    );
    assertEqual(document.getElementById("sceneSub").textContent, "no session",
      "the collapsed bar's status line did not say there is no session");
  });

  await test("with the IK service down the panel says the one line that fixes it",
    async () => {
      routes.scene = Object.assign({}, SCENE_OK, {
        ghost_available: false,
        ik: {available: false, arm_ids: [], tip_frame: "flange"},
      });
      await reconnect();
      emit(frame());
      await settle(4);
      assertEqual(
        note(),
        'Pose editing needs the IK service. Start it with '
        + '"ros2 launch franka_ik franka_ik.launch.py".',
        "the IK-offline sentence is not the one the table specifies",
      );
      assertEqual(document.getElementById("sceneSub").textContent, "IK offline",
        "the collapsed bar did not report IK offline");
      routes.scene = JSON.parse(JSON.stringify(SCENE_OK));
      await reconnect();
      emit(frame());
      await settle(4);
    });

  await test("the checker's own sentences are rendered verbatim, never re-authored",
    async () => {
      const cellNote = "The workspace model is not installed, so poses are not "
        + "collision-checked and the cell is not drawn.";
      routes.scene = Object.assign({}, SCENE_OK, {
        cell: null, cell_source: "unavailable", cell_note: cellNote,
      });
      await reconnect();
      emit(frame());
      await settle(4);
      assertEqual(note(), cellNote, "the server's cell sentence was not rendered verbatim");

      const interlockNote = "The cell model was built for a different robot "
        + "description, so poses are not collision-checked.";
      routes.scene = Object.assign({}, SCENE_OK, {
        checker: {available: true, profile: "dual", interlock: "mismatch",
                  note: interlockNote},
      });
      await reconnect();
      emit(frame());
      await settle(4);
      assertEqual(note(), interlockNote,
        "the server's interlock sentence was not rendered verbatim");
      routes.scene = JSON.parse(JSON.stringify(SCENE_OK));
      await reconnect();
      emit(frame());
      await settle(4);
    });

  await test("a single-arm session builds one ghost toggle and reports one arm",
    async () => {
      const single = frame({arm_ids: ["panda1"]});
      single.session.arms = "panda1";
      single.session.arm_mode = "single";
      emit(single);
      await settle(4);
      const toggles = document.getElementById("ghostSeg").children;
      assertEqual(toggles.length, 1, "a one-arm session built the wrong number of toggles");
      assertEqual(toggles[0].dataset.arm, "panda1", "the toggle names the wrong arm");
      assert(document.getElementById("sceneSub").textContent.indexOf("1 arm") === 0,
        `the status line reads ${document.getElementById("sceneSub").textContent}`);
      emit(frame());
      await settle(4);
      assertEqual(document.getElementById("ghostSeg").children.length, 2,
        "the second arm's toggle did not come back");
      assertEqual(document.getElementById("sceneSub").textContent,
        "2 arms live · cell drawn",
        "the collapsed bar must say what opening the panel would be worth");
    });

  await test("a frame with a missing joint holds it, and the console keeps rendering",
    async () => {
      const held = frame({armExtras: {panda1: {
        positions: [HOME[0], null, HOME[2], HOME[3], HOME[4], HOME[5], HOME[6]],
        positions_stale: true,
      }}});
      let threw = null;
      try {
        emit(held);
      } catch (error) {
        threw = error;
      }
      await settle(4);
      assertEqual(threw, null, `a null joint threw out of the render pass: ${threw}`);
      // The console is still alive: its own chrome still tracks the frame.
      assertEqual(document.getElementById("hintText").textContent, held.hint,
        "the console stopped rendering after a frame with a missing joint");
      emit(frame());
      await settle(4);
    });

  await test("the Copy affordance follows the ghost, the verdict and the clock",
    async () => {
      emit(frame());
      await settle(6);
      const copyButton = ghostControl("copy", "panda1");
      const verdict = ghostControl("verdict", "panda1");
      assertEqual(copyButton.hidden, true, "Copy was offered before a ghost differed");

      // The ghost cannot be shown until the scene has mounted, and the mount is
      // asynchronous: toggling before it lands sets a flag nothing acts on.
      await waitFor(() => document.querySelector("#sceneView canvas"),
        "the panel to mount its canvas");
      const canvas = document.querySelector("#sceneView canvas");

      const toggle = document.getElementById("ghostSeg").children[0];
      toggle.click();
      await settle(4);
      assertEqual(toggle.getAttribute("aria-pressed"), "true", "the ghost toggle did not latch");
      assertEqual(copyButton.hidden, true,
        "Copy appeared while the ghost still matched reality");

      // One real drag, all the way through the console's own request path.
      solveResponse = {
        ok: true, arm_id: "panda1", solved: true,
        positions: HOME.map((value, index) => value + (index === 0 ? 0.4 : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict: {status: "clear", min_clearance: 0.04, offending_links: [],
                  reason: null, reason_code: null, checker: "cell_model"},
        copy: {joints_deg: [22.92, -45, 0, -135, 0, 90, 45],
               joints_rad: [0.4, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398],
               snippet: "# Ghost pose for panda1, authored in the Franka console."},
      };
      posted.length = 0;
      await grabHandle(canvas, {x: 40, y: 20});
      assert(posted.some((entry) => entry.path === "/api/ghost/solve"),
        "the drag never reached the solve route");
      const request = posted.find((entry) => entry.path === "/api/ghost/solve").body;
      assertEqual(request.arm_id, "panda1", "the solve named the wrong arm");
      assert(request.scene && Array.isArray(request.scene.panda1)
        && Array.isArray(request.scene.panda2),
        `the solve carried no rendered scene: ${JSON.stringify(request.scene)}`);
      assert(request.scene.panda1 !== null && request.scene.panda2 !== null,
        "the rendered scene carried a null pose instead of omitting the arm");

      await settle(4);
      assertEqual(copyButton.hidden, false, "Copy stayed hidden after the ghost moved");
      assertEqual(copyButton.disabled, false, "Copy was refused on a clear verdict");
      assertEqual(verdict.className, "scene-verdict clear",
        `the verdict chip reads ${verdict.className}`);
      assert(ghostControl("degrees", "panda1").hidden === false,
        "the panel showed no degrees for a diverged ghost");
      assert(ghostControl("degrees", "panda1").textContent.indexOf("°") > 0,
        "the panel showed radians where it must show degrees");
    });

  await test("Copy writes the server's snippet and shows exactly what it wrote",
    async () => {
      const copyButton = ghostControl("copy", "panda1");
      const snippet = ghostControl("snippet", "panda1");
      assertEqual(copyButton.textContent, "Copy pose — panda1",
        "the Copy button did not name the arm it would copy");
      assertEqual(snippet.hidden, true, "a snippet was on screen before anything was copied");
      // The lab's own journey: the console reached by IP is not a secure
      // context, so navigator.clipboard is undefined and the execCommand path
      // is the one that actually runs. Exercise that one.
      const originalClipboard = navigator.clipboard;
      const originalExec = document.execCommand;
      let written = null;
      Object.defineProperty(navigator, "clipboard", {value: undefined, configurable: true});
      document.execCommand = function (command) {
        if (command === "copy") {
          written = document.activeElement && document.activeElement.value;
        }
        return true;
      };
      // Read the result SYNCHRONOUSLY: the execCommand path and the panel
      // update both run inside the click, and the acknowledgement expires on a
      // timer — so anything that waited first would be racing that timer
      // rather than testing the copy.
      try {
        copyButton.click();
      } finally {
        document.execCommand = originalExec;
        Object.defineProperty(navigator, "clipboard",
          {value: originalClipboard, configurable: true});
      }
      assertEqual(written, "# Ghost pose for panda1, authored in the Franka console.",
        "the clipboard did not receive the server's snippet unchanged");
      assertEqual(ghostControl("toast", "panda1").hidden, false,
        "copying gave no acknowledgement");
      assertEqual(snippet.hidden, false, "the copied snippet was not shown");
      // Byte for byte. The panel neither assembles nor edits the snippet, so
      // what is read here is exactly what was pasted — including any line the
      // server puts in it about a pose that was not collision-checked.
      assertEqual(snippet.textContent,
        "# Ghost pose for panda1, authored in the Franka console.",
        "the snippet on screen is not the one the server sent");
    });

  await test("a collision verdict disables Copy and shows the server's sentence",
    async () => {
      const copyButton = ghostControl("copy", "panda1");
      const verdict = ghostControl("verdict", "panda1");
      const reason = "Panda 1's forearm would leave the work area through the "
        + "table top by 21 mm.";
      solveResponse = {
        ok: true, arm_id: "panda1", solved: true,
        positions: HOME.map((value, index) => value + (index === 0 ? 0.5 : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict: {status: "collision", min_clearance: -0.021,
                  offending_links: ["panda1_link5"], reason,
                  reason_code: "contact", checker: "cell_model"},
        copy: {joints_deg: [28.6, -45, 0, -135, 0, 90, 45],
               joints_rad: [0.5, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398],
               snippet: "# Ghost pose for panda1, authored in the Franka console."},
      };
      const canvas = document.querySelector("#sceneView canvas");
      await grabHandle(canvas, {x: 30, y: 30});
      await settle(4);
      assertEqual(verdict.textContent, reason,
        "the collision sentence was not the server's, rendered verbatim");
      assertEqual(verdict.className, "scene-verdict collision",
        "the verdict chip did not take the collision treatment");
      assertEqual(copyButton.hidden, false, "Copy vanished on a collision instead of refusing");
      assertEqual(copyButton.disabled, true, "Copy stayed available on a colliding pose");
    });

  await test("Reset ghost puts the ghost back, and says what it went back to",
    async () => {
      // The pose the ghost is about to become. A reset moves the ghost with
      // no gesture behind it, so nothing would ask the checker about the
      // pose it lands on -- and the sentence from the pose it left would sit
      // there describing a cell that is gone. The console asks again.
      solveResponse = {
        ok: true, arm_id: "panda1", solved: true,
        positions: HOME.slice(), positions_deg: [], redundancy_value: HOME[6],
        solve_reason: null,
        verdict: {status: "clear", min_clearance: 0.04, offending_links: [],
                  reason: null, reason_code: null, contacts: [],
                  checker: "cell_model"},
        copy: {joints_deg: [0, -45, 0, -135, 0, 90, 45], joints_rad: [],
               snippet: "# Ghost pose for panda1."},
      };
      ghostControl("reset", "panda1").click();
      await settle(6);
      assertEqual(ghostControl("copy", "panda1").hidden, true,
        "Copy survived a reset");
      assertEqual(ghostControl("verdict", "panda1").textContent,
        "Clear of everything in the cell model.",
        "the row did not report the pose the ghost was reset to");
      assertEqual(ghostControl("snippet", "panda1").hidden, true,
        "a snippet describing the old pose survived a reset");
    });


  await test("the toolbar says how to move the hand, in plain words", async () => {
    const hint = document.getElementById("sceneHint");
    const toolbar = document.getElementById("sceneToolbar");
    assert(hint, "the ghost toolbar carries no line explaining the handles");
    assert(toolbar.contains(hint),
      "the hint does not live inside the ghost toolbar, so closing the panel "
      + "would leave it on screen with nothing to explain");
    const words = hint.textContent.toLowerCase();
    // The four handles, named. The operator reported the third translation
    // axis as missing because the ONLY place Shift was written down was a
    // comment in a source file; this line is the fix for that, so a hint that
    // has stopped naming Shift has stopped being the fix.
    ["knob", "shift", "arrow", "ring"].forEach((word) => {
      assert(words.indexOf(word) >= 0,
        `the hint never mentions the ${word}, so an operator would still have `
        + "to be told this by a person");
    });
    assert(!/[_(){}<>[\]=]/.test(hint.textContent),
      `the hint reads like source, not like a sentence: ${hint.textContent}`);
    ["quaternion", "flange", "gizmo", "modifier", "ndc", "urdf"].forEach((jargon) => {
      assert(words.indexOf(jargon) < 0,
        `the hint says "${jargon}", which is not a word this console explains`);
    });

    emit(frame());
    await settle(4);
    if (body.hidden) {
      bar.click();
      await settle(4);
    }
    const toggles = Array.from(document.getElementById("ghostSeg").children);
    for (const toggle of toggles) {
      if (toggle.getAttribute("aria-pressed") === "true") {
        toggle.click();
        await settle(4);
      }
    }
    await waitFor(() => hint.hidden === true, "the hint to go with the last ghost");
    assertEqual(hint.hidden, true,
      "the toolbar explains handles that are not on screen: no ghost is shown, "
      + "so there is no knob, no arrow and no ring to explain");
    toggles[0].click();
    await waitFor(() => hint.hidden === false,
      "the hint to arrive with the ghost it explains");
    bar.click();
    await settle(4);
    assertEqual(toolbar.hidden, true,
      "the ghost toolbar survived the panel closing, and the hint with it");
    bar.click();
    await settle(4);
    assertEqual(hint.hidden, false,
      "the hint did not come back when the panel reopened on a shown ghost");
  });


  /* ================= the rotation gizmo, through the console ============== */

  // Sweep the WHOLE canvas for a gesture the caller recognises. Two moves per
  // grab, so a gesture identifies itself by its own request shape: a turn
  // pins the target position and moves the quaternion; a hand drag does the
  // opposite. No case below has to know where a handle happens to be drawn.
  async function sweepFor(canvas, accept, what, by = {x: 26, y: 18}) {
    const sweepGrabs = [];
    const size = canvas.getBoundingClientRect();
    const step = 20;
    for (let downY = 10; downY < size.height; downY += step) {
      for (let downX = 10; downX < size.width; downX += step) {
        // Read the rect EVERY time and work in canvas-relative offsets. The
        // panel above the canvas grows and shrinks as affordances appear --
        // a Copy acknowledgement is enough to wrap the toolbar -- and a
        // viewport coordinate captured before that is a coordinate somewhere
        // else afterwards.
        const at = canvasPoint(canvas, {x: downX, y: downY});
        const down = pointer("pointerdown", canvas, at);
        if (!down.defaultPrevented) {
          pointer("pointerup", canvas, at);
          continue;
        }
        const solves = await dragFrom(canvas, at, by);
        sweepGrabs.push(solves.length + ":"
          + (solves.length ? solves[0].arm_id : "none"));
        if (solves.length > 0 && accept(solves)) {
          return {offset: {x: downX, y: downY}, solves};
        }
      }
    }
    throw new Error(`no gesture anywhere on the canvas produced ${what}`
      + ` (${Math.round(size.width)}x${Math.round(size.height)} canvas,`
      + ` grabs: ${JSON.stringify(sweepGrabs)})`);
  }

  function canvasPoint(canvas, offset) {
    const bounds = canvas.getBoundingClientRect();
    return {x: bounds.left + offset.x, y: bounds.top + offset.y};
  }

  /** Two moves and a release from a pointer that is already down. */
  async function dragFrom(canvas, at, by) {
    posted.length = 0;
    pointer("pointermove", canvas, {x: at.x + by.x, y: at.y + by.y});
    await settle(3);
    pointer("pointermove", canvas, {x: at.x + 2 * by.x, y: at.y + 2 * by.y});
    await settle(3);
    pointer("pointerup", canvas, {x: at.x + 2 * by.x, y: at.y + 2 * by.y});
    await settle(4);
    return posted.filter((entry) => entry.path === "/api/ghost/solve")
      .map((entry) => entry.body);
  }

  const samePlace = (a, b) => a.every((value, index) => Math.abs(value - b[index]) < 1e-9);
  const sameTurn = (a, b) => quaternionAngle(a, b) < 1e-9;
  // A TURN, in the wire shape: the hand stays exactly where it is and its
  // orientation moves. A hand drag is the mirror image of this and is
  // rejected here, so the two can never be confused for one another.
  const isTurn = (solves) => solves.length >= 2
    && solves.every((body) => samePlace(body.target.position, solves[0].target.position))
    && !sameTurn(solves[solves.length - 1].target.orientation,
                 solves[0].target.orientation);

  await test("an axis ring on the console turns the hand and the arm follows",
    async () => {
      emit(frame());
      await settle(4);
      const canvas = document.querySelector("#sceneView canvas");
      const toggle = document.getElementById("ghostSeg").children[0];
      if (toggle.getAttribute("aria-pressed") !== "true") {
        toggle.click();
        await settle(6);
      }
      // The snippet the server would build for THIS request, carrying the
      // quaternion the gizmo authored. That is what makes "Copy carries the
      // orientation" a thing this case can read rather than assume.
      solveResponse = (body) => ({
        ok: true, arm_id: body.arm_id, solved: true,
        positions: HOME.map((value, index) => value + (index === 6 ? 0.22 : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict: {status: "clear", min_clearance: 0.04, offending_links: [],
                  reason: null, reason_code: null, checker: "cell_model"},
        copy: {joints_deg: [0, -45, 0, -135, 0, 90, 57.6],
               joints_rad: [0, -0.785398, 0, -2.356194, 0, 1.570796, 1.005398],
               snippet: "# Ghost pose for " + body.arm_id + ", quat "
                 + body.target.orientation.map((v) => v.toFixed(6)).join(",")},
      });
      const found = await sweepFor(canvas, isTurn, "a turn of the hand");
      const turned = found.solves[found.solves.length - 1];
      assertEqual(turned.arm_id, "panda1", "the turn named the wrong arm");
      assert(Array.isArray(turned.target.orientation)
        && turned.target.orientation.length === 4,
        "the console sent no quaternion for a turn");
      assert(turned.scene && Array.isArray(turned.scene.panda1),
        "the turn carried no rendered scene for the collision check");

      // The verdict travelled: a clear answer leaves Copy offered and armed.
      await settle(4);
      const copyButton = ghostControl("copy", "panda1");
      assertEqual(copyButton.hidden, false, "Copy was not offered after a turn");
      assertEqual(copyButton.disabled, false, "Copy was refused on a clear turn");
      assertEqual(ghostControl("verdict", "panda1").className, "scene-verdict clear",
        "the verdict chip did not follow a turn");

      // And Copy carries the orientation the ring authored, byte for byte.
      const expected = "# Ghost pose for panda1, quat "
        + turned.target.orientation.map((v) => v.toFixed(6)).join(",");
      const originalClipboard = navigator.clipboard;
      const originalExec = document.execCommand;
      let written = null;
      Object.defineProperty(navigator, "clipboard", {value: undefined, configurable: true});
      document.execCommand = function (command) {
        if (command === "copy") {
          written = document.activeElement && document.activeElement.value;
        }
        return true;
      };
      try {
        copyButton.click();
      } finally {
        document.execCommand = originalExec;
        Object.defineProperty(navigator, "clipboard",
          {value: originalClipboard, configurable: true});
      }
      assertEqual(written, expected,
        "the copied snippet is not the one the server computed for the authored orientation");
      assertEqual(ghostControl("snippet", "panda1").textContent, expected,
        "the snippet on screen is not the one that was copied");

      // The same ring again, with a colliding answer: the tinting path a turn
      // takes is the hand drag's, and it still fires.
      const reason = "Panda 1's wrist would leave the work area by 8 mm.";
      solveResponse = (body) => ({
        ok: true, arm_id: body.arm_id, solved: true,
        positions: HOME.map((value, index) => value + (index === 6 ? 0.3 : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict: {status: "collision", min_clearance: -0.008,
                  offending_links: ["panda1_link7"], reason,
                  reason_code: "contact", checker: "cell_model"},
        copy: {joints_deg: [0, -45, 0, -135, 0, 90, 62], joints_rad: [],
               snippet: "# Ghost pose for panda1."},
      });
      const again = (await sweepFor(canvas, isTurn, "a second turn of the hand",
        {x: -30, y: -20})).solves;
      assertEqual(again[again.length - 1].arm_id, "panda1",
        "the second turn named the wrong arm");
      await settle(4);
      assertEqual(ghostControl("verdict", "panda1").textContent, reason,
        "a colliding turn did not get the server's sentence");
      assertEqual(ghostControl("verdict", "panda1").className, "scene-verdict collision",
        "a colliding turn did not take the collision treatment");
      assertEqual(ghostControl("copy", "panda1").disabled, true,
        "Copy stayed available on a colliding turn");
      ghostControl("reset", "panda1").click();
      await settle(4);
    });

  /* ================= two ghosts, two of everything ======================== */

  await test("with both ghosts up, each arm has its own Copy and its own degrees",
    async () => {
      emit(frame());
      await settle(4);
      const canvas = document.querySelector("#sceneView canvas");
      // Put the view back where the default frames it, whatever the cases
      // above left it looking at.
      canvas.dispatchEvent(new MouseEvent("dblclick", {bubbles: true, cancelable: true}));
      await settle(2);
      // One ghost at a time, each moved by a real gesture of its own. Doing
      // it in this order is also the journey the defect was found on: work
      // on panda1, then bring panda2 up beside it.
      const seg = Array.from(document.getElementById("ghostSeg").children);
      assertEqual(seg.length, 2, "the two-arm session did not offer two ghost toggles");
      seg.forEach((node) => {
        if (node.getAttribute("aria-pressed") === "true") {
          node.click();
        }
      });
      await settle(4);

      // Two arms, two different answers. Everything below asks whether the
      // panel kept them apart.
      const degreesFor = {
        panda1: [22.92, -45, 0, -135, 0, 90, 45],
        panda2: [-17.19, -45, 0, -135, 0, 90, 45],
      };
      solveResponse = (body) => ({
        ok: true, arm_id: body.arm_id, solved: true,
        positions: HOME.map((value, index) => value
          + (index === 0 ? (body.arm_id === "panda1" ? 0.4 : -0.3) : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict: {status: "clear", min_clearance: 0.04, offending_links: [],
                  reason: null, reason_code: null, checker: "cell_model"},
        copy: {joints_deg: degreesFor[body.arm_id], joints_rad: [],
               snippet: "# Ghost pose for " + body.arm_id
                 + ", authored in the Franka console."},
      });

      seg[0].click();
      await settle(6);
      await sweepFor(canvas, (solves) => solves.some(
        (body) => body.arm_id === "panda1"), "a gesture on panda1's ghost");
      seg[1].click();
      await settle(6);
      await sweepFor(canvas, (solves) => solves.some(
        (body) => body.arm_id === "panda2"), "a gesture on panda2's ghost");
      assertEqual(seg.map((node) => node.getAttribute("aria-pressed")).join(","),
        "true,true", "both ghosts must be up for this case");
      // ...and panda1 is STILL reachable with its neighbour up. A second hand
      // on the canvas must not take the first one's handles away.
      await sweepFor(canvas, (solves) => solves.some(
        (body) => body.arm_id === "panda1"),
      "a gesture on panda1's ghost while panda2's is up too", {x: -30, y: -20});
      await settle(4);

      const copies = shownCopyButtons();
      assertEqual(copies.length, 2,
        `both ghosts differ from reality, so both must offer Copy; ${copies.length} did`);
      assertEqual(copies.map((node) => node.textContent).sort().join(" | "),
        "Copy pose — panda1 | Copy pose — panda2",
        "the two Copy affordances do not name their own arms");

      const degrees = ["panda1", "panda2"].map((armId) => ghostControl("degrees", armId));
      degrees.forEach((node, index) => {
        const armId = index === 0 ? "panda1" : "panda2";
        assertEqual(node.hidden, false, `${armId} has no degrees readout of its own`);
        assert(node.textContent.indexOf(armId) === 0,
          `${armId}'s degrees line does not say which arm it belongs to`);
        assert(node.textContent.indexOf(String(degreesFor[armId][0])) > 0,
          `${armId}'s degrees line reads ${node.textContent}, which is not its own pose`);
      });
      assert(degrees[0].textContent !== degrees[1].textContent,
        "both arms are showing the same joint angles");

      // Copy panda2, and only panda2.
      const originalClipboard = navigator.clipboard;
      const originalExec = document.execCommand;
      let written = null;
      Object.defineProperty(navigator, "clipboard", {value: undefined, configurable: true});
      document.execCommand = function (command) {
        if (command === "copy") {
          written = document.activeElement && document.activeElement.value;
        }
        return true;
      };
      try {
        ghostControl("copy", "panda2").click();
      } finally {
        document.execCommand = originalExec;
        Object.defineProperty(navigator, "clipboard",
          {value: originalClipboard, configurable: true});
      }
      assertEqual(written,
        "# Ghost pose for panda2, authored in the Franka console.",
        "Copy on panda2 put panda1's pose on the clipboard");
      assertEqual(ghostControl("toast", "panda2").hidden, false,
        "panda2's copy gave no acknowledgement");
      assertEqual(ghostControl("toast", "panda1").hidden, true,
        "copying panda2 acknowledged on panda1's row");
      assertEqual(ghostControl("snippet", "panda1").hidden, true,
        "panda2's snippet was shown under panda1");

      // Reset panda2, and only panda2.
      ghostControl("reset", "panda2").click();
      await settle(4);
      assertEqual(ghostControl("copy", "panda2").hidden, true,
        "panda2's Copy survived panda2's reset");
      assertEqual(ghostControl("copy", "panda1").hidden, false,
        "resetting panda2 took panda1's Copy away with it");
      assert(ghostControl("degrees", "panda1").textContent.indexOf("22.92") > 0,
        "resetting panda2 changed what panda1 reads out");
    });


  /* ============ one answer, one line per arm: verdict attribution ========= */

  // THE DEFECT, from the operator's own screenshot (2026-09-04). The check is
  // asked about the whole cell and answers once, and the console used to write
  // that one sentence into the row of whichever arm had just been dragged. So
  // Panda 1's row read "Panda 2 joint 4 is 4.0° past its limit." above Panda
  // 1's own degrees, and went on reading it after Panda 2 had moved clear,
  // because nothing refreshed a row until its own arm was dragged again.
  //
  // Every case below drives the console the way the operator did: a real
  // gesture on one ghost, and then a reading of BOTH rows.

  const CLEAR_LINE = "Clear of everything in the cell model.";
  const PANDA2_LIMIT = "Panda 2 joint 4 is 4.0° past its limit.";
  const PANDA1_TABLE = "Panda 1's forearm would leave the work area through "
    + "the table top by 21 mm.";
  const CROSS_PAIR = "Panda 1's wrist would hit Panda 2's forearm — 12 mm too close.";
  const NOT_RECHECKED = "The cell was not checked again after that change, "
    + "so the lines above it were cleared. Move a ghost to ask again.";
  // An answer that never arrives at all: a 503, a dropped connection, an IK
  // service that has gone away between one request and the next.
  const NO_ANSWER = {ok: false, error: "solve_failed",
                     detail: "The IK service dropped the connection."};
  // An answer that arrives and refuses: the pose is one the solver will not
  // reach, which is what a re-check of a pose sitting at a joint limit gets.
  const refusal = (body) => ({
    ok: true, arm_id: body.arm_id, solved: false, positions: null,
    verdict: null, copy: null,
    solve_reason: "Reaching that point would push a joint past its limit."});
  const PANDA2_SELF = "Panda 2's forearm would hit its own base — 12 mm too close.";

  const LIMIT_OF_PANDA2 = {
    kind: "joint_limit", arm_id: "panda2", a: "panda2_joint4", b: "",
    distance: -0.0698132, sentence: PANDA2_LIMIT};
  const TABLE_OF_PANDA1 = {
    kind: "containment", arm_id: "panda1", a: "panda1_link5_v1",
    b: "work_area.z_min", distance: -0.021, sentence: PANDA1_TABLE};
  const CROSS_OF_BOTH = {
    kind: "cross_arm", arm_id: "panda1", a: "panda1_link6_v0",
    b: "panda2_link5_v1", distance: -0.012, sentence: CROSS_PAIR};
  const SELF_OF_PANDA2 = {
    kind: "self", arm_id: "panda2", a: "panda2_link5_v1", b: "panda2_link0_v0",
    distance: -0.012, sentence: PANDA2_SELF};

  // The server's per-kind link rule, restated small for a fixture: a joint
  // limit names no link at all, a containment names only its own side, and a
  // volume id loses its `_v<n>`. Getting this right here is what makes the
  // tint assertions below mean anything.
  const LINK_FIELDS = {
    self: ["a", "b"], cross_arm: ["a", "b"], containment: ["a"],
    environment: ["a"], keep_out: ["a"], joint_limit: []};

  function linksOf(contacts) {
    const found = [];
    contacts.forEach((item) => (LINK_FIELDS[item.kind] || []).forEach((field) => {
      const name = String(item[field]).replace(/_v\d+$/, "");
      if (/^panda[12]_link[0-8]$/.test(name) && found.indexOf(name) < 0) {
        found.push(name);
      }
    }));
    return found.sort();
  }

  // The server's per-arm attribution, restated small for a fixture. A contact
  // names an arm when the checker attributed it (`arm_id`) or when either
  // side of the pair is one of that arm's parts, and a cross-arm pair names
  // both. The page holds no copy of this rule any more -- it reads `arms` --
  // so this mirrors `ghost.arms_payload`, which is computed over the WHOLE
  // contact tuple before the wire list is truncated.
  function armsOf(contacts) {
    const arms = {};
    ["panda1", "panda2"].forEach((armId) => {
      const prefix = armId + "_";
      const mine = contacts.filter((item) => item.arm_id === armId
        || String(item.a).indexOf(prefix) === 0
        || String(item.b).indexOf(prefix) === 0);
      arms[armId] = mine.length
        ? {status: "collision", reason: mine[0].sentence,
           offending_links: linksOf(mine).filter(
             (name) => name.indexOf(prefix) === 0)}
        : {status: "clear", reason: null, offending_links: []};
    });
    return arms;
  }

  // One whole-cell answer, exactly the shape the server sends: the three
  // whole-cell fields, the itemised `contacts` for reading, and the per-arm
  // `arms` the page draws its rows from.
  function wholeCell(contacts) {
    return function (body) {
      const verdict = contacts.length
        ? {status: "collision", min_clearance: -0.012,
           offending_links: linksOf(contacts), reason: contacts[0].sentence,
           reason_code: "contact", contacts, arms: armsOf(contacts),
           checker: "cell_model"}
        : {status: "clear", min_clearance: 0.04, offending_links: [],
           reason: null, reason_code: null, contacts: [],
           arms: armsOf([]), checker: "cell_model"};
      return {
        ok: true, arm_id: body.arm_id, solved: true,
        positions: HOME.map((value, index) => value + (index === 0
          ? (body.arm_id === "panda1" ? 0.36 : -0.28) : 0)),
        positions_deg: [], redundancy_value: HOME[6], solve_reason: null,
        verdict,
        copy: {joints_deg: [20.6, -45, 0, -135, 0, 90, 45], joints_rad: [],
               snippet: "# Ghost pose for " + body.arm_id + "."},
      };
    };
  }

  const line = (armId) => ghostControl("verdict", armId).textContent;
  const chip = (armId) => ghostControl("verdict", armId).className;

  // A gesture that is known to have reached ONE named arm's solve route. The
  // grab point is remembered per arm and re-tried before the canvas is swept
  // again, because a sweep is the expensive way to ask a cheap question.
  const grabbedAt = {};
  let nudge = 1;

  async function gestureOn(armId) {
    const canvas = document.querySelector("#sceneView canvas");
    const accept = (solves) => solves.some((entry) => entry.arm_id === armId);
    nudge = -nudge;
    const by = {x: 22 * nudge, y: 14 * nudge};
    if (grabbedAt[armId]) {
      const at = canvasPoint(canvas, grabbedAt[armId]);
      const down = pointer("pointerdown", canvas, at);
      if (down.defaultPrevented) {
        const solves = await dragFrom(canvas, at, by);
        if (solves.length && accept(solves)) {
          return solves;
        }
      } else {
        pointer("pointerup", canvas, at);
      }
    }
    const found = await sweepFor(canvas, accept,
      `a gesture on ${armId}'s ghost`, by);
    grabbedAt[armId] = found.offset;
    return found.solves;
  }

  /** Both ghosts up, both reachable, and the panel open. */
  async function bothGhostsUp() {
    emit(frame());
    await settle(4);
    if (body.hidden) {
      bar.click();
      await settle(4);
    }
    const seg = Array.from(document.getElementById("ghostSeg").children);
    for (const node of seg) {
      if (node.getAttribute("aria-pressed") !== "true") {
        node.click();
        await settle(6);
      }
    }
    return seg;
  }

  await test("a fault on one arm never becomes the other arm's line", async () => {
    await bothGhostsUp();
    solveResponse = wholeCell([LIMIT_OF_PANDA2]);
    await gestureOn("panda1");
    await settle(4);
    // THE SCREENSHOT. panda1 was the arm dragged, so the whole-cell sentence
    // used to land here. Nothing is wrong with panda1.
    assertEqual(line("panda1"), CLEAR_LINE,
      "panda1's row is carrying panda2's fault");
    assertEqual(chip("panda1"), "scene-verdict clear",
      "panda1 was tinted for a fault that is not its own");
    assertEqual(line("panda2"), PANDA2_LIMIT,
      "the arm the fault belongs to did not get the sentence");
    assertEqual(chip("panda2"), "scene-verdict collision",
      "the offending arm was not tinted");
  });

  await test("a row is rewritten by its neighbour's check, untouched itself",
    async () => {
      await bothGhostsUp();
      solveResponse = wholeCell([TABLE_OF_PANDA1]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), PANDA1_TABLE,
        "panda1's own fault did not reach panda1's row");

      // panda1 moves clear, and the only thing that happens afterwards is a
      // gesture on panda2. This is the half of the defect that outlived the
      // pose: the stale sentence used to sit there until panda1 was dragged.
      solveResponse = wholeCell([]);
      posted.length = 0;
      await gestureOn("panda2");
      await settle(4);
      assert(!posted.some((entry) => entry.path === "/api/ghost/solve"
        && entry.body.arm_id === "panda1"),
      "panda1 was solved after all, so this proves nothing about refreshing");
      assertEqual(line("panda1"), CLEAR_LINE,
        "panda1's row kept a sentence about a pose that has since moved");
      assertEqual(chip("panda1"), "scene-verdict clear",
        "panda1 stayed tinted for a fault the cell no longer has");
      assertEqual(line("panda2"), CLEAR_LINE, "panda2's own row did not refresh");
    });

  await test("a contact between the two arms is the fault of both rows", async () => {
    await bothGhostsUp();
    solveResponse = wholeCell([CROSS_OF_BOTH]);
    await gestureOn("panda1");
    await settle(4);
    assertEqual(line("panda1"), CROSS_PAIR,
      "the arm that was dragged did not get the pair it is half of");
    assertEqual(line("panda2"), CROSS_PAIR,
      "the other half of the pair was not told about it");
    assertEqual(chip("panda1"), "scene-verdict collision", "panda1 was not tinted");
    assertEqual(chip("panda2"), "scene-verdict collision", "panda2 was not tinted");
  });

  await test("one arm's own parts touching is never the other arm's line", async () => {
    await bothGhostsUp();
    solveResponse = wholeCell([SELF_OF_PANDA2]);
    // Dragged on panda1, and the fault is entirely inside panda2. Attributing
    // by the arm that was solved would put it on exactly the wrong row.
    await gestureOn("panda1");
    await settle(4);
    assertEqual(line("panda1"), CLEAR_LINE,
      "panda2's own self-contact was printed on panda1's row");
    assertEqual(line("panda2"), PANDA2_SELF,
      "the arm whose parts would touch was not told");
  });

  await test("resetting one ghost re-checks the cell the other one is still in",
    async () => {
      await bothGhostsUp();
      solveResponse = wholeCell([CROSS_OF_BOTH]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), CROSS_PAIR, "the pair did not reach panda1's row");

      // A reset changes the cell with no gesture behind it, so nothing would
      // ask again -- and panda1's sentence would go on describing a panda2
      // that is no longer there.
      solveResponse = wholeCell([]);
      posted.length = 0;
      ghostControl("reset", "panda2").click();
      await waitFor(() => line("panda1") === CLEAR_LINE,
        "panda1's row to be re-checked after panda2 was reset");
      assert(posted.some((entry) => entry.path === "/api/ghost/solve"),
        "the reset asked nothing, so panda1's sentence outlived its cell");
      assertEqual(chip("panda1"), "scene-verdict clear",
        "panda1 stayed tinted for an arm that has gone back to the robot");
      // And the re-check moved nothing: it carries no new pose, so panda1's
      // Copy and its degrees still describe the pose the operator authored.
      assertEqual(ghostControl("copy", "panda1").hidden, false,
        "a re-check took panda1's Copy away, so it was treated as a new pose");
    });

  await test("a ghost taken off the screen leaves no sentence behind it", async () => {
    await bothGhostsUp();
    // The pair, deliberately: it puts a sentence on the REMAINING arm's row
    // that only a re-check can remove. A fault of panda2's alone never
    // reached panda1's row in the first place, so hiding panda2 would clear
    // it whether the page asked again or not, and the case would pass with
    // the re-check on hide deleted.
    solveResponse = wholeCell([CROSS_OF_BOTH]);
    await gestureOn("panda1");
    await settle(4);
    assertEqual(line("panda1"), CROSS_PAIR, "the pair did not reach panda1's row");
    assertEqual(line("panda2"), CROSS_PAIR, "the pair did not reach panda2's row");

    const seg = Array.from(document.getElementById("ghostSeg").children);
    solveResponse = wholeCell([]);
    posted.length = 0;
    seg[1].click();
    await waitFor(() => ghostControl("verdict", "panda2").textContent === "",
      "panda2's row to empty when panda2's ghost left the screen");
    await waitFor(() => line("panda1") === CLEAR_LINE,
      "panda1's row to be re-checked for the cell panda2 left");
    assert(posted.some((entry) => entry.path === "/api/ghost/solve"),
      "hiding a ghost asked nothing, so panda1's sentence outlived its cell");
    assertEqual(chip("panda1"), "scene-verdict clear",
      "panda1 stayed tinted for an arm that is no longer drawn");
    seg[1].click();
    await settle(6);
  });

  await test("an answer with no attribution in it tells nobody they are clear",
    async () => {
      // The itemised list is bounded for the wire, so absence from it is not
      // evidence of anything -- which is why the page no longer reads it.
      // An answer that carries no `arms` map is an answer whose attribution
      // this page does not have, and both rows say what the cell said.
      // Reading worse than the truth is a bug; reading clear when something
      // is not is a lie, and this fails towards the bug.
      await bothGhostsUp();
      const unattributed = wholeCell([SELF_OF_PANDA2]);
      solveResponse = (body) => {
        const answer = unattributed(body);
        delete answer.verdict.arms;
        return answer;
      };
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), PANDA2_SELF,
        "an unattributed refusal let panda1 read clear");
      assertEqual(line("panda2"), PANDA2_SELF,
        "an unattributed refusal let panda2 read clear");
      assertEqual(ghostControl("copy", "panda1").disabled, true,
        "Copy opened on a pose no attribution had cleared");
    });

  await test("a refusal about an arm with no ghost still reaches the screen",
    async () => {
      // The checker looks at every arm in the cell, drawn as a ghost or
      // standing where it is measured -- so a refusal can name an arm that
      // has no row. With the sentence attributed and nowhere to put it, the
      // one row on screen used to read the clear line for a cell the checker
      // had refused, with Copy re-enabled on it.
      await bothGhostsUp();
      const seg = Array.from(document.getElementById("ghostSeg").children);
      solveResponse = wholeCell([]);
      seg[1].click();
      await waitFor(() => ghostControl("verdict", "panda2").textContent === "",
        "panda2's ghost to leave the screen");

      solveResponse = wholeCell([SELF_OF_PANDA2]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), PANDA2_SELF,
        "the whole cell was refused and the only row on screen read clear");
      assertEqual(chip("panda1"), "scene-verdict collision",
        "a refused cell left the row on screen tinted clear");
      assertEqual(ghostControl("copy", "panda1").disabled, true,
        "Copy opened on a pose the checker never cleared");

      seg[1].click();
      await settle(6);
    });

  await test("a re-check the solver refuses is asked of the other ghost",
    async () => {
      await bothGhostsUp();
      solveResponse = wholeCell([CROSS_OF_BOTH]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), CROSS_PAIR, "the pair did not reach panda1's row");

      // The re-check target is the FK of the pose already drawn, and a pose
      // sitting at a joint limit is one the solver can refuse -- which used
      // to end the matter silently, with every row still describing the cell
      // as it was before the reset.
      const clear = wholeCell([]);
      solveResponse = (body) => (body.arm_id === "panda1"
        ? {ok: true, arm_id: "panda1", solved: false, positions: null,
           verdict: null, copy: null,
           solve_reason: "Reaching that point would push a joint past its limit."}
        : clear(body));
      ghostControl("reset", "panda2").click();
      await waitFor(() => line("panda1") === CLEAR_LINE,
        "the refused re-check to fall through to the other ghost");
      assertEqual(chip("panda1"), "scene-verdict clear",
        "panda1 stayed tinted for a cell that has gone");
    });

  await test("a re-check nothing can answer empties the rows and says so",
    async () => {
      await bothGhostsUp();
      solveResponse = wholeCell([CROSS_OF_BOTH]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), CROSS_PAIR, "the pair did not reach panda1's row");

      // Neither ghost can be re-checked. A stale sentence is a claim about a
      // cell that is gone; a blank row claims nothing, and the panel says in
      // words why the rows are blank.
      solveResponse = refusal;
      ghostControl("reset", "panda2").click();
      await waitFor(() => line("panda1") === "",
        "panda1's row to be emptied by a re-check that never came back");
      assertEqual(chip("panda1"), "scene-verdict",
        "panda1 kept a tint for a cell nothing has checked");
      assertEqual(note(), NOT_RECHECKED,
        "the panel did not say why its rows went blank");

      // And the panel comes back: one answered solve refreshes the rows and
      // takes the note away with them.
      solveResponse = wholeCell([]);
      await gestureOn("panda1");
      await waitFor(() => line("panda1") === CLEAR_LINE,
        "a later solve to put the rows back");
      assertEqual(noteHidden(), true, "the note outlived the answer that fixed it");
    });

  await test("an answer already on the wire does not discharge the re-check",
    async () => {
      await bothGhostsUp();
      solveResponse = wholeCell([CROSS_OF_BOTH]);
      await gestureOn("panda1");
      await settle(4);
      assertEqual(line("panda1"), CROSS_PAIR, "the pair did not reach panda1's row");

      // THE RACE, in the order it happens in life. The drag's last answer is
      // still on the wire when the operator takes panda2's ghost off the
      // screen, so the re-check queues up behind it. That answer was computed
      // for the cell panda2 was still in: it refreshes nothing about the cell
      // as it is now, and the re-check still owes the rows an answer. Letting
      // it discharge the round left panda1 reading a cross-arm sentence about
      // a ghost that is no longer drawn, with the panel saying nothing.
      const stale = wholeCell([CROSS_OF_BOTH]);
      let release = null;
      solveResponse = (body) => {
        if (release === null && body.arm_id === "panda1") {
          return new Promise((resolve) => {
            release = () => resolve(stale(body));
          });
        }
        return refusal(body);
      };
      await gestureOn("panda1");
      assert(release !== null, "the drag's answer is not on the wire, so there is no race");

      const seg = Array.from(document.getElementById("ghostSeg").children);
      seg[1].click();
      await settle(2);
      release();
      await waitFor(() => line("panda1") === "",
        "panda1's row to be emptied by the re-check that nothing answered");
      assertEqual(chip("panda1"), "scene-verdict",
        "panda1 kept a tint for a cell nothing has checked");
      assertEqual(note(), NOT_RECHECKED,
        "the panel did not say why its rows went blank");

      solveResponse = wholeCell([]);
      seg[1].click();
      await settle(6);
    });

  window.fetch = realFetch;
  window.scrollBy = realScrollBy;
}
