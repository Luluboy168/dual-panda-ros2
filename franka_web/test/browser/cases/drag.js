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
  forwardKinematics,
  invertRigidMatrix,
  multiplyMatrices,
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
    schema_version: 4,
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
      payload = solveResponse || {ok: true, solved: false, solve_reason: null};
    }
    return Promise.resolve(new Response(JSON.stringify(payload), {
      status: payload.ok === false ? 503 : 200,
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
      const copyButton = document.getElementById("btnGhostCopy");
      const verdict = document.getElementById("sceneVerdict");
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
      assert(document.getElementById("sceneDegrees").hidden === false,
        "the panel showed no degrees for a diverged ghost");
      assert(document.getElementById("sceneDegrees").textContent.indexOf("°") > 0,
        "the panel showed radians where it must show degrees");
    });

  await test("Copy writes the server's snippet and shows exactly what it wrote",
    async () => {
      const copyButton = document.getElementById("btnGhostCopy");
      const snippet = document.getElementById("sceneSnippet");
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
      assertEqual(document.getElementById("sceneToast").hidden, false,
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
      const copyButton = document.getElementById("btnGhostCopy");
      const verdict = document.getElementById("sceneVerdict");
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

  await test("Reset ghost puts the ghost back and takes Copy away", async () => {
    document.getElementById("btnGhostReset").click();
    await settle(4);
    assertEqual(document.getElementById("btnGhostCopy").hidden, true,
      "Copy survived a reset");
    assertEqual(document.getElementById("sceneVerdict").textContent, "",
      "the verdict survived a reset");
    assertEqual(document.getElementById("sceneSnippet").hidden, true,
      "a snippet describing the old pose survived a reset");
  });

  window.fetch = realFetch;
  window.scrollBy = realScrollBy;
}
