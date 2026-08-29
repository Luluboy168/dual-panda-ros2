// [THROWAWAY] Session C standalone browser interaction suite.

import {mountSolidScene} from "../../../web/ghost/scene.js";
import {shortestAngleDiff} from "../../../web/ghost/drag.js";
import {loadModel} from "./urdf.js";

const INITIAL = [0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4];
const LOWER = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
const UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
const JOG_STEP = 0.034906585;

function measuredMap(arm1 = INITIAL, arm2 = INITIAL) {
  const result = {};
  for (const [armId, values] of [["panda1", arm1], ["panda2", arm2]]) {
    values.forEach((value, index) => {
      result[`${armId}_joint${index + 1}`] = value;
    });
  }
  return result;
}

function matrixOf(object) {
  object.updateWorldMatrix(true, false);
  return JSON.stringify([...object.matrixWorld.elements]);
}

function pointerEvent(type, point, pointerId = 41) {
  return new PointerEvent(type, {
    bubbles: true,
    cancelable: true,
    pointerId,
    pointerType: "mouse",
    isPrimary: true,
    button: type === "pointerdown" ? 0 : -1,
    buttons: type === "pointerup" || type === "pointercancel" ? 0 : 1,
    clientX: point.clientX,
    clientY: point.clientY,
  });
}

function dispatchKey(key, options = {}) {
  window.dispatchEvent(new KeyboardEvent("keydown", {
    bubbles: true,
    cancelable: true,
    key,
    ...options,
  }));
}

function startDrag(handle, armIndex, jointIndex, pointerId = 41) {
  handle.selectJoint(armIndex, jointIndex);
  const angle = handle.getGhost(armIndex)[jointIndex];
  const point = handle.dragController.testing.coordinatesForAngle(armIndex, jointIndex, angle);
  handle.renderer.domElement.dispatchEvent(pointerEvent("pointerdown", point, pointerId));
  return {angle, point, pointerId};
}

function dragTo(handle, armIndex, jointIndex, angle, pointerId = 41) {
  const point = handle.dragController.testing.coordinatesForAngle(armIndex, jointIndex, angle);
  handle.renderer.domElement.dispatchEvent(pointerEvent("pointermove", point, pointerId));
}

function endDrag(handle, point, pointerId = 41, type = "pointerup") {
  handle.renderer.domElement.dispatchEvent(pointerEvent(type, point, pointerId));
}

function setJoint(positions, jointIndex, value) {
  const result = [...positions];
  result[jointIndex] = value;
  return result;
}

export async function runDragCases({test, assert, assertEqual, assertNear, assertArrayNear}) {
  const {model, manifest} = await loadModel();
  const host = document.createElement("div");
  host.style.cssText = "display:grid; grid-template-columns:800px 340px; width:1140px";
  const container = document.createElement("div");
  container.style.cssText = "width:800px; height:600px";
  const side = document.createElement("aside");
  const panel = document.createElement("div");
  const resetButton = document.createElement("button");
  resetButton.type = "button";
  resetButton.textContent = "Reset ghost (R)";
  const deltaOutput = document.createElement("output");
  const applyButton = document.createElement("button");
  applyButton.type = "button";
  applyButton.textContent = "Apply — sends target, arm ramps at the configured slew";
  side.append(panel, resetButton, deltaOutput, applyButton);
  host.append(container, side);
  document.body.append(host);

  const assetBase = new URL("../../../web/assets/", import.meta.url);
  const handle = await mountSolidScene(container, {
    model,
    manifest,
    assetBase,
    assetFetch: (url) => window.fetch(url),
    initialArm: 1,
    jogStepRad: JOG_STEP,
    ui: {panel, resetButton, deltaOutput, applyButton},
  });
  handle.setMeasured(measuredMap());

  await test("ghosts are translucent, tinted, geometry-sharing, and selected-arm-only", () => {
    assertEqual(handle.ghostLinkMeshes.length, 16, "one eight-link ghost per arm is required");
    for (let index = 0; index < 8; index += 1) {
      const solid = handle.linkMeshes.find(
        (mesh) => mesh.userData.linkName === `panda1_link${index}`,
      );
      const arm1 = handle.ghostLinkMeshes.find(
        (mesh) => mesh.userData.linkName === `panda1_link${index}`,
      );
      const arm2 = handle.ghostLinkMeshes.find(
        (mesh) => mesh.userData.linkName === `panda2_link${index}`,
      );
      assertEqual(arm1.geometry, solid.geometry, `arm 1 link${index} geometry is not shared`);
      assertEqual(arm2.geometry, solid.geometry, `arm 2 link${index} geometry is not shared`);
      for (const material of arm1.material) {
        assertNear(material.opacity, 0.35, 1e-12, "ghost opacity");
        assert(material.transparent, "ghost material must be transparent");
        assertEqual(material.depthWrite, false, "ghost must not write depth");
        assertEqual(material.side, THREE.FrontSide, "ghost must use FrontSide");
      }
      assertEqual(arm1.renderOrder, 10, "ghost must render after the solid");
      assert(
        arm1.material[0].color.getHex() !== arm2.material[0].color.getHex(),
        "per-arm ghost tints are indistinguishable",
      );
    }
    assert(handle.ghostGraph.linkObjects.get("panda1_link0").visible, "selected ghost hidden");
    assert(!handle.ghostGraph.linkObjects.get("panda2_link0").visible, "unselected ghost visible");
  });

  await test("accessible panel contains seven rows per arm and honest Stage 5 controls", async () => {
    assertEqual(panel.querySelectorAll('[data-arm-panel="1"] .joint-row').length, 7);
    assertEqual(panel.querySelectorAll('[data-arm-panel="2"] .joint-row').length, 7);
    assertEqual(panel.querySelectorAll("input[type=number][aria-label]").length, 14,
      "each joint needs a labelled numeric input");
    assertEqual(panel.querySelectorAll(".ghost-visibility input[type=checkbox]").length, 2,
      "each arm needs a labelled visibility control");
    assertEqual(
      applyButton.textContent,
      "Apply — sends target, arm ramps at the configured slew",
    );
    assert(applyButton.disabled, "Stage 5 Apply placeholder must be disabled");
    const page = await (await fetch(new URL("../../../web/index.html", import.meta.url))).text();
    assert(
      page.includes("PROTOTYPE — no robot connection. Apply prints the payload."),
      "prototype honesty banner is missing",
    );
    assert(page.includes("Joint state stale —"), "stale-age honesty message is missing");
    assert(!/Cartesian|end-effector|\bIK\b/.test(page), "out-of-scope Cartesian UI leaked in");
  });

  await test("list, link mapping, and neighbour handles share joint selection", () => {
    panel.querySelector('[data-select-joint="1:2"]').click();
    assertEqual(handle.dragController.testing.selectedArm, 1);
    assertEqual(handle.dragController.testing.selectedJoint, 2);
    assertEqual(
      JSON.stringify(handle.dragController.testing.selectionForLink("panda2_link6")),
      JSON.stringify({armIndex: 2, jointIndex: 5}),
    );
    assertEqual(handle.dragController.testing.selectionForLink("panda1_link0"), null);
    const selected = handle.dragController.testing.handles.get("1:2");
    const previous = handle.dragController.testing.handles.get("1:1");
    const next = handle.dragController.testing.handles.get("1:3");
    assert(selected.group.visible, "selected handle hidden");
    assert(previous.group.visible && next.group.visible, "neighbour handles hidden");
    assert(previous.materials.reachable.opacity < selected.materials.reachable.opacity,
      "neighbour handle is not faint");

    const neighbourAngle = (LOWER[3] + UPPER[3]) / 2;
    const neighbourPoint = handle.dragController.testing.coordinatesForAngle(1, 3, neighbourAngle);
    handle.renderer.domElement.dispatchEvent(pointerEvent("pointerdown", neighbourPoint, 45));
    assertEqual(handle.dragController.testing.selectedJoint, 3,
      "synthetic pointer selection on neighbour handle did not use selectJoint");

    dispatchKey("Escape");
    const link7 = handle.linkMeshes.find(
      (mesh) => mesh.userData.linkName === "panda2_link7",
    );
    const linkPoint = handle.dragController.testing.coordinatesForObject(link7);
    handle.renderer.domElement.dispatchEvent(pointerEvent("pointerdown", linkPoint, 46));
    assertEqual(handle.dragController.testing.selectedArm, 2,
      "synthetic link-mesh pick did not select its arm");
    assert(handle.dragController.testing.selectedJoint !== null,
      "synthetic link-mesh pick did not select a proximal joint");
    assert(!handle.dragController.testing.dragging, "solid link mesh became draggable");
  });

  await test("joint-plane handle carries fence arc, tones, measured tick, and knob", () => {
    handle.setFence(1, {
      lower: LOWER.map((value) => value + 0.1),
      upper: UPPER.map((value) => value - 0.1),
      source: "session",
    });
    handle.selectJoint(1, 3);
    handle.setGhost(1, setJoint(INITIAL, 3, -1.3));
    const arc = handle.dragController.testing.handles.get("1:3");
    assertEqual(arc.reachable.geometry.type, "TorusGeometry", "reachable arc is not toroidal");
    assertNear(arc.reachable.userData.arcStart, LOWER[3] + 0.1, 1e-12);
    assertNear(arc.reachable.userData.arcEnd, UPPER[3] - 0.1, 1e-12);
    assertEqual(arc.remainders.length, 2, "out-of-fence remainder is not shown at both ends");
    assert(arc.materials.remainder.opacity < arc.materials.reachable.opacity,
      "out-of-fence tone is not faint");
    assert(arc.pending && arc.pending.material === arc.materials.pending,
      "pending measured-to-ghost segment missing");
    assert(arc.tick.visible && arc.knob.visible, "measured tick or ghost knob missing");
    assertNear(arc.group.position.length(), 0, 1e-12, "handle is not centred on joint origin");
    const localAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(arc.group.quaternion);
    const jointAxis = new THREE.Vector3(...model.joints.find(
      (joint) => joint.name === "panda1_joint4",
    ).axis).normalize();
    assertNear(localAxis.dot(jointAxis), 1, 1e-12, "handle normal does not match joint axis");
    const childMesh = handle.ghostLinkMeshes.find(
      (mesh) => mesh.userData.linkName === "panda1_link4",
    );
    const expectedRadius = Math.min(
      0.19,
      Math.max(0.075, childMesh.geometry.boundingSphere.radius * 1.1),
    );
    assertNear(arc.radius, expectedRadius, 1e-12, "handle radius ignores child link bounds");
    handle.setFence(1, {lower: LOWER, upper: UPPER, source: "urdf"});
  });

  // Stage 5's done-criterion names BOTH one-sided joints, but only joint 4's arc geometry was
  // machine-asserted; joint 6 had a clamp test and nothing that pins its arc. Joint 6 is the
  // opposite-handed case -- its span runs from a hard stop just below zero far into positive
  // rotation -- so a sign or endpoint mix-up that survives joint 4 would show up here.
  await test("one-sided joint 6 carries the same fence arc geometry as joint 4", () => {
    handle.setFence(1, {
      lower: LOWER.map((value) => value + 0.1),
      upper: UPPER.map((value) => value - 0.1),
      source: "session",
    });
    handle.selectJoint(1, 5);
    handle.setGhost(1, setJoint(INITIAL, 5, 1.4));
    const arc = handle.dragController.testing.handles.get("1:5");
    assertEqual(arc.reachable.geometry.type, "TorusGeometry", "reachable arc is not toroidal");
    assertNear(arc.reachable.userData.arcStart, LOWER[5] + 0.1, 1e-12);
    assertNear(arc.reachable.userData.arcEnd, UPPER[5] - 0.1, 1e-12);
    assert(
      arc.reachable.userData.arcEnd > arc.reachable.userData.arcStart,
      "joint 6 arc runs backwards",
    );
    assertEqual(arc.remainders.length, 2, "out-of-fence remainder is not shown at both ends");
    assert(arc.materials.remainder.opacity < arc.materials.reachable.opacity,
      "out-of-fence tone is not faint");
    assert(arc.pending && arc.pending.material === arc.materials.pending,
      "pending measured-to-ghost segment missing");
    assert(arc.tick.visible && arc.knob.visible, "measured tick or ghost knob missing");
    assertNear(arc.group.position.length(), 0, 1e-12, "handle is not centred on joint origin");
    const localAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(arc.group.quaternion);
    const jointAxis = new THREE.Vector3(...model.joints.find(
      (joint) => joint.name === "panda1_joint6",
    ).axis).normalize();
    assertNear(localAxis.dot(jointAxis), 1, 1e-12, "handle normal does not match joint axis");
    const childMesh = handle.ghostLinkMeshes.find(
      (mesh) => mesh.userData.linkName === "panda1_link6",
    );
    const expectedRadius = Math.min(
      0.19,
      Math.max(0.075, childMesh.geometry.boundingSphere.radius * 1.1),
    );
    assertNear(arc.radius, expectedRadius, 1e-12, "handle radius ignores child link bounds");
    handle.setFence(1, {lower: LOWER, upper: UPPER, source: "urdf"});
  });

  await test("known pointer-plane drag changes joint 1 by the requested angle", () => {
    handle.setGhost(1, INITIAL);
    let capturedPointer = null;
    const nativeCapture = handle.renderer.domElement.setPointerCapture.bind(
      handle.renderer.domElement,
    );
    handle.renderer.domElement.setPointerCapture = (pointerId) => {
      capturedPointer = pointerId;
      nativeCapture(pointerId);
    };
    const active = startDrag(handle, 1, 0, 51);
    assert(handle.dragController.testing.dragging, "pointerdown on knob did not begin drag");
    assert(!handle.orbitControls.enabled, "camera was not suppressed during joint drag");
    assertEqual(capturedPointer, 51, "joint drag did not request pointer capture");
    dragTo(handle, 1, 0, active.angle + 0.34, 51);
    endDrag(handle, handle.dragController.testing.coordinatesForAngle(1, 0, active.angle + 0.34), 51);
    assertNear(handle.getGhost(1)[0], 0.34, 1e-3, "joint-plane drag delta");
    assert(handle.orbitControls.enabled, "camera did not resume after pointerup");
    handle.renderer.domElement.setPointerCapture = nativeCapture;
  });

  await test("near-axis pointer frames are ignored", () => {
    handle.setGhost(1, INITIAL);
    const active = startDrag(handle, 1, 0, 52);
    const centre = handle.dragController.testing.coordinatesForAngle(1, 0, 0, 0);
    handle.renderer.domElement.dispatchEvent(pointerEvent("pointermove", centre, 52));
    assertNear(handle.getGhost(1)[0], 0, 1e-12, "degenerate frame changed the ghost");
    endDrag(handle, active.point, 52, "pointercancel");
    assert(!handle.dragController.testing.dragging, "pointercancel left drag active");
    startDrag(handle, 1, 0, 521);
    window.dispatchEvent(new Event("blur"));
    assert(!handle.dragController.testing.dragging, "window blur left drag active");
    assert(handle.orbitControls.enabled, "window blur did not restore camera controls");
  });

  await test("joint 4 clamps exactly at -0.0698 through 200 pointer moves", () => {
    const start = setJoint(INITIAL, 3, -2.0);
    handle.setGhost(1, start);
    const active = startDrag(handle, 1, 3, 53);
    for (let index = 1; index <= 200; index += 1) {
      dragTo(handle, 1, 3, active.angle + index * 0.02, 53);
      assert(handle.getGhost(1)[3] <= -0.0698, `joint 4 exceeded upper fence at event ${index}`);
    }
    endDrag(handle, active.point, 53);
    assertNear(handle.getGhost(1)[3], -0.0698, 1e-12);
    const row = handle.dragController.testing.rows.get("1:3");
    assertEqual(row.status.textContent, "at upper limit");
    assert(row.root.classList.contains("limit-pulse-upper"), "upper arc end did not pulse");
    const knob = handle.dragController.testing.handles.get("1:3").knob;
    assertNear(Math.atan2(knob.position.y, knob.position.x), -0.0698, 1e-12,
      "knob rendered beyond the clamped upper limit");
  });

  await test("joint 6 clamps exactly at -0.0175 toward negative", () => {
    handle.setGhost(1, setJoint(INITIAL, 5, 1.0));
    const active = startDrag(handle, 1, 5, 54);
    for (let index = 1; index <= 160; index += 1) {
      dragTo(handle, 1, 5, active.angle - index * 0.015, 54);
    }
    endDrag(handle, active.point, 54);
    assertNear(handle.getGhost(1)[5], -0.0175, 1e-12);
    assertEqual(handle.dragController.testing.rows.get("1:5").status.textContent, "at lower limit");
  });

  await test("400 seam-crossing deltas totalling 3pi accumulate without sign snap", () => {
    handle.setGhost(1, INITIAL);
    const active = startDrag(handle, 1, 0, 55);
    for (let index = 1; index <= 400; index += 1) {
      dragTo(handle, 1, 0, active.angle + index * (3 * Math.PI / 400), 55);
      assert(handle.getGhost(1)[0] >= 0, `wrap accumulator went negative at event ${index}`);
    }
    endDrag(handle, active.point, 55);
    assertNear(handle.getGhost(1)[0], 2.8973, 1e-12);
    assertNear(shortestAngleDiff(-Math.PI + 0.01, Math.PI - 0.01), 0.02, 1e-12);
  });

  await test("setFence narrows every clamp and rejects a wider-than-URDF fence", () => {
    const narrowUpper = UPPER.map((value) => value - 0.1);
    handle.setFence(1, {lower: LOWER, upper: narrowUpper, source: "session"});
    handle.setGhost(1, UPPER);
    assertArrayNear(handle.getGhost(1), narrowUpper, 1e-12, "narrow fence clamp");
    let threw = false;
    try {
      const widerLower = [...LOWER];
      widerLower[0] -= 0.001;
      handle.setFence(1, {lower: widerLower, upper: UPPER, source: "session"});
    } catch (error) {
      threw = error instanceof RangeError;
    }
    assert(threw, "wider-than-URDF fence must throw");
    assertEqual(
      handle.dragController.testing.handles.get("1:0").group.userData.fenceSource,
      "session",
      "rendered fence lost its session provenance",
    );
    threw = false;
    try {
      handle.setFence(1, {lower: LOWER, upper: narrowUpper, source: "urdf"});
    } catch (error) {
      threw = error instanceof RangeError;
    }
    assert(threw, "a narrowed fence must not claim URDF provenance");
    threw = false;
    try {
      const almostUrdf = [...LOWER];
      almostUrdf[0] -= 5e-11;
      handle.setFence(1, {lower: almostUrdf, upper: UPPER, source: "session"});
    } catch (error) {
      threw = error instanceof RangeError;
    }
    assert(threw, "even a sub-epsilon wider-than-URDF fence must throw");
    handle.setFence(1, {lower: LOWER, upper: UPPER, source: "urdf"});
  });

  await test("zero-width session fence remains stable and exactly clamped", () => {
    const lower = [...LOWER];
    const upper = [...UPPER];
    lower[0] = 0.25;
    upper[0] = 0.25;
    handle.setFence(1, {lower, upper, source: "session"});
    handle.setGhost(1, setJoint(INITIAL, 0, -2));
    assertNear(handle.getGhost(1)[0], 0.25, 1e-12, "zero-width clamp changed");
    const fixed = handle.dragController.testing.handles.get("1:0");
    assertEqual(fixed.reachable, null, "zero-width reachable arc should have no fake span");
    assertNear(fixed.endpoints.lower.position.distanceTo(fixed.endpoints.upper.position), 0, 1e-12,
      "zero-width endpoints diverged");
    handle.setMeasured({panda1_joint1: 0.2});
    assert(fixed.pending.visible, "pending segment vanished at a zero-width fence");
    handle.setFence(1, {lower: LOWER, upper: UPPER, source: "urdf"});
    handle.setMeasured(measuredMap());
  });

  await test("measured samples validate atomically before either arm changes", () => {
    const before1 = handle.getMeasured(1);
    const before2 = handle.getMeasured(2);
    let threw = false;
    try {
      handle.dragController.setMeasured({
        panda1_joint1: before1[0] + 0.1,
        panda2_joint7: Number.NaN,
      });
    } catch (error) {
      threw = error instanceof TypeError;
    }
    assert(threw, "non-finite measured joint must reject the whole sample");
    assertArrayNear(handle.getMeasured(1), before1, 0, "arm 1 partially accepted bad sample");
    assertArrayNear(handle.getMeasured(2), before2, 0, "arm 2 partially accepted bad sample");
  });

  await test("keyboard nudges, fine step, plus/minus, and Tab share the clamp path", () => {
    handle.setGhost(1, INITIAL);
    handle.selectJoint(1, 0);
    for (let index = 0; index < 10; index += 1) {
      dispatchKey("ArrowRight");
    }
    assertNear(handle.getGhost(1)[0], 10 * JOG_STEP, 1e-12, "ten 2-degree nudges");
    dispatchKey("+", {shiftKey: true});
    assertNear(handle.getGhost(1)[0], 10 * JOG_STEP + Math.PI / 360, 1e-12, "fine nudge");
    dispatchKey("-");
    assertNear(handle.getGhost(1)[0], 9 * JOG_STEP + Math.PI / 360, 1e-12, "minus nudge");
    dispatchKey("Tab");
    assertEqual(handle.dragController.testing.selectedJoint, 1, "Tab did not select next joint");
    dispatchKey("Tab", {shiftKey: true});
    assertEqual(handle.dragController.testing.selectedJoint, 0, "Shift+Tab did not select previous");
  });

  await test("numeric degrees, radian tooltip, Reset button, and R stay honest", () => {
    const input = handle.dragController.testing.rows.get("1:0").input;
    input.value = "30";
    input.dispatchEvent(new Event("change", {bubbles: true}));
    assertNear(handle.getGhost(1)[0], Math.PI / 6, 1e-12, "degree field did not store radians");
    assertEqual(input.title, `${(Math.PI / 6).toFixed(4)} rad`, "radian tooltip is not four decimals");
    resetButton.click();
    assertArrayNear(handle.getGhost(1), INITIAL, 1e-12, "Reset button did not sync measured");
    handle.setGhost(1, setJoint(INITIAL, 0, 0.4));
    dispatchKey("R");
    assertArrayNear(handle.getGhost(1), INITIAL, 1e-12, "R did not sync measured");
    handle.selectJoint(1, 1);
    const beforeEmpty = handle.getGhost(1);
    input.value = "";
    input.dispatchEvent(new Event("change", {bubbles: true}));
    assertArrayNear(handle.getGhost(1), beforeEmpty, 0, "empty numeric input changed ghost");
    assertEqual(handle.dragController.testing.selectedJoint, 1,
      "empty numeric input changed selection");
    assertEqual(input.value, "", "empty numeric input was not left inert");
  });

  await test("setEnabled(false) blocks pointer, keyboard, numeric input, reset, and Apply", () => {
    handle.setGhost(1, setJoint(INITIAL, 0, 0.2));
    handle.selectJoint(1, 0);
    handle.setEnabled(false);
    const before = handle.getGhost(1);
    const coords = handle.dragController.testing.coordinatesForAngle(1, 0, before[0]);
    handle.renderer.domElement.dispatchEvent(pointerEvent("pointerdown", coords, 56));
    dispatchKey("ArrowRight");
    const input = handle.dragController.testing.rows.get("1:0").input;
    input.value = "50";
    input.dispatchEvent(new Event("change", {bubbles: true}));
    resetButton.click();
    assertArrayNear(handle.getGhost(1), before, 1e-12, "disabled input changed ghost");
    assert(!handle.dragController.testing.dragging, "disabled pointerdown began drag");
    assert(applyButton.disabled, "Apply enabled in read-only mode");
    assertEqual(applyButton.dataset.disabledReason, "read-only");
    handle.setEnabled(true);
  });

  await test("solid and ghost state remain isolated in both directions", () => {
    handle.setGhost(1, setJoint(INITIAL, 0, 0.45));
    const ghostBefore = handle.getGhost(1);
    handle.setMeasured({panda1_joint1: -0.35});
    assertArrayNear(handle.getGhost(1), ghostBefore, 1e-12, "measured update moved ghost");
    const solidBefore = matrixOf(handle.linkObjects.get("panda1_link7"));
    const active = startDrag(handle, 1, 0, 57);
    dragTo(handle, 1, 0, active.angle + 0.2, 57);
    endDrag(handle, active.point, 57);
    assertEqual(matrixOf(handle.linkObjects.get("panda1_link7")), solidBefore,
      "ghost drag moved solid robot");
  });

  await test("Escape restores an in-flight drag exactly, then deselects", () => {
    handle.setGhost(1, setJoint(INITIAL, 0, 0.23123456789));
    const before = handle.getGhost(1)[0];
    const active = startDrag(handle, 1, 0, 58);
    dragTo(handle, 1, 0, active.angle + 0.45, 58);
    assert(handle.getGhost(1)[0] !== before, "drag did not change before Escape");
    dispatchKey("Escape");
    assertEqual(handle.getGhost(1)[0], before, "Escape did not restore q0 exactly");
    assert(!handle.dragController.testing.dragging, "Escape left drag active");
    assert(handle.orbitControls.enabled, "Escape did not restore camera controls");
    dispatchKey("Escape");
    assertEqual(handle.dragController.testing.selectedJoint, null, "second Escape did not deselect");
  });

  await test("visibility, arm selection, programmatic setGhost, and sync methods work", () => {
    handle.selectArm(2);
    assert(!handle.ghostGraph.linkObjects.get("panda1_link0").visible, "old arm ghost remained");
    assert(handle.ghostGraph.linkObjects.get("panda2_link0").visible, "new arm ghost hidden");
    handle.setGhostVisible(1, true);
    assert(handle.ghostGraph.linkObjects.get("panda1_link0").visible, "visibility hook failed");
    handle.setGhostVisible(2, true);
    handle.selectArm(1);
    handle.selectArm(2);
    assert(handle.ghostGraph.linkObjects.get("panda1_link0").visible,
      "explicit arm 1 visibility was lost on selection");
    assert(handle.ghostGraph.linkObjects.get("panda2_link0").visible,
      "explicit arm 2 visibility was lost on selection");
    handle.setGhost(2, new Array(7).fill(100));
    assertArrayNear(handle.getGhost(2), UPPER, 1e-12, "programmatic setGhost did not clamp");
    handle.syncGhostToMeasured(2);
    assertArrayNear(handle.getGhost(2), INITIAL, 1e-12, "sync hook did not restore measured");
  });

  await test("stale state greys only solid, disables Apply, and reports large delta", () => {
    handle.selectArm(1);
    handle.setGhost(1, setJoint(handle.getMeasured(1), 0, handle.getMeasured(1)[0] + Math.PI / 3));
    assertEqual(deltaOutput.textContent, "60.0°", "large ghost delta is not conspicuous");
    const solidMaterial = handle.linkMeshes[0].material[0];
    const liveOpacity = solidMaterial.opacity;
    const liveColor = solidMaterial.color.getHex();
    const ghostOpacity = handle.ghostLinkMeshes[0].material[0].opacity;
    handle.setStale(true);
    assert(handle.stale && container.classList.contains("joint-state-stale"), "stale hook absent");
    assertNear(solidMaterial.opacity, liveOpacity, 1e-12, "stale solid became translucent");
    assertEqual(solidMaterial.transparent, false, "stale solid stopped being solid");
    assert(solidMaterial.color.getHex() !== liveColor, "stale solid color was not greyed");
    assertNear(handle.ghostLinkMeshes[0].material[0].opacity, ghostOpacity, 1e-12,
      "stale state altered ghost appearance");
    assert(applyButton.disabled, "stale state enabled Apply");
    assertEqual(applyButton.dataset.disabledReason, "stale");
    handle.setStale(false);
    assertNear(solidMaterial.opacity, liveOpacity, 1e-12, "live solid opacity was not restored");
  });

  await test("pending arc reuses one fixed BufferGeometry under update stress", () => {
    handle.setGhostVisible(1, true);
    handle.setGhost(1, INITIAL);
    const stats = handle.dragController.testing.allocationStats;
    const allocationsBefore = stats.pendingBufferGeometries;
    const pending = handle.dragController.testing.handles.get("1:0").pending;
    const geometry = pending.geometry;
    const NativeTorusGeometry = THREE.TorusGeometry;
    let hotPathTorusAllocations = 0;
    THREE.TorusGeometry = function countedTorusGeometry(...args) {
      hotPathTorusAllocations += 1;
      return new NativeTorusGeometry(...args);
    };
    THREE.TorusGeometry.prototype = NativeTorusGeometry.prototype;
    try {
      const active = startDrag(handle, 1, 0, 591);
      for (let index = 1; index <= 600; index += 1) {
        dragTo(handle, 1, 0, active.angle + 0.4 * Math.sin(index * 0.07), 591);
      }
      endDrag(handle, active.point, 591);
      for (let index = 0; index < 400; index += 1) {
        handle.setMeasured({panda1_joint1: 0.2 * Math.sin(index * 0.11)});
      }
    } finally {
      THREE.TorusGeometry = NativeTorusGeometry;
    }
    assertEqual(pending.geometry, geometry, "pending arc geometry identity changed");
    assertEqual(hotPathTorusAllocations, 0,
      "pointer/state hot path allocated TorusGeometry");
    assertEqual(stats.pendingBufferGeometries, allocationsBefore,
      "pointer/state hot path allocated pending BufferGeometry");
    assertEqual(geometry.userData.fixedCapacity, 96 * 6 * 6,
      "pending arc buffer capacity changed under stress");
  });

  await test("dispose is idempotent and removes listeners, RAF, canvas, and captures", () => {
    handle.selectJoint(1, 0);
    const active = startDrag(handle, 1, 0, 59);
    assert(handle.listenerCount > 0 && handle.animationActive, "mounted scene lacks tracked resources");
    handle.dispose();
    handle.dispose();
    assertEqual(handle.dragController.listenerCount, 0, "drag listeners leaked");
    assertEqual(handle.orbitControls.listenerCount, 0, "orbit listeners leaked");
    assertEqual(handle.listenerCount, 0, "scene listener count leaked");
    assert(!handle.animationActive, "animation frame remains active");
    assert(!handle.renderer.domElement.isConnected, "canvas remains attached");
    assert(!handle.dragController.testing.dragging, "active pointer capture survived dispose");
    endDrag(handle, active.point, 59);
  });

  host.remove();
}
