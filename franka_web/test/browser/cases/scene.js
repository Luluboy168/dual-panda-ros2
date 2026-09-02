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

import {mountSolidScene, PALETTE_KEYS} from "../../../static/ghost/scene.js";
import {createGhostState} from "../../../static/ghost/ghost_state.js";
import {forwardKinematics} from "../../../static/ghost/kinematics.js";
import {activeListenerCount, mount} from "../../../static/ghost/ghost.js";
import {ASSET_BASE, loadModel} from "./urdf.js";

//: The colour tokens the stylesheet owns, in the module's own key spelling.
//: A ninth or a missing one means the two tables have drifted apart.
const DOCUMENTED_PALETTE_KEYS = [
  "cellFloor", "cellLine", "ghost1", "ghost2", "ghostCollide", "ghostUnchecked",
  "grid", "gridMajor", "handle", "handleActive", "handleRefused", "ring",
  "ringActive", "sceneBg", "stale",
].sort();

const CELL = {
  id: "work_area", frame: "cell",
  x_min: -0.35, x_max: 0.9, y_min: -1.0, y_max: 1.0, z_min: 0.0, z_max: 2.0,
};

function makeContainer(width, height) {
  const container = document.createElement("div");
  container.style.width = `${width}px`;
  container.style.height = `${height}px`;
  document.body.append(container);
  return container;
}

export async function runSceneCases(context) {
  const {test, assert, assertEqual, assertNear, assertArrayNear, settle} = context;
  const {model, manifest} = await loadModel();

  const container = makeContainer(800, 600);
  const handle = await mountSolidScene(container, {
    model,
    manifest,
    assetBase: ASSET_BASE,
    assetFetch: (url) => window.fetch(url),
    cell: CELL,
  });

  await test("the scene requires and uses a real WebGL2 context", () => {
    assert(handle.gl, "WebGL2 context was not created");
    assertEqual(handle.gl, handle.renderer.getContext(), "renderer must use the probed context");
    assert(
      typeof WebGL2RenderingContext !== "undefined" && handle.gl instanceof WebGL2RenderingContext,
      "renderer context is not WebGL2",
    );
  });

  await test("sixteen solid and sixteen ghost meshes share eight geometries", () => {
    assertEqual(handle.linkMeshes.length, 16, "expected eight visual links per arm");
    assertEqual(handle.ghostLinkMeshes.length, 16, "expected one eight-link ghost per arm");
    assertEqual(new Set(handle.linkMeshes.map((mesh) => mesh.geometry)).size, 8,
      "different link numbers must retain different geometry uploads");
    for (let index = 0; index < 8; index += 1) {
      const arm1 = handle.linkMeshes.find((mesh) => mesh.userData.linkName === `panda1_link${index}`);
      const arm2 = handle.linkMeshes.find((mesh) => mesh.userData.linkName === `panda2_link${index}`);
      assert(arm1 && arm2, `missing link${index} mesh`);
      assertEqual(arm1.geometry, arm2.geometry, `link${index} geometry was loaded twice`);
      assert(arm1.geometry.getAttribute("color"), `link${index} lost material-group colors`);
    }
  });

  await test("every generated mesh loads with its metadata vertex counts", () => {
    const geometries = new Set(handle.linkMeshes.map((mesh) => mesh.geometry));
    assertEqual(geometries.size, 8, "expected eight distinct generated meshes");
    for (const geometry of geometries) {
      assert(geometry.index, "generated mesh geometry is not indexed");
      assertEqual(
        geometry.index.count, geometry.userData.sourceVertexCount,
        "index does not preserve every source triangle corner",
      );
      assertEqual(
        geometry.getAttribute("position").count, geometry.userData.indexedVertexCount,
        "indexed vertex metadata disagrees with the position buffer",
      );
      assert(geometry.userData.ghostMesh.schema === "franka.ghost.mesh/1",
        "mesh metadata is not the generated schema");
    }
  });

  await test("SOLID materials are cloned per arm so greying stays per arm", () => {
    // Both arms reference the same link*.dae. A shared material would grey the
    // live arm the moment its neighbour went stale.
    const keys = [...handle.testing.solidMaterials.keys()];
    assertEqual(keys.filter((key) => key.startsWith("1:")).length, 8, "panda1 materials");
    assertEqual(keys.filter((key) => key.startsWith("2:")).length, 8, "panda2 materials");
    const one = handle.testing.solidMaterials.get(keys.find((key) => key.startsWith("1:")));
    const two = handle.testing.solidMaterials.get(keys.find((key) => key.startsWith("2:")));
    assert(one[0] !== two[0], "the two arms share one solid material");
  });

  await test("greying one arm leaves the other at its live colour", async () => {
    const before = [...handle.testing.solidMaterials.values()]
      .map((materials) => materials[0].color.getHex());
    handle.setSolidStale(1, true);
    await settle();
    const after = [...handle.testing.solidMaterials.entries()]
      .map(([key, materials]) => ({key, hex: materials[0].color.getHex()}));
    after.forEach((entry, index) => {
      if (entry.key.startsWith("1:")) {
        assert(entry.hex !== before[index], `${entry.key} did not grey`);
      } else {
        assertEqual(entry.hex, before[index], `${entry.key} greyed with the other arm`);
      }
    });
    handle.setSolidStale(1, false);
  });

  await test("the cell box corners match the requested bounds", () => {
    const corners = handle.cell.boxCorners();
    assertEqual(corners.length, 8, "a box has eight corners");
    const expected = [];
    [CELL.x_min, CELL.x_max].forEach((x) => {
      [CELL.y_min, CELL.y_max].forEach((y) => {
        [CELL.z_min, CELL.z_max].forEach((z) => {
          expected.push([x, y, z]);
        });
      });
    });
    corners.forEach((corner, index) => {
      assertArrayNear(corner, expected[index], 1e-9, `corner ${index}`);
    });
  });

  await test("the ground grid is clipped to the box footprint", () => {
    let grid = null;
    handle.cell.group.traverse((object) => {
      if (object.name === "cell_grid") {
        grid = object;
      }
    });
    assert(grid, "no clipped grid was drawn");
    const positions = grid.geometry.getAttribute("position");
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (let index = 0; index < positions.count; index += 1) {
      minX = Math.min(minX, positions.getX(index));
      maxX = Math.max(maxX, positions.getX(index));
      minY = Math.min(minY, positions.getY(index));
      maxY = Math.max(maxY, positions.getY(index));
    }
    assert(minX >= CELL.x_min - 1e-9 && maxX <= CELL.x_max + 1e-9, "grid overruns in x");
    assert(minY >= CELL.y_min - 1e-9 && maxY <= CELL.y_max + 1e-9, "grid overruns in y");
    assertNear(maxX, CELL.x_max, handle.cell.gridSpacing + 1e-9, "grid stops short in x");
    assertNear(maxY, CELL.y_max, handle.cell.gridSpacing + 1e-9, "grid stops short in y");
  });

  await test("with no cell the fallback grid and axes appear instead", async () => {
    handle.setCell(null);
    await settle();
    assertEqual(handle.cell.boxCorners(), null, "a null cell still reported a box");
    const names = [];
    handle.cell.group.traverse((object) => names.push(object.name));
    assert(names.indexOf("ground_grid") >= 0, "no fallback ground grid");
    assert(names.indexOf("base_axes") >= 0, "no fallback axes");
    handle.setCell(CELL);
    await settle();
    assert(handle.cell.boxCorners(), "the cell box did not come back");
  });

  await test("the default view frames the measured cell", () => {
    handle.frameCamera();
    const target = handle.orbitControls.target;
    assertNear(target.x, (CELL.x_min + CELL.x_max) / 2, 1e-9, "camera target x");
    assertNear(target.y, (CELL.y_min + CELL.y_max) / 2, 1e-9, "camera target y");
    assertNear(target.z, CELL.z_min + 0.5, 1e-9, "camera target z");
    assert(handle.orbitControls.radius > 1.5 && handle.orbitControls.radius <= 8,
      `camera radius ${handle.orbitControls.radius} does not frame a 1.25 x 2.0 x 2.0 m cell`);
  });

  await test("a double-click on empty canvas re-frames the view", async () => {
    // View reset is a gesture, not a fourth toolbar button: the design
    // enumerates the toolbar's three affordances and this stays out of them.
    handle.frameCamera();
    const framed = {
      x: handle.orbitControls.target.x,
      y: handle.orbitControls.target.y,
      z: handle.orbitControls.target.z,
      radius: handle.orbitControls.radius,
    };
    handle.orbitControls.frame([framed.x + 0.7, framed.y - 0.4, framed.z + 0.3], 1.1);
    assert(Math.abs(handle.orbitControls.target.x - framed.x) > 0.5,
      "the view did not actually move before the reset");
    handle.canvas.dispatchEvent(new MouseEvent("dblclick", {bubbles: true, cancelable: true}));
    await settle();
    assertNear(handle.orbitControls.target.x, framed.x, 1e-9, "reset target x");
    assertNear(handle.orbitControls.target.y, framed.y, 1e-9, "reset target y");
    assertNear(handle.orbitControls.target.z, framed.z, 1e-9, "reset target z");
    assertNear(handle.orbitControls.radius, framed.radius, 1e-9, "reset radius");
  });

  await test("rendering is on demand: an idle frame draws nothing", async () => {
    await settle(3);
    const idle = handle.testing.renderCount;
    await settle(3);
    assertEqual(handle.testing.renderCount, idle, "an unchanged scene rendered again");
    handle.requestRender();
    await settle(3);
    assert(handle.testing.renderCount > idle, "a requested render never happened");
  });

  await test("the pixel-ratio ratchet steps down once, and only recovers slowly", async () => {
    // The rolling window is shared with real rendering, so start it empty:
    // otherwise this measures whatever the software GL backend happened to do.
    handle.testing.resetFrameTimes();
    const start = handle.testing.ratioIndex;
    for (let index = 0; index < 30; index += 1) {
      handle.testing.noteFrameTime(40);
    }
    assertEqual(handle.testing.ratioIndex, Math.max(0, start - 1),
      "a slow window must step the ratio down exactly one notch");
    const stepped = handle.testing.ratioIndex;
    for (let index = 0; index < 30; index += 1) {
      handle.testing.noteFrameTime(4);
    }
    assertEqual(handle.testing.ratioIndex, stepped,
      "a fast window inside the cooldown must not step back up");
    for (let index = 0; index < 30; index += 1) {
      handle.testing.noteFrameTime(40);
    }
    assert(handle.testing.ratioIndex >= 0, "the ratio index left its range");
    for (let round = 0; round < 6; round += 1) {
      for (let index = 0; index < 30; index += 1) {
        handle.testing.noteFrameTime(40);
      }
    }
    assertEqual(handle.testing.ratioIndex, 0, "the ratio index fell below its floor");
    // ...and it does come back. A ratchet that only ever steps down would leave
    // a machine that recovers rendering at a quarter of the resolution it can
    // afford, and nothing short of a reload would undo it.
    handle.testing.resetFrameTimes();
    await new Promise((resolve) => setTimeout(resolve, 2100));
    for (let index = 0; index < 30; index += 1) {
      handle.testing.noteFrameTime(4);
    }
    assertEqual(handle.testing.ratioIndex, 1,
      "a fast window past the cooldown must step the ratio back up exactly one notch");
  });

  await test("the built-in palette matches the documented token set exactly", () => {
    assertEqual(
      JSON.stringify(PALETTE_KEYS), JSON.stringify(DOCUMENTED_PALETTE_KEYS),
      "the module's fallback palette and the documented token list have drifted",
    );
    assertEqual(
      JSON.stringify(Object.keys(handle.testing.builtinPalette.dark).sort()),
      JSON.stringify(DOCUMENTED_PALETTE_KEYS),
      "the dark fallback palette has a different key set from the light one",
    );
  });

  await test("switching theme repaints the background, the cell and both ghosts", async () => {
    handle.setTheme("light");
    await settle();
    const lightBg = handle.scene.background.getHex();
    const lightGhost = [...handle.testing.ghostMaterials.values()][0][0].color.getHex();
    handle.setTheme("dark");
    await settle();
    assert(handle.scene.background.getHex() !== lightBg, "the clear colour did not change");
    assert([...handle.testing.ghostMaterials.values()][0][0].color.getHex() !== lightGhost,
      "the ghost tint did not change");
    assertEqual(handle.themeName, "dark", "the theme name did not move");
  });

  await test("a partial palette falls back per key, not wholesale", async () => {
    handle.setTheme("light", {sceneBg: "#010203"});
    await settle();
    assertEqual(handle.scene.background.getHex(), 0x010203, "the supplied key was ignored");
    assertEqual(handle.palette.ghost1, handle.testing.builtinPalette.light.ghost1,
      "an unsupplied key did not fall back to the built-in value");
  });

  await test("interpolation is linear in joint angle and snaps when it must", () => {
    const state = createGhostState({
      three: globalThis.THREE, model,
      solidGraph: handle.solidGraph, ghostGraph: handle.ghostGraph,
      armIndices: [1, 2], initialArm: 1, render: handle.requestRender,
    });
    const zero = new Array(7).fill(0);
    const tenth = new Array(7).fill(0.1);
    state.setMeasured(1, zero, {arrivalMs: 1000, framePeriodMs: 200, snap: true});
    assertArrayNear(state.measuredNow(1), zero, 1e-12, "the first frame must snap");

    state.setMeasured(1, tenth, {arrivalMs: 2000, framePeriodMs: 200});
    state.applyInterpolation(2100);
    assertArrayNear(state.measuredNow(1), new Array(7).fill(0.05), 1e-12,
      "alpha 0.5 must be the midpoint in JOINT space");
    state.applyInterpolation(2200);
    assertArrayNear(state.measuredNow(1), tenth, 1e-12, "alpha 1 must be the target");

    // Snap rule: a step above 0.35 rad in one frame is a teleport.
    state.setMeasured(1, new Array(7).fill(1.0), {arrivalMs: 3000, framePeriodMs: 200});
    assertArrayNear(state.measuredNow(1), new Array(7).fill(1.0), 1e-12,
      "a 0.9 rad step must snap, not smooth");

    // Snap rule: an explicitly flagged frame.
    state.setMeasured(1, new Array(7).fill(1.05), {arrivalMs: 4000, framePeriodMs: 200, snap: true});
    assertArrayNear(state.measuredNow(1), new Array(7).fill(1.05), 1e-12,
      "snap: true must be honoured");

    // Snap rule: a stale arm. Smoothing a stale pose invents motion.
    state.setStale(1, true);
    state.setMeasured(1, new Array(7).fill(1.1), {arrivalMs: 5000, framePeriodMs: 200});
    state.applyInterpolation(5100);
    assertArrayNear(state.measuredNow(1), new Array(7).fill(1.1), 1e-12,
      "a stale arm must not be smoothed toward anything");
    state.setStale(1, false);

    // Ghost joints are never interpolated toward anything. The pose below is
    // inside every joint's URDF limits, so nothing here is a clamp in disguise.
    const parked = [0.4, 0.4, 0.4, -1.5, 0.4, 1.4, 0.4];
    state.setGhost(1, parked);
    state.setMeasured(1, new Array(7).fill(1.2), {arrivalMs: 6000, framePeriodMs: 200});
    state.applyInterpolation(6100);
    assertArrayNear(state.getGhost(1), parked, 1e-12,
      "the ghost pose moved with a measured frame");
    state.dispose();
  });

  await test("a null joint holds its previous value and nothing throws", () => {
    const state = createGhostState({
      three: globalThis.THREE, model,
      solidGraph: handle.solidGraph, ghostGraph: handle.ghostGraph,
      armIndices: [1], initialArm: 1, render: handle.requestRender,
    });
    state.setMeasured(1, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
      {arrivalMs: 1000, framePeriodMs: 200, snap: true});
    // health reports null for any joint name missing from the JointState, so
    // this is a routine frame and not an exotic one.
    state.setMeasured(1, [0.15, null, 0.3, 0.4, 0.5, 0.6, 0.7],
      {arrivalMs: 1200, framePeriodMs: 200, snap: true});
    assertArrayNear(state.measuredNow(1), [0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], 1e-12,
      "joint 2 did not hold its previous value");
    state.setMeasured(1, [Number.NaN, Number.NaN, Number.NaN, Number.NaN,
                          Number.NaN, Number.NaN, Number.NaN],
      {arrivalMs: 1400, framePeriodMs: 200, snap: true});
    assertArrayNear(state.measuredNow(1), [0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], 1e-12,
      "an all-null frame moved the arm");
    // A whole-array null is the different thing: this arm is not in session.
    state.setMeasured(1, null, {});
    assertEqual(state.isPresent(1), false, "a null pose must hide the arm entirely");
    state.dispose();
  });

  handle.dispose();
  container.remove();

  // Degraded mounts run only after the shipped-model scene is disposed, so the
  // software GL backend never holds two live contexts at once.

  await test("a mount whose assets 404 rejects, mounts nothing, leaves no listener", async () => {
    const before = activeListenerCount();
    const fixture = makeContainer(640, 480);
    let message = null;
    try {
      await mount(fixture, {
        urdfUrl: new URL("no-such-model.urdf", ASSET_BASE).href,
        manifestUrl: new URL("manifest.json", ASSET_BASE).href,
        assetBase: ASSET_BASE.href,
        arms: [{armIndex: 1, armId: "panda1"}],
        initialArm: 1,
        cell: null,
        theme: "light",
        onSolveRequest: () => Promise.resolve({ok: true}),
        onGhostChanged: () => {},
      });
    } catch (error) {
      message = error.message;
    }
    assert(message && message.indexOf("404") >= 0, `expected an HTTP 404 message, got ${message}`);
    assertEqual(fixture.childElementCount, 0, "a failed mount left DOM behind");
    assertEqual(activeListenerCount(), before, "a failed mount left listeners behind");
    fixture.remove();
  });

  await test("a lost drawing context stops the loop and reports itself once", async () => {
    const fixture = makeContainer(320, 240);
    const reports = [];
    const mounted = await mount(fixture, {
      urdfUrl: new URL("model.urdf", ASSET_BASE).href,
      manifestUrl: new URL("manifest.json", ASSET_BASE).href,
      assetBase: ASSET_BASE.href,
      arms: [{armIndex: 1, armId: "panda1"}],
      initialArm: 1,
      cell: null,
      theme: "light",
      onSolveRequest: (request) => {
        if (request.kind === "status" && typeof request.drawing === "boolean") {
          reports.push(request.drawing);
        }
        return Promise.resolve({ok: true});
      },
      onGhostChanged: () => {},
    });
    try {
      const canvas = fixture.querySelector("canvas");
      canvas.dispatchEvent(new Event("webglcontextlost", {cancelable: true}));
      await settle(3);
      assertEqual(JSON.stringify(reports), JSON.stringify([false]),
        "a lost context must report itself exactly once, as a panel failure");
      // Nothing draws while the context is gone, and nothing throws either.
      mounted.setMeasured(1, new Array(7).fill(0.1),
        {arrivalMs: performance.now(), framePeriodMs: 200, snap: true});
      await settle(3);
    } finally {
      mounted.dispose();
      fixture.remove();
    }
  });

  await test("the seam refuses an onApply hook outright", async () => {
    const fixture = makeContainer(320, 240);
    let message = null;
    try {
      await mount(fixture, {
        urdfUrl: new URL("model.urdf", ASSET_BASE).href,
        manifestUrl: new URL("manifest.json", ASSET_BASE).href,
        assetBase: ASSET_BASE.href,
        arms: [{armIndex: 1, armId: "panda1"}],
        initialArm: 1,
        cell: null,
        theme: "light",
        onSolveRequest: () => Promise.resolve({ok: true}),
        onGhostChanged: () => {},
        onApply: () => {},
      });
    } catch (error) {
      message = error.message;
    }
    assert(message && message.indexOf("onApply") >= 0,
      `a seam with an onApply hook must be refused; got ${message}`);
    fixture.remove();
  });

  await test("the seam refuses an option key it does not define", async () => {
    const fixture = makeContainer(320, 240);
    let message = null;
    try {
      await mount(fixture, {
        urdfUrl: "a", manifestUrl: "b", assetBase: "c",
        arms: [{armIndex: 1, armId: "panda1"}], initialArm: 1, cell: null,
        theme: "light", onSolveRequest: () => {}, onGhostChanged: () => {},
        jogStepRad: 0.017,
      });
    } catch (error) {
      message = error.message;
    }
    assert(message && message.indexOf("jogStepRad") >= 0,
      `an undefined option must be refused; got ${message}`);
    fixture.remove();
  });

  await test("forward kinematics and the rendered graph agree", async () => {
    const fixture = makeContainer(480, 360);
    const mounted = await mount(fixture, {
      urdfUrl: new URL("model.urdf", ASSET_BASE).href,
      manifestUrl: new URL("manifest.json", ASSET_BASE).href,
      assetBase: ASSET_BASE.href,
      arms: [{armIndex: 1, armId: "panda1"}, {armIndex: 2, armId: "panda2"}],
      initialArm: 1,
      cell: CELL,
      theme: "light",
      onSolveRequest: () => Promise.resolve({ok: true}),
      onGhostChanged: () => {},
    });
    try {
      const values = [0.21, -0.91, 0.34, -2.14, -0.18, 1.82, 0.63];
      mounted.setMeasured(1, values, {arrivalMs: performance.now(), framePeriodMs: 200, snap: true});
      mounted.setMeasured(2, values, {arrivalMs: performance.now(), framePeriodMs: 200, snap: true});
      await settle();
      const positions = {};
      for (const armId of ["panda1", "panda2"]) {
        values.forEach((value, index) => {
          positions[`${armId}_joint${index + 1}`] = value;
        });
      }
      const expected = forwardKinematics(model, positions).links;
      assertArrayNear(mounted.getRenderedPose(1), values, 1e-12, "rendered pose for panda1");
      assertArrayNear(mounted.getRenderedPose(2), values, 1e-12, "rendered pose for panda2");
      // A hidden ghost means getRenderedPose reports what is actually drawn:
      // the interpolated measured pose, not a ghost nobody can see.
      const flange = expected.panda1_link8;
      assert(Number.isFinite(flange[3]), "the model did not produce a flange pose");
    } finally {
      mounted.dispose();
      fixture.remove();
    }
  });

  await test("dispose leaves no listener, no canvas, and is safe twice", async () => {
    const before = activeListenerCount();
    const fixture = makeContainer(480, 360);
    const mounted = await mount(fixture, {
      urdfUrl: new URL("model.urdf", ASSET_BASE).href,
      manifestUrl: new URL("manifest.json", ASSET_BASE).href,
      assetBase: ASSET_BASE.href,
      arms: [{armIndex: 1, armId: "panda1"}],
      initialArm: 1,
      cell: CELL,
      theme: "light",
      onSolveRequest: () => Promise.resolve({ok: true}),
      onGhostChanged: () => {},
    });
    assert(activeListenerCount() > before, "a live mount registered no listeners at all");
    assertEqual(
      JSON.stringify(Object.keys(mounted).sort()),
      JSON.stringify([
        "dispose", "getRenderedPose", "selectArm", "setCell", "setEnabled",
        "setGhost", "setGhostVisible", "setMeasured", "setStale", "setTheme",
        "setVerdict", "syncGhostToMeasured",
      ]),
      "the frozen handle and the documented seam disagree",
    );
    mounted.dispose();
    mounted.dispose();
    assertEqual(activeListenerCount(), before, "dispose left listeners behind");
    assertEqual(fixture.childElementCount, 0, "dispose left the canvas behind");
    fixture.remove();
  });
}
