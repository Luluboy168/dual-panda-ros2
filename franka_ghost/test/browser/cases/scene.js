// [THROWAWAY] Session C standalone browser render suite.

import {mountSolidScene} from "../../../web/ghost/scene.js";
import {forwardKinematics} from "../../../web/ghost/kinematics.js";
import {parseUrdf} from "../../../web/ghost/urdf.js";
import {loadModel} from "./urdf.js";

// Every rpy in the shipped dual URDF is single-axis (a +-pi/2 roll, or zero), so scene.js's
// three.js "ZYX" Euler order and the wrong "XYZ" order are numerically identical on it: mutating
// scene.js:218 to "XYZ" left the whole suite green. kinematics.js has the plan-mandated
// non-commuting case; this is the equivalent guard for the renderer, which is the durable half
// that moves into franka_web and will meet a compound rpy the first time a tilted base or garmi
// is loaded. Every rpy below has all three components non-zero, so Rz*Ry*Rx differs from Rx*Ry*Rz.
const COMPOUND_RPY = [
  [0.61, -0.43, 0.92],
  [-0.37, 0.55, -0.81],
  [0.88, 0.29, -0.64],
  [-0.52, -0.71, 0.33],
  [0.44, 0.96, 0.77],
  [-0.83, 0.38, -0.25],
  [0.19, -0.62, 1.07],
  [0.73, 0.51, -0.94],
  [-0.28, 0.84, 0.46],
];
const COMPOUND_XYZ = [
  [0.031, -0.017, 0.204],
  [-0.042, 0.061, 0.128],
  [0.019, 0.083, -0.037],
  [-0.075, -0.024, 0.166],
  [0.058, -0.049, 0.091],
  [0.013, 0.072, -0.055],
  [-0.066, 0.038, 0.147],
  [0.027, -0.081, 0.062],
  [0.049, 0.015, -0.033],
];
// The probe model carries no <visual>, so no mesh has to load for it.
const emptyManifest = {schema: "franka.ghost.manifest/1", meshes: {}};
const COMPOUND_AXES = [
  [0, 0, 1],
  [0, -1, 0],
  [1, 0, 0],
  [0, 1, 0],
  [-1, 0, 0],
  [0, 0, -1],
  [0.5773502691896258, 0.5773502691896258, 0.5773502691896258],
];

function compoundRpyUrdf() {
  const parts = ['<robot name="compound_rpy_probe">', '  <link name="base_link"/>'];
  for (const armId of ["panda1", "panda2"]) {
    for (let link = 0; link <= 8; link += 1) {
      parts.push(`  <link name="${armId}_link${link}"/>`);
    }
    const mountRpy = COMPOUND_RPY[8].join(" ");
    const mountXyz = COMPOUND_XYZ[8].join(" ");
    parts.push(
      `  <joint name="${armId}_mount" type="fixed">`,
      '    <parent link="base_link"/>',
      `    <child link="${armId}_link0"/>`,
      `    <origin xyz="${mountXyz}" rpy="${mountRpy}"/>`,
      "  </joint>",
    );
    for (let index = 0; index < 7; index += 1) {
      parts.push(
        `  <joint name="${armId}_joint${index + 1}" type="revolute">`,
        `    <parent link="${armId}_link${index}"/>`,
        `    <child link="${armId}_link${index + 1}"/>`,
        `    <origin xyz="${COMPOUND_XYZ[index].join(" ")}" rpy="${COMPOUND_RPY[index].join(" ")}"/>`,
        `    <axis xyz="${COMPOUND_AXES[index].join(" ")}"/>`,
        '    <limit lower="-3.0" upper="3.0" velocity="2.175" effort="87"/>',
        "  </joint>",
      );
    }
    parts.push(
      `  <joint name="${armId}_flange" type="fixed">`,
      `    <parent link="${armId}_link7"/>`,
      `    <child link="${armId}_link8"/>`,
      `    <origin xyz="${COMPOUND_XYZ[7].join(" ")}" rpy="${COMPOUND_RPY[7].join(" ")}"/>`,
      "  </joint>",
    );
  }
  parts.push("</robot>");
  return parts.join("\n");
}

function worldMatrix(link) {
  link.updateWorldMatrix(true, false);
  return [...link.matrixWorld.elements];
}

function translationDistance(left, right) {
  return Math.hypot(left[12] - right[12], left[13] - right[13], left[14] - right[14]);
}

function rowMajorWorldMatrix(link) {
  const elements = worldMatrix(link);
  const result = [];
  for (let row = 0; row < 4; row += 1) {
    for (let column = 0; column < 4; column += 1) {
      result.push(elements[column * 4 + row]);
    }
  }
  return result;
}

export async function runSceneCases({test, assert, assertEqual}) {
  const {model, manifest} = await loadModel();
  const container = document.createElement("div");
  container.style.cssText = "width: 800px; height: 600px";
  document.body.append(container);
  const assetBase = new URL("../../../web/assets/", import.meta.url);
  const handle = await mountSolidScene(container, {
    model,
    manifest,
    assetBase,
    assetFetch: (url) => window.fetch(url),
    jogStepRad: Math.PI / 90,
  });

  await test("solid scene requires and uses a real WebGL2 context", () => {
    assert(handle.gl, "WebGL2 context was not created");
    assertEqual(handle.gl, handle.renderer.getContext(), "renderer must use the probed context");
    assert(
      typeof WebGL2RenderingContext !== "undefined" && handle.gl instanceof WebGL2RenderingContext,
      "renderer context is not WebGL2",
    );
  });

  await test("scene contains 16 solid and 16 ghost shared-geometry link meshes", () => {
    assertEqual(handle.linkMeshes.length, 16, "expected eight visual links per arm");
    assertEqual(handle.ghostLinkMeshes.length, 16, "expected one eight-link ghost per arm");
    const solidMeshes = [];
    const ghostMeshes = [];
    handle.scene.traverse((object) => {
      if (object.isMesh && object.userData.role === "solid") {
        solidMeshes.push(object);
      } else if (object.isMesh && object.userData.role === "ghost") {
        ghostMeshes.push(object);
      }
    });
    assertEqual(solidMeshes.length, 16, "unexpected solid mesh count");
    assertEqual(ghostMeshes.length, 16, "unexpected ghost mesh count");
    assertEqual(new Set(handle.linkMeshes.map((mesh) => mesh.geometry)).size, 8,
      "different link numbers must retain different geometry uploads");
    for (let index = 0; index < 8; index += 1) {
      const arm1 = handle.linkMeshes.find((mesh) => mesh.userData.linkName === `panda1_link${index}`);
      const arm2 = handle.linkMeshes.find((mesh) => mesh.userData.linkName === `panda2_link${index}`);
      assert(arm1 && arm2, `missing link${index} mesh clone`);
      assertEqual(arm1.geometry, arm2.geometry, `link${index} geometry was loaded twice`);
      assertEqual(arm1.geometry.groups.length, 1, `link${index} is more than one draw call`);
      assert(arm1.geometry.getAttribute("color"), `link${index} lost material-group colors`);
      const ghost1 = handle.ghostLinkMeshes.find(
        (mesh) => mesh.userData.linkName === `panda1_link${index}`,
      );
      assertEqual(arm1.geometry, ghost1.geometry, `link${index} ghost duplicated geometry`);
    }
  });

  await test("real mesh triangles use indexed raster vertices without changing topology", () => {
    const geometries = new Set(handle.linkMeshes.map((mesh) => mesh.geometry));
    for (const geometry of geometries) {
      assert(geometry.index, "real mesh geometry is not indexed");
      assertEqual(
        geometry.index.count,
        geometry.userData.sourceVertexCount,
        "index does not preserve every source triangle corner",
      );
      assertEqual(
        geometry.getAttribute("position").count,
        geometry.userData.indexedVertexCount,
        "indexed vertex metadata disagrees with the position buffer",
      );
      assert(
        geometry.userData.indexedVertexCount < geometry.userData.sourceVertexCount * 0.25,
        "real mesh raster vertex count was not reduced by at least 75%",
      );
    }
  });

  await test("rendered link matrices agree with the pure kinematics model", () => {
    const positions = {};
    const values = [0.21, -0.91, 0.34, -2.14, -0.18, 1.82, 0.63];
    for (const armId of ["panda1", "panda2"]) {
      values.forEach((value, index) => {
        positions[`${armId}_joint${index + 1}`] = value;
      });
    }
    handle.setMeasured(positions);
    const expected = forwardKinematics(model, positions).links;
    for (const [name, link] of handle.linkObjects) {
      const actual = rowMajorWorldMatrix(link);
      actual.forEach((value, index) => {
        if (Math.abs(value - expected[name][index]) > 1e-10) {
          throw new Error(`${name} world matrix differs at ${index}`);
        }
      });
    }
  });

  await test("measured updates are isolated by arm", () => {
    const panda2Before = new Map();
    for (const [name, link] of handle.linkObjects) {
      if (name.startsWith("panda2_")) {
        panda2Before.set(name, worldMatrix(link));
      }
    }
    const link3Before = worldMatrix(handle.linkObjects.get("panda1_link3"));
    handle.setMeasured({panda1_joint2: -1.0});
    const link3After = worldMatrix(handle.linkObjects.get("panda1_link3"));
    assert(
      translationDistance(link3Before, link3After) > 0.001,
      "panda1_link3 moved by no more than 1 mm",
    );
    for (const [name, before] of panda2Before) {
      const after = worldMatrix(handle.linkObjects.get(name));
      assertEqual(JSON.stringify(after), JSON.stringify(before), `${name} changed with panda1`);
    }
  });

  await test("partial and unknown measured maps preserve existing joints", () => {
    handle.setMeasured({panda1_joint1: 0.4, panda1_joint2: -0.8});
    handle.setMeasured({panda1_joint2: -0.6});
    assertEqual(handle.jointNodes.get("panda1_joint1").userData.value, 0.4);
    assertEqual(handle.jointNodes.get("panda1_joint2").userData.value, -0.6);
    const before = worldMatrix(handle.linkObjects.get("panda1_link7"));
    handle.setMeasured({not_a_robot_joint: Infinity});
    const after = worldMatrix(handle.linkObjects.get("panda1_link7"));
    assertEqual(JSON.stringify(after), JSON.stringify(before), "unknown joint changed the scene");
  });

  handle.dispose();
  container.remove();

  // Mounted only after the shipped-model scene is disposed, so the software GL backend never
  // holds two live WebGL2 contexts at once.
  await test("scene rotation order matches fixed-axis kinematics on non-commuting rpy", async () => {
    const probeModel = parseUrdf(compoundRpyUrdf(), emptyManifest);
    const probeContainer = document.createElement("div");
    probeContainer.style.cssText = "width: 640px; height: 480px";
    document.body.append(probeContainer);
    const probeHandle = await mountSolidScene(probeContainer, {
      model: probeModel,
      manifest: emptyManifest,
      assetBase: new URL("../../../web/assets/", import.meta.url),
      assetFetch: (url) => window.fetch(url),
      jogStepRad: Math.PI / 90,
    });
    try {
      const positions = {};
      const values = [0.37, -1.21, 0.84, -2.03, 1.16, -0.58, 2.42];
      for (const armId of ["panda1", "panda2"]) {
        values.forEach((value, index) => {
          positions[`${armId}_joint${index + 1}`] = value;
        });
      }
      probeHandle.setMeasured(positions);

      const expected = forwardKinematics(probeModel, positions).links;
      let worstError = 0;
      for (const [name, link] of probeHandle.linkObjects) {
        const actual = rowMajorWorldMatrix(link);
        actual.forEach((value, index) => {
          worstError = Math.max(worstError, Math.abs(value - expected[name][index]));
        });
      }
      assert(
        worstError < 1e-10,
        `compound-rpy world matrices diverge from forwardKinematics by ${worstError};`
        + ' scene.js must compose URDF fixed-axis rpy as a three.js "ZYX" Euler',
      );

      // Prove the assertion above can actually fail: the same links composed in three's default
      // "XYZ" order must be visibly different, so a flipped rotation order cannot slip through.
      const flippedRoot = probeHandle.linkObjects.get("panda1_link1");
      const originNode = probeHandle.solidGraph.jointOrigins.get("panda1_joint1");
      const zyx = [...originNode.rotation.toArray()];
      originNode.rotation.set(zyx[0], zyx[1], zyx[2], "XYZ");
      probeHandle.scene.updateMatrixWorld(true);
      const flipped = rowMajorWorldMatrix(flippedRoot);
      const divergence = Math.max(
        ...flipped.map((value, index) => Math.abs(value - expected["panda1_link1"][index])),
      );
      originNode.rotation.set(zyx[0], zyx[1], zyx[2], "ZYX");
      probeHandle.scene.updateMatrixWorld(true);
      assert(
        divergence > 1e-3,
        `chosen rpy is commuting after all (XYZ differs by only ${divergence}); `
        + "the guard would not catch a flipped rotation order",
      );
    } finally {
      probeHandle.dispose();
      probeContainer.remove();
    }
  });
}
