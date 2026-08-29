// [DURABLE] Browser contract test retained when the ghost moves into franka_web.

import {parseUrdf, resolveMeshUri} from "../../../web/ghost/urdf.js";

const EXPECTED_LIMITS = [
  [-2.8973, 2.8973, 2.1750, 87],
  [-1.7628, 1.7628, 2.1750, 87],
  [-2.8973, 2.8973, 2.1750, 87],
  [-3.0718, -0.0698, 2.1750, 87],
  [-2.8973, 2.8973, 2.6100, 12],
  [-0.0175, 3.7525, 2.6100, 12],
  [-2.8973, 2.8973, 2.6100, 12],
];

async function loadModel() {
  const assetBase = new URL("../../../web/assets/", import.meta.url);
  const [urdfResponse, manifestResponse] = await Promise.all([
    fetch(new URL("model.urdf", assetBase)),
    fetch(new URL("manifest.json", assetBase)),
  ]);
  if (!urdfResponse.ok || !manifestResponse.ok) {
    throw new Error(
      `generated assets unavailable: URDF=${urdfResponse.status}, manifest=${manifestResponse.status}`,
    );
  }
  const [urdfText, manifest] = await Promise.all([
    urdfResponse.text(),
    manifestResponse.json(),
  ]);
  return {model: parseUrdf(urdfText, manifest), manifest};
}

export async function runUrdfCases({test, assert, assertEqual, assertNear}) {
  await test("URDF parser rejects malformed XML", () => {
    let threw = false;
    try {
      parseUrdf("<robot><link></robot>");
    } catch (_error) {
      threw = true;
    }
    assert(threw, "malformed XML must throw");
  });

  const {model, manifest} = await loadModel();

  await test("dual Panda joint counts and root", () => {
    assertEqual(model.joints.filter((joint) => joint.type === "revolute").length, 14);
    assertEqual(model.joints.filter((joint) => joint.type === "fixed").length, 4);
    assertEqual(model.roots.length, 1);
    assertEqual(model.roots[0], "base_link");
  });

  await test("dual Panda revolute limits", () => {
    for (const armId of ["panda1", "panda2"]) {
      EXPECTED_LIMITS.forEach((expected, index) => {
        const name = `${armId}_joint${index + 1}`;
        const joint = model.joints.find((candidate) => candidate.name === name);
        assert(joint, `missing ${name}`);
        assertNear(joint.limit.lower, expected[0], 1e-9, `${name} lower`);
        assertNear(joint.limit.upper, expected[1], 1e-9, `${name} upper`);
        assertNear(joint.limit.velocity, expected[2], 1e-9, `${name} velocity`);
        assertNear(joint.limit.effort, expected[3], 1e-9, `${name} effort`);
      });
    }
  });

  await test("mesh package URIs resolve through manifest", () => {
    let meshCount = 0;
    for (const link of Object.values(model.links)) {
      if (!link.visual) {
        continue;
      }
      meshCount += 1;
      const expected = resolveMeshUri(link.visual.meshUri, manifest);
      assert(expected, `manifest has no entry for ${link.visual.meshUri}`);
      assertEqual(link.visual.assetUri, expected, `wrong resolution for ${link.visual.meshUri}`);
    }
    assertEqual(meshCount, 16, "expected link0..link7 visual meshes for both arms");
  });
}

export {loadModel};
