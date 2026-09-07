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

import {parseUrdf, resolveMeshUri} from "../../../static/ghost/urdf.js";

const EXPECTED_LIMITS = [
  [-2.8973, 2.8973, 2.1750, 87],
  [-1.7628, 1.7628, 2.1750, 87],
  [-2.8973, 2.8973, 2.1750, 87],
  [-3.0718, -0.0698, 2.1750, 87],
  [-2.8973, 2.8973, 2.6100, 12],
  [-0.0175, 3.7525, 2.6100, 12],
  [-2.8973, 2.8973, 2.6100, 12],
];

export const ASSET_BASE = new URL("../../../static/ghost/assets/", import.meta.url);

async function loadModel() {
  const [urdfResponse, manifestResponse] = await Promise.all([
    fetch(new URL("model.urdf", ASSET_BASE)),
    fetch(new URL("manifest.json", ASSET_BASE)),
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
  await test("URDF parser rejects malformed XML", async () => {
    // DOMParser's own <parsererror> element carries an inline style attribute,
    // which the production policy blocks — so parsing bad XML necessarily
    // trips one style-src-attr violation that is the BROWSER's, not ours. It
    // is caught, accounted for here, and removed, so the suite's "zero
    // violations" rule keeps its teeth everywhere else.
    const problems = window.__ghostHarnessProblems;
    const before = problems.length;
    let threw = false;
    try {
      parseUrdf("<robot><link></robot>");
    } catch (_error) {
      threw = true;
    }
    await new Promise((resolve) => setTimeout(resolve, 0));
    const raised = problems.slice(before);
    problems.length = before;
    assert(threw, "malformed XML must throw");
    assert(
      raised.every((problem) => problem.kind === "securitypolicyviolation"
        && problem.message.indexOf("style-src") === 0),
      `parsing bad XML raised something other than the expected parsererror style `
      + `violation: ${JSON.stringify(raised)}`,
    );
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

  await test("the two arm bases stand one metre apart", () => {
    // The scene draws the lab, so a base separation that has drifted draws the
    // WRONG lab, convincingly. The generated model is the single source of the
    // separation, so this is the place to hold it.
    const origins = ["panda1", "panda2"].map((armId) => {
      const joint = model.joints.find(
        (candidate) => candidate.name === `${armId}_joint_base_link`,
      );
      assert(joint, `missing ${armId}_joint_base_link`);
      return joint.origin.xyz;
    });
    assertNear(origins[0][1] - origins[1][1], 1.00, 1e-9, "base separation in y");
    assertNear(origins[0][0], origins[1][0], 1e-9, "base x");
    assertNear(origins[0][2], origins[1][2], 1e-9, "base z");
  });
}

export {loadModel};
