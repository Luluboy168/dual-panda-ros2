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

import {
  axisAngleMatrix,
  fixedAxisXyzMatrix,
  forwardKinematics,
  identityMatrix,
  invertRigidMatrix,
  multiplyMatrices,
  quaternionFromMatrix,
  translationFromMatrix,
  translationMatrix,
} from "../../../static/ghost/kinematics.js";
import {loadModel} from "./urdf.js";

const INITIAL = [0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4];
// Captured with tf2_echo -p 12 on fake_dual_state_only, ROS_DOMAIN_ID=82.
// The live fake JointState was all-zero despite the xacro's initial_position
// joint parameters; that observed discrepancy is recorded in Session C's
// reconciliation log rather than hidden by substituting the planned pose.
// The y components were DERIVED, not re-captured, at the base-separation
// correction: at all-zero joints link7's y is exactly the mounting offset, so
// +-0.26 became +-0.50 with x and z unchanged (the mounting joint is a pure
// y translation). The "observed fake-state link7 transforms match
// robot_state_publisher tf2" case below asserts these values against forward
// kinematics of the regenerated model and is the guard on that reasoning.
const TF2_INITIAL_LINK7 = {
  panda1_link7: {translation: [0.088, 0.5, 1.033], quaternion: [1, 0, 0, 0]},
  panda2_link7: {translation: [0.088, -0.5, 1.033], quaternion: [1, 0, 0, 0]},
};

function multiply(left, right) {
  const result = new Array(16).fill(0);
  for (let row = 0; row < 4; row += 1) {
    for (let column = 0; column < 4; column += 1) {
      for (let inner = 0; inner < 4; inner += 1) {
        result[row * 4 + column] += left[row * 4 + inner] * right[inner * 4 + column];
      }
    }
  }
  return result;
}

function independentOrigin({xyz, rpy}) {
  const [x, y, z] = xyz;
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const translate = [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
  const rotateX = [1, 0, 0, 0, 0, cr, -sr, 0, 0, sr, cr, 0, 0, 0, 0, 1];
  const rotateY = [cp, 0, sp, 0, 0, 1, 0, 0, -sp, 0, cp, 0, 0, 0, 0, 1];
  const rotateZ = [cy, -sy, 0, 0, sy, cy, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  return multiply(translate, multiply(rotateZ, multiply(rotateY, rotateX)));
}

function independentZeroTransform(model, linkName) {
  const byChild = new Map(model.joints.map((joint) => [joint.child, joint]));
  const chain = [];
  let cursor = linkName;
  while (byChild.has(cursor)) {
    const joint = byChild.get(cursor);
    chain.unshift(joint);
    cursor = joint.parent;
  }
  return chain.reduce(
    (transform, joint) => multiply(transform, independentOrigin(joint.origin)),
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
  );
}

export async function runKinematicsCases({test, assertArrayNear, assertNear}) {
  const {model} = await loadModel();

  await test("fixed-axis XYZ uses Rz * Ry * Rx for non-commuting RPY", () => {
    const actual = fixedAxisXyzMatrix([0.3, 0.5, 0.7]);
    const expected = [
      0.6712121661589577, -0.5070818727544463, 0.5406867876359134, 0,
      0.5653542083811438, 0.8219543695041275, 0.06903356805788474, 0,
      -0.479425538604203, 0.2593433800522308, 0.8383866435942036, 0,
      0, 0, 0, 1,
    ];
    assertArrayNear(actual, expected, 1e-14, "non-commuting RPY matrix");
  });

  await test("initial poses differ only by the 1.00 m arm-base translation", () => {
    const positions = {};
    for (const armId of ["panda1", "panda2"]) {
      INITIAL.forEach((value, index) => {
        positions[`${armId}_joint${index + 1}`] = value;
      });
    }
    const transforms = forwardKinematics(model, positions).links;
    const panda1 = translationFromMatrix(transforms.panda1_link7);
    const panda2 = translationFromMatrix(transforms.panda2_link7);
    assertNear(panda1[0], panda2[0], 1e-12, "link7 x");
    assertNear(panda1[2], panda2[2], 1e-12, "link7 z");
    assertNear(panda1[1] - panda2[1], 1.00, 1e-12, "link7 y separation");
    assertArrayNear(
      quaternionFromMatrix(transforms.panda1_link7),
      quaternionFromMatrix(transforms.panda2_link7),
      1e-12,
      "link7 orientations",
    );
  });

  await test("observed fake-state link7 transforms match robot_state_publisher tf2", () => {
    const positions = {};
    for (const armId of ["panda1", "panda2"]) {
      Array(7).fill(0).forEach((value, index) => {
        positions[`${armId}_joint${index + 1}`] = value;
      });
    }
    const transforms = forwardKinematics(model, positions).links;
    for (const [linkName, expected] of Object.entries(TF2_INITIAL_LINK7)) {
      assertArrayNear(
        translationFromMatrix(transforms[linkName]),
        expected.translation,
        1e-6,
        `${linkName} tf2 translation`,
      );
      const quaternion = quaternionFromMatrix(transforms[linkName]);
      const directError = Math.hypot(...quaternion.map((value, i) => value - expected.quaternion[i]));
      const negatedError = Math.hypot(...quaternion.map((value, i) => value + expected.quaternion[i]));
      if (Math.min(directError, negatedError) > 1e-6) {
        throw new Error(`${linkName} tf2 quaternion differs: ${JSON.stringify(quaternion)}`);
      }
    }
  });

  await test("all-zero FK equals an independently composed identity chain", () => {
    const transforms = forwardKinematics(model, {}).links;
    for (const linkName of ["panda1_link7", "panda2_link7"]) {
      assertArrayNear(
        transforms[linkName],
        independentZeroTransform(model, linkName),
        1e-12,
        `${linkName} zero transform`,
      );
    }
  });

  await test("invertRigidMatrix round-trips twenty random rigid transforms", () => {
    const identity = identityMatrix();
    let seed = 20260902;
    const next = () => {
      // A tiny deterministic generator: a failing case must be reproducible.
      seed = (seed * 1103515245 + 12345) % 2147483648;
      return seed / 2147483648;
    };
    for (let trial = 0; trial < 20; trial += 1) {
      const rigid = multiplyMatrices(
        translationMatrix([next() * 4 - 2, next() * 4 - 2, next() * 4 - 2]),
        multiplyMatrices(
          fixedAxisXyzMatrix([next() * 6 - 3, next() * 6 - 3, next() * 6 - 3]),
          axisAngleMatrix([next() + 0.1, next() - 0.5, next() - 0.5], next() * 6 - 3),
        ),
      );
      assertArrayNear(
        multiplyMatrices(rigid, invertRigidMatrix(rigid)), identity, 1e-12,
        `trial ${trial} M * inverse(M)`,
      );
      assertArrayNear(
        multiplyMatrices(invertRigidMatrix(rigid), rigid), identity, 1e-12,
        `trial ${trial} inverse(M) * M`,
      );
    }
  });

  await test("invertRigidMatrix refuses anything that is not a 4x4", () => {
    for (const bad of [new Array(15).fill(0), null, "identity",
                       [...Array(15).fill(0), Number.NaN]]) {
      let threw = false;
      try {
        invertRigidMatrix(bad);
      } catch (_error) {
        threw = true;
      }
      if (!threw) {
        throw new Error(`invertRigidMatrix accepted ${JSON.stringify(bad)}`);
      }
    }
  });

  await test("a flange pose round-trips through the arm base frame", () => {
    // The property the drag depends on: expressing the hand in <arm>_link0 and
    // composing it back with the base must reproduce the world pose exactly.
    const positions = {};
    const values = [0.21, -0.91, 0.34, -2.14, -0.18, 1.82, 0.63];
    values.forEach((value, index) => {
      positions[`panda1_joint${index + 1}`] = value;
    });
    const links = forwardKinematics(model, positions).links;
    const local = multiplyMatrices(
      invertRigidMatrix(links.panda1_link0), links.panda1_link8,
    );
    assertArrayNear(
      multiplyMatrices(links.panda1_link0, local), links.panda1_link8, 1e-12,
      "base-relative flange pose",
    );
    assertArrayNear(
      quaternionFromMatrix(multiplyMatrices(links.panda1_link0, local)),
      quaternionFromMatrix(links.panda1_link8), 1e-12, "flange orientation",
    );
  });
}
