// [DURABLE] Browser contract test retained when the ghost moves into franka_web.

import {
  fixedAxisXyzMatrix,
  forwardKinematics,
  quaternionFromMatrix,
  translationFromMatrix,
} from "../../../web/ghost/kinematics.js";
import {loadModel} from "./urdf.js";

const INITIAL = [0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4];
// Captured with tf2_echo -p 12 on fake_dual_state_only, ROS_DOMAIN_ID=82.
// The live fake JointState was all-zero despite the xacro's initial_position
// joint parameters; that observed discrepancy is recorded in Session C's
// reconciliation log rather than hidden by substituting the planned pose.
// The y components were DERIVED, not re-captured, at the base-separation
// correction: at all-zero joints link7's y is exactly the mounting offset, so
// +-0.26 became +-0.50 with x and z unchanged (the mounting joint is a pure
// y translation). The "all-zero FK equals an independently composed identity
// chain" case below re-derives the same chain from the model and is the guard
// on that reasoning.
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
}
