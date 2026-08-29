// [DURABLE] Moves unchanged into franka_web at the Session C merge.

const EPSILON = 1e-15;

function requireMatrix(matrix, name) {
  if (!Array.isArray(matrix) || matrix.length !== 16 || matrix.some((v) => !Number.isFinite(v))) {
    throw new TypeError(`${name} must be a 16-element finite matrix`);
  }
}

export function identityMatrix() {
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
}

/** Multiply row-major 4x4 matrices acting on column vectors. */
export function multiplyMatrices(left, right) {
  requireMatrix(left, "left");
  requireMatrix(right, "right");
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

export function translationMatrix(xyz) {
  if (!Array.isArray(xyz) || xyz.length !== 3 || xyz.some((v) => !Number.isFinite(v))) {
    throw new TypeError("xyz must contain three finite numbers");
  }
  const result = identityMatrix();
  [result[3], result[7], result[11]] = xyz;
  return result;
}

/**
 * URDF rpy is fixed-axis XYZ: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * This is the Session C Stage 3 convention (equivalent to a ZYX Euler
 * construction in three.js), not three.js's default XYZ composition.
 */
export function fixedAxisXyzMatrix(rpy) {
  if (!Array.isArray(rpy) || rpy.length !== 3 || rpy.some((v) => !Number.isFinite(v))) {
    throw new TypeError("rpy must contain three finite numbers");
  }
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0,
    -sp, cp * sr, cp * cr, 0,
    0, 0, 0, 1,
  ];
}

export function axisAngleMatrix(axis, angle) {
  if (!Array.isArray(axis) || axis.length !== 3 || axis.some((v) => !Number.isFinite(v))) {
    throw new TypeError("axis must contain three finite numbers");
  }
  if (!Number.isFinite(angle)) {
    throw new TypeError("angle must be finite");
  }
  const length = Math.hypot(...axis);
  if (length <= EPSILON) {
    throw new Error("joint axis must be non-zero");
  }
  const [x, y, z] = axis.map((value) => value / length);
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  const oneMinusCosine = 1 - cosine;
  return [
    cosine + x * x * oneMinusCosine,
    x * y * oneMinusCosine - z * sine,
    x * z * oneMinusCosine + y * sine,
    0,
    y * x * oneMinusCosine + z * sine,
    cosine + y * y * oneMinusCosine,
    y * z * oneMinusCosine - x * sine,
    0,
    z * x * oneMinusCosine - y * sine,
    z * y * oneMinusCosine + x * sine,
    cosine + z * z * oneMinusCosine,
    0,
    0, 0, 0, 1,
  ];
}

export function originMatrix(origin) {
  if (!origin || !Array.isArray(origin.xyz) || !Array.isArray(origin.rpy)) {
    throw new TypeError("origin must contain xyz and rpy vectors");
  }
  return multiplyMatrices(translationMatrix(origin.xyz), fixedAxisXyzMatrix(origin.rpy));
}

function jointValue(jointPositions, name) {
  const value = jointPositions instanceof Map ? jointPositions.get(name) : jointPositions[name];
  if (value === undefined) {
    return 0;
  }
  if (!Number.isFinite(value)) {
    throw new TypeError(`joint position ${name} must be finite`);
  }
  return value;
}

/**
 * Compute link and joint-frame transforms from each URDF root.
 *
 * T(parent->child) = Translate(origin.xyz) * RPY(origin.rpy) * Rot(axis, q).
 * Returned matrices are row-major and act on column vectors.
 */
export function forwardKinematics(model, jointPositions = {}) {
  if (!model || !model.links || !Array.isArray(model.joints) || !Array.isArray(model.roots)) {
    throw new TypeError("model must be a parsed URDF model");
  }

  const childrenByParent = new Map();
  for (const joint of model.joints) {
    if (!childrenByParent.has(joint.parent)) {
      childrenByParent.set(joint.parent, []);
    }
    childrenByParent.get(joint.parent).push(joint);
  }

  const links = {};
  const joints = {};
  const queue = [];
  for (const root of model.roots) {
    links[root] = identityMatrix();
    queue.push(root);
  }

  while (queue.length > 0) {
    const parentName = queue.shift();
    for (const joint of childrenByParent.get(parentName) || []) {
      let local = originMatrix(joint.origin);
      if (joint.type === "revolute" || joint.type === "continuous") {
        local = multiplyMatrices(local, axisAngleMatrix(joint.axis, jointValue(jointPositions, joint.name)));
      } else if (joint.type !== "fixed") {
        throw new Error(`unsupported joint type ${joint.type} on ${joint.name}`);
      }
      joints[joint.name] = multiplyMatrices(links[parentName], originMatrix(joint.origin));
      links[joint.child] = multiplyMatrices(links[parentName], local);
      queue.push(joint.child);
    }
  }

  const linkNames = Object.keys(model.links);
  if (Object.keys(links).length !== linkNames.length) {
    const missing = linkNames.filter((name) => !Object.hasOwn(links, name));
    throw new Error(`URDF joint graph is disconnected or cyclic: ${missing.join(", ")}`);
  }
  return {links, joints};
}

export function translationFromMatrix(matrix) {
  requireMatrix(matrix, "matrix");
  return [matrix[3], matrix[7], matrix[11]];
}

/** Return an [x, y, z, w] unit quaternion from a rigid transform. */
export function quaternionFromMatrix(matrix) {
  requireMatrix(matrix, "matrix");
  const m00 = matrix[0];
  const m11 = matrix[5];
  const m22 = matrix[10];
  const trace = m00 + m11 + m22;
  let x;
  let y;
  let z;
  let w;
  if (trace > 0) {
    const scale = 2 * Math.sqrt(trace + 1);
    w = 0.25 * scale;
    x = (matrix[9] - matrix[6]) / scale;
    y = (matrix[2] - matrix[8]) / scale;
    z = (matrix[4] - matrix[1]) / scale;
  } else if (m00 > m11 && m00 > m22) {
    const scale = 2 * Math.sqrt(1 + m00 - m11 - m22);
    w = (matrix[9] - matrix[6]) / scale;
    x = 0.25 * scale;
    y = (matrix[1] + matrix[4]) / scale;
    z = (matrix[2] + matrix[8]) / scale;
  } else if (m11 > m22) {
    const scale = 2 * Math.sqrt(1 + m11 - m00 - m22);
    w = (matrix[2] - matrix[8]) / scale;
    x = (matrix[1] + matrix[4]) / scale;
    y = 0.25 * scale;
    z = (matrix[6] + matrix[9]) / scale;
  } else {
    const scale = 2 * Math.sqrt(1 + m22 - m00 - m11);
    w = (matrix[4] - matrix[1]) / scale;
    x = (matrix[2] + matrix[8]) / scale;
    y = (matrix[6] + matrix[9]) / scale;
    z = 0.25 * scale;
  }
  const length = Math.hypot(x, y, z, w);
  return [x / length, y / length, z / length, w / length];
}
