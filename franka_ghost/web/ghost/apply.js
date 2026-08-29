// [DURABLE] Moves unchanged into franka_web at the Session C merge.

export const GHOST_APPLY_SCHEMA = "franka.ghost.apply/1";
export const URDF_LOWER = Object.freeze([
  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
]);
export const URDF_UPPER = Object.freeze([
  2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
]);

const FORBIDDEN_FIELDS = new Set([
  "velocities",
  "accelerations",
  "effort",
  "efforts",
  "time_from_start",
  "frame_id",
  "duration",
  "emitted_at",
  "stamp",
  "stamp_ns",
  "timestamp",
  "timestamp_ns",
  "wall_clock",
  "wall_clock_ns",
]);

function isFiniteNumber(value) {
  return typeof value === "number" && Number.isFinite(value);
}

function sevenFinite(values) {
  return Array.isArray(values) && values.length === 7 && values.every(isFiniteNumber);
}

function cloneSeven(values, label) {
  if (!sevenFinite(values)) {
    throw new TypeError(`${label} must contain exactly seven finite numbers`);
  }
  return [...values];
}

function armIdForIndex(armIndex) {
  if (!Number.isInteger(armIndex) || (armIndex !== 1 && armIndex !== 2)) {
    throw new RangeError("arm_index must be the integer 1 or 2");
  }
  return `panda${armIndex}`;
}

export function canonicalJointNames(armId) {
  if (armId !== "panda1" && armId !== "panda2") {
    throw new RangeError("arm_id must be panda1 or panda2");
  }
  return Array.from({length: 7}, (_unused, index) => `${armId}_joint${index + 1}`);
}

function forbiddenPaths(value, path = "$") {
  const paths = [];
  if (Array.isArray(value)) {
    value.forEach((child, index) => paths.push(...forbiddenPaths(child, `${path}[${index}]`)));
    return paths;
  }
  if (!value || typeof value !== "object") {
    return paths;
  }
  for (const [key, child] of Object.entries(value)) {
    const lowered = key.toLowerCase();
    if (FORBIDDEN_FIELDS.has(lowered)
        || lowered.endsWith("_timestamp")
        || lowered.endsWith("_timestamp_ns")) {
      paths.push(`${path}.${key}`);
    }
    paths.push(...forbiddenPaths(child, `${path}.${key}`));
  }
  return paths;
}

/** Return every Contract C1 violation without mutating the candidate event. */
export function validateGhostApplyEvent(event, previousEpoch = null) {
  if (!event || typeof event !== "object" || Array.isArray(event)) {
    return ["C1: event must be a JSON object"];
  }
  const errors = [];
  if (event.schema !== GHOST_APPLY_SCHEMA) {
    errors.push(`C1: schema must be ${JSON.stringify(GHOST_APPLY_SCHEMA)}`);
  }

  const armIndexValid = Number.isInteger(event.arm_index)
    && (event.arm_index === 1 || event.arm_index === 2);
  const expectedArmId = armIndexValid ? `panda${event.arm_index}` : null;
  if (!armIndexValid || event.arm_id !== expectedArmId) {
    errors.push("I5: arm_index must be 1 or 2 and agree with arm_id panda1 or panda2");
  }

  const expectedNames = expectedArmId ? canonicalJointNames(expectedArmId) : [];
  if (JSON.stringify(event.joint_names) !== JSON.stringify(expectedNames)) {
    errors.push(`I1: joint_names must equal ${JSON.stringify(expectedNames)}`);
  }

  const positionsValid = sevenFinite(event.positions);
  if (!positionsValid) {
    errors.push("I2: positions must contain exactly 7 finite numbers");
  }

  const fence = event.fence;
  const lower = fence && typeof fence === "object" ? fence.lower : null;
  const upper = fence && typeof fence === "object" ? fence.upper : null;
  const lowerValid = sevenFinite(lower);
  const upperValid = sevenFinite(upper);
  if (!lowerValid || !upperValid) {
    errors.push("I3: fence lower and upper must each contain exactly 7 finite numbers");
  } else {
    lower.forEach((low, index) => {
      const high = upper[index];
      if (low > high) {
        errors.push(`I3: fence.lower[${index}]=${low} > fence.upper[${index}]=${high}`);
      } else if (positionsValid && event.positions[index] < low) {
        errors.push(
          `I3: positions[${index}]=${event.positions[index]} < fence.lower[${index}]=${low}`,
        );
      } else if (positionsValid && event.positions[index] > high) {
        errors.push(
          `I3: positions[${index}]=${event.positions[index]} > fence.upper[${index}]=${high}`,
        );
      }
      if (low < URDF_LOWER[index]) {
        errors.push(`I4: fence.lower[${index}]=${low} < URDF lower ${URDF_LOWER[index]}`);
      }
      if (high > URDF_UPPER[index]) {
        errors.push(`I4: fence.upper[${index}]=${high} > URDF upper ${URDF_UPPER[index]}`);
      }
    });
  }
  if (!fence || (fence.source !== "urdf" && fence.source !== "session")) {
    errors.push("I4: fence.source must be 'urdf' or 'session'");
  }

  if (!Number.isInteger(event.ghost_epoch) || event.ghost_epoch < 1) {
    errors.push("I6: ghost_epoch must be a positive integer");
  } else if (previousEpoch !== null && event.ghost_epoch <= previousEpoch) {
    errors.push(
      `I6: ghost_epoch=${event.ghost_epoch} must be greater than previous epoch ${previousEpoch}`,
    );
  }

  const forbidden = forbiddenPaths(event);
  if (forbidden.length > 0) {
    errors.push(`I7: prohibited command/timing fields present at ${forbidden.join(", ")}`);
  }
  if (!sevenFinite(event.measured_at_apply)) {
    errors.push("C1: measured_at_apply must contain exactly 7 finite numbers");
  }
  return errors;
}

/** Throw one error containing all Contract C1 violations. */
export function assertGhostApplyEvent(event, previousEpoch = null) {
  const errors = validateGhostApplyEvent(event, previousEpoch);
  if (errors.length > 0) {
    throw new Error(errors.join("\n"));
  }
  return event;
}

/** Snapshot and validate one arm's joint-space ghost pose. */
export function buildGhostApplyEvent({
  armIndex,
  positions,
  fence,
  measuredAtApply,
  ghostEpoch,
  previousEpoch = null,
}) {
  const armId = armIdForIndex(armIndex);
  const event = {
    schema: GHOST_APPLY_SCHEMA,
    arm_index: armIndex,
    arm_id: armId,
    joint_names: canonicalJointNames(armId),
    positions: cloneSeven(positions, "positions"),
    fence: {
      lower: cloneSeven(fence && fence.lower, "fence.lower"),
      upper: cloneSeven(fence && fence.upper, "fence.upper"),
      source: fence && fence.source,
    },
    measured_at_apply: cloneSeven(measuredAtApply, "measuredAtApply"),
    ghost_epoch: ghostEpoch,
  };
  return assertGhostApplyEvent(event, previousEpoch);
}

// RECONCILIATION: SESSION_A section 6.13 has no whole-pose endpoint; see plan section C1
/**
 * Prototype pass-through and the sole Session-A adaptation point.
 *
 * Session A must choose whether to add a whole-pose endpoint, decompose into
 * fixed jogs, or leave Apply prototype-only. Until then this deliberately
 * preserves the frozen C1 event byte-for-byte at the object level.
 */
export function toWebV1Payload(event) {
  assertGhostApplyEvent(event);
  return event;
}
