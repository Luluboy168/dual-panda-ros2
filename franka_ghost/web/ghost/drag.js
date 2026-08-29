// [DURABLE] Moves unchanged into franka_web at the Session C merge.

const FULL_TURN = 2 * Math.PI;
const DEGREES_PER_RADIAN = 180 / Math.PI;
const FINE_JOG_STEP = 0.5 / DEGREES_PER_RADIAN;
const LIMIT_EPSILON = 1e-10;
const ARC_SEGMENTS = 96;
const ARC_TUBE_SEGMENTS = 6;
const ARC_VERTICES_PER_QUAD = 6;

function armIdForIndex(armIndex) {
  if (armIndex !== 1 && armIndex !== 2) {
    throw new RangeError("armIndex must be 1 or 2");
  }
  return `panda${armIndex}`;
}

function requireJointIndex(jointIndex) {
  if (!Number.isInteger(jointIndex) || jointIndex < 0 || jointIndex >= 7) {
    throw new RangeError("jointIndex must be an integer from 0 through 6");
  }
}

function cloneSeven(values, name) {
  if (!Array.isArray(values) || values.length !== 7
      || values.some((value) => !Number.isFinite(value))) {
    throw new TypeError(`${name} must contain exactly seven finite numbers`);
  }
  return [...values];
}

/** Return the wrapped angular step in (-pi, pi]. */
export function shortestAngleDiff(next, previous) {
  if (!Number.isFinite(next) || !Number.isFinite(previous)) {
    throw new TypeError("angles must be finite");
  }
  let delta = (next - previous) % FULL_TURN;
  if (delta <= -Math.PI) {
    delta += FULL_TURN;
  } else if (delta > Math.PI) {
    delta -= FULL_TURN;
  }
  return delta;
}

function armFromLinkName(linkName) {
  const match = /^(panda([12]))_link([1-7])$/.exec(linkName || "");
  return match ? {armIndex: Number(match[2]), jointIndex: Number(match[3]) - 1} : null;
}

function createMaterial(three, color, opacity) {
  return new three.MeshBasicMaterial({
    color,
    opacity,
    transparent: true,
    depthWrite: false,
    depthTest: false,
    side: three.DoubleSide,
  });
}

function positionAtAngle(object, radius, angle) {
  object.position.set(radius * Math.cos(angle), radius * Math.sin(angle), 0);
}

function makeArcMesh(three, radius, thickness, start, end, material, name) {
  const span = Math.max(0, end - start);
  if (span <= 1e-12) {
    return null;
  }
  const segments = Math.max(8, Math.ceil(96 * span / FULL_TURN));
  const geometry = new three.TorusGeometry(radius, thickness, 6, segments, span);
  const mesh = new three.Mesh(geometry, material);
  mesh.name = name;
  mesh.rotation.z = start;
  mesh.renderOrder = 20;
  mesh.userData.arcStart = start;
  mesh.userData.arcEnd = end;
  mesh.userData.arcSpan = span;
  return mesh;
}

function writeTorusVertex(array, offset, radius, thickness, arcAngle, tubeAngle) {
  const tubeRadius = radius + thickness * Math.cos(tubeAngle);
  array[offset] = tubeRadius * Math.cos(arcAngle);
  array[offset + 1] = tubeRadius * Math.sin(arcAngle);
  array[offset + 2] = thickness * Math.sin(tubeAngle);
}

/**
 * Create one fixed-capacity pending-arc mesh.
 *
 * Pointer and measured-state updates rewrite this buffer in place. In
 * particular, they must never create/dispose TorusGeometry: joint state can
 * arrive at display rate and pointermove can be substantially faster.
 */
function makeReusableArcMesh(three, radius, thickness, material, name, allocationStats) {
  const vertexCapacity = ARC_SEGMENTS * ARC_TUBE_SEGMENTS * ARC_VERTICES_PER_QUAD;
  const positions = new Float32Array(vertexCapacity * 3);
  const geometry = new three.BufferGeometry();
  const attribute = new three.BufferAttribute(positions, 3);
  const setAttribute = geometry.setAttribute
    ? geometry.setAttribute.bind(geometry)
    : geometry.addAttribute.bind(geometry);
  setAttribute("position", attribute);
  geometry.setDrawRange(0, 0);
  geometry.boundingSphere = new three.Sphere(
    new three.Vector3(0, 0, 0),
    radius + thickness,
  );
  geometry.userData.fixedCapacity = vertexCapacity;
  allocationStats.pendingBufferGeometries += 1;

  const mesh = new three.Mesh(geometry, material);
  mesh.name = name;
  mesh.renderOrder = 21;
  mesh.visible = false;
  mesh.userData.arcStart = 0;
  mesh.userData.arcEnd = 0;
  mesh.userData.arcSpan = 0;
  mesh.userData.fixedBuffer = true;

  function update(start, end) {
    const span = Math.min(FULL_TURN, Math.max(0, end - start));
    mesh.rotation.z = start;
    mesh.userData.arcStart = start;
    mesh.userData.arcEnd = start + span;
    mesh.userData.arcSpan = span;
    if (span <= 1e-12) {
      geometry.setDrawRange(0, 0);
      mesh.visible = false;
      return;
    }

    const arcSegments = Math.max(1, Math.ceil(ARC_SEGMENTS * span / FULL_TURN));
    let vertexOffset = 0;
    for (let arcIndex = 0; arcIndex < arcSegments; arcIndex += 1) {
      const arc0 = span * arcIndex / arcSegments;
      const arc1 = span * (arcIndex + 1) / arcSegments;
      for (let tubeIndex = 0; tubeIndex < ARC_TUBE_SEGMENTS; tubeIndex += 1) {
        const tube0 = FULL_TURN * tubeIndex / ARC_TUBE_SEGMENTS;
        const tube1 = FULL_TURN * (tubeIndex + 1) / ARC_TUBE_SEGMENTS;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc0, tube0);
        vertexOffset += 3;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc1, tube0);
        vertexOffset += 3;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc1, tube1);
        vertexOffset += 3;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc0, tube0);
        vertexOffset += 3;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc1, tube1);
        vertexOffset += 3;
        writeTorusVertex(positions, vertexOffset, radius, thickness, arc0, tube1);
        vertexOffset += 3;
      }
    }
    const vertexCount = vertexOffset / 3;
    geometry.setDrawRange(0, vertexCount);
    attribute.updateRange.offset = 0;
    attribute.updateRange.count = vertexOffset;
    attribute.needsUpdate = true;
    mesh.visible = true;
  }

  return {mesh, update};
}

function addTrackedListener(registry, target, type, listener, options) {
  target.addEventListener(type, listener, options);
  registry.push({target, type, listener, options});
}

/**
 * Install joint-space-only ghost editing.
 *
 * The controller owns every clamp path and every interaction listener. It has
 * no transport or motion-system semantics.
 */
export function createJointDragController({
  three,
  canvas,
  camera,
  model,
  solidGraph,
  ghostGraph,
  render,
  orbitControls,
  initialArm = 1,
  armIndices = [1, 2],
  jogStepRad,
  ui = {},
}) {
  if (!Number.isFinite(jogStepRad) || jogStepRad <= 0) {
    throw new TypeError("jogStepRad must be positive and finite");
  }
  armIdForIndex(initialArm);
  if (!Array.isArray(armIndices) || armIndices.length < 1
      || armIndices.length > 2 || new Set(armIndices).size !== armIndices.length
      || armIndices.some((armIndex) => armIndex !== 1 && armIndex !== 2)
      || !armIndices.includes(initialArm)) {
    throw new TypeError("armIndices must contain one or two unique configured arms");
  }

  const listeners = [];
  const timers = new Map();
  const states = new Map();
  const handles = new Map();
  const rowElements = new Map();
  const pickTargets = [];
  const raycaster = new three.Raycaster();
  const pointerNdc = new three.Vector2();
  const dragPlane = new three.Plane();
  const worldOrigin = new three.Vector3();
  const worldAxis = new three.Vector3();
  const worldPoint = new three.Vector3();
  const allocationStats = {pendingBufferGeometries: 0};
  let selectedArm = initialArm;
  let selectedJoint = null;
  let enabled = true;
  let stale = false;
  let disposed = false;
  let drag = null;

  for (const armIndex of armIndices) {
    const armId = armIdForIndex(armIndex);
    const joints = Array.from({length: 7}, (_unused, index) => {
      const name = `${armId}_joint${index + 1}`;
      const joint = model.joints.find((candidate) => candidate.name === name);
      if (!joint || !joint.limit) {
        throw new Error(`model is missing limited revolute joint ${name}`);
      }
      return joint;
    });
    states.set(armIndex, {
      armId,
      joints,
      urdfLower: joints.map((joint) => joint.limit.lower),
      urdfUpper: joints.map((joint) => joint.limit.upper),
      fenceLower: joints.map((joint) => joint.limit.lower),
      fenceUpper: joints.map((joint) => joint.limit.upper),
      fenceSource: "urdf",
      measured: new Array(7).fill(Number.NaN),
      ghost: new Array(7).fill(Number.NaN),
      requestedVisible: armIndex === initialArm,
      visibilityExplicit: false,
    });
  }

  function stateFor(armIndex) {
    armIdForIndex(armIndex);
    const state = states.get(armIndex);
    if (!state) {
      throw new RangeError("armIndex is not configured for this mount");
    }
    return state;
  }

  function armReady(state) {
    return state.ghost.every(Number.isFinite);
  }

  function updateArmVisibility(armIndex) {
    const state = stateFor(armIndex);
    const root = ghostGraph.linkObjects.get(`${state.armId}_link0`);
    if (root) {
      root.visible = state.requestedVisible && armReady(state);
    }
  }

  function limitText(state, jointIndex) {
    const value = state.ghost[jointIndex];
    if (!Number.isFinite(value)) {
      return "waiting for measured value";
    }
    if (Math.abs(value - state.fenceLower[jointIndex]) <= LIMIT_EPSILON) {
      return "at lower limit";
    }
    if (Math.abs(value - state.fenceUpper[jointIndex]) <= LIMIT_EPSILON) {
      return "at upper limit";
    }
    const measured = state.measured[jointIndex];
    return Number.isFinite(measured)
      ? `${((value - measured) * DEGREES_PER_RADIAN).toFixed(1)}° pending`
      : "ghost set; measured unavailable";
  }

  function updateDeltaOutput() {
    if (!ui.deltaOutput) {
      return;
    }
    const state = stateFor(selectedArm);
    const deltas = state.ghost.map((value, index) => (
      Number.isFinite(value) && Number.isFinite(state.measured[index])
        ? Math.abs(value - state.measured[index])
        : 0
    ));
    const maximum = Math.max(0, ...deltas) * DEGREES_PER_RADIAN;
    ui.deltaOutput.textContent = `${maximum.toFixed(1)}°`;
    ui.deltaOutput.title = `Maximum absolute ghost-to-measured delta: ${maximum.toFixed(3)} degrees`;
  }

  function updateApplyPlaceholder() {
    if (!ui.applyButton) {
      return;
    }
    if (typeof ui.onStateChange === "function") {
      ui.onStateChange();
      return;
    }
    // Stage 5 deliberately has no Apply implementation. Stage 6 replaces this
    // disabled placeholder after validating the output contract.
    ui.applyButton.disabled = true;
    ui.applyButton.setAttribute("aria-disabled", "true");
    const reason = stale
      ? "Joint state is stale; Apply is unavailable"
      : enabled
        ? "Apply is implemented in Session C Stage 6"
        : "Ghost editing is read-only";
    ui.applyButton.title = reason;
    ui.applyButton.dataset.disabledReason = stale ? "stale" : enabled ? "stage6" : "read-only";
  }

  function updateRow(armIndex, jointIndex) {
    const state = stateFor(armIndex);
    const row = rowElements.get(`${armIndex}:${jointIndex}`);
    if (!row) {
      return;
    }
    const measured = state.measured[jointIndex];
    const ghost = state.ghost[jointIndex];
    row.root.classList.toggle(
      "selected",
      selectedArm === armIndex && selectedJoint === jointIndex,
    );
    row.button.setAttribute(
      "aria-pressed",
      selectedArm === armIndex && selectedJoint === jointIndex ? "true" : "false",
    );
    row.measured.textContent = Number.isFinite(measured)
      ? `${(measured * DEGREES_PER_RADIAN).toFixed(1)}°`
      : "—";
    row.ghost.textContent = Number.isFinite(ghost)
      ? `${(ghost * DEGREES_PER_RADIAN).toFixed(1)}°`
      : "—";
    row.status.textContent = limitText(state, jointIndex);
    row.input.disabled = !enabled || !Number.isFinite(ghost);
    row.input.value = Number.isFinite(ghost) ? (ghost * DEGREES_PER_RADIAN).toFixed(2) : "";
    row.input.min = (state.fenceLower[jointIndex] * DEGREES_PER_RADIAN).toFixed(4);
    row.input.max = (state.fenceUpper[jointIndex] * DEGREES_PER_RADIAN).toFixed(4);
    row.input.title = Number.isFinite(ghost) ? `${ghost.toFixed(4)} rad` : "Measured value pending";
    const span = state.fenceUpper[jointIndex] - state.fenceLower[jointIndex];
    const fraction = (value) => Number.isFinite(value)
      ? span === 0 ? 50 : 100 * (value - state.fenceLower[jointIndex]) / span
      : 0;
    row.measuredMarker.style.left = `${Math.min(100, Math.max(0, fraction(measured)))}%`;
    row.ghostMarker.style.left = `${Math.min(100, Math.max(0, fraction(ghost)))}%`;
  }

  function refreshRows(armIndex = null) {
    for (const candidateArm of armIndex === null ? armIndices : [armIndex]) {
      for (let jointIndex = 0; jointIndex < 7; jointIndex += 1) {
        updateRow(candidateArm, jointIndex);
      }
    }
    updateDeltaOutput();
    updateApplyPlaceholder();
  }

  function pulseLimit(armIndex, jointIndex, side) {
    const key = `${armIndex}:${jointIndex}`;
    const row = rowElements.get(key);
    const handle = handles.get(key);
    if (row) {
      row.root.classList.remove("limit-pulse-lower", "limit-pulse-upper");
      row.root.classList.add(`limit-pulse-${side}`);
    }
    const endpoint = handle && handle.endpoints[side];
    if (endpoint) {
      endpoint.material.color.set(side === "lower" ? 0xff726b : 0xffc45c);
      endpoint.scale.setScalar(1.8);
    }
    clearTimeout(timers.get(key));
    timers.set(key, setTimeout(() => {
      if (row) {
        row.root.classList.remove("limit-pulse-lower", "limit-pulse-upper");
      }
      if (endpoint) {
        endpoint.material.color.set(0x8fa9ba);
        endpoint.scale.setScalar(1);
      }
      timers.delete(key);
      render();
    }, 240));
  }

  /** The only clamp used by drag, keyboard, numeric input and setGhost. */
  function clampJoint(armIndex, jointIndex, requested) {
    requireJointIndex(jointIndex);
    if (!Number.isFinite(requested)) {
      throw new TypeError("joint value must be finite");
    }
    const state = stateFor(armIndex);
    if (requested < state.fenceLower[jointIndex]) {
      pulseLimit(armIndex, jointIndex, "lower");
      return state.fenceLower[jointIndex];
    }
    if (requested > state.fenceUpper[jointIndex]) {
      pulseLimit(armIndex, jointIndex, "upper");
      return state.fenceUpper[jointIndex];
    }
    return requested;
  }

  function setNodeAngle(graph, name, value) {
    const node = graph.jointNodes.get(name);
    if (!node) {
      return;
    }
    node.userData.value = value;
    node.quaternion.setFromAxisAngle(node.userData.axis, value);
  }

  function updatePendingArc(handle, state, jointIndex) {
    const measured = state.measured[jointIndex];
    const ghost = state.ghost[jointIndex];
    if (!Number.isFinite(measured) || !Number.isFinite(ghost)
        || Math.abs(measured - ghost) <= 1e-12) {
      handle.pendingArc.update(0, 0);
      return;
    }
    handle.pendingArc.update(
      Math.min(measured, ghost),
      Math.max(measured, ghost),
    );
  }

  function refreshDynamicHandle(armIndex, jointIndex) {
    const state = stateFor(armIndex);
    const handle = handles.get(`${armIndex}:${jointIndex}`);
    if (!handle) {
      return;
    }
    const measured = state.measured[jointIndex];
    const ghost = state.ghost[jointIndex];
    handle.tick.visible = Number.isFinite(measured);
    handle.knob.visible = Number.isFinite(ghost);
    if (Number.isFinite(measured)) {
      positionAtAngle(handle.tick, handle.radius, measured);
      handle.tick.rotation.z = measured;
    }
    if (Number.isFinite(ghost)) {
      positionAtAngle(handle.knob, handle.radius, ghost);
    }
    updatePendingArc(handle, state, jointIndex);
  }

  function disposeStaticParts(handle) {
    const materials = new Set();
    handle.group.traverse((object) => {
      if (object.geometry && typeof object.geometry.dispose === "function") {
        object.geometry.dispose();
      }
      if (object.material) {
        for (const material of Array.isArray(object.material) ? object.material : [object.material]) {
          materials.add(material);
        }
      }
    });
    materials.forEach((material) => material.dispose());
  }

  function removeHandleFromPickTargets(handle) {
    for (let index = pickTargets.length - 1; index >= 0; index -= 1) {
      let cursor = pickTargets[index];
      while (cursor) {
        if (cursor === handle.group) {
          pickTargets.splice(index, 1);
          break;
        }
        cursor = cursor.parent;
      }
    }
  }

  function buildHandle(armIndex, jointIndex) {
    const state = stateFor(armIndex);
    const joint = state.joints[jointIndex];
    const key = `${armIndex}:${jointIndex}`;
    const previous = handles.get(key);
    if (previous) {
      clearTimeout(timers.get(key));
      timers.delete(key);
      removeHandleFromPickTargets(previous);
      previous.group.parent.remove(previous.group);
      disposeStaticParts(previous);
    }

    const originNode = ghostGraph.jointOrigins.get(joint.name);
    const childMesh = ghostGraph.linkMeshes.find(
      (mesh) => mesh.userData.linkName === joint.child,
    );
    const childRadius = childMesh && childMesh.geometry.boundingSphere
      ? childMesh.geometry.boundingSphere.radius
      : 0.1;
    const radius = Math.min(0.19, Math.max(0.075, childRadius * 1.1));
    const group = new three.Group();
    group.name = `${joint.name}_rotation_handle`;
    group.userData.armIndex = armIndex;
    group.userData.jointIndex = jointIndex;
    group.userData.fenceLower = state.fenceLower[jointIndex];
    group.userData.fenceUpper = state.fenceUpper[jointIndex];
    group.userData.fenceSource = state.fenceSource;
    group.userData.urdfLower = state.urdfLower[jointIndex];
    group.userData.urdfUpper = state.urdfUpper[jointIndex];
    group.userData.radius = radius;
    group.quaternion.setFromUnitVectors(
      new three.Vector3(0, 0, 1),
      new three.Vector3(...joint.axis).normalize(),
    );
    originNode.add(group);

    const materials = {
      reachable: createMaterial(three, armIndex === 1 ? 0x55c9ff : 0xbd85ff, 0.78),
      pending: createMaterial(three, 0xffb454, 0.98),
      remainder: createMaterial(three, 0x687887, 0.22),
      tick: createMaterial(three, 0xe8f3ff, 1),
      knob: createMaterial(three, armIndex === 1 ? 0x78ddff : 0xd3a8ff, 1),
      endpointLower: createMaterial(three, 0x8fa9ba, 0.82),
      endpointUpper: createMaterial(three, 0x8fa9ba, 0.82),
    };
    const reachable = makeArcMesh(
      three,
      radius,
      0.0055,
      state.fenceLower[jointIndex],
      state.fenceUpper[jointIndex],
      materials.reachable,
      `${joint.name}_reachable_arc`,
    );
    const remainders = [
      makeArcMesh(
        three,
        radius,
        0.003,
        state.urdfLower[jointIndex],
        state.fenceLower[jointIndex],
        materials.remainder,
        `${joint.name}_lower_remainder`,
      ),
      makeArcMesh(
        three,
        radius,
        0.003,
        state.fenceUpper[jointIndex],
        state.urdfUpper[jointIndex],
        materials.remainder,
        `${joint.name}_upper_remainder`,
      ),
    ].filter(Boolean);
    const tick = new three.Mesh(
      new three.BoxGeometry(0.028, 0.004, 0.006),
      materials.tick,
    );
    tick.name = `${joint.name}_measured_tick`;
    tick.renderOrder = 22;
    const knob = new three.Mesh(
      new three.SphereGeometry(0.017, 16, 10),
      materials.knob,
    );
    knob.name = `${joint.name}_ghost_knob`;
    knob.renderOrder = 23;
    knob.userData.handleKind = "knob";
    knob.userData.armIndex = armIndex;
    knob.userData.jointIndex = jointIndex;

    function endpoint(side, angle, material) {
      const mesh = new three.Mesh(new three.SphereGeometry(0.007, 10, 6), material);
      mesh.name = `${joint.name}_${side}_endpoint`;
      mesh.renderOrder = 22;
      mesh.userData.handleKind = "joint-handle";
      mesh.userData.armIndex = armIndex;
      mesh.userData.jointIndex = jointIndex;
      positionAtAngle(mesh, radius, angle);
      return mesh;
    }
    const endpoints = {
      lower: endpoint("lower", state.fenceLower[jointIndex], materials.endpointLower),
      upper: endpoint("upper", state.fenceUpper[jointIndex], materials.endpointUpper),
    };
    const pendingArc = makeReusableArcMesh(
      three,
      radius,
      0.009,
      materials.pending,
      `${state.armId}_joint${jointIndex + 1}_pending_arc`,
      allocationStats,
    );
    const pending = pendingArc.mesh;
    pending.userData.handleKind = "joint-handle";
    pending.userData.armIndex = armIndex;
    pending.userData.jointIndex = jointIndex;
    for (const object of [
      reachable,
      ...remainders,
      pending,
      tick,
      knob,
      endpoints.lower,
      endpoints.upper,
    ]) {
      if (!object) {
        continue;
      }
      if (!object.userData.handleKind) {
        object.userData.handleKind = "joint-handle";
        object.userData.armIndex = armIndex;
        object.userData.jointIndex = jointIndex;
      }
      group.add(object);
      pickTargets.push(object);
    }
    const handle = {
      armIndex,
      jointIndex,
      group,
      radius,
      reachable,
      remainders,
      pending,
      pendingArc,
      tick,
      knob,
      endpoints,
      materials,
    };
    handles.set(key, handle);
    refreshDynamicHandle(armIndex, jointIndex);
    return handle;
  }

  function refreshHandleVisibility() {
    for (const handle of handles.values()) {
      const distance = selectedJoint === null || handle.armIndex !== selectedArm
        ? Number.POSITIVE_INFINITY
        : Math.abs(handle.jointIndex - selectedJoint);
      handle.group.visible = enabled
        && stateFor(handle.armIndex).requestedVisible
        && armReady(stateFor(handle.armIndex))
        && distance <= 1;
      const selected = distance === 0;
      const opacityScale = selected ? 1 : 0.28;
      for (const material of Object.values(handle.materials)) {
        const base = material === handle.materials.remainder ? 0.22
          : material === handle.materials.reachable ? 0.78
            : 1;
        material.opacity = base * opacityScale;
      }
      handle.group.userData.selected = selected;
      handle.group.userData.neighbour = distance === 1;
    }
  }

  function applyGhostValue(armIndex, jointIndex, requested) {
    const state = stateFor(armIndex);
    const value = clampJoint(armIndex, jointIndex, requested);
    state.ghost[jointIndex] = value;
    setNodeAngle(ghostGraph, state.joints[jointIndex].name, value);
    updateArmVisibility(armIndex);
    refreshDynamicHandle(armIndex, jointIndex);
    refreshHandleVisibility();
    updateRow(armIndex, jointIndex);
    updateDeltaOutput();
    ghostGraph.root.updateMatrixWorld(true);
    render();
    return value;
  }

  function setMeasured(measured) {
    if (!measured || typeof measured !== "object" || Array.isArray(measured)) {
      throw new TypeError("measured state must be a by-name map");
    }
    const updates = [];
    for (const armIndex of armIndices) {
      const state = stateFor(armIndex);
      state.joints.forEach((joint, jointIndex) => {
        if (!Object.hasOwn(measured, joint.name)) {
          return;
        }
        const value = measured[joint.name];
        if (!Number.isFinite(value)) {
          throw new TypeError(`measured joint ${joint.name} must be finite`);
        }
        updates.push({armIndex, jointIndex, value});
      });
    }
    // Validation above is deliberately a separate pass: one malformed known
    // joint rejects the entire sample without partially moving either arm.
    for (const {armIndex, jointIndex, value} of updates) {
      const state = stateFor(armIndex);
      state.measured[jointIndex] = value;
      if (!Number.isFinite(state.ghost[jointIndex])) {
        applyGhostValue(armIndex, jointIndex, value);
      }
      refreshDynamicHandle(armIndex, jointIndex);
      updateRow(armIndex, jointIndex);
    }
    for (const armIndex of armIndices) {
      const state = stateFor(armIndex);
      updateArmVisibility(armIndex);
    }
    refreshHandleVisibility();
    updateDeltaOutput();
    render();
  }

  function setGhost(armIndex, positions) {
    const values = cloneSeven(positions, "positions");
    values.forEach((value, jointIndex) => applyGhostValue(armIndex, jointIndex, value));
  }

  function syncGhostToMeasured(armIndex) {
    const state = stateFor(armIndex);
    state.measured.forEach((value, jointIndex) => {
      if (Number.isFinite(value)) {
        applyGhostValue(armIndex, jointIndex, value);
      }
    });
  }

  function setFence(armIndex, fence) {
    if (!fence || typeof fence !== "object") {
      throw new TypeError("fence must be an object");
    }
    const lower = cloneSeven(fence.lower, "fence.lower");
    const upper = cloneSeven(fence.upper, "fence.upper");
    if (fence.source !== "urdf" && fence.source !== "session") {
      throw new TypeError("fence.source must be 'urdf' or 'session'");
    }
    const state = stateFor(armIndex);
    lower.forEach((value, index) => {
      if (value < state.urdfLower[index]
          || upper[index] > state.urdfUpper[index]) {
        throw new RangeError(`fence for joint ${index + 1} may only narrow the URDF limits`);
      }
      if (value > upper[index]) {
        throw new RangeError(`fence lower exceeds upper for joint ${index + 1}`);
      }
      if (fence.source === "urdf"
          && (value !== state.urdfLower[index] || upper[index] !== state.urdfUpper[index])) {
        throw new RangeError("source 'urdf' requires the exact URDF limits");
      }
    });
    state.fenceLower = lower;
    state.fenceUpper = upper;
    state.fenceSource = fence.source;
    for (let jointIndex = 0; jointIndex < 7; jointIndex += 1) {
      buildHandle(armIndex, jointIndex);
      if (Number.isFinite(state.ghost[jointIndex])) {
        applyGhostValue(armIndex, jointIndex, state.ghost[jointIndex]);
      }
    }
    refreshRows(armIndex);
    refreshHandleVisibility();
    render();
  }

  function setGhostVisible(armIndex, visible) {
    if (typeof visible !== "boolean") {
      throw new TypeError("ghost visibility must be a boolean");
    }
    stateFor(armIndex).requestedVisible = visible;
    stateFor(armIndex).visibilityExplicit = true;
    const checkbox = ui.panel && ui.panel.querySelector(
      `[data-ghost-visible="${armIndex}"]`,
    );
    if (checkbox) {
      checkbox.checked = visible;
    }
    updateArmVisibility(armIndex);
    refreshHandleVisibility();
    render();
  }

  function selectArm(armIndex) {
    stateFor(armIndex);
    if (drag) {
      finishDrag(false);
    }
    selectedArm = armIndex;
    for (const candidate of armIndices) {
      if (!stateFor(candidate).visibilityExplicit) {
        stateFor(candidate).requestedVisible = candidate === armIndex;
      }
      updateArmVisibility(candidate);
      const section = ui.panel && ui.panel.querySelector(`[data-arm-panel="${candidate}"]`);
      if (section) {
        section.classList.toggle("active", candidate === armIndex);
      }
      const checkbox = ui.panel && ui.panel.querySelector(
        `[data-ghost-visible="${candidate}"]`,
      );
      if (checkbox) {
        checkbox.checked = stateFor(candidate).requestedVisible;
      }
    }
    refreshRows();
    refreshHandleVisibility();
    render();
    if (typeof ui.onSelectArm === "function") {
      ui.onSelectArm(armIndex);
    }
  }

  function selectJoint(armIndex, jointIndex) {
    requireJointIndex(jointIndex);
    selectArm(armIndex);
    selectedJoint = jointIndex;
    refreshRows();
    refreshHandleVisibility();
    render();
  }

  function deselectJoint() {
    selectedJoint = null;
    refreshRows();
    refreshHandleVisibility();
    render();
  }

  function setEnabled(nextEnabled) {
    if (typeof nextEnabled !== "boolean") {
      throw new TypeError("enabled must be a boolean");
    }
    if (!nextEnabled && drag) {
      finishDrag(true);
    }
    enabled = nextEnabled;
    if (ui.panel) {
      ui.panel.classList.toggle("read-only", !enabled);
      ui.panel.setAttribute("aria-disabled", enabled ? "false" : "true");
    }
    if (ui.resetButton) {
      ui.resetButton.disabled = !enabled;
    }
    refreshRows();
    refreshHandleVisibility();
    updateApplyPlaceholder();
  }

  function setStale(nextStale) {
    stale = Boolean(nextStale);
    if (ui.panel) {
      ui.panel.classList.toggle("state-stale", stale);
    }
    updateApplyPlaceholder();
  }

  function buildPanel() {
    if (!ui.panel) {
      return;
    }
    ui.panel.replaceChildren();
    for (const armIndex of armIndices) {
      const state = stateFor(armIndex);
      const section = document.createElement("section");
      section.className = `arm-joints${armIndex === initialArm ? " active" : ""}`;
      section.dataset.armPanel = String(armIndex);
      section.setAttribute("aria-labelledby", `ghost-arm-${armIndex}-heading`);

      const heading = document.createElement("div");
      heading.className = "arm-heading";
      const armButton = document.createElement("button");
      armButton.type = "button";
      armButton.className = "arm-select";
      armButton.dataset.selectArm = String(armIndex);
      armButton.id = `ghost-arm-${armIndex}-heading`;
      armButton.textContent = `Arm ${armIndex} · ${state.armId}`;
      const visibilityLabel = document.createElement("label");
      visibilityLabel.className = "ghost-visibility";
      const visibility = document.createElement("input");
      visibility.type = "checkbox";
      visibility.dataset.ghostVisible = String(armIndex);
      visibility.checked = armIndex === initialArm;
      visibilityLabel.append(visibility, document.createTextNode(" ghost"));
      heading.append(armButton, visibilityLabel);
      section.append(heading);

      const list = document.createElement("div");
      list.className = "joint-list";
      list.setAttribute("role", "list");
      state.joints.forEach((joint, jointIndex) => {
        const row = document.createElement("div");
        row.className = "joint-row";
        row.dataset.armIndex = String(armIndex);
        row.dataset.jointIndex = String(jointIndex);
        row.setAttribute("role", "listitem");

        const button = document.createElement("button");
        button.type = "button";
        button.className = "joint-select";
        button.dataset.selectJoint = `${armIndex}:${jointIndex}`;
        button.setAttribute("aria-pressed", "false");
        const name = document.createElement("strong");
        name.textContent = `J${jointIndex + 1}`;
        const measured = document.createElement("span");
        measured.className = "joint-measured";
        const ghost = document.createElement("span");
        ghost.className = "joint-ghost-value";
        button.append(name, measured, ghost);

        const bar = document.createElement("div");
        bar.className = "joint-limit-bar";
        bar.setAttribute("aria-hidden", "true");
        const measuredMarker = document.createElement("i");
        measuredMarker.className = "measured-marker";
        const ghostMarker = document.createElement("i");
        ghostMarker.className = "ghost-marker";
        bar.append(measuredMarker, ghostMarker);

        const label = document.createElement("label");
        label.className = "joint-number";
        const labelText = document.createElement("span");
        labelText.textContent = "degrees";
        const input = document.createElement("input");
        input.type = "number";
        input.step = "0.1";
        input.inputMode = "decimal";
        input.dataset.jointInput = `${armIndex}:${jointIndex}`;
        input.setAttribute("aria-label", `${joint.name} ghost angle in degrees`);
        label.append(labelText, input);

        const status = document.createElement("span");
        status.className = "joint-limit-status";
        status.setAttribute("aria-live", "polite");
        row.append(button, bar, label, status);
        list.append(row);
        rowElements.set(`${armIndex}:${jointIndex}`, {
          root: row,
          button,
          measured,
          ghost,
          input,
          status,
          measuredMarker,
          ghostMarker,
        });
      });
      section.append(list);
      ui.panel.append(section);
    }
  }

  function pointerRay(event) {
    const bounds = canvas.getBoundingClientRect();
    pointerNdc.set(
      2 * (event.clientX - bounds.left) / bounds.width - 1,
      1 - 2 * (event.clientY - bounds.top) / bounds.height,
    );
    raycaster.setFromCamera(pointerNdc, camera);
    return raycaster.ray;
  }

  function angleInDragPlane(point, activeDrag) {
    worldPoint.copy(point).sub(activeDrag.origin);
    if (worldPoint.length() < 0.02) {
      return null;
    }
    return Math.atan2(worldPoint.dot(activeDrag.basisV), worldPoint.dot(activeDrag.basisU));
  }

  function planeHit(event, activeDrag) {
    pointerRay(event);
    return raycaster.ray.intersectPlane(activeDrag.plane, worldPoint)
      ? worldPoint.clone()
      : null;
  }

  function beginDrag(event, handle) {
    if (!enabled || selectedArm !== handle.armIndex || selectedJoint !== handle.jointIndex) {
      return false;
    }
    const state = stateFor(handle.armIndex);
    const q0 = state.ghost[handle.jointIndex];
    if (!Number.isFinite(q0)) {
      return false;
    }
    handle.group.updateWorldMatrix(true, false);
    handle.group.getWorldPosition(worldOrigin);
    const worldQuaternion = handle.group.getWorldQuaternion(new three.Quaternion());
    const basisU = new three.Vector3(1, 0, 0).applyQuaternion(worldQuaternion).normalize();
    const basisV = new three.Vector3(0, 1, 0).applyQuaternion(worldQuaternion).normalize();
    worldAxis.crossVectors(basisU, basisV).normalize();
    dragPlane.setFromNormalAndCoplanarPoint(worldAxis, worldOrigin);
    const activeDrag = {
      pointerId: event.pointerId,
      armIndex: handle.armIndex,
      jointIndex: handle.jointIndex,
      q0,
      total: 0,
      lastAngle: 0,
      origin: worldOrigin.clone(),
      basisU,
      basisV,
      plane: dragPlane.clone(),
    };
    const hit = planeHit(event, activeDrag);
    const anchor = hit && angleInDragPlane(hit, activeDrag);
    if (anchor === null || anchor === undefined) {
      return false;
    }
    activeDrag.lastAngle = anchor;
    drag = activeDrag;
    orbitControls.cancel();
    orbitControls.setEnabled(false);
    canvas.classList.add("joint-dragging");
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch (_error) {
      // Synthetic PointerEvents do not become active pointers in Chromium.
      // A real primary pointer always takes this path successfully.
    }
    return true;
  }

  function finishDrag(restore) {
    if (!drag) {
      return false;
    }
    const activeDrag = drag;
    drag = null;
    if (restore) {
      applyGhostValue(activeDrag.armIndex, activeDrag.jointIndex, activeDrag.q0);
    }
    try {
      if (canvas.hasPointerCapture(activeDrag.pointerId)) {
        canvas.releasePointerCapture(activeDrag.pointerId);
      }
    } catch (_error) {
      // See the synthetic PointerEvent note in beginDrag().
    }
    canvas.classList.remove("joint-dragging");
    orbitControls.setEnabled(enabled);
    return true;
  }

  function suppressEvent(event) {
    event.preventDefault();
    event.stopImmediatePropagation();
  }

  function isEffectivelyVisible(object) {
    let cursor = object;
    while (cursor) {
      if (!cursor.visible) {
        return false;
      }
      cursor = cursor.parent;
    }
    return true;
  }

  function onPointerDown(event) {
    if (!enabled || event.button !== 0) {
      return;
    }
    pointerRay(event);
    const intersections = raycaster.intersectObjects(
      pickTargets.filter(isEffectivelyVisible),
      false,
    );
    const knobHit = intersections.find((hit) => hit.object.userData.handleKind === "knob"
      && hit.object.userData.armIndex === selectedArm
      && hit.object.userData.jointIndex === selectedJoint);
    if (knobHit) {
      suppressEvent(event);
      beginDrag(event, handles.get(`${selectedArm}:${selectedJoint}`));
      return;
    }
    const handleHit = intersections[0];
    if (handleHit) {
      suppressEvent(event);
      selectJoint(handleHit.object.userData.armIndex, handleHit.object.userData.jointIndex);
      return;
    }
    const meshHits = raycaster.intersectObjects(
      [...solidGraph.linkMeshes, ...ghostGraph.linkMeshes].filter(isEffectivelyVisible),
      false,
    );
    for (const hit of meshHits) {
      const selection = armFromLinkName(hit.object.userData.linkName);
      if (selection) {
        suppressEvent(event);
        selectJoint(selection.armIndex, selection.jointIndex);
        return;
      }
    }
  }

  function onPointerMove(event) {
    if (!drag || event.pointerId !== drag.pointerId) {
      return;
    }
    suppressEvent(event);
    const hit = planeHit(event, drag);
    const angle = hit && angleInDragPlane(hit, drag);
    if (angle === null || angle === undefined) {
      return;
    }
    drag.total += shortestAngleDiff(angle, drag.lastAngle);
    drag.lastAngle = angle;
    applyGhostValue(drag.armIndex, drag.jointIndex, drag.q0 + drag.total);
  }

  function onPointerUp(event) {
    if (drag && event.pointerId === drag.pointerId) {
      suppressEvent(event);
      finishDrag(false);
    }
  }

  function onBlur() {
    finishDrag(false);
  }

  function onPanelClick(event) {
    if (!enabled) {
      return;
    }
    const armButton = event.target.closest("[data-select-arm]");
    if (armButton) {
      selectArm(Number(armButton.dataset.selectArm));
      return;
    }
    const jointButton = event.target.closest("[data-select-joint]");
    if (jointButton) {
      const [armIndex, jointIndex] = jointButton.dataset.selectJoint.split(":").map(Number);
      selectJoint(armIndex, jointIndex);
    }
  }

  function onPanelChange(event) {
    const visibility = event.target.closest("[data-ghost-visible]");
    if (visibility) {
      setGhostVisible(Number(visibility.dataset.ghostVisible), visibility.checked);
      return;
    }
    const input = event.target.closest("[data-joint-input]");
    if (!input || !enabled) {
      return;
    }
    if (input.value.trim() === "") {
      return;
    }
    const [armIndex, jointIndex] = input.dataset.jointInput.split(":").map(Number);
    const degrees = Number(input.value);
    if (Number.isFinite(degrees)) {
      selectJoint(armIndex, jointIndex);
      applyGhostValue(armIndex, jointIndex, degrees / DEGREES_PER_RADIAN);
    } else {
      updateRow(armIndex, jointIndex);
    }
  }

  function onKeyDown(event) {
    if (disposed) {
      return;
    }
    const inNumericInput = event.target instanceof HTMLInputElement;
    if (event.key === "Escape") {
      event.preventDefault();
      if (!finishDrag(true)) {
        deselectJoint();
      }
      return;
    }
    if (!enabled || inNumericInput) {
      return;
    }
    if (event.key === "r" || event.key === "R") {
      event.preventDefault();
      syncGhostToMeasured(selectedArm);
      return;
    }
    if (event.key === "Tab" && selectedJoint !== null) {
      event.preventDefault();
      selectedJoint = (selectedJoint + (event.shiftKey ? 6 : 1)) % 7;
      refreshRows();
      refreshHandleVisibility();
      render();
      return;
    }
    if (selectedJoint === null) {
      return;
    }
    const negative = event.key === "ArrowLeft" || event.key === "-" || event.key === "_";
    const positive = event.key === "ArrowRight" || event.key === "+" || event.key === "=";
    if (!negative && !positive) {
      return;
    }
    event.preventDefault();
    const step = event.shiftKey ? FINE_JOG_STEP : jogStepRad;
    const value = stateFor(selectedArm).ghost[selectedJoint];
    if (Number.isFinite(value)) {
      applyGhostValue(selectedArm, selectedJoint, value + (positive ? step : -step));
    }
  }

  function onReset() {
    if (enabled) {
      syncGhostToMeasured(selectedArm);
    }
  }

  function projectCoordinates(point) {
    point.project(camera);
    const bounds = canvas.getBoundingClientRect();
    return {
      clientX: bounds.left + (point.x + 1) * bounds.width / 2,
      clientY: bounds.top + (1 - point.y) * bounds.height / 2,
    };
  }

  function coordinatesForAngle(armIndex, jointIndex, angle, radiusScale = 1) {
    const handle = handles.get(`${armIndex}:${jointIndex}`);
    if (!handle) {
      throw new Error("handle is unavailable");
    }
    handle.group.updateWorldMatrix(true, false);
    const point = new three.Vector3(
      handle.radius * radiusScale * Math.cos(angle),
      handle.radius * radiusScale * Math.sin(angle),
      0,
    );
    handle.group.localToWorld(point);
    return projectCoordinates(point);
  }

  function coordinatesForObject(object) {
    if (!object || !object.geometry || !object.geometry.boundingSphere) {
      throw new TypeError("object needs geometry with a bounding sphere");
    }
    object.updateWorldMatrix(true, false);
    const point = object.geometry.boundingSphere.center.clone();
    object.localToWorld(point);
    return projectCoordinates(point);
  }

  buildPanel();
  for (const armIndex of armIndices) {
    for (let jointIndex = 0; jointIndex < 7; jointIndex += 1) {
      buildHandle(armIndex, jointIndex);
    }
    updateArmVisibility(armIndex);
  }
  selectArm(initialArm);
  refreshRows();
  refreshHandleVisibility();
  updateApplyPlaceholder();

  addTrackedListener(listeners, canvas, "pointerdown", onPointerDown, true);
  addTrackedListener(listeners, canvas, "pointermove", onPointerMove, true);
  addTrackedListener(listeners, canvas, "pointerup", onPointerUp, true);
  addTrackedListener(listeners, canvas, "pointercancel", onPointerUp, true);
  addTrackedListener(listeners, window, "blur", onBlur);
  addTrackedListener(listeners, window, "keydown", onKeyDown);
  if (ui.panel) {
    addTrackedListener(listeners, ui.panel, "click", onPanelClick);
    addTrackedListener(listeners, ui.panel, "change", onPanelChange);
  }
  if (ui.resetButton) {
    addTrackedListener(listeners, ui.resetButton, "click", onReset);
  }

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    finishDrag(false);
    while (listeners.length > 0) {
      const listener = listeners.pop();
      listener.target.removeEventListener(
        listener.type,
        listener.listener,
        listener.options,
      );
    }
    for (const timer of timers.values()) {
      clearTimeout(timer);
    }
    timers.clear();
    for (const handle of handles.values()) {
      removeHandleFromPickTargets(handle);
      if (handle.group.parent) {
        handle.group.parent.remove(handle.group);
      }
      disposeStaticParts(handle);
    }
    handles.clear();
    pickTargets.length = 0;
  }

  return {
    setMeasured,
    setFence,
    setGhost,
    syncGhostToMeasured,
    setGhostVisible,
    selectArm,
    selectJoint,
    setEnabled,
    setStale,
    clampJoint,
    dispose,
    get listenerCount() {
      return listeners.length;
    },
    getGhost(armIndex) {
      return [...stateFor(armIndex).ghost];
    },
    getMeasured(armIndex) {
      return [...stateFor(armIndex).measured];
    },
    testing: {
      handles,
      rows: rowElements,
      allocationStats,
      coordinatesForAngle,
      coordinatesForObject,
      selectionForLink: armFromLinkName,
      get selectedArm() {
        return selectedArm;
      },
      get selectedJoint() {
        return selectedJoint;
      },
      get dragging() {
        return Boolean(drag);
      },
      get enabled() {
        return enabled;
      },
    },
  };
}
