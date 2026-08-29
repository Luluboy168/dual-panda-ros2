// [DURABLE] Frozen Session C mount seam; moves unchanged into franka_web.

import {buildGhostApplyEvent, toWebV1Payload, URDF_LOWER, URDF_UPPER} from "./apply.js";
import {parseUrdf} from "./urdf.js";
import {mountSolidScene} from "./scene.js";

const STALE_AFTER_MS = 500;
let mountNumber = 0;

function requireString(value, name) {
  if (typeof value !== "string" || value.length === 0) {
    throw new TypeError(`${name} must be a non-empty string`);
  }
  return value;
}

function requireArmIndex(value, configuredArms) {
  if (!Number.isInteger(value) || !configuredArms.has(value)) {
    throw new RangeError("armIndex must identify an arm configured for this mount");
  }
  return value;
}

function requireSevenFinite(values, name) {
  if (!Array.isArray(values) || values.length !== 7
      || values.some((value) => typeof value !== "number" || !Number.isFinite(value))) {
    throw new TypeError(`${name} must contain exactly seven finite numbers`);
  }
  return [...values];
}

function validateOptions(options) {
  if (!options || typeof options !== "object" || Array.isArray(options)) {
    throw new TypeError("options must be an object");
  }
  const urdfUrl = requireString(options.urdfUrl, "urdfUrl");
  const manifestUrl = requireString(options.manifestUrl, "manifestUrl");
  const assetBase = requireString(options.assetBase, "assetBase");
  if (!Array.isArray(options.arms) || options.arms.length < 1 || options.arms.length > 2) {
    throw new TypeError("arms must contain one or two arm descriptors");
  }
  const arms = options.arms.map((arm) => {
    if (!arm || typeof arm !== "object"
        || !Number.isInteger(arm.armIndex)
        || (arm.armIndex !== 1 && arm.armIndex !== 2)
        || arm.armId !== `panda${arm.armIndex}`) {
      throw new TypeError("each arm must pair armIndex 1/2 with armId panda1/panda2");
    }
    return Object.freeze({armIndex: arm.armIndex, armId: arm.armId});
  });
  if (new Set(arms.map((arm) => arm.armIndex)).size !== arms.length) {
    throw new TypeError("arms must not contain duplicates");
  }
  const configuredArms = new Map(arms.map((arm) => [arm.armIndex, arm]));
  requireArmIndex(options.initialArm, configuredArms);
  if (typeof options.jogStepRad !== "number" || !Number.isFinite(options.jogStepRad)
      || options.jogStepRad <= 0) {
    throw new TypeError("jogStepRad must be positive and finite");
  }
  if (typeof options.onApply !== "function") {
    throw new TypeError("onApply must be a function");
  }
  return {
    urdfUrl,
    manifestUrl,
    assetBase,
    arms,
    configuredArms,
    initialArm: options.initialArm,
    jogStepRad: options.jogStepRad,
    onApply: options.onApply,
  };
}

function buildMountUi(container) {
  mountNumber += 1;
  const headingId = `franka-ghost-joint-editor-${mountNumber}`;
  const workspace = document.createElement("div");
  workspace.className = "workspace ghost-module";
  workspace.dataset.ghostReady = "false";

  const viewport = document.createElement("section");
  viewport.className = "viewport";
  viewport.setAttribute("aria-label", "Robot viewport");
  const startup = document.createElement("div");
  startup.className = "startup";
  startup.textContent = "Loading model and WebGL renderer…";
  viewport.append(startup);

  const controls = document.createElement("aside");
  controls.className = "controls";
  controls.setAttribute("aria-labelledby", headingId);
  const controlsHeading = document.createElement("div");
  controlsHeading.className = "controls-heading";
  const titleGroup = document.createElement("div");
  const eyebrow = document.createElement("p");
  eyebrow.className = "eyebrow";
  eyebrow.textContent = "Selected-arm ghost";
  const title = document.createElement("h2");
  title.id = headingId;
  title.textContent = "Joint editor";
  titleGroup.append(eyebrow, title);
  const editMode = document.createElement("span");
  editMode.className = "edit-mode";
  editMode.textContent = "waiting";
  controlsHeading.append(titleGroup, editMode);

  const help = document.createElement("p");
  help.className = "controls-help";
  help.textContent = "Select a link, arc, handle, or row. Drag the knob in the joint plane; arrow keys nudge the selected joint.";
  const panel = document.createElement("div");
  panel.className = "joint-panels";
  panel.setAttribute("aria-label", "Per-arm joint values");
  const actions = document.createElement("div");
  actions.className = "ghost-actions";
  const resetButton = document.createElement("button");
  resetButton.type = "button";
  resetButton.className = "secondary-action";
  resetButton.textContent = "Reset ghost (R)";
  const deltaReadout = document.createElement("div");
  deltaReadout.className = "delta-readout";
  deltaReadout.append(document.createTextNode("max |ghost − measured|"));
  const deltaOutput = document.createElement("output");
  deltaOutput.textContent = "0.0°";
  deltaReadout.append(deltaOutput);
  const applyButton = document.createElement("button");
  applyButton.type = "button";
  applyButton.className = "apply-action";
  applyButton.disabled = true;
  applyButton.textContent = "Apply — sends target, arm ramps at the configured slew";
  const applyStatus = document.createElement("p");
  applyStatus.className = "apply-note ghost-apply-status";
  applyStatus.setAttribute("role", "status");
  applyStatus.textContent = "Waiting for a complete, fresh measured pose.";
  actions.append(resetButton, deltaReadout, applyButton, applyStatus);
  controls.append(controlsHeading, help, panel, actions);
  workspace.append(viewport, controls);
  container.replaceChildren(workspace);
  return {
    workspace,
    viewport,
    controls,
    panel,
    resetButton,
    deltaOutput,
    applyButton,
    applyStatus,
    editMode,
  };
}

function fetchChecked(url, signal) {
  return fetch(url, {signal}).then((response) => {
    if (!response.ok) {
      throw new Error(`${url}: HTTP ${response.status}`);
    }
    return response;
  });
}

/** Mount the frozen Contract C3 joint-space ghost module. */
export function mount(container, options) {
  if (!(container instanceof Element)) {
    throw new TypeError("container must be a DOM Element");
  }
  const three = globalThis.THREE;
  if (!three || String(three.REVISION) !== "111") {
    throw new Error(`three.js r111 is required; found revision ${three && three.REVISION || "none"}`);
  }
  const config = validateOptions(options);
  const ui = buildMountUi(container);
  const fences = new Map();
  const measured = new Map();
  const pendingGhost = new Map();
  const visibility = new Map();
  const visibilityExplicit = new Set();
  for (const arm of config.arms) {
    fences.set(arm.armIndex, {
      lower: [...URDF_LOWER],
      upper: [...URDF_UPPER],
      source: "urdf",
    });
    measured.set(arm.armIndex, new Array(7).fill(Number.NaN));
    visibility.set(arm.armIndex, arm.armIndex === config.initialArm);
  }

  let selectedArm = config.initialArm;
  let sceneHandle = null;
  let ready = false;
  let disposed = false;
  let requestedEnabled = true;
  let stale = true;
  let freshnessTimer = null;
  let lastEpoch = 0;
  let applyPending = false;
  let applyFeedback = null;
  const abortController = new AbortController();

  function ensureActive() {
    if (disposed) {
      throw new Error("ghost mount is disposed");
    }
  }

  function selectedPoseReady() {
    return ready
      && measured.get(selectedArm).every(Number.isFinite)
      && sceneHandle.getGhost(selectedArm).every(Number.isFinite);
  }

  function refreshApplyUi() {
    if (disposed) {
      return;
    }
    const editable = ready && requestedEnabled;
    ui.applyButton.disabled = !editable || stale || !selectedPoseReady() || applyPending;
    ui.applyButton.setAttribute("aria-disabled", ui.applyButton.disabled ? "true" : "false");
    ui.editMode.textContent = !ready ? "waiting" : editable ? "editable" : "read-only";
    ui.editMode.classList.toggle("read-only", !editable);
    ui.applyStatus.classList.toggle("error-detail", Boolean(applyFeedback && applyFeedback.error));
    if (!ready) {
      ui.applyStatus.textContent = "Waiting for the model to finish loading.";
    } else if (stale) {
      ui.applyStatus.textContent = "Apply refused: measured joint state is stale.";
    } else if (!requestedEnabled) {
      ui.applyStatus.textContent = "Apply refused: ghost editing is read-only.";
    } else if (!selectedPoseReady()) {
      ui.applyStatus.textContent = "Apply refused: selected arm needs seven finite measured joints.";
    } else if (applyPending) {
      ui.applyStatus.textContent = "Apply validation is in progress…";
    } else if (applyFeedback) {
      ui.applyStatus.textContent = applyFeedback.text;
    } else {
      ui.applyStatus.textContent = "Ready to emit a validated prototype payload; nothing is published.";
    }
  }

  function applyEffectiveState() {
    if (sceneHandle) {
      sceneHandle.setStale(stale);
      // Staleness gates Apply and greys only the solid robot. The ghost stays
      // editable so an operator never mistakes stale input for a lost draft.
      sceneHandle.setEnabled(requestedEnabled);
    }
    refreshApplyUi();
  }

  function markFresh() {
    stale = false;
    clearTimeout(freshnessTimer);
    freshnessTimer = setTimeout(() => {
      if (disposed) {
        return;
      }
      stale = true;
      applyEffectiveState();
    }, STALE_AFTER_MS);
    applyEffectiveState();
  }

  function setMeasured(map) {
    ensureActive();
    if (!map || typeof map !== "object" || Array.isArray(map)) {
      throw new TypeError("measured state must be a by-name map");
    }
    const accepted = [];
    for (const arm of config.arms) {
      for (let index = 0; index < 7; index += 1) {
        const name = `${arm.armId}_joint${index + 1}`;
        if (!Object.hasOwn(map, name)) {
          continue;
        }
        const value = map[name];
        // SESSION_A frame contract section 6.11 rule 2: positions are extracted from the incoming
        // 14-name JointState BY NAME, and "a name that is absent yields null at that index and
        // sets positions_stale: true". franka_web builds this map by zipping joint_names with
        // positions (README wiring 2), so those nulls arrive here verbatim. A null therefore means
        // "no reading for this joint in this frame", which is the same thing an omitted key means:
        // hold the last known value, exactly as unknown names are ignored. Throwing here would
        // take out franka_web's whole state callback over one stale joint name.
        // NaN / Infinity are treated identically -- a non-finite float is also "no reading".
        if (value === null || value === undefined) {
          continue;
        }
        if (typeof value !== "number") {
          // Not a number and not the documented null placeholder: a real caller bug, and the
          // frame is rejected atomically (nothing in it is applied).
          throw new TypeError(`measured joint ${name} must be a number or null`);
        }
        if (!Number.isFinite(value)) {
          continue;
        }
        accepted.push({armIndex: arm.armIndex, index, name, value});
      }
    }
    const acceptedMap = Object.fromEntries(accepted.map(({name, value}) => [name, value]));
    if (sceneHandle) {
      sceneHandle.setMeasured(acceptedMap);
    }
    for (const {armIndex, index, value} of accepted) {
      measured.get(armIndex)[index] = value;
    }
    if (accepted.length > 0) {
      markFresh();
    } else {
      refreshApplyUi();
    }
  }

  function setFence(armIndex, fence) {
    ensureActive();
    requireArmIndex(armIndex, config.configuredArms);
    if (!fence || fence.source !== "session") {
      throw new TypeError("public setFence requires source 'session'");
    }
    const lower = requireSevenFinite(fence.lower, "fence.lower");
    const upper = requireSevenFinite(fence.upper, "fence.upper");
    lower.forEach((value, index) => {
      if (value < URDF_LOWER[index] || upper[index] > URDF_UPPER[index]) {
        throw new RangeError(`fence for joint ${index + 1} may only narrow the URDF limits`);
      }
      if (value > upper[index]) {
        throw new RangeError(`fence lower exceeds upper for joint ${index + 1}`);
      }
    });
    const snapshot = {lower, upper, source: "session"};
    fences.set(armIndex, snapshot);
    if (sceneHandle) {
      sceneHandle.setFence(armIndex, snapshot);
    }
    refreshApplyUi();
  }

  function setGhost(armIndex, positions7) {
    ensureActive();
    requireArmIndex(armIndex, config.configuredArms);
    const positions = requireSevenFinite(positions7, "positions7");
    pendingGhost.set(armIndex, positions);
    if (sceneHandle) {
      sceneHandle.setGhost(armIndex, positions);
    }
    refreshApplyUi();
  }

  function syncGhostToMeasured(armIndex) {
    ensureActive();
    requireArmIndex(armIndex, config.configuredArms);
    if (!measured.get(armIndex).every(Number.isFinite)) {
      throw new Error("cannot sync ghost before all seven measured joints are finite");
    }
    if (sceneHandle) {
      sceneHandle.syncGhostToMeasured(armIndex);
    } else {
      pendingGhost.set(armIndex, [...measured.get(armIndex)]);
    }
    refreshApplyUi();
  }

  function setGhostVisible(armIndex, visible) {
    ensureActive();
    requireArmIndex(armIndex, config.configuredArms);
    if (typeof visible !== "boolean") {
      throw new TypeError("ghost visibility must be a boolean");
    }
    visibility.set(armIndex, visible);
    visibilityExplicit.add(armIndex);
    if (sceneHandle) {
      sceneHandle.setGhostVisible(armIndex, visible);
    }
  }

  function selectArm(armIndex) {
    ensureActive();
    requireArmIndex(armIndex, config.configuredArms);
    selectedArm = armIndex;
    if (sceneHandle) {
      sceneHandle.selectArm(armIndex);
    }
    refreshApplyUi();
  }

  function setEnabled(enabled) {
    ensureActive();
    if (typeof enabled !== "boolean") {
      throw new TypeError("enabled must be a boolean");
    }
    requestedEnabled = enabled;
    applyEffectiveState();
  }

  async function onApplyClick() {
    if (ui.applyButton.disabled || applyPending || disposed) {
      refreshApplyUi();
      return;
    }
    applyFeedback = null;
    applyPending = true;
    refreshApplyUi();
    try {
      const candidateEpoch = lastEpoch + 1;
      const event = buildGhostApplyEvent({
        armIndex: selectedArm,
        positions: sceneHandle.getGhost(selectedArm),
        fence: fences.get(selectedArm),
        measuredAtApply: sceneHandle.getMeasured(selectedArm),
        ghostEpoch: candidateEpoch,
        previousEpoch: lastEpoch || null,
      });
      const payload = toWebV1Payload(event);
      lastEpoch = candidateEpoch;
      ui.applyStatus.textContent = `Emitting arm ${selectedArm} ghost epoch ${candidateEpoch}…`;
      await config.onApply(payload);
      applyFeedback = {
        error: false,
        text: `Apply emitted: arm ${selectedArm}, ghost epoch ${candidateEpoch}.`,
      };
    } catch (error) {
      applyFeedback = {error: true, text: `Apply refused: ${error.message}`};
    } finally {
      applyPending = false;
      refreshApplyUi();
    }
  }
  ui.applyButton.addEventListener("click", onApplyClick);

  Promise.all([
    fetchChecked(config.urdfUrl, abortController.signal).then((response) => response.text()),
    fetchChecked(config.manifestUrl, abortController.signal).then((response) => response.json()),
  ]).then(async ([urdfText, manifest]) => {
    if (disposed) {
      return;
    }
    const model = parseUrdf(urdfText, manifest);
    const mounted = await mountSolidScene(ui.viewport, {
      model,
      manifest,
      assetBase: config.assetBase,
      assetFetch: (url) => fetchChecked(url, abortController.signal),
      initialArm: config.initialArm,
      arms: config.arms,
      jogStepRad: config.jogStepRad,
      ui: {
        panel: ui.panel,
        resetButton: ui.resetButton,
        applyButton: ui.applyButton,
        deltaOutput: ui.deltaOutput,
        onStateChange: refreshApplyUi,
        onSelectArm(armIndex) {
          selectedArm = armIndex;
          refreshApplyUi();
        },
      },
    });
    if (disposed) {
      mounted.dispose();
      return;
    }
    sceneHandle = mounted;
    const initialMap = {};
    for (const arm of config.arms) {
      measured.get(arm.armIndex).forEach((value, index) => {
        if (Number.isFinite(value)) {
          initialMap[`${arm.armId}_joint${index + 1}`] = value;
        }
      });
    }
    if (Object.keys(initialMap).length > 0) {
      sceneHandle.setMeasured(initialMap);
    }
    for (const arm of config.arms) {
      const fence = fences.get(arm.armIndex);
      if (fence.source === "session") {
        sceneHandle.setFence(arm.armIndex, fence);
      }
      if (pendingGhost.has(arm.armIndex)) {
        sceneHandle.setGhost(arm.armIndex, pendingGhost.get(arm.armIndex));
      }
      if (visibilityExplicit.has(arm.armIndex)) {
        sceneHandle.setGhostVisible(arm.armIndex, visibility.get(arm.armIndex));
      }
    }
    sceneHandle.selectArm(selectedArm);
    ready = true;
    ui.workspace.dataset.ghostReady = "true";
    applyEffectiveState();
  }).catch((error) => {
    if (disposed || error.name === "AbortError") {
      return;
    }
    ui.workspace.dataset.ghostReady = "error";
    ui.viewport.replaceChildren();
    const detail = document.createElement("div");
    detail.className = "startup error-detail";
    detail.textContent = error.message;
    ui.viewport.append(detail);
    ui.applyStatus.textContent = `Apply refused: ${error.message}`;
    ui.applyStatus.classList.add("error-detail");
    console.error(error);
  });

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    clearTimeout(freshnessTimer);
    abortController.abort();
    ui.applyButton.removeEventListener("click", onApplyClick);
    if (sceneHandle) {
      sceneHandle.dispose();
      sceneHandle = null;
    }
    ui.workspace.remove();
  }

  return Object.freeze({
    setMeasured,
    setFence,
    setGhost,
    syncGhostToMeasured,
    setGhostVisible,
    selectArm,
    setEnabled,
    dispose,
  });
}
