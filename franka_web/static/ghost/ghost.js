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

// The mount seam.
//
// This module tree receives URLs and callbacks and returns geometry and joint
// numbers. It never learns that HTTP, ROS or a server exist: it performs no
// I/O of its own beyond fetching the two URLs it is handed and the mesh assets
// those name, and every solve leaves through onSolveRequest. The driver is the
// only translator.

import {parseUrdf} from "./urdf.js";
import {mountSolidScene} from "./scene.js";
import {createGhostState} from "./ghost_state.js";
import {createHandDrag} from "./hand_drag.js";

const OPTION_KEYS = Object.freeze([
  "urdfUrl", "manifestUrl", "assetBase", "arms", "initialArm",
  "cell", "theme", "onSolveRequest", "onGhostChanged",
]);

export const HANDLE_METHODS = Object.freeze([
  "dispose", "getRenderedPose", "selectArm", "setCell", "setEnabled",
  "setGhost", "setGhostVisible", "setMeasured", "setStale", "setTheme",
  "setVerdict", "syncGhostToMeasured",
]);

const liveMounts = new Set();

/** Live listener total across every mount; the browser suite drives it to 0. */
export function activeListenerCount() {
  let total = 0;
  for (const mount of liveMounts) {
    total += mount.listenerCount();
  }
  return total;
}

function requireString(value, name) {
  if (typeof value !== "string" || value.length === 0) {
    throw new TypeError(`${name} must be a non-empty string`);
  }
  return value;
}

function validateOptions(options) {
  if (!options || typeof options !== "object" || Array.isArray(options)) {
    throw new TypeError("options must be an object");
  }
  if (typeof options.onApply !== "undefined") {
    // A positive assertion, not an oversight: there is no web-side motion path
    // from a ghost pose, so there is no hook here for one to attach to.
    throw new TypeError("onApply is not part of this seam; the ghost commands nothing");
  }
  for (const key of Object.keys(options)) {
    if (!OPTION_KEYS.includes(key)) {
      throw new TypeError(`unknown mount option ${key}`);
    }
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
  const armIndices = arms.map((arm) => arm.armIndex);
  if (new Set(armIndices).size !== arms.length) {
    throw new TypeError("arms must not contain duplicates");
  }
  if (!armIndices.includes(options.initialArm)) {
    throw new TypeError("initialArm must identify an arm configured for this mount");
  }
  if (typeof options.onSolveRequest !== "function") {
    throw new TypeError("onSolveRequest must be a function");
  }
  if (typeof options.onGhostChanged !== "function"
      && typeof options.onGhostChanged !== "undefined") {
    throw new TypeError("onGhostChanged must be a function when given");
  }
  return {
    urdfUrl,
    manifestUrl,
    assetBase,
    arms,
    armIndices,
    initialArm: options.initialArm,
    cell: options.cell === undefined ? null : options.cell,
    theme: options.theme === "dark" ? "dark" : "light",
    onSolveRequest: options.onSolveRequest,
    onGhostChanged: options.onGhostChanged || null,
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

/** Mount the scene into `container`. Resolves with the handle, or rejects. */
export async function mount(container, options) {
  if (!(container instanceof Element)) {
    throw new TypeError("container must be a DOM Element");
  }
  const three = globalThis.THREE;
  if (!three || String(three.REVISION) !== "111") {
    throw new Error(
      `three.js r111 is required; found revision ${three && three.REVISION || "none"}`,
    );
  }
  const config = validateOptions(options);
  const abort = new AbortController();

  let scene = null;
  let ghostState = null;
  let handDrag = null;
  let resizeObserver = null;
  let disposed = false;

  function listenerCount() {
    return (scene ? scene.listenerCount : 0)
      + (handDrag ? handDrag.listenerCount : 0)
      + (ghostState ? ghostState.listenerCount : 0)
      + (resizeObserver ? 1 : 0);
  }
  const registration = {listenerCount};

  try {
    const [urdfText, manifest] = await Promise.all([
      fetchChecked(config.urdfUrl, abort.signal).then((response) => response.text()),
      fetchChecked(config.manifestUrl, abort.signal).then((response) => response.json()),
    ]);
    const model = parseUrdf(urdfText, manifest);

    scene = await mountSolidScene(container, {
      model,
      manifest,
      assetBase: config.assetBase,
      assetFetch: (url) => fetchChecked(url, abort.signal),
      arms: config.arms,
      theme: config.theme,
      cell: config.cell,
      onTick(now) {
        if (ghostState) {
          ghostState.applyInterpolation(now);
        }
      },
      onContextChange(alive) {
        // A lost drawing context is the driver's to explain -- it owns every
        // sentence the panel shows -- so it travels the one status channel and
        // this module says nothing about it itself.
        config.onSolveRequest({kind: "status", armIndex: null, text: null,
                               verdict: null, drawing: alive === true});
      },
    });

    ghostState = createGhostState({
      three,
      model,
      solidGraph: scene.solidGraph,
      ghostGraph: scene.ghostGraph,
      armIndices: config.armIndices,
      initialArm: config.initialArm,
      render: scene.requestRender,
      onChange(armIndex, positions7, differs) {
        if (config.onGhostChanged) {
          config.onGhostChanged(armIndex, positions7, differs);
        }
      },
    });

    handDrag = createHandDrag({
      three,
      canvas: scene.canvas,
      camera: scene.camera,
      model,
      ghostGraph: scene.ghostGraph,
      ghostState,
      orbitControls: scene.orbitControls,
      render: scene.requestRender,
      onSolveRequest: config.onSolveRequest,
      palette: scene.palette,
      onStatus(update) {
        // One callback, three request shapes. The module never learns that an
        // endpoint exists; the driver routes on `kind`, and a status carries
        // no request at all.
        config.onSolveRequest({
          kind: "status",
          armIndex: update.armIndex,
          text: update.text,
          verdict: update.verdict,
        });
      },
    });
  } catch (error) {
    abort.abort();
    if (handDrag) {
      handDrag.dispose();
    }
    if (ghostState) {
      ghostState.dispose();
    }
    if (scene) {
      scene.dispose();
    }
    container.replaceChildren();
    throw error;
  }

  if (typeof ResizeObserver === "function") {
    // window.resize never fires when the PANEL changes size -- expanding the
    // collapsed bar, the desktop split appearing, a device toolbar -- and
    // without this the canvas keeps rendering at the aspect it was mounted at.
    //
    // The work is deferred to the next frame and coalesced, because resizing
    // the renderer inside the observation callback re-enters the observer and
    // the browser reports "ResizeObserver loop completed with undelivered
    // notifications" as an uncaught error -- once per settling frame, in every
    // viewer's console, for a resize that was going to happen anyway.
    let resizeQueued = false;
    resizeObserver = new ResizeObserver(function () {
      if (resizeQueued || disposed) {
        return;
      }
      resizeQueued = true;
      requestAnimationFrame(function () {
        resizeQueued = false;
        if (!disposed) {
          scene.resize();
        }
      });
    });
    resizeObserver.observe(container);
  }
  liveMounts.add(registration);

  function refreshArm(armIndex) {
    scene.setGhostRootVisible(armIndex, ghostState.isGhostVisible(armIndex));
    handDrag.refresh();
  }

  function setMeasured(armIndex, positions7, meta) {
    ghostState.setMeasured(armIndex, positions7, meta);
    scene.setArmPresent(armIndex, ghostState.isPresent(armIndex));
    refreshArm(armIndex);
  }

  function setStale(armIndex, isStale) {
    ghostState.setStale(armIndex, isStale);
    scene.setSolidStale(armIndex, isStale);
  }

  function setGhost(armIndex, positions7) {
    ghostState.setGhost(armIndex, positions7);
    refreshArm(armIndex);
  }

  function syncGhostToMeasured(armIndex) {
    ghostState.syncGhostToMeasured(armIndex);
    // The target orientation is re-captured here and nowhere else: G2 authors
    // position only, so "reset" is the one moment the hand's orientation is
    // allowed to change.
    handDrag.captureTarget(armIndex);
    refreshArm(armIndex);
  }

  function setGhostVisible(armIndex, visible) {
    ghostState.setGhostVisible(armIndex, visible);
    if (visible) {
      ghostState.syncGhostToMeasured(armIndex);
      handDrag.captureTarget(armIndex);
    }
    refreshArm(armIndex);
  }

  function selectArm(armIndex) {
    ghostState.selectArm(armIndex);
    handDrag.refresh();
  }

  function setEnabled(isEnabled) {
    handDrag.setEnabled(isEnabled === true);
  }

  function setVerdict(armIndex, verdict) {
    ghostState.setVerdict(armIndex, verdict);
    scene.setGhostTint(armIndex, verdict);
  }

  function setCell(cell) {
    scene.setCell(cell === undefined ? null : cell);
  }

  function setTheme(name, palette) {
    scene.setTheme(name, palette);
    handDrag.setPalette(scene.palette);
  }

  /**
   * What the user currently SEES for this arm: the ghost pose when its ghost
   * is shown, the interpolated measured pose when it is not, and null when the
   * arm is not in the session at all.
   */
  function getRenderedPose(armIndex) {
    if (!ghostState.isPresent(armIndex)) {
      return null;
    }
    return ghostState.isGhostVisible(armIndex)
      ? ghostState.getGhost(armIndex)
      : ghostState.measuredNow(armIndex);
  }

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    abort.abort();
    if (resizeObserver) {
      resizeObserver.disconnect();
      resizeObserver = null;
    }
    handDrag.dispose();
    ghostState.dispose();
    scene.dispose();
    liveMounts.delete(registration);
    container.replaceChildren();
  }

  return Object.freeze({
    dispose,
    getRenderedPose,
    selectArm,
    setCell,
    setEnabled,
    setGhost,
    setGhostVisible,
    setMeasured,
    setStale,
    setTheme,
    setVerdict,
    syncGhostToMeasured,
  });
}
