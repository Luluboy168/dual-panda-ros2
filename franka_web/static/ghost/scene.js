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

// Provenance: the Session C renderer prototype, moved unchanged at the ghost merge.

import {loadMeshGeometries} from "./meshes.js";
import {createCell} from "./cell.js";

// Fallback only. The host page's stylesheet owns these values; the driver
// passes the resolved palette in at mount and on every theme change. The KEY
// SET here is the contract: it must match the documented token list exactly,
// and the browser suite pins the two together.
const BUILTIN_PALETTE = {
  light: {
    sceneBg: "#F3F6F9", grid: "#D7DEE6", gridMajor: "#8494A3",
    cellLine: "#8494A3", cellFloor: "#F3F6F9",
    ghost1: "#2557C7", ghost2: "#7A46B8",
    ghostCollide: "#C23430", ghostUnchecked: "#8494A3",
    handle: "#2557C7", handleActive: "#17222E", handleRefused: "#C23430",
    elbow: "#B26A00", elbowActive: "#7A4400", stale: "#9AA4AE",
    axisX: "#C0392B", axisY: "#1E8449", axisZ: "#2471C7",
  },
  dark: {
    sceneBg: "#1C2531", grid: "#29323F", gridMajor: "#6C7987",
    cellLine: "#6C7987", cellFloor: "#1C2531",
    ghost1: "#7FA5F4", ghost2: "#BC8DF0",
    ghostCollide: "#F1706A", ghostUnchecked: "#6C7987",
    handle: "#7FA5F4", handleActive: "#E6EBF1", handleRefused: "#F1706A",
    elbow: "#E0A33A", elbowActive: "#F5C46A", stale: "#6B747E",
    axisX: "#F0837A", axisY: "#4FBF83", axisZ: "#6FA8F0",
  },
};

export const PALETTE_KEYS = Object.freeze(Object.keys(BUILTIN_PALETTE.light).sort());

// Adaptive raster resolution. The prototype pinned this at 0.5 for a software
// WebGL fallback, which makes a hardware-GL machine look worse than it is.
const RATIO_STEPS = [0.5, 0.75, 1.0, 1.5];
const RATIO_SAMPLE_COUNT = 30;
const RATIO_STEP_DOWN_MS = 33;
const RATIO_STEP_UP_MS = 12;
//: A step in EITHER direction starts the cooldown. Counting it from step-ups
//: alone would let the very first fast window undo a step-down immediately,
//: which is the oscillation this whole ratchet exists to avoid.
const RATIO_STEP_COOLDOWN_MS = 2000;

const DEFAULT_CELL_SPAN_M = 2.4;

function requireThree() {
  const three = globalThis.THREE;
  if (!three || typeof three.WebGLRenderer !== "function") {
    throw new Error("three.js r111 classic global is not loaded");
  }
  if (String(three.REVISION) !== "111") {
    throw new Error(`three.js r111 is required; found revision ${three.REVISION || "unknown"}`);
  }
  return three;
}

function rendererSize(container) {
  const bounds = container.getBoundingClientRect();
  return {
    width: Math.max(1, Math.round(bounds.width || container.clientWidth || 960)),
    height: Math.max(1, Math.round(bounds.height || container.clientHeight || 640)),
  };
}

function disposeMaterialMap(materialMap) {
  for (const materials of materialMap.values()) {
    materials.forEach((material) => material.dispose());
  }
}

function disposeGeometryAssets(geometryAssets) {
  for (const asset of geometryAssets.values()) {
    asset.geometry.dispose();
    asset.materials.forEach((material) => material.dispose());
  }
}

function disposeRenderer(renderer, canvas, alreadyLost) {
  renderer.dispose();
  // Forcing a loss on a context that is ALREADY lost cannot work and makes
  // three.js warn about a missing extension it can no longer query -- noise in
  // every viewer's console for a context that is already gone.
  if (!alreadyLost && typeof renderer.forceContextLoss === "function") {
    renderer.forceContextLoss();
  }
  canvas.remove();
}

function armIndexForLink(linkName) {
  return String(linkName).startsWith("panda1_") ? 1 : 2;
}

function installOrbitControls(three, canvas, camera, render) {
  const target = new three.Vector3(0, 0, 0.45);
  let radius = 2.25;
  let azimuth = 0.78;
  let polar = 1.02;
  // A Map, not a single pointer: two fingers are the only way to zoom or pan
  // on a touch screen, and the prototype bound pan to `button > 0`, which no
  // touch pointer ever reports.
  const pointers = new Map();
  let pinch = null;
  let enabled = true;
  let listenerCount = 0;

  function updateCamera() {
    const sinPolar = Math.sin(polar);
    camera.position.set(
      target.x + radius * sinPolar * Math.cos(azimuth),
      target.y + radius * sinPolar * Math.sin(azimuth),
      target.z + radius * Math.cos(polar),
    );
    camera.up.set(0, 0, 1);
    camera.lookAt(target);
    render();
  }

  function pointerList() {
    return [...pointers.values()];
  }

  function pinchState() {
    const [a, b] = pointerList();
    return {
      distance: Math.hypot(a.x - b.x, a.y - b.y),
      midX: (a.x + b.x) / 2,
      midY: (a.y + b.y) / 2,
    };
  }

  function panBy(dx, dy) {
    const scale = radius * 0.0016;
    const right = new three.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
    const up = new three.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);
    target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
  }

  function onPointerDown(event) {
    if (!enabled || event.defaultPrevented || event.button > 2) {
      return;
    }
    pointers.set(event.pointerId, {
      id: event.pointerId, x: event.clientX, y: event.clientY, button: event.button,
    });
    if (pointers.size === 2) {
      pinch = pinchState();
    }
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch (_error) {
      // Synthetic PointerEvents used by the browser suite are not active OS pointers.
    }
  }

  function onPointerMove(event) {
    if (!enabled || !pointers.has(event.pointerId)) {
      return;
    }
    const pointer = pointers.get(event.pointerId);
    const dx = event.clientX - pointer.x;
    const dy = event.clientY - pointer.y;
    pointer.x = event.clientX;
    pointer.y = event.clientY;

    if (pointers.size >= 2) {
      // Two fingers: pinch to zoom, drag the midpoint to pan. One finger keeps
      // orbiting, which is what makes the canvas usable at 375 px.
      const next = pinchState();
      if (pinch) {
        if (pinch.distance > 0 && next.distance > 0) {
          radius = Math.min(8, Math.max(0.45, radius * (pinch.distance / next.distance)));
        }
        panBy(next.midX - pinch.midX, next.midY - pinch.midY);
      }
      pinch = next;
      updateCamera();
      return;
    }

    if (pointer.button === 0) {
      // TRACKBALL, not camera-joystick: the cursor grabs the world and the
      // world follows it, as if a ball were rolling under the finger. Drag
      // right and the near face travels right, which walks the camera the
      // other way round -- so azimuth DECREASES with +dx. Drag down and the
      // near face tips down, which lifts the camera over the top of the cell
      // -- so polar DECREASES with +dy. The pitch sign is the one this scene
      // shipped backwards; the yaw sign was already the metaphor's, and is
      // written out here so a future edit cannot "fix" it into disagreement.
      azimuth -= dx * 0.006;
      polar = Math.min(Math.PI - 0.08, Math.max(0.08, polar - dy * 0.006));
    } else {
      panBy(dx, dy);
    }
    updateCamera();
  }

  function releasePointer(event) {
    if (!pointers.has(event.pointerId)) {
      return;
    }
    pointers.delete(event.pointerId);
    pinch = pointers.size === 2 ? pinchState() : null;
    try {
      if (canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
    } catch (_error) {
      // See the synthetic PointerEvent note in onPointerDown().
    }
  }

  function onWheel(event) {
    if (!enabled) {
      return;
    }
    event.preventDefault();
    radius = Math.min(8, Math.max(0.45, radius * Math.exp(event.deltaY * 0.001)));
    updateCamera();
  }

  function onContextMenu(event) {
    event.preventDefault();
  }

  const registered = [
    ["pointerdown", onPointerDown],
    ["pointermove", onPointerMove],
    ["pointerup", releasePointer],
    ["pointercancel", releasePointer],
    ["wheel", onWheel, {passive: false}],
    ["contextmenu", onContextMenu],
  ];
  registered.forEach(([type, listener, options]) => {
    canvas.addEventListener(type, listener, options);
    listenerCount += 1;
  });
  updateCamera();

  function cancel() {
    for (const pointer of pointerList()) {
      try {
        if (canvas.hasPointerCapture(pointer.id)) {
          canvas.releasePointerCapture(pointer.id);
        }
      } catch (_error) {
        // See the synthetic PointerEvent note in onPointerDown().
      }
    }
    pointers.clear();
    pinch = null;
  }

  function dispose() {
    registered.forEach(([type, listener, options]) => {
      canvas.removeEventListener(type, listener, options);
      listenerCount -= 1;
    });
    cancel();
  }

  return {
    cancel,
    dispose,
    /** Frame a world-space point at a world-space distance. */
    frame(nextTarget, nextRadius) {
      target.set(nextTarget[0], nextTarget[1], nextTarget[2]);
      radius = Math.min(8, Math.max(0.45, nextRadius));
      updateCamera();
    },
    setEnabled(nextEnabled) {
      enabled = Boolean(nextEnabled);
      if (!enabled) {
        cancel();
      }
    },
    get enabled() {
      return enabled;
    },
    get listenerCount() {
      return listenerCount;
    },
    get radius() {
      return radius;
    },
    //: The two angles the drag above writes, readable. A case that measures
    //: something AT a framing -- how much of one handle another covers, say --
    //: has to be able to name the framing it was at and to get back to it.
    //: Without these it can only walk a path and report whichever framings the
    //: camera happened to pass through, which is a sample of nothing.
    get azimuth() {
      return azimuth;
    },
    get polar() {
      return polar;
    },
    get target() {
      return target;
    },
    get activePointerCount() {
      return pointers.size;
    },
  };
}

function createRobotGraph(
  three,
  scene,
  model,
  geometryAssets,
  {role = "solid", materialsForLink = (asset) => asset.materials} = {},
) {
  const root = new three.Group();
  root.name = `${role}_robot`;
  scene.add(root);
  const linkObjects = new Map();
  const jointNodes = new Map();
  const jointOrigins = new Map();
  const linkMeshes = [];
  const childrenByParent = new Map();

  for (const joint of model.joints) {
    if (!childrenByParent.has(joint.parent)) {
      childrenByParent.set(joint.parent, []);
    }
    childrenByParent.get(joint.parent).push(joint);
  }

  function buildLink(linkName) {
    const link = model.links[linkName];
    const linkObject = new three.Object3D();
    linkObject.name = linkName;
    linkObject.userData.linkName = linkName;
    linkObjects.set(linkName, linkObject);

    if (link.visual && link.visual.assetUri) {
      const asset = geometryAssets.get(link.visual.assetUri);
      if (!asset) {
        throw new Error(`loaded geometry missing for ${link.visual.assetUri}`);
      }
      const mesh = new three.Mesh(asset.geometry, materialsForLink(asset, linkName));
      mesh.name = `${role}_${linkName}_mesh`;
      mesh.userData.linkName = linkName;
      mesh.userData.role = role;
      mesh.renderOrder = role === "ghost" ? 10 : 0;
      linkObject.add(mesh);
      linkMeshes.push(mesh);
    }

    for (const joint of childrenByParent.get(linkName) || []) {
      const originNode = new three.Object3D();
      originNode.name = `${joint.name}_origin`;
      originNode.position.fromArray(joint.origin.xyz);
      // URDF fixed-axis XYZ is Rz * Ry * Rx; this is three's ZYX Euler order.
      originNode.rotation.set(...joint.origin.rpy, "ZYX");
      originNode.userData.joint = joint;
      linkObject.add(originNode);
      jointOrigins.set(joint.name, originNode);

      let childParent = originNode;
      if (joint.type === "revolute" || joint.type === "continuous") {
        const rotationNode = new three.Object3D();
        rotationNode.name = joint.name;
        rotationNode.userData.joint = joint;
        rotationNode.userData.axis = new three.Vector3(...joint.axis).normalize();
        rotationNode.userData.value = 0;
        originNode.add(rotationNode);
        jointNodes.set(joint.name, rotationNode);
        childParent = rotationNode;
      } else if (joint.type !== "fixed") {
        throw new Error(`unsupported joint type ${joint.type} on ${joint.name}`);
      }
      childParent.add(buildLink(joint.child));
    }
    return linkObject;
  }

  for (const rootName of model.roots) {
    root.add(buildLink(rootName));
  }
  return {root, linkObjects, jointNodes, jointOrigins, linkMeshes};
}

/** Build the solid + ghost dual-Panda renderer for one panel. */
export async function mountSolidScene(container, {
  model,
  manifest,
  assetBase,
  assetFetch,
  arms = [
    {armIndex: 1, armId: "panda1"},
    {armIndex: 2, armId: "panda2"},
  ],
  theme = "light",
  palette: initialPalette = null,
  cell: initialCell = null,
  onTick = null,
  onContextChange = null,
}) {
  const three = requireThree();
  if (!(container instanceof Element)) {
    throw new TypeError("container must be a DOM Element");
  }
  if (!model || !model.links || !Array.isArray(model.joints)) {
    throw new TypeError("model must be a parsed URDF model");
  }

  const canvas = document.createElement("canvas");
  canvas.className = "ghost-canvas";
  canvas.style.width = "100%";
  canvas.style.height = "100%";
  canvas.setAttribute("aria-label", "Live dual Panda cell with editable ghost poses");
  container.replaceChildren(canvas);

  const contextAttributes = {alpha: false, antialias: false, depth: true, stencil: false};
  const gl = canvas.getContext("webgl2", contextAttributes);
  if (!gl) {
    throw new Error("WebGL2 is required for the Franka ghost renderer");
  }
  const renderer = new three.WebGLRenderer({canvas, context: gl, antialias: false});
  renderer.sortObjects = false;

  const scene = new three.Scene();
  const camera = new three.PerspectiveCamera(42, 1, 0.01, 50);
  const hemisphere = new three.HemisphereLight(0xe8f3ff, 0x17212d, 1.1);
  const keyLight = new three.DirectionalLight(0xffffff, 1.35);
  keyLight.position.set(1.8, -1.6, 2.8);
  scene.add(hemisphere, keyLight);

  let themeName = BUILTIN_PALETTE[theme] ? theme : "light";
  let palette = Object.assign({}, BUILTIN_PALETTE[themeName], initialPalette || {});

  const cellHandle = createCell(three, scene, palette);

  let geometryAssets;
  try {
    geometryAssets = await loadMeshGeometries(manifest, assetBase, assetFetch);
  } catch (error) {
    cellHandle.dispose();
    disposeRenderer(renderer, canvas);
    throw error;
  }

  // Ghost materials are per (arm, link) so one offending link can be tinted on
  // its own. The SOLID materials are cloned per arm for the same reason: both
  // arms reference the same link*.dae, so a shared material would grey the
  // live arm whenever its neighbour went stale.
  const ghostMaterials = new Map();
  const solidMaterials = new Map();

  function baseArmTint(armIndex) {
    return new three.Color(armIndex === 1 ? palette.ghost1 : palette.ghost2);
  }

  function ghostMaterialsForLink(asset, linkName) {
    const armIndex = armIndexForLink(linkName);
    const key = `${armIndex}:${asset.metadata.source}`;
    if (!ghostMaterials.has(key)) {
      const materials = asset.materials.map((source) => {
        const material = source.clone();
        material.color = material.uniforms.tint.value;
        material.userData.sourceColor = source.color.clone();
        material.userData.armIndex = armIndex;
        material.userData.linkName = linkName;
        material.opacity = 0.35;
        material.uniforms.opacity.value = material.opacity;
        material.transparent = true;
        material.depthWrite = false;
        material.side = three.FrontSide;
        return material;
      });
      ghostMaterials.set(key, materials);
    }
    return ghostMaterials.get(key);
  }

  function solidMaterialsForLink(asset, linkName) {
    const armIndex = armIndexForLink(linkName);
    const key = `${armIndex}:${asset.metadata.source}`;
    if (!solidMaterials.has(key)) {
      const materials = asset.materials.map((source) => {
        const material = source.clone();
        material.color = material.uniforms.tint.value;
        material.color.copy(source.color);
        material.userData.sourceColor = source.color.clone();
        material.userData.armIndex = armIndex;
        material.userData.linkName = linkName;
        material.userData.liveOpacity = material.opacity;
        material.userData.liveTransparent = material.transparent;
        return material;
      });
      solidMaterials.set(key, materials);
    }
    return solidMaterials.get(key);
  }

  let solidGraph;
  let ghostGraph;
  try {
    solidGraph = createRobotGraph(three, scene, model, geometryAssets, {
      role: "solid",
      materialsForLink: solidMaterialsForLink,
    });
    ghostGraph = createRobotGraph(three, scene, model, geometryAssets, {
      role: "ghost",
      materialsForLink: ghostMaterialsForLink,
    });
  } catch (error) {
    disposeMaterialMap(ghostMaterials);
    disposeMaterialMap(solidMaterials);
    cellHandle.dispose();
    disposeGeometryAssets(geometryAssets);
    disposeRenderer(renderer, canvas);
    throw error;
  }

  const configuredArmIndices = arms.map((arm) => arm.armIndex);
  const present = new Map([[1, false], [2, false]]);
  const ghostShown = new Map([[1, false], [2, false]]);
  const staleArms = new Map([[1, false], [2, false]]);
  const verdicts = new Map([[1, null], [2, null]]);
  for (const armIndex of [1, 2]) {
    present.set(armIndex, configuredArmIndices.includes(armIndex));
  }

  let disposed = false;
  let contextLost = false;
  let animationFrame = null;
  let renderNeeded = true;
  let renderCount = 0;
  let ratioIndex = RATIO_STEPS.length - 1;
  let frameTimes = [];
  let lastStepMs = 0;

  function requestRender() {
    renderNeeded = true;
  }

  function applyRatio() {
    renderer.setPixelRatio(
      Math.min(globalThis.devicePixelRatio || 1, RATIO_STEPS[ratioIndex]),
    );
    resize();
  }

  function noteFrameTime(ms) {
    frameTimes.push(ms);
    if (frameTimes.length < RATIO_SAMPLE_COUNT) {
      return;
    }
    const mean = frameTimes.reduce((total, value) => total + value, 0) / frameTimes.length;
    frameTimes = [];
    const now = performance.now();
    if (mean > RATIO_STEP_DOWN_MS && ratioIndex > 0) {
      ratioIndex -= 1;
      lastStepMs = now;
      applyRatio();
    } else if (mean < RATIO_STEP_UP_MS && ratioIndex < RATIO_STEPS.length - 1
               && now - lastStepMs > RATIO_STEP_COOLDOWN_MS) {
      ratioIndex += 1;
      lastStepMs = now;
      applyRatio();
    }
  }

  function animate(now) {
    animationFrame = requestAnimationFrame(animate);
    // A collapsed panel is genuinely free: `hidden` gives the container no
    // layout box, so the loop costs one early return per frame and no GPU
    // work at all. This is also why the seam needs no pause() method.
    if (disposed || contextLost || document.hidden || canvas.clientWidth === 0) {
      return;
    }
    if (typeof onTick === "function") {
      onTick(now);                          // may call requestRender()
    }
    if (!renderNeeded) {
      return;
    }
    const started = performance.now();
    renderer.render(scene, camera);
    renderCount += 1;
    noteFrameTime(performance.now() - started);
    renderNeeded = false;
  }

  function resize() {
    if (disposed) {
      return;
    }
    const {width, height} = rendererSize(container);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    requestRender();
  }

  function frameCamera() {
    // The default view must show the real lab: the prototype's fixed target
    // and radius were tuned against a base separation that has since been
    // corrected, and "confirm the metre of separation by eye" is only a
    // meaningful check from a sane default view.
    const corners = cellHandle.boxCorners();
    let span = [DEFAULT_CELL_SPAN_M, DEFAULT_CELL_SPAN_M, DEFAULT_CELL_SPAN_M];
    let centre = [0, 0, 0.5];
    if (corners) {
      const axis = (index) => corners.map((corner) => corner[index]);
      const low = [0, 1, 2].map((index) => Math.min(...axis(index)));
      const high = [0, 1, 2].map((index) => Math.max(...axis(index)));
      span = [0, 1, 2].map((index) => high[index] - low[index]);
      centre = [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2] + 0.5];
    }
    const diagonal = Math.hypot(span[0], span[1], span[2]);
    const halfFov = (camera.fov * Math.PI) / 360;
    orbitControls.frame(centre, (0.55 * diagonal) / Math.tan(halfFov));
  }

  function retintSolids() {
    for (const materials of solidMaterials.values()) {
      materials.forEach((material) => {
        const armIndex = material.userData.armIndex;
        material.color.copy(material.userData.sourceColor);
        if (staleArms.get(armIndex) === true) {
          material.color.lerp(new three.Color(palette.stale), 0.82);
        }
        material.opacity = material.userData.liveOpacity;
        material.transparent = material.userData.liveTransparent;
        material.needsUpdate = true;
      });
    }
  }

  function retintGhosts() {
    for (const materials of ghostMaterials.values()) {
      materials.forEach((material) => {
        const armIndex = material.userData.armIndex;
        const verdict = verdicts.get(armIndex);
        const status = verdict ? verdict.status : null;
        const offending = (verdict && verdict.offending_links) || [];
        let opacity = 0.35;
        let tint;
        if (status === "collision" && offending.indexOf(material.userData.linkName) >= 0) {
          tint = new three.Color(palette.ghostCollide);
        } else if (status === "unchecked" || status === "pending") {
          // A distinct NEUTRAL treatment, not the clear colour and not the
          // collision colour, at a slightly higher opacity. "Not checked" must
          // never be mistakable for "checked and fine".
          tint = new three.Color(palette.ghostUnchecked);
          opacity = 0.45;
        } else {
          tint = baseArmTint(armIndex);
        }
        material.color.copy(material.userData.sourceColor).lerp(tint, 0.62);
        material.opacity = opacity;
        material.uniforms.opacity.value = opacity;
        material.needsUpdate = true;
      });
    }
  }

  function applyPalette(next) {
    palette = Object.assign({}, BUILTIN_PALETTE[themeName], next || {});
    renderer.setClearColor(new three.Color(palette.sceneBg), 1);
    scene.background = new three.Color(palette.sceneBg);
    cellHandle.setPalette(palette);
    retintGhosts();
    retintSolids();
    requestRender();
  }

  function setTheme(nextName, nextPalette) {
    themeName = BUILTIN_PALETTE[nextName] ? nextName : themeName;
    applyPalette(nextPalette);
  }

  function setCell(cell) {
    cellHandle.setCell(cell);
    frameCamera();
    requestRender();
  }

  function setArmPresent(armIndex, isPresent) {
    present.set(armIndex, Boolean(isPresent));
    refreshVisibility();
  }

  function setGhostRootVisible(armIndex, visible) {
    ghostShown.set(armIndex, Boolean(visible));
    refreshVisibility();
  }

  function refreshVisibility() {
    for (const armIndex of [1, 2]) {
      const configured = configuredArmIndices.includes(armIndex);
      const solidRoot = solidGraph.linkObjects.get(`panda${armIndex}_link0`);
      const ghostRoot = ghostGraph.linkObjects.get(`panda${armIndex}_link0`);
      const live = configured && present.get(armIndex) === true;
      if (solidRoot) {
        solidRoot.visible = live;
      }
      if (ghostRoot) {
        ghostRoot.visible = live && ghostShown.get(armIndex) === true;
      }
    }
    requestRender();
  }

  function setSolidStale(armIndex, isStale) {
    staleArms.set(armIndex, Boolean(isStale));
    retintSolids();
    requestRender();
  }

  function setGhostTint(armIndex, verdict) {
    verdicts.set(armIndex, verdict || null);
    retintGhosts();
    requestRender();
  }

  const orbitControls = installOrbitControls(three, canvas, camera, requestRender);

  // View reset is a double-click on empty canvas, not a fourth toolbar button:
  // the toolbar's three affordances are enumerated by the design and a fourth
  // one would be the first thing to argue about. Removable in one line.
  function onDoubleClick(event) {
    if (event.defaultPrevented) {
      return;
    }
    event.preventDefault();
    frameCamera();
  }

  // A lost context is not a broken page. The canvas stops drawing and says so
  // through the callback; the console around it is untouched.
  function onContextLost(event) {
    event.preventDefault();
    contextLost = true;
    if (typeof onContextChange === "function") {
      onContextChange(false);
    }
  }

  function onContextRestored() {
    contextLost = false;
    applyRatio();
    applyPalette(null);
    resize();
    if (typeof onContextChange === "function") {
      onContextChange(true);
    }
  }

  const ownListeners = [
    ["resize", resize, window],
    ["dblclick", onDoubleClick, canvas],
    ["webglcontextlost", onContextLost, canvas],
    ["webglcontextrestored", onContextRestored, canvas],
  ];
  ownListeners.forEach(([type, listener, target]) => {
    target.addEventListener(type, listener);
  });
  let resizeListenerCount = ownListeners.length;
  applyRatio();
  applyPalette(initialPalette);
  cellHandle.setCell(initialCell);
  refreshVisibility();
  frameCamera();
  resize();

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    cancelAnimationFrame(animationFrame);
    animationFrame = null;
    ownListeners.forEach(([type, listener, target]) => {
      target.removeEventListener(type, listener);
    });
    resizeListenerCount = 0;
    orbitControls.dispose();
    cellHandle.dispose();
    disposeMaterialMap(ghostMaterials);
    disposeMaterialMap(solidMaterials);
    disposeGeometryAssets(geometryAssets);
    disposeRenderer(renderer, canvas, contextLost || gl.isContextLost());
  }

  scene.updateMatrixWorld(true);
  animate(performance.now());

  return {
    canvas,
    renderer,
    gl,
    scene,
    camera,
    model,
    solidGraph,
    ghostGraph,
    orbitControls,
    cell: cellHandle,
    linkObjects: solidGraph.linkObjects,
    jointNodes: solidGraph.jointNodes,
    linkMeshes: solidGraph.linkMeshes,
    ghostLinkMeshes: ghostGraph.linkMeshes,
    requestRender,
    resize,
    frameCamera,
    setTheme,
    setCell,
    setArmPresent,
    setGhostRootVisible,
    setSolidStale,
    setGhostTint,
    dispose,
    get palette() {
      return palette;
    },
    get themeName() {
      return themeName;
    },
    get listenerCount() {
      return resizeListenerCount + orbitControls.listenerCount;
    },
    get animationActive() {
      return animationFrame !== null;
    },
    get contextLost() {
      return contextLost;
    },
    testing: {
      noteFrameTime,
      // The rolling window is shared with real rendering, so a test that wants
      // to drive the ratchet has to start from an empty one.
      resetFrameTimes() {
        frameTimes = [];
      },
      builtinPalette: BUILTIN_PALETTE,
      ghostMaterials,
      solidMaterials,
      get renderCount() {
        return renderCount;
      },
      get ratioIndex() {
        return ratioIndex;
      },
    },
  };
}
