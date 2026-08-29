// [DURABLE] Moves unchanged into franka_web at the Session C merge.

import {loadMeshGeometries} from "./meshes.js";
import {createJointDragController} from "./drag.js";

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

function disposeRenderer(renderer, canvas) {
  renderer.dispose();
  if (typeof renderer.forceContextLoss === "function") {
    renderer.forceContextLoss();
  }
  canvas.remove();
}

function installOrbitControls(three, canvas, camera, render) {
  const target = new three.Vector3(0, 0, 0.45);
  let radius = 2.25;
  let azimuth = 0.78;
  let polar = 1.02;
  let pointer = null;
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

  function onPointerDown(event) {
    if (!enabled || event.defaultPrevented || event.button > 2) {
      return;
    }
    pointer = {id: event.pointerId, x: event.clientX, y: event.clientY, button: event.button};
    try {
      canvas.setPointerCapture(event.pointerId);
    } catch (_error) {
      // Synthetic PointerEvents used by the browser suite are not active OS pointers.
    }
  }

  function onPointerMove(event) {
    if (!enabled || !pointer || pointer.id !== event.pointerId) {
      return;
    }
    const dx = event.clientX - pointer.x;
    const dy = event.clientY - pointer.y;
    pointer.x = event.clientX;
    pointer.y = event.clientY;
    if (pointer.button === 0) {
      azimuth -= dx * 0.006;
      polar = Math.min(Math.PI - 0.08, Math.max(0.08, polar + dy * 0.006));
    } else {
      const scale = radius * 0.0016;
      const right = new three.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
      const up = new three.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);
      target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
    }
    updateCamera();
  }

  function releasePointer(event) {
    if (!pointer || pointer.id !== event.pointerId) {
      return;
    }
    if (canvas.hasPointerCapture(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    pointer = null;
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
    if (pointer && canvas.hasPointerCapture(pointer.id)) {
      canvas.releasePointerCapture(pointer.id);
    }
    pointer = null;
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

/** Build the solid and joint-space ghost dual-Panda renderer. */
export async function mountSolidScene(container, {
  model,
  manifest,
  assetBase,
  assetFetch,
  initialArm = 1,
  arms = [
    {armIndex: 1, armId: "panda1"},
    {armIndex: 2, armId: "panda2"},
  ],
  jogStepRad,
  ui = {},
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
  canvas.setAttribute(
    "aria-label",
    "Live solid dual Panda with joint-space ghost preview",
  );
  container.replaceChildren(canvas);

  const contextAttributes = {alpha: false, antialias: false, depth: true, stencil: false};
  const gl = canvas.getContext("webgl2", contextAttributes);
  if (!gl) {
    throw new Error("WebGL2 is required for the Franka ghost renderer");
  }
  const renderer = new three.WebGLRenderer({canvas, context: gl, antialias: false});
  renderer.setClearColor(0x0b1118, 1);
  // The generated visual meshes are detailed enough that full device-pixel
  // resolution is fill-rate bound under software WebGL. Preserve geometry and
  // interaction fidelity while bounding raster cost for the offline fallback.
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 0.5));
  renderer.sortObjects = false;

  const scene = new three.Scene();
  scene.background = new three.Color(0x0b1118);
  const camera = new three.PerspectiveCamera(42, 1, 0.01, 50);
  const hemisphere = new three.HemisphereLight(0xe8f3ff, 0x17212d, 1.1);
  const keyLight = new three.DirectionalLight(0xffffff, 1.35);
  keyLight.position.set(1.8, -1.6, 2.8);
  scene.add(hemisphere, keyLight);

  const grid = new three.GridHelper(2.4, 24, 0x4d6475, 0x263645);
  grid.name = "ground_grid";
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.002;
  scene.add(grid);
  const axes = new three.AxesHelper(0.22);
  axes.name = "base_axes";
  scene.add(axes);

  let geometryAssets;
  try {
    geometryAssets = await loadMeshGeometries(manifest, assetBase, assetFetch);
  } catch (error) {
    disposeRenderer(renderer, canvas);
    throw error;
  }
  const ghostMaterials = new Map();
  const armTints = new Map([
    [1, new three.Color(0x29b9ee)],
    [2, new three.Color(0xa96ee8)],
  ]);
  function ghostMaterialsForLink(asset, linkName) {
    const armIndex = linkName.startsWith("panda1_") ? 1 : 2;
    const key = `${armIndex}:${asset.metadata.source}`;
    if (!ghostMaterials.has(key)) {
      const materials = asset.materials.map((source) => {
        const material = source.clone();
        material.color = material.uniforms.tint.value;
        material.color.copy(source.color).lerp(armTints.get(armIndex), 0.48);
        material.opacity = 0.35;
        material.uniforms.opacity.value = material.opacity;
        material.transparent = true;
        material.depthWrite = false;
        material.side = three.FrontSide;
        material.userData.armTint = armIndex;
        return material;
      });
      ghostMaterials.set(key, materials);
    }
    return ghostMaterials.get(key);
  }
  let solidGraph;
  let ghostGraph;
  try {
    solidGraph = createRobotGraph(three, scene, model, geometryAssets, {role: "solid"});
    ghostGraph = createRobotGraph(three, scene, model, geometryAssets, {
      role: "ghost",
      materialsForLink: ghostMaterialsForLink,
    });
  } catch (error) {
    disposeMaterialMap(ghostMaterials);
    disposeGeometryAssets(geometryAssets);
    disposeRenderer(renderer, canvas);
    throw error;
  }
  const configuredArmIndices = arms.map((arm) => arm.armIndex);
  for (const armIndex of [1, 2]) {
    if (!configuredArmIndices.includes(armIndex)) {
      solidGraph.linkObjects.get(`panda${armIndex}_link0`).visible = false;
      ghostGraph.linkObjects.get(`panda${armIndex}_link0`).visible = false;
    }
  }
  let disposed = false;
  let animationFrame = null;
  let renderNeeded = true;

  function requestRender() {
    renderNeeded = true;
  }

  function animate() {
    if (disposed) {
      return;
    }
    renderer.render(scene, camera);
    renderNeeded = false;
    animationFrame = requestAnimationFrame(animate);
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

  const orbitControls = installOrbitControls(three, canvas, camera, requestRender);
  window.addEventListener("resize", resize);
  let resizeListenerCount = 1;
  resize();

  let dragController;
  try {
    dragController = createJointDragController({
      three,
      canvas,
      camera,
      model,
      solidGraph,
      ghostGraph,
      render: requestRender,
      orbitControls,
      initialArm,
      armIndices: configuredArmIndices,
      jogStepRad,
      ui,
    });
  } catch (error) {
    window.removeEventListener("resize", resize);
    resizeListenerCount = 0;
    orbitControls.dispose();
    disposeMaterialMap(ghostMaterials);
    disposeGeometryAssets(geometryAssets);
    disposeRenderer(renderer, canvas);
    throw error;
  }

  const solidMaterials = [...geometryAssets.values()].flatMap((asset) => asset.materials);
  const staleColor = new three.Color(0x70777e);
  solidMaterials.forEach((material) => {
    material.userData.liveColor = material.color.clone();
    material.userData.liveOpacity = material.opacity;
    material.userData.liveTransparent = material.transparent;
  });
  let stateStale = false;

  function setMeasured(measured) {
    if (!measured || typeof measured !== "object" || Array.isArray(measured)) {
      throw new TypeError("measured state must be a by-name map");
    }
    for (const [name, value] of Object.entries(measured)) {
      if (solidGraph.jointNodes.has(name) && !Number.isFinite(value)) {
        throw new TypeError(`measured joint ${name} must be finite`);
      }
    }
    for (const [name, value] of Object.entries(measured)) {
      const node = solidGraph.jointNodes.get(name);
      if (!node) {
        continue;
      }
      node.userData.value = value;
      node.quaternion.setFromAxisAngle(node.userData.axis, value);
    }
    scene.updateMatrixWorld(true);
    dragController.setMeasured(measured);
    requestRender();
  }

  function setStale(nextStale) {
    stateStale = Boolean(nextStale);
    solidMaterials.forEach((material) => {
      if (stateStale) {
        material.color.copy(material.userData.liveColor).lerp(staleColor, 0.82);
        // Staleness changes provenance legibility, not scene occlusion: keep
        // the measured robot solid while greying it, and never touch ghosts.
        material.opacity = material.userData.liveOpacity;
        material.transparent = material.userData.liveTransparent;
      } else {
        material.color.copy(material.userData.liveColor);
        material.opacity = material.userData.liveOpacity;
        material.transparent = material.userData.liveTransparent;
      }
      material.needsUpdate = true;
    });
    container.classList.toggle("joint-state-stale", stateStale);
    dragController.setStale(stateStale);
    requestRender();
  }

  function setEnabled(nextEnabled) {
    dragController.setEnabled(nextEnabled);
    container.classList.toggle("ghost-read-only", !nextEnabled);
    requestRender();
  }

  function dispose() {
    if (disposed) {
      return;
    }
    disposed = true;
    cancelAnimationFrame(animationFrame);
    animationFrame = null;
    window.removeEventListener("resize", resize);
    resizeListenerCount = 0;
    dragController.dispose();
    orbitControls.dispose();
    disposeMaterialMap(ghostMaterials);
    disposeGeometryAssets(geometryAssets);
    disposeRenderer(renderer, canvas);
  }

  scene.updateMatrixWorld(true);
  animate();
  return {
    setMeasured,
    setFence: dragController.setFence,
    setGhost: dragController.setGhost,
    syncGhostToMeasured: dragController.syncGhostToMeasured,
    setGhostVisible: dragController.setGhostVisible,
    selectArm: dragController.selectArm,
    selectJoint: dragController.selectJoint,
    setEnabled,
    setStale,
    dispose,
    renderer,
    gl,
    scene,
    camera,
    linkObjects: solidGraph.linkObjects,
    jointNodes: solidGraph.jointNodes,
    linkMeshes: solidGraph.linkMeshes,
    solidGraph,
    ghostGraph,
    ghostLinkMeshes: ghostGraph.linkMeshes,
    dragController,
    orbitControls,
    getGhost: dragController.getGhost,
    getMeasured: dragController.getMeasured,
    get listenerCount() {
      return resizeListenerCount + orbitControls.listenerCount + dragController.listenerCount;
    },
    get stale() {
      return stateStale;
    },
    get animationActive() {
      return animationFrame !== null;
    },
  };
}
