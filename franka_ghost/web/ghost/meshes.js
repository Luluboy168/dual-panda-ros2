// [DURABLE] Moves unchanged into franka_web at the Session C merge.

const MESH_SCHEMA = "franka.ghost.mesh/1";

function requireThree() {
  const three = globalThis.THREE;
  if (!three || typeof three.BufferGeometry !== "function") {
    throw new Error("three.js r111 classic global is not loaded");
  }
  return three;
}

async function fetchOk(fetchAsset, url, kind) {
  if (typeof fetchAsset !== "function") {
    throw new TypeError("asset fetch function is required");
  }
  const response = await fetchAsset(url);
  if (!response.ok) {
    throw new Error(`failed to load ${kind} ${url}: HTTP ${response.status}`);
  }
  return response;
}

function validateLayoutEntry(entry, byteLength) {
  if (!entry || entry.type !== "float32" || !Number.isInteger(entry.items)
      || !Number.isInteger(entry.count) || !Number.isInteger(entry.byteOffset)
      || entry.items <= 0 || entry.count < 0 || entry.byteOffset < 0) {
    throw new Error("invalid float32 mesh layout entry");
  }
  const end = entry.byteOffset + entry.items * entry.count * Float32Array.BYTES_PER_ELEMENT;
  if (end > byteLength) {
    throw new Error(`${entry.name} mesh layout exceeds binary payload`);
  }
}

function makeVertexColors(three, metadata, vertexCount) {
  if (!Array.isArray(metadata.groups) || metadata.groups.length === 0) {
    throw new Error("mesh metadata must contain at least one material group");
  }
  const colors = new Float32Array(vertexCount * 3);
  let opacity = null;
  metadata.groups.forEach((group) => {
    if (!Array.isArray(group.color) || group.color.length !== 3
        || group.color.some((value) => !Number.isFinite(value))) {
      throw new Error("mesh material group has an invalid color");
    }
    const groupOpacity = Number(group.opacity);
    if (!Number.isFinite(groupOpacity) || groupOpacity < 0 || groupOpacity > 1) {
      throw new Error("mesh material group has an invalid opacity");
    }
    if (opacity === null) {
      opacity = groupOpacity;
    } else if (opacity !== groupOpacity) {
      throw new Error("one mesh cannot mix material-group opacity values");
    }
    for (let vertex = group.start; vertex < group.start + group.count; vertex += 1) {
      colors.set(group.color, vertex * 3);
    }
  });
  return {colors, opacity};
}

function indexForRaster(positions, normals, colors, vertexCount) {
  const vertexByPositionAndColor = new Map();
  const indexedPositions = [];
  const indexedNormals = [];
  const indexedColors = [];
  const indices = new Uint32Array(vertexCount);

  for (let vertex = 0; vertex < vertexCount; vertex += 1) {
    const offset = vertex * 3;
    // MeshBasic/Lambert materials share vertices only where both the geometric
    // position and Franka material color match. Triangle topology and color
    // boundaries therefore remain exact; normals are averaged for smooth CAD
    // shading instead of transforming six duplicate vertices at every corner.
    const key = `${positions[offset]},${positions[offset + 1]},${positions[offset + 2]};`
      + `${colors[offset]},${colors[offset + 1]},${colors[offset + 2]}`;
    let indexedVertex = vertexByPositionAndColor.get(key);
    if (indexedVertex === undefined) {
      indexedVertex = indexedPositions.length / 3;
      vertexByPositionAndColor.set(key, indexedVertex);
      indexedPositions.push(
        positions[offset], positions[offset + 1], positions[offset + 2],
      );
      indexedNormals.push(normals[offset], normals[offset + 1], normals[offset + 2]);
      indexedColors.push(colors[offset], colors[offset + 1], colors[offset + 2]);
    } else {
      const indexedOffset = indexedVertex * 3;
      indexedNormals[indexedOffset] += normals[offset];
      indexedNormals[indexedOffset + 1] += normals[offset + 1];
      indexedNormals[indexedOffset + 2] += normals[offset + 2];
    }
    indices[vertex] = indexedVertex;
  }

  for (let offset = 0; offset < indexedNormals.length; offset += 3) {
    const length = Math.hypot(
      indexedNormals[offset], indexedNormals[offset + 1], indexedNormals[offset + 2],
    );
    if (length > 0) {
      indexedNormals[offset] /= length;
      indexedNormals[offset + 1] /= length;
      indexedNormals[offset + 2] /= length;
    }
  }
  return {
    colors: new Float32Array(indexedColors),
    indices,
    normals: new Float32Array(indexedNormals),
    positions: new Float32Array(indexedPositions),
  };
}

function makeVertexLitMaterial(three, opacity) {
  const material = new three.ShaderMaterial({
    uniforms: {
      opacity: {value: opacity},
      tint: {value: new three.Color(0xffffff)},
    },
    vertexShader: `
      attribute vec3 color;
      uniform vec3 tint;
      varying vec3 shadedColor;
      void main() {
        vec3 surfaceNormal = normalize(normalMatrix * normal);
        vec3 keyDirection = normalize(vec3(0.45, -0.65, 0.61));
        float key = max(dot(surfaceNormal, keyDirection), 0.0);
        float hemisphere = 0.5 + 0.5 * surfaceNormal.z;
        float light = 0.40 + 0.45 * key + 0.15 * hemisphere;
        shadedColor = color * tint * light;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      precision mediump float;
      uniform float opacity;
      varying vec3 shadedColor;
      void main() {
        gl_FragColor = vec4(shadedColor, opacity);
      }
    `,
    opacity,
    transparent: opacity < 1,
    side: three.FrontSide,
  });
  // Keep the conventional Material fields available to the scene's tint and
  // transparency logic while the uniforms drive the lightweight shader.
  material.color = material.uniforms.tint.value;
  return material;
}

/** Load one generated C5 mesh, retaining the binary buffer through its attributes. */
export async function loadGhostMesh(metadataUrl, fetchAsset) {
  const three = requireThree();
  const metadataResponse = await fetchOk(fetchAsset, metadataUrl, "mesh metadata");
  const metadata = await metadataResponse.json();
  if (!metadata || metadata.schema !== MESH_SCHEMA || metadata.up_axis !== "Z_UP"
      || metadata.unit_m !== 1 || typeof metadata.bin !== "string") {
    throw new Error(`invalid ghost mesh metadata ${metadataUrl}`);
  }

  const binaryUrl = new URL(metadata.bin, metadataResponse.url || metadataUrl);
  const binary = await (await fetchOk(fetchAsset, binaryUrl, "mesh binary")).arrayBuffer();
  const layoutByName = new Map((metadata.layout || []).map((entry) => [entry.name, entry]));
  const position = layoutByName.get("position");
  const normal = layoutByName.get("normal");
  if (!position || !normal || position.items !== 3 || normal.items !== 3
      || position.count !== normal.count) {
    throw new Error("mesh layout must contain matching vec3 position and normal arrays");
  }
  validateLayoutEntry(position, binary.byteLength);
  validateLayoutEntry(normal, binary.byteLength);

  const sourcePositions = new Float32Array(
    binary, position.byteOffset, position.count * position.items,
  );
  const sourceNormals = new Float32Array(
    binary, normal.byteOffset, normal.count * normal.items,
  );

  let coveredVertices = 0;
  metadata.groups.forEach((group) => {
    if (!Number.isInteger(group.start) || !Number.isInteger(group.count)
        || group.start !== coveredVertices || group.count <= 0) {
      throw new Error("mesh material groups must tile the vertex range in order");
    }
    coveredVertices += group.count;
  });
  if (coveredVertices !== position.count) {
    throw new Error("mesh material groups do not cover every vertex");
  }
  const {colors: sourceColors, opacity} = makeVertexColors(three, metadata, position.count);
  const indexed = indexForRaster(sourcePositions, sourceNormals, sourceColors, position.count);

  const geometry = new three.BufferGeometry();
  const addAttribute = geometry.setAttribute
    ? geometry.setAttribute.bind(geometry)
    : geometry.addAttribute.bind(geometry);
  addAttribute("position", new three.BufferAttribute(indexed.positions, position.items));
  addAttribute("normal", new three.BufferAttribute(indexed.normals, normal.items));
  addAttribute("color", new three.BufferAttribute(indexed.colors, 3));
  geometry.setIndex(new three.BufferAttribute(indexed.indices, 1));
  // Bake group colors into a vertex attribute so each link remains one draw
  // call while preserving the exact Franka grey/black material boundaries.
  geometry.addGroup(0, position.count, 0);
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  geometry.userData.ghostMesh = metadata;
  geometry.userData.sourceVertexCount = position.count;
  geometry.userData.indexedVertexCount = indexed.positions.length / 3;
  const material = makeVertexLitMaterial(three, opacity);
  return {geometry, materials: [material], metadata};
}

/** Load each unique manifest geometry exactly once. */
export async function loadMeshGeometries(manifest, assetBase, fetchAsset) {
  if (!manifest || manifest.schema !== "franka.ghost.manifest/1" || !manifest.meshes) {
    throw new Error("invalid ghost asset manifest");
  }
  const baseUrl = new URL(String(assetBase), document.baseURI);
  const assetPaths = [...new Set(Object.values(manifest.meshes))];
  const loaded = await Promise.all(assetPaths.map(async (assetPath) => [
    assetPath,
    await loadGhostMesh(new URL(assetPath, baseUrl), fetchAsset),
  ]));
  return new Map(loaded);
}
