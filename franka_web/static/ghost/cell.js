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

// The measured cell: the allowed-volume box outline, the table top it stands
// on, and a ground grid clipped to the box footprint so the two can never
// disagree about where the table is.
//
// THE CELL FRAME IS THE RENDERER ROOT FRAME. The cell frame is fixed at the
// midpoint of the two joint-1 axes on the table surface, and under symmetric
// mounting it coincides with base_link, which is the root of the model this
// renderer draws. The six bounds are therefore drawn with NO transform, and
// this module must never grow a transform parameter: a cell that ever
// declares a non-identity dual anchor is a contract amendment, not a
// client-side fix.

const GRID_SPACING_M = 0.1;
const MAJOR_EVERY = 10;
const GRID_DROP_M = 0.002;
const FALLBACK_SPAN_M = 2.4;
const FALLBACK_DIVISIONS = 24;
const FALLBACK_AXES_M = 0.22;

const BOUND_KEYS = ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"];

function isBounded(cell) {
  return Boolean(cell) && BOUND_KEYS.every(
    (key) => Number.isFinite(cell[key]),
  ) && cell.x_max > cell.x_min && cell.y_max > cell.y_min && cell.z_max > cell.z_min;
}

function gridLineOffsets(low, high) {
  // Lines land on exact multiples of the spacing so "every tenth line" is a
  // property of the world, not of where the box happens to start.
  const first = Math.ceil(low / GRID_SPACING_M - 1e-9);
  const last = Math.floor(high / GRID_SPACING_M + 1e-9);
  const offsets = [];
  for (let step = first; step <= last; step += 1) {
    offsets.push({value: step * GRID_SPACING_M, major: step % MAJOR_EVERY === 0});
  }
  return offsets;
}

/** Build the cell group. Returns the small handle scene.js drives. */
export function createCell(three, scene, palette) {
  const group = new three.Group();
  group.name = "measured_cell";
  scene.add(group);

  let colours = Object.assign({}, palette);
  let bounds = null;
  const owned = [];

  const boxMaterial = new three.LineDashedMaterial({
    color: new three.Color(colours.cellLine || "#8494A3"),
    dashSize: 0.06,
    gapSize: 0.04,
  });
  const floorMaterial = new three.MeshBasicMaterial({
    color: new three.Color(colours.cellFloor || "#F3F6F9"),
    transparent: true,
    opacity: 0.55,
    depthWrite: false,
    side: three.DoubleSide,
  });
  const gridMaterial = new three.LineBasicMaterial({
    color: new three.Color(colours.grid || "#D7DEE6"),
  });
  const gridMajorMaterial = new three.LineBasicMaterial({
    color: new three.Color(colours.gridMajor || "#8494A3"),
  });

  function clearGroup() {
    while (owned.length > 0) {
      const object = owned.pop();
      group.remove(object);
      if (object.geometry && typeof object.geometry.dispose === "function") {
        object.geometry.dispose();
      }
      if (object.dispose && object !== group) {
        // GridHelper / AxesHelper own their own material.
        if (typeof object.dispose === "function") {
          object.dispose();
        }
      }
    }
  }

  function buildFallback() {
    const grid = new three.GridHelper(
      FALLBACK_SPAN_M, FALLBACK_DIVISIONS,
      new three.Color(colours.gridMajor || "#8494A3"),
      new three.Color(colours.grid || "#D7DEE6"),
    );
    grid.name = "ground_grid";
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -GRID_DROP_M;
    group.add(grid);
    owned.push(grid);

    const axes = new three.AxesHelper(FALLBACK_AXES_M);
    axes.name = "base_axes";
    group.add(axes);
    owned.push(axes);
  }

  function buildBox() {
    const width = bounds.x_max - bounds.x_min;
    const depth = bounds.y_max - bounds.y_min;
    const height = bounds.z_max - bounds.z_min;

    const box = new three.LineSegments(
      new three.EdgesGeometry(new three.BoxBufferGeometry(width, depth, height)),
      boxMaterial,
    );
    box.name = "allowed_volume";
    box.position.set(
      (bounds.x_min + bounds.x_max) / 2,
      (bounds.y_min + bounds.y_max) / 2,
      (bounds.z_min + bounds.z_max) / 2,
    );
    box.computeLineDistances();
    group.add(box);
    owned.push(box);

    const floor = new three.Mesh(
      new three.PlaneBufferGeometry(width, depth), floorMaterial,
    );
    floor.name = "table_top";
    floor.position.set(
      (bounds.x_min + bounds.x_max) / 2,
      (bounds.y_min + bounds.y_max) / 2,
      bounds.z_min,
    );
    floor.renderOrder = -10;
    group.add(floor);
    owned.push(floor);

    const minor = [];
    const major = [];
    const z = bounds.z_min - GRID_DROP_M;
    gridLineOffsets(bounds.x_min, bounds.x_max).forEach((line) => {
      (line.major ? major : minor).push(
        line.value, bounds.y_min, z, line.value, bounds.y_max, z,
      );
    });
    gridLineOffsets(bounds.y_min, bounds.y_max).forEach((line) => {
      (line.major ? major : minor).push(
        bounds.x_min, line.value, z, bounds.x_max, line.value, z,
      );
    });
    [[minor, gridMaterial, "cell_grid"], [major, gridMajorMaterial, "cell_grid_major"]]
      .forEach(([points, material, name]) => {
        if (points.length === 0) {
          return;
        }
        const geometry = new three.BufferGeometry();
        geometry.setAttribute(
          "position", new three.BufferAttribute(new Float32Array(points), 3),
        );
        const lines = new three.LineSegments(geometry, material);
        lines.name = name;
        group.add(lines);
        owned.push(lines);
      });
  }

  function setCell(cell) {
    clearGroup();
    bounds = isBounded(cell)
      ? {
        id: cell.id, frame: cell.frame,
        x_min: cell.x_min, x_max: cell.x_max,
        y_min: cell.y_min, y_max: cell.y_max,
        z_min: cell.z_min, z_max: cell.z_max,
      }
      : null;
    if (bounds) {
      buildBox();
    } else {
      buildFallback();
    }
  }

  function setPalette(next) {
    colours = Object.assign({}, next);
    boxMaterial.color.set(colours.cellLine || "#8494A3");
    floorMaterial.color.set(colours.cellFloor || "#F3F6F9");
    gridMaterial.color.set(colours.grid || "#D7DEE6");
    gridMajorMaterial.color.set(colours.gridMajor || "#8494A3");
    if (!bounds) {
      // GridHelper bakes its colours into vertex colours at construction.
      setCell(null);
    }
  }

  /** The eight corners of the allowed volume, or null when there is no box. */
  function boxCorners() {
    if (!bounds) {
      return null;
    }
    const corners = [];
    [bounds.x_min, bounds.x_max].forEach((x) => {
      [bounds.y_min, bounds.y_max].forEach((y) => {
        [bounds.z_min, bounds.z_max].forEach((z) => {
          corners.push([x, y, z]);
        });
      });
    });
    return corners;
  }

  function dispose() {
    clearGroup();
    scene.remove(group);
    boxMaterial.dispose();
    floorMaterial.dispose();
    gridMaterial.dispose();
    gridMajorMaterial.dispose();
  }

  return {
    group,
    setCell,
    setPalette,
    boxCorners,
    dispose,
    get gridSpacing() {
      return GRID_SPACING_M;
    },
  };
}
