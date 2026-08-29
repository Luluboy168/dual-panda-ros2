// [DURABLE] Moves unchanged into franka_web at the Session C merge.

function directChild(element, tagName) {
  return Array.from(element.children).find((child) => child.tagName === tagName) || null;
}

function requiredAttribute(element, name, context) {
  const value = element.getAttribute(name);
  if (value === null || value === "") {
    throw new Error(`${context} is missing required attribute ${name}`);
  }
  return value;
}

function numberAttribute(element, name, context) {
  const value = Number(requiredAttribute(element, name, context));
  if (!Number.isFinite(value)) {
    throw new Error(`${context}.${name} must be a finite number`);
  }
  return value;
}

function vectorAttribute(element, name, fallback, context) {
  const text = element ? element.getAttribute(name) : null;
  if (text === null || text.trim() === "") {
    return [...fallback];
  }
  const values = text.trim().split(/\s+/).map(Number);
  if (values.length !== 3 || values.some((value) => !Number.isFinite(value))) {
    throw new Error(`${context}.${name} must contain exactly three finite numbers`);
  }
  return values;
}

/** Resolve a package mesh URI through the generated Session C manifest. */
export function resolveMeshUri(meshUri, manifest) {
  if (!manifest || typeof manifest !== "object" || !manifest.meshes) {
    return null;
  }
  const resolved = manifest.meshes[meshUri];
  return typeof resolved === "string" && resolved.length > 0 ? resolved : null;
}

/**
 * Parse the small URDF subset used by the dual Panda browser model.
 *
 * This intentionally ignores collision, inertial, safety, transmission and
 * ros2_control data. The returned object contains plain browser data only.
 */
export function parseUrdf(urdfText, manifest = null) {
  if (typeof urdfText !== "string" || urdfText.trim() === "") {
    throw new TypeError("urdfText must be a non-empty string");
  }

  const documentNode = new DOMParser().parseFromString(urdfText, "application/xml");
  const parserError = documentNode.querySelector("parsererror");
  if (parserError) {
    throw new Error(`invalid URDF XML: ${parserError.textContent.trim()}`);
  }
  const robot = documentNode.documentElement;
  if (!robot || robot.tagName !== "robot") {
    throw new Error("URDF root element must be <robot>");
  }

  const links = {};
  for (const linkElement of Array.from(robot.children).filter(
    (child) => child.tagName === "link",
  )) {
    const name = requiredAttribute(linkElement, "name", "link");
    if (Object.hasOwn(links, name)) {
      throw new Error(`duplicate link ${name}`);
    }

    let visual = null;
    const visualElement = directChild(linkElement, "visual");
    const geometryElement = visualElement && directChild(visualElement, "geometry");
    const meshElement = geometryElement && directChild(geometryElement, "mesh");
    if (meshElement) {
      const meshUri = requiredAttribute(meshElement, "filename", `link ${name} mesh`);
      visual = {
        meshUri,
        assetUri: resolveMeshUri(meshUri, manifest),
      };
    }
    links[name] = {name, visual};
  }

  const joints = [];
  const childLinks = new Set();
  const jointNames = new Set();
  for (const jointElement of Array.from(robot.children).filter(
    (child) => child.tagName === "joint",
  )) {
    const name = requiredAttribute(jointElement, "name", "joint");
    if (jointNames.has(name)) {
      throw new Error(`duplicate joint ${name}`);
    }
    jointNames.add(name);

    const type = requiredAttribute(jointElement, "type", `joint ${name}`);
    const parentElement = directChild(jointElement, "parent");
    const childElement = directChild(jointElement, "child");
    const parent = requiredAttribute(parentElement, "link", `joint ${name} parent`);
    const child = requiredAttribute(childElement, "link", `joint ${name} child`);
    if (!Object.hasOwn(links, parent) || !Object.hasOwn(links, child)) {
      throw new Error(`joint ${name} references an unknown link`);
    }
    if (childLinks.has(child)) {
      throw new Error(`link ${child} has more than one parent joint`);
    }
    childLinks.add(child);

    const originElement = directChild(jointElement, "origin");
    const origin = {
      xyz: vectorAttribute(originElement, "xyz", [0, 0, 0], `joint ${name} origin`),
      rpy: vectorAttribute(originElement, "rpy", [0, 0, 0], `joint ${name} origin`),
    };

    const axisElement = directChild(jointElement, "axis");
    const axis = type === "fixed"
      ? null
      : vectorAttribute(axisElement, "xyz", [1, 0, 0], `joint ${name} axis`);

    const limitElement = directChild(jointElement, "limit");
    const limit = limitElement ? {
      lower: numberAttribute(limitElement, "lower", `joint ${name} limit`),
      upper: numberAttribute(limitElement, "upper", `joint ${name} limit`),
      velocity: numberAttribute(limitElement, "velocity", `joint ${name} limit`),
      effort: numberAttribute(limitElement, "effort", `joint ${name} limit`),
    } : null;
    if (type === "revolute" && !limit) {
      throw new Error(`revolute joint ${name} is missing <limit>`);
    }

    joints.push({name, type, parent, child, origin, axis, limit});
  }

  const roots = Object.keys(links).filter((name) => !childLinks.has(name));
  if (roots.length === 0 && Object.keys(links).length > 0) {
    throw new Error("URDF joint graph has no root link");
  }
  return {links, joints, roots};
}
