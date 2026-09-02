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

// The browser suite's driver. It lives in a file rather than inline in
// harness.html because the page is served under the production
// Content-Security-Policy, which refuses inline script.

const verdictElement = document.getElementById("verdict");
const failures = [];
const problems = [];
let tests = 0;
let finished = false;

function describe(value) {
  try {
    return typeof value === "string" ? value : JSON.stringify(value);
  } catch (_error) {
    return String(value);
  }
}

const originalConsoleError = console.error.bind(console);
const originalConsoleWarn = console.warn.bind(console);
console.error = (...args) => {
  problems.push({kind: "console.error", message: args.map(describe).join(" ")});
  originalConsoleError(...args);
};
console.warn = (...args) => {
  problems.push({kind: "console.warn", message: args.map(describe).join(" ")});
  originalConsoleWarn(...args);
};
window.addEventListener("error", (event) => {
  problems.push({
    kind: "uncaught exception",
    message: event.error && event.error.stack ? event.error.stack : event.message,
  });
});
window.addEventListener("unhandledrejection", (event) => {
  const reason = event.reason;
  problems.push({
    kind: "unhandled rejection",
    message: reason && reason.stack ? reason.stack : describe(reason),
  });
});
// A silent violation list is evidence only once the listener is known to fire
// AND the page is known to carry the policy. The self-check below proves both
// before the suite is allowed to trust that silence.
window.addEventListener("securitypolicyviolation", (event) => {
  problems.push({
    kind: "securitypolicyviolation",
    message: `${event.violatedDirective}: ${event.blockedURI || ""} `
      + `(${event.sourceFile || "inline"})`,
  });
});
window.__ghostHarnessProblems = problems;

function serialiseError(error) {
  return error && error.stack ? error.stack : String(error);
}

function finish(extra = {}) {
  if (finished) {
    return;
  }
  finished = true;
  clearTimeout(hardTimeout);
  const verdict = {
    ok: failures.length === 0 && problems.length === 0 && !extra.timeout,
    tests,
    failures,
    browser_problems: problems,
    ...extra,
  };
  const encoded = JSON.stringify(verdict);
  verdictElement.textContent = encoded;
  verdictElement.dataset.status = "complete";
  document.title = verdict.ok ? "PASS" : "FAIL";
  // The suite cannot run under --virtual-time-budget: requestAnimationFrame
  // fires once there and then stops, so a scene that renders on demand never
  // renders and a drag that coalesces to a frame never sends. The runner
  // therefore lets real time pass, and the page says when it is done. Same
  // origin, so connect-src 'self' allows it; #verdict stays the record for a
  // runner that would rather read the DOM.
  try {
    fetch("/__ghost_verdict", {method: "POST", body: encoded});
  } catch (_error) {
    // A runner that reads #verdict instead is equally welcome.
  }
}

const hardTimeout = setTimeout(() => {
  failures.push({name: "harness hard timeout", error: "browser suite exceeded 120000 ms"});
  finish({timeout: true});
}, 120000);

const context = {
  async test(name, callback) {
    tests += 1;
    try {
      await callback();
    } catch (error) {
      failures.push({name, error: serialiseError(error)});
    }
  },
  assert(condition, message = "assertion failed") {
    if (!condition) {
      throw new Error(message);
    }
  },
  assertEqual(actual, expected, message = "values differ") {
    if (actual !== expected) {
      throw new Error(
        `${message}: expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`,
      );
    }
  },
  assertNear(actual, expected, tolerance, message = "numbers differ") {
    if (!Number.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
      throw new Error(`${message}: expected ${expected} ± ${tolerance}, got ${actual}`);
    }
  },
  assertArrayNear(actual, expected, tolerance, message = "arrays differ") {
    if (!Array.isArray(actual) || actual.length !== expected.length) {
      throw new Error(`${message}: array lengths differ`);
    }
    actual.forEach((value, index) => {
      context.assertNear(value, expected[index], tolerance, `${message} at index ${index}`);
    });
  },
  /** Let the render loop and any queued promise work run. */
  async settle(frames = 2) {
    for (let index = 0; index < frames; index += 1) {
      await new Promise((resolve) => requestAnimationFrame(resolve));
    }
    await new Promise((resolve) => setTimeout(resolve, 0));
  },
};

await context.test("the content-security policy is live and observed", async () => {
  // ONE deliberate violation, caught and then dropped from the list. A runner
  // that forgets the policy header, or a listener that never fires, fails here
  // instead of quietly passing every CSP assertion downstream.
  const before = problems.length;
  const probe = document.createElement("div");
  // A PARSED style attribute is what the policy blocks; the CSSOM writes the
  // shipped modules use are not a policy subject at all.
  probe.setAttribute("style", "color: rgb(1, 2, 3)");
  document.body.append(probe);
  const applied = getComputedStyle(probe).color;
  await new Promise((resolve) => setTimeout(resolve, 0));
  probe.remove();
  const caught = problems.slice(before)
    .filter((problem) => problem.kind === "securitypolicyviolation");
  problems.length = before;
  context.assert(
    caught.length > 0,
    "no securitypolicyviolation fired for a parsed style attribute: either the runner "
    + "served no Content-Security-Policy header or the listener is dead, and every CSP "
    + `assertion below would pass by construction (the colour applied as ${applied})`,
  );
});

try {
  const [
    {runUrdfCases},
    {runKinematicsCases},
    {runSceneCases},
    {runDragCases},
  ] = await Promise.all([
    import("./cases/urdf.js"),
    import("./cases/kinematics.js"),
    import("./cases/scene.js"),
    import("./cases/drag.js"),
  ]);
  await runUrdfCases(context);
  await runKinematicsCases(context);
  await runSceneCases(context);
  await runDragCases(context);
} catch (error) {
  failures.push({name: "suite setup", error: serialiseError(error)});
}
await new Promise((resolve) => setTimeout(resolve, 0));
finish();
