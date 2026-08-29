// [DURABLE] Contract C1/C3 browser checks retained when merged into franka_web.

import * as ghostModule from "../../../web/ghost/ghost.js";
import {
  assertGhostApplyEvent,
  buildGhostApplyEvent,
  toWebV1Payload,
  URDF_LOWER,
  URDF_UPPER,
  validateGhostApplyEvent,
} from "../../../web/ghost/apply.js";

const INITIAL = [0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4];
const ARBITRARY_1 = [0.21, -0.9, 0.31, -2.15, -0.22, 1.91, 0.61];
const ARBITRARY_2 = [-0.18, -0.7, -0.27, -2.31, 0.24, 1.72, -0.52];
const HANDLE_KEYS = [
  "dispose",
  "selectArm",
  "setEnabled",
  "setFence",
  "setGhost",
  "setGhostVisible",
  "setMeasured",
  "syncGhostToMeasured",
];

function measuredMap(arm1 = INITIAL, arm2 = INITIAL) {
  const result = {};
  for (const [armId, positions] of [["panda1", arm1], ["panda2", arm2]]) {
    positions.forEach((position, index) => {
      result[`${armId}_joint${index + 1}`] = position;
    });
  }
  return result;
}

function baseEvent() {
  return buildGhostApplyEvent({
    armIndex: 1,
    positions: ARBITRARY_1,
    fence: {lower: URDF_LOWER, upper: URDF_UPPER, source: "urdf"},
    measuredAtApply: INITIAL,
    ghostEpoch: 1,
  });
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

async function waitFor(predicate, message, timeoutMs = 7000) {
  const deadline = performance.now() + timeoutMs;
  while (!predicate()) {
    if (performance.now() > deadline) {
      throw new Error(message);
    }
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
}

async function postC1(event) {
  const response = await fetch("/apply", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(event),
  });
  if (!response.ok) {
    throw new Error(`POST /apply returned HTTP ${response.status}`);
  }
  return response.json();
}

export async function runApplyCases({
  test,
  assert,
  assertEqual,
  assertArrayNear,
}) {
  await test("apply.js builds a cloned C1 event and prototype adaptation is identity", () => {
    const positions = [...ARBITRARY_1];
    const measured = [...INITIAL];
    const event = buildGhostApplyEvent({
      armIndex: 1,
      positions,
      fence: {lower: URDF_LOWER, upper: URDF_UPPER, source: "urdf"},
      measuredAtApply: measured,
      ghostEpoch: 1,
    });
    positions[0] = 2;
    measured[0] = 2;
    assertArrayNear(event.positions, ARBITRARY_1, 0, "positions were not snapshotted");
    assertArrayNear(event.measured_at_apply, INITIAL, 0, "measured pose was not snapshotted");
    assertEqual(toWebV1Payload(event), event, "prototype adaptation must pass C1 through");
  });

  const invalidCases = [
    ["I1", (event) => event.joint_names.reverse(), null],
    ["I2", (event) => { event.positions[0] = Number.NaN; }, null],
    ["I3", (event) => { event.positions[3] = event.fence.upper[3] + 0.01; }, null],
    ["I4", (event) => { event.fence.lower[0] = URDF_LOWER[0] - 0.01; }, null],
    ["I5", (event) => { event.arm_id = "panda2"; }, null],
    ["I6", () => {}, 1],
    ["I7", (event) => { event.nested = {time_from_start: {sec: 0}}; }, null],
  ];
  for (const [invariant, mutate, previousEpoch] of invalidCases) {
    await test(`apply.js refuses ${invariant} before emission`, () => {
      const event = clone(baseEvent());
      mutate(event);
      const errors = validateGhostApplyEvent(event, previousEpoch);
      assert(errors.some((error) => error.startsWith(`${invariant}:`)), errors.join("; "));
      let threw = false;
      try {
        assertGhostApplyEvent(event, previousEpoch);
      } catch (error) {
        threw = error.message.includes(`${invariant}:`);
      }
      assert(threw, `${invariant} did not throw before emission`);
    });
  }

  await test("prototype Apply endpoint rejects an invalid C1 browser roundtrip", async () => {
    const invalid = clone(baseEvent());
    invalid.positions[3] = invalid.fence.upper[3] + 0.01;
    const verdict = await postC1(invalid);
    assert(!verdict.accepted, "invalid C1 event was accepted by POST /apply");
    assert(
      verdict.errors.some((error) => error.startsWith("I3:")),
      `invalid roundtrip did not report I3: ${JSON.stringify(verdict.errors)}`,
    );
  });

  await test("single-arm C3 retains pre-ready sync and rejects measured frames atomically", async () => {
    const host = document.createElement("div");
    host.style.cssText = "width:1100px;height:650px";
    document.body.append(host);
    const emitted = [];
    const handle = ghostModule.mount(host, {
      urdfUrl: new URL("../../../web/assets/model.urdf", import.meta.url).href,
      manifestUrl: new URL("../../../web/assets/manifest.json", import.meta.url).href,
      assetBase: new URL("../../../web/assets/", import.meta.url).href,
      arms: [{armIndex: 2, armId: "panda2"}],
      initialArm: 2,
      jogStepRad: Math.PI / 90,
      onApply(event) {
        emitted.push(event);
      },
    });
    try {
      assertEqual(JSON.stringify(Object.keys(handle).sort()), JSON.stringify(HANDLE_KEYS));
      handle.setMeasured(measuredMap());
      handle.syncGhostToMeasured(2);

      let rejectedAtomically = false;
      try {
        handle.setMeasured({
          panda2_joint1: ARBITRARY_2[0],
          // A value that is neither a number nor the documented null placeholder is a caller
          // bug, and the whole frame must be refused without applying its valid half.
          panda2_joint7: "0.0",
        });
      } catch (error) {
        rejectedAtomically = error.message.includes("panda2_joint7");
      }
      assert(rejectedAtomically, "mixed valid/invalid measured frame did not throw");

      await waitFor(
        () => host.querySelector('[data-ghost-ready="true"]'),
        "single-arm public mount did not become ready",
      );
      assertEqual(host.querySelectorAll("[data-arm-panel]").length, 1);
      assert(host.querySelector('[data-arm-panel="2"]'), "arm 2 panel is missing");
      assert(!host.querySelector('[data-arm-panel="1"]'), "unconfigured arm 1 panel leaked in");

      // Refresh staleness without changing the pose established before readiness.
      handle.setMeasured({panda2_joint2: INITIAL[1]});
      const applyButton = host.querySelector(".apply-action");
      await waitFor(() => !applyButton.disabled, "single-arm Apply did not enable");
      applyButton.click();
      await waitFor(() => emitted.length === 1, "single-arm Apply did not emit");
      assertEqual(emitted[0].arm_index, 2);
      assertEqual(emitted[0].arm_id, "panda2");
      assertArrayNear(
        emitted[0].positions,
        INITIAL,
        0,
        "pre-ready sync was not retained until renderer readiness",
      );
      assertArrayNear(
        emitted[0].measured_at_apply,
        INITIAL,
        0,
        "invalid measured frame partially changed the accepted snapshot",
      );
    } finally {
      handle.dispose();
      handle.dispose();
      host.remove();
    }
  });

  // SESSION_A_WEB_V1_PLAN.md section 6.11 frame rule 2: positions are extracted from the incoming
  // 14-name JointState by name, and "a name that is absent yields null at that index and sets
  // positions_stale: true". README wiring 2 has franka_web build this map by zipping joint_names
  // with positions, so those nulls arrive at setMeasured verbatim. Before this test, setMeasured
  // threw on them, which would have taken out franka_web's whole state callback at merge over one
  // stale joint name.
  await test("setMeasured holds the last value for Session A's null and non-finite entries", async () => {
    const host = document.createElement("div");
    host.style.cssText = "width:1100px;height:650px";
    document.body.append(host);
    const emitted = [];
    const handle = ghostModule.mount(host, {
      urdfUrl: new URL("../../../web/assets/model.urdf", import.meta.url).href,
      manifestUrl: new URL("../../../web/assets/manifest.json", import.meta.url).href,
      assetBase: new URL("../../../web/assets/", import.meta.url).href,
      arms: [{armIndex: 2, armId: "panda2"}],
      initialArm: 2,
      jogStepRad: Math.PI / 90,
      onApply(event) {
        emitted.push(event);
      },
    });
    try {
      handle.setMeasured(measuredMap());
      await waitFor(
        () => host.querySelector('[data-ghost-ready="true"]'),
        "null-tolerance mount did not become ready",
      );

      // Exactly the shape a zipped Session A frame produces when two names are missing from the
      // JointState, plus the non-finite floats any producer may emit for "no reading".
      const held = [...INITIAL];
      held[0] = ARBITRARY_2[0];
      held[5] = ARBITRARY_2[5];
      handle.setMeasured({
        panda2_joint1: ARBITRARY_2[0],
        panda2_joint2: null,
        panda2_joint3: undefined,
        panda2_joint4: Number.NaN,
        panda2_joint5: Number.POSITIVE_INFINITY,
        panda2_joint6: ARBITRARY_2[5],
        panda2_joint7: null,
      });

      // A frame carrying no reading at all is equally tolerated and changes nothing.
      handle.setMeasured({
        panda2_joint1: null,
        panda2_joint2: null,
        panda2_joint3: null,
        panda2_joint4: null,
        panda2_joint5: null,
        panda2_joint6: null,
        panda2_joint7: null,
      });

      handle.syncGhostToMeasured(2);
      const applyButton = host.querySelector(".apply-action");
      await waitFor(() => !applyButton.disabled, "Apply did not enable after a partially null frame");
      applyButton.click();
      await waitFor(() => emitted.length === 1, "Apply did not emit after a partially null frame");
      assertArrayNear(
        emitted[0].measured_at_apply,
        held,
        0,
        "null and non-finite entries did not hold their last known values",
      );
      assertArrayNear(
        emitted[0].positions,
        held,
        0,
        "ghost synced to something other than the held measured pose",
      );
    } finally {
      handle.dispose();
      host.remove();
    }
  });

  await test("ghost.js exports only mount and returns exactly the frozen C3 handle", async () => {
    assertEqual(JSON.stringify(Object.keys(ghostModule)), JSON.stringify(["mount"]));
    const host = document.createElement("div");
    host.style.cssText = "width:1100px;height:650px";
    document.body.append(host);
    let callbackMode = "post";
    const emitted = [];
    const roundTrips = [];
    const handle = ghostModule.mount(host, {
      urdfUrl: new URL("../../../web/assets/model.urdf", import.meta.url).href,
      manifestUrl: new URL("../../../web/assets/manifest.json", import.meta.url).href,
      assetBase: new URL("../../../web/assets/", import.meta.url).href,
      arms: [
        {armIndex: 1, armId: "panda1"},
        {armIndex: 2, armId: "panda2"},
      ],
      initialArm: 1,
      jogStepRad: Math.PI / 90,
      onApply(event) {
        emitted.push(event);
        if (callbackMode === "reject") {
          throw new Error("test callback refused the event");
        }
        const request = postC1(event);
        roundTrips.push(request);
        return request.then((verdict) => {
          if (!verdict.accepted) {
            throw new Error(verdict.errors.join("; "));
          }
          return verdict;
        });
      },
    });
    assertEqual(JSON.stringify(Object.keys(handle).sort()), JSON.stringify(HANDLE_KEYS));

    // The synchronous facade must safely retain state while its assets load.
    handle.setMeasured(measuredMap());
    handle.setGhost(1, ARBITRARY_1);
    await waitFor(
      () => host.querySelector('[data-ghost-ready="true"]'),
      "public mount did not become ready",
    );
    handle.setMeasured(measuredMap());
    const applyButton = host.querySelector(".apply-action");
    const applyStatus = host.querySelector(".ghost-apply-status");
    await waitFor(() => !applyButton.disabled, "Apply did not enable for a fresh complete pose");

    applyButton.click();
    await waitFor(() => emitted.length === 1, "arm 1 Apply did not synchronously snapshot");
    const first = emitted[0];
    // Mutation after the press must not change either snapshot in the event.
    handle.setMeasured(measuredMap(ARBITRARY_2, INITIAL));
    handle.setGhost(1, INITIAL);
    const firstVerdict = await roundTrips[0];
    assert(firstVerdict.accepted, firstVerdict.errors && firstVerdict.errors.join("; "));
    assertEqual(JSON.stringify(firstVerdict.echo), JSON.stringify(first));
    assertEqual(first.arm_id, "panda1");
    assertEqual(first.arm_index, 1);
    assertEqual(first.ghost_epoch, 1);
    assertEqual(
      JSON.stringify(first.joint_names),
      JSON.stringify(Array.from({length: 7}, (_unused, index) => `panda1_joint${index + 1}`)),
    );
    assertArrayNear(first.positions, ARBITRARY_1, 0, "arm 1 payload did not match ghost");
    assertArrayNear(first.measured_at_apply, INITIAL, 0, "measured snapshot changed after press");

    handle.selectArm(2);
    handle.setGhostVisible(2, true);
    handle.setGhost(2, ARBITRARY_2);
    handle.setMeasured(measuredMap());
    await waitFor(() => !applyButton.disabled, "arm 2 Apply did not enable");
    applyButton.click();
    await waitFor(() => emitted.length === 2, "arm 2 Apply did not emit");
    const secondVerdict = await roundTrips[1];
    assert(secondVerdict.accepted, secondVerdict.errors && secondVerdict.errors.join("; "));
    assertEqual(emitted[1].arm_id, "panda2");
    assertEqual(emitted[1].arm_index, 2);
    assertEqual(emitted[1].ghost_epoch, 2);
    assertArrayNear(emitted[1].positions, ARBITRARY_2, 0, "arm 2 payload mismatch");

    // Freshness disables Apply and greys only the solid; editing remains enabled.
    await waitFor(() => applyButton.disabled && applyStatus.textContent.includes("stale"),
      "stale measured state did not visibly refuse Apply", 1500);
    assert(!host.querySelector(".viewport").classList.contains("ghost-read-only"),
      "staleness incorrectly made ghost editing read-only");

    // A callback-side refusal must remain visible instead of escaping the UI.
    callbackMode = "reject";
    handle.setMeasured(measuredMap());
    await waitFor(() => !applyButton.disabled, "Apply did not recover after a fresh frame");
    applyButton.click();
    await waitFor(() => applyStatus.textContent.includes("test callback refused"),
      "callback refusal was not rendered in the UI");
    assertEqual(emitted[2].ghost_epoch, 3, "mount epoch did not increment globally");

    handle.setEnabled(false);
    assert(applyButton.disabled, "read-only C3 state left Apply enabled");
    handle.dispose();
    handle.dispose();
    host.remove();
  });
}
