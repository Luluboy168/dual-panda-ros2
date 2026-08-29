<!-- [DURABLE] Session C contract and mechanical merge inventory. -->

# Franka ghost frontend

This package is the standalone, fake-hardware-only Session C prototype for the
dual-Panda joint-space ghost editor. It contains no IK, Cartesian interaction,
controller publishing, or real-robot connection. The standalone HTTP server,
polling transport, host page, and synthetic/live state sources are explicitly
throwaway; the browser module and asset pipeline are the merge product.

## Frozen Contract C3 — the ghost module mount API

The following section is copied verbatim from
`SESSION_C_GHOST_PLAN.md` §4 C3. It is the merge seam consumed by
`franka_web`.

### C3 — the ghost module mount API (the merge seam)

`franka_web` will call exactly this. Nothing else in `web/ghost/` is public.

```js
// web/ghost/ghost.js
export function mount(container, options) -> GhostHandle

// options:
{
  urdfUrl:      "/assets/model.urdf",    // string, required
  manifestUrl:  "/assets/manifest.json", // string, required
  assetBase:    "/assets/",              // string, required; prefix for mesh files
  arms: [                                // array, 1 or 2 entries
    { armIndex: 1, armId: "panda1" },
    { armIndex: 2, armId: "panda2" }
  ],
  initialArm:   1,                       // integer; which arm the ghost starts on
  jogStepRad:   0.034906585,             // number; keyboard nudge = 2 deg. MUST equal Session A's
                                         //   capabilities.jog_step_rad (SESSION_A section 6.1);
                                         //   franka_web passes it in, never hardcoded twice.
  onApply:      (GhostApplyEvent) => {}  // required; see C1
}
// SIMPLICITY CUT: onSelect and onGhostChange were removed from the frozen seam. No consumer in
// SESSION_A_WEB_V1_PLAN.md reads either, and onGhostChange fired per drag frame is a per-frame
// callback across a merge boundary that nothing needed. Adding a callback later is additive.

// GhostHandle:
{
  setMeasured(map),      // map: {"panda1_joint1": 0.12, ...}. Partial maps allowed;
                         //   unknown names ignored; missing joints keep their last value.
                         //   A present name whose value is null/undefined/NaN/Infinity counts
                         //   as ABSENT and keeps its last value — Session A §6.11 rule 2 puts
                         //   null at the index of any name missing from the JointState, and
                         //   wiring #2 below zips those nulls straight into this map, so this
                         //   must never throw. A present non-number that is not null does
                         //   throw, and rejects the whole frame without applying any of it.
                         //   MATCH BY NAME — never positional. Safe to call at 20-60 Hz.
  setFence(armIndex, {lower:[7], upper:[7], source:"session"}),
                         // narrows the clamp. MUST reject (throw) a fence wider than the URDF
                         //   limits — invariant I4.
  setGhost(armIndex, positions7),   // programmatic ghost placement; clamped on the way in
  syncGhostToMeasured(armIndex),    // snap ghost onto the solid robot
  setGhostVisible(armIndex, bool),
  selectArm(armIndex),
  setEnabled(bool),      // false => read-only: no picking, no drag, no Apply. franka_web calls
                         //   setEnabled(false) whenever the session is not in Motion mode,
                         //   or the arm's Enable toggle is off, or the arm is faulted.
  dispose()              // release GL context, listeners, RAF; idempotent
}
```

Three rules that make this seam hold:

- The ghost module **never** performs I/O of its own beyond fetching `urdfUrl` / `manifestUrl` /
  mesh assets. No polling, no websockets, no `POST`. State arrives via `setMeasured`; output leaves
  via `onApply`. The throwaway `transport.js` is what wires those to the dev server, and it is the
  *only* file that knows the dev server exists.
- The ghost module **never** imports from outside `web/ghost/` except `../vendor/three.min.js`.
- The ghost module has **no knowledge of controller names, topics, services, or ROS**. Grep
  `web/ghost/` for `impedance`, `rclpy`, `topic`, `service`, `/dual_arm` — all must be zero hits.
  Make this a test (Stage 8).

## 8. Merge path into `franka_web`

The merge is a directory move plus three callback wirings, provided C3 held.

**Moves unchanged** into `franka_web`'s asset tree:

| From | To | Note |
| --- | --- | --- |
| `franka_ghost/web/ghost/*.js` | `franka_web/franka_web/static/ghost/` — Session A's package layout puts served assets under `franka_web/franka_web/static/` (`SESSION_A_WEB_V1_PLAN.md` §2), installed via `install(DIRECTORY static ...)` | the whole point of the session |
| `franka_ghost/franka_ghost/urdf_export.py` | `franka_web`'s asset build step | same xacro args |
| `franka_ghost/franka_ghost/mesh_convert.py` | same | C5 format unchanged |
| `franka_ghost/web/assets/` layout | `franka_web` serves the identical relative paths under its own `static/` root | manifest-driven |
| `franka_ghost/test/browser/cases/{urdf,kinematics,apply}.js` | `franka_web`'s browser suite | unchanged assertions |
| C3 (`mount()`) and C1 (`GhostApplyEvent`) | the seam itself | frozen |

**Deleted at merge:**

| File | Replaced by |
| --- | --- |
| `franka_ghost/franka_ghost/dev_server.py` | `franka_web`'s HTTP server |
| `franka_ghost/franka_ghost/joint_source.py` | `franka_web`'s Health-card joint stream |
| `franka_ghost/scripts/franka_ghost_dev_server.py` | — |
| `franka_ghost/web/transport.js` (polling) | `franka_web`'s websocket feed |
| `franka_ghost/web/index.html`, `style.css` | `franka_web`'s single page, Control card |
| `POST /apply` echo endpoint | `franka_web`'s real jog/target endpoint |
| the `demo` synthetic source | live state |
| the "PROTOTYPE — no robot connection" banner | — |
| `test/test_dev_server.py`, `test/live/` | `franka_web`'s own integration tests |

**The three wirings `franka_web` performs:**

1. `mount(el, {..., onApply: e => webv1.sendJogTarget(e)})` — `franka_web` converts C1 → C2 and
   owns the ≥ 10 Hz republish, the enable service call, and the ROS clock stamp. The ghost stays
   ignorant of all of it.
2. On every Health-card state frame (Session A §6.11; the transport is SSE in that build):
   `handle.setMeasured({...})` built by zipping each arm's `joint_names` with its `positions`.
   Same by-name matching. **No filtering is required on `franka_web`'s side:** §6.11 rule 2 puts
   `null` at the index of any name absent from the incoming `JointState` (and sets
   `positions_stale`), and `setMeasured` treats such an entry as absent — that joint holds its last
   known value and the rest of the frame still lands. One stale joint name can never throw out of
   the state callback.
3. On session start and on every gains-file change, from Session A's `POST /api/gains` response
   (§6.5), whose `fence` object is keyed by **`arm_id`**, not by `arm_N`:
   `handle.setFence(n, {lower: resp.fence[armId].position_lower, upper: resp.fence[armId].position_upper, source: "session"})`
   — the ghost then clamps to the *session's* fence, which the validator has already proven lies
   inside the Panda policy (`controller_config_validator.py:415-422`). Plus
   `handle.setEnabled(motionMode && armEnabled && !faulted)`, derived from the §6.11 frame's
   `session.mode`, `arms[armId].motion.enabled` and `fault.active`.

**Safety properties preserved by construction:** the ghost cannot move a robot (it has no ROS and
no network egress, C3); it cannot render an unreachable pose (I3/I4 clamp); it cannot outlive its
fence (`setFence` narrows only); and it goes read-only the moment `franka_web` says the session is
not in Motion mode.

---

## Complete per-file ownership classification

The §8 inventory above is authoritative. Section 2 additionally marks
`asset_prep.py` durable; its package initializer and asset-pipeline unit tests
move conceptually with that retained code. Files needed only to build or test this standalone ROS
package are throwaway even where their behavior is re-created in `franka_web`.
Generated `web/assets/` content is intentionally gitignored, but its relative
layout is durable under the §8 move table.

| Classification | Files |
| --- | --- |
| `[DURABLE]` | `README.md`; `franka_ghost/{__init__,asset_prep,mesh_convert,urdf_export}.py`; `web/ghost/*.js`; `test/{test_mesh_convert,test_urdf_export}.py`; `test/browser/cases/{urdf,kinematics,apply}.js`; generated `web/assets/` layout |
| `[THROWAWAY]` | `.gitignore`; `CMakeLists.txt`; `package.xml`; `franka_ghost/{dev_server,joint_source}.py`; `scripts/*.py`; `web/{index.html,style.css,transport.js}`; `test/{test_apply_payload,test_dev_server,test_module_purity}.py`; `test/golden_ghost_apply_event.json`; `test/browser/{harness.html,run_browser_tests.py,test_browser.py}`; `test/browser/cases/{drag,scene}.js`; `test/live/` |

JSON does not support comments, so the golden event data file is classified in
this table instead of being made invalid with a header comment. Generated and
vendored gitignored files are likewise classified by directory rather than
modified after generation.
