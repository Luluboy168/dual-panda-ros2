<!--
Copyright 2026 The multipanda_ros2 Authors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Franka IK

`franka_ik` is a standalone, deterministic inverse-kinematics service for one to four Panda
chains loaded from a URDF string. It is deliberately robot-stack-free: URDF plus parameters plus
request in, deterministic solutions out. It reads no live state, publishes no command, and never
contacts a robot.

The v1 backend is numeric-only: Orocos KDL
`ChainIkSolverPos_LMA`, with fixed joint 7, an iteration bound, joint-limit post-filtering, and an
independent KDL FK acceptance check. The package has no dependency on MoveIt, controller manager,
Franka hardware, `franka_web`, or the reviewed robot stack.

## Launch and inspect

Build and source the workspace, then use the session's interactive ROS domain:

```bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=SHM
ros2 launch franka_ik franka_ik.launch.py
```

The default launch renders the package-owned, URDF-only dual-arm xacro. It contains no
`ros2_control` block and requires no network address. To load another kinematic description, pass
`robot_description_file:=/absolute/path/to/model.urdf.xacro` and an appropriate
`arm_ids:='[panda]'` or other configured list.

In a second sourced shell with the same domain, inspect the loaded chains:

```bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=SHM
ros2 service call /franka_ik_service/chain_info \
  franka_ik_interfaces/srv/GetChainInfo '{}'
```

A complete, reachable flange request for the default `panda1` chain is:

```bash
ros2 service call /franka_ik_service/solve_ik \
  franka_ik_interfaces/srv/SolveIk \
  "{request: {frame_id: panda1_link0, arm_id: panda1, tip_frame: 0, \
  target_pose: {position: {x: 0.17255052551411307, y: 0.12230509713297047, \
  z: 1.0619921789728903}, orientation: {x: -0.7853152885879714, \
  y: 0.3802913936116545, z: 0.020837360558426443, w: 0.4880821220449024}}, \
  seed_positions: [1.292828163875247, -0.12392445309347258, -0.6692734092177752, \
  -0.39325855124207026, 0.42768355848674866, 1.2486806990419492, \
  1.8392452272436794], redundancy_mode: 0, redundancy_value: 0.0, \
  max_solutions: 1, solver: 0, position_tolerance: 0.0, \
  orientation_tolerance: 0.0, joint_limit_margin: 0.0}}"
```

The example uses an exact corpus witness, requests the flange in the arm base frame, takes joint
7 from the seed, selects the node's default solver, and uses the configured acceptance
tolerances. A caller owns the seven-element seed; the service never supplies one from robot state.

## Frame conventions

For arm id `A`, the kinematic chain is `A_link0` to `A_link8`. `A_link8` is the default flange
tip. If the loaded URDF also contains `A_hand_tcp`, `TIP_HAND_TCP` is available; otherwise that
request returns `RESULT_UNSUPPORTED_TIP`.

`target_pose` may be expressed in `""`, `A_link0`, or the parsed URDF root. An empty frame and
`A_link0` both mean the arm base. A root-frame target is converted as
`base_T_target = inverse(root_T_base) * root_T_target`, using the transform parsed from the URDF.
The service has no TF listener and never hardcodes a mounting transform.

## 5. THE CONTRACT

This section is normative. `franka_web` may rely on every statement in it. Any change after
Stage 4 lands is a breaking change requiring a coordinated update of both sessions.

### 5.1 Package split, and why

- **`franka_ik_interfaces`** — `ament_cmake` + `rosidl_generate_interfaces` only. No C++, no node,
  no dependency beyond `std_msgs`, `geometry_msgs`, `builtin_interfaces`. This is the *only*
  package `franka_web` depends on.
- **`franka_ik`** — the solver library, the service node, the launch file. Depends on
  `franka_ik_interfaces`.

Rationale: ROS 2 discourages generating interfaces in a package that also builds libraries linking
against them, and — more importantly here — a pure-rosidl package gives `franka_web` a
dependency edge that carries no Eigen, no KDL, and no vendored third-party code. The track brief's
"standalone package `franka_ik`" is satisfied: one functional unit, two ament packages, no
dependency on `franka_web` in either direction.

### 5.2 `franka_ik_interfaces/msg/IkRequest.msg`

```
# One IK query.

# ---- reference frame of target_pose -------------------------------------
# ""  or  "<arm_id>_link0"  -> the arm's own base frame (recommended; mounting-independent)
# "<urdf_root>"             -> the URDF root frame ("base_link" for the dual description)
# anything else             -> RESULT_UNKNOWN_FRAME
string<=128 frame_id

# ---- which arm ----------------------------------------------------------
# Must equal one of the arm ids the node loaded at configure time.
# "panda1"/"panda2" for the dual description, "panda" for the single one.
string<=64 arm_id

# ---- what pose to solve for ---------------------------------------------
uint8 TIP_FLANGE=0     # <arm_id>_link8    - the URDF flange; hand-independent; the default
uint8 TIP_HAND_TCP=1   # <arm_id>_hand_tcp - only when the loaded description includes a hand
uint8 tip_frame 0

# Pose of tip_frame expressed in frame_id. The quaternion is normalised by the service;
# a norm outside [1e-6, 1e6] or any non-finite component is RESULT_BAD_REQUEST.
geometry_msgs/Pose target_pose

# ---- seed ---------------------------------------------------------------
# REQUIRED, always 7 values, radians, in the canonical joint1..joint7 order.
# The service NEVER reads robot state; the caller owns "where the arm is now".
# Non-finite -> RESULT_BAD_REQUEST. Outside the joint limits -> RESULT_SEED_OUT_OF_LIMITS.
float64[7] seed_positions

# ---- 7-DOF redundancy ---------------------------------------------------
uint8 REDUNDANCY_FROM_SEED=0  # q7 := seed_positions[6]  (drag-continuity default)
uint8 REDUNDANCY_FIXED=1      # q7 := redundancy_value (rad, inside joint 7's limits)
# SIMPLICITY CUT: REDUNDANCY_SCAN (sweep q7 across its range) was removed. Nothing on the locked
# ladder searches over q7 -- a drag re-solves from the previous seed. Adding a third mode later is
# additive (enum value 2 stays free); building it now is a workspace-search feature with no caller.
uint8 redundancy_mode 0
float64 redundancy_value 0.0

# ---- output shaping -----------------------------------------------------
# 0    -> REACHABILITY PROBE: solutions[] stays empty, only `result` is meaningful. Cheapest path.
# 1..4 -> return at most this many solutions, ordered by seed_distance ascending (best first).
#         4 is the analytic branch count; there is never a fifth distinct answer to rank.
# >4   -> RESULT_BAD_REQUEST
uint8 max_solutions 1

uint8 SOLVER_DEFAULT=0   # whatever the node's default_solver parameter says
uint8 SOLVER_ANALYTIC=1  # RESULT_BAD_REQUEST if the analytic backend was not built
uint8 SOLVER_NUMERIC=2
uint8 solver 0

# ---- acceptance thresholds (0.0 means "use the node default") -----------
float64 position_tolerance 0.0     # m,   max accepted ||p_fk - p_target||
float64 orientation_tolerance 0.0  # rad, max accepted angle(R_target^T * R_fk)
float64 joint_limit_margin 0.0     # rad, shrink EVERY joint limit symmetrically by this much;
                                   # must be in [0, joint_limit_margin_max]; lets a caller with a
                                   # stricter controller fence get solutions that fit inside it
```

### 5.3 `franka_ik_interfaces/msg/IkSolution.msg`

```
float64[7] positions          # radians, canonical joint1..joint7 order
float64 redundancy_value      # q7 actually realised (always == positions[6])
float64 position_error        # m,   ||FK(positions).p - target.p||       (always recomputed by FK)
float64 orientation_error     # rad, angle(R_target^T * FK(positions).R)
float64 seed_distance         # rad, max_i |positions[i] - seed_positions[i]|
uint8 BRANCH_NUMERIC=255
uint8 branch                  # analytic branch index 0..3, or BRANCH_NUMERIC
```

`position_error` / `orientation_error` are **always** measured by running the service's own KDL FK
on the returned joint vector. They are never copied from the solver's internal residual. A caller
can therefore trust them as an independent check.

### 5.4 `franka_ik_interfaces/msg/IkResult.msg`

```
uint8 RESULT_SUCCESS=0                  # solutions[] holds >=1 entry (or max_solutions was 0)
uint8 RESULT_BAD_REQUEST=1              # malformed field: non-finite, bad enum, max_solutions>4,
                                        #   degenerate quaternion, out-of-range redundancy_value,
                                        #   joint_limit_margin out of range, unavailable solver
uint8 RESULT_UNKNOWN_ARM=2              # arm_id is not one of the configured chains
uint8 RESULT_UNKNOWN_FRAME=3            # frame_id is neither "" / "<arm_id>_link0" nor the root
uint8 RESULT_UNSUPPORTED_TIP=4          # TIP_HAND_TCP requested but the description has no hand
uint8 RESULT_SEED_OUT_OF_LIMITS=5       # a seed value lies outside the (margin-adjusted) limits
uint8 RESULT_UNREACHABLE=6              # provably outside the arm's geometric workspace
uint8 RESULT_LIMITS_VIOLATED=7          # geometrically reachable, but every branch needs a joint
                                        #   outside its (margin-adjusted) limit
uint8 RESULT_TOLERANCE_NOT_MET=8        # a candidate converged but FK error exceeds tolerance
uint8 RESULT_ITERATION_BUDGET_EXHAUSTED=9  # numeric backend hit maxiter without converging
uint8 RESULT_NO_ACCEPTABLE_SOLUTION=10  # deterministic numeric search returned no FK-verified,
                                        #   in-limit candidate; global infeasibility is not proven
uint8 RESULT_INTERNAL_ERROR=11
uint8 result

string<=256 message                     # short, bounded, human-readable; NEVER machine-parsed
franka_ik_interfaces/IkSolution[<=4] solutions
uint8 solver_used                       # SOLVER_ANALYTIC or SOLVER_NUMERIC (never SOLVER_DEFAULT)
uint16 iterations                       # analytic: branches examined (<=4)
                                        # numeric:  LMA iterations actually run
builtin_interfaces/Duration solve_time  # steady-clock time strictly inside the solver
```

**RESULT_UNREACHABLE vs RESULT_LIMITS_VIOLATED.** The vendored analytic solver returns `NaN` for
both cases, so `franka_ik` classifies them itself with an independent closed-form test on the
wrist centre: reachable iff the shoulder→wrist-centre distance lies in
`[|L24 − L46|, L24 + L46]` (with `L24 = 0.326591870689`, `L46 = 0.392762332715` from §4.2) after
removing the joint-1 rotation and the tip offset. Reachable but no branch survives ⇒
`RESULT_LIMITS_VIOLATED`; test fails ⇒ `RESULT_UNREACHABLE`. **The vendored header is never
patched** — this classifier lives in `franka_ik`'s own code and has its own tests (§8.3). The
numeric backend classifies the same way for consistency, so the code a caller sees does not depend
on which backend ran.

### 5.5 Services

`franka_ik_interfaces/srv/SolveIk.srv`
```
franka_ik_interfaces/IkRequest request
---
franka_ik_interfaces/IkResult result
```

⚠ **SIMPLICITY CUT — there is no `SolveIkBatch`.** The batch service, its `BATCH_*` codes, its
64-entry bound and the `max_batch_size` parameter were all removed. Nothing on the locked ladder
issues a batch: a drag loop makes one solve per frame, and the workspace sweep that would want a
batch belongs to Session D, whose plan does not ask for one. A reachability probe is just
`SolveIk` with `max_solutions: 0`, and a caller that wants many can call in a loop at the measured
≥ 200 Hz. Adding `SolveIkBatch` later is purely additive.

`franka_ik_interfaces/srv/GetChainInfo.srv`
```
---
string urdf_root_frame                        # "base_link" (dual) or "panda_link0" (single)
string<=64 urdf_sha256                        # hex sha256 of the exact URDF the node parsed
franka_ik_interfaces/ChainInfo[<=4] chains
uint8 default_solver                          # SOLVER_ANALYTIC or SOLVER_NUMERIC
bool analytic_backend_available
string<=64 package_version                    # franka_ik's package.xml <version>
```

`franka_ik_interfaces/msg/ChainInfo.msg`
```
string<=64 arm_id
string base_frame                   # <arm_id>_link0
string flange_frame                 # <arm_id>_link8
string hand_tcp_frame               # <arm_id>_hand_tcp, or "" when the description has no hand
string[7] joint_names               # <arm_id>_joint1 .. <arm_id>_joint7, in order
float64[7] position_lower
float64[7] position_upper
float64[7] velocity_limit
geometry_msgs/Transform root_to_base   # the fixed urdf_root -> base_frame transform
```

`GetChainInfo` is what lets `franka_web` render limits, label joints, and verify at start-up that
the IK node and the bringup are talking about the same robot (compare `urdf_sha256`).

### 5.6 Resolved names, node identity, QoS

| Thing | Value |
| --- | --- |
| Executable | `franka_ik_service_node` |
| Default node name | `franka_ik_service` |
| Default namespace | `/` |
| Services | `~/solve_ik` → `/franka_ik_service/solve_ik`; `~/chain_info` → `/franka_ik_service/chain_info` |
| Service QoS | `rclcpp::ServicesQoS()` (default reliable) — do not customise |
| Callback group | one `MutuallyExclusive` group; a `SingleThreadedExecutor`. One operator at a time is a locked product decision; concurrency here would only add nondeterminism. |
| Lifecycle | plain `rclcpp::Node` (not lifecycle). URDF parsing happens in the constructor; a parse failure throws and the process exits non-zero. |

### 5.7 Parameters

| Parameter | Type | Default | Constraint |
| --- | --- | --- | --- |
| `robot_description` | string | *(required)* | URDF XML. Empty ⇒ node exits non-zero with a clear message. |
| `arm_ids` | string[] | `["panda1","panda2"]` | 1–4 entries, each matching `[A-Za-z][A-Za-z0-9_]*`, each present in the URDF as `<id>_link0 … <id>_link8` + 7 revolute joints. Unknown id ⇒ exit non-zero. |
| `default_solver` | string | `"numeric"` | `"analytic"` \| `"numeric"`. `"analytic"` with no analytic backend built ⇒ exit non-zero. |
| `position_tolerance` | double | `1.0e-4` | m, in `(0, 1e-2]` |
| `orientation_tolerance` | double | `1.0e-3` | rad, in `(0, 1e-1]` |
| `numeric_max_iterations` | int | `40` | in `[10, 2000]`; Stage-5 whole-corpus p99/max were 18/20 iterations, retaining 2x headroom |
| `numeric_eps` | double | `1.0e-6` | in `(0, 1e-2]` |
| `joint_limit_margin_max` | double | `0.100` | rad, in `[0, 0.5]` |

All parameters are read once at construction and are **not** dynamically reconfigurable in v1.
Every out-of-range value is a hard startup failure with a message naming the parameter — never a
silent clamp.

### 5.8 Contract invariants (assert these in tests, state them in the README)

1. The service is a **pure function** of `(robot_description, parameters, IkRequest)`. It has no
   internal mutable state between calls beyond scratch buffers.
2. Every returned `positions` vector satisfies the margin-adjusted URDF limits, inclusive.
3. Every returned `positions` vector satisfies `position_error ≤ position_tolerance` **and**
   `orientation_error ≤ orientation_tolerance`, as measured by the service's own FK.
4. `solutions` is ordered by `seed_distance` ascending, ties broken by `branch` ascending.
5. `result == RESULT_SUCCESS` means the backend found at least one valid witness. If
   `max_solutions == 0`, that witness is deliberately not serialized and `solutions` is empty;
   otherwise `solutions` is non-empty. A failed reachability probe never becomes success merely
   because `max_solutions == 0`.
6. Identical requests produce byte-identical responses apart from `solve_time`.
7. The service never blocks on anything but computation: no file I/O, no network, no logging
   inside the solve path above `RCLCPP_DEBUG`.

### 5.9 v1 caveat — exactly one solution, always

⚠ **The numeric-only v1 returns exactly ONE solution.** A `RESULT_SUCCESS` response with
`max_solutions >= 1` always carries a single `IkSolution`, with `branch = BRANCH_NUMERIC` (255) and
`solver_used = SOLVER_NUMERIC`, no matter which of 1..4 was requested. §5.2's *"ordered by
`seed_distance` ascending (best first) — 4 is the analytic branch count"* and invariant 4's
tie-breaking rule describe the ranking of **a future analytic backend**; in v1 they are vacuously
true and the ranking machinery is inert. `max_solutions` in 1..4 therefore behaves exactly like
`max_solutions: 1`; only `max_solutions: 0` (the reachability probe) behaves differently. A caller
must not build a UI around ranked alternatives that v1 will never return.

---

## Measured performance

All figures below were measured in a Release build on the checked-in 2,000-pose
`reachable_random` corpus. The service measurement used sequential localhost shared-memory calls.
The production iteration default is 40; the entire 6,700-witness reachable corpus measured
iteration p50/p95/p99/max of 14/17/18/20, leaving 2x headroom over the observed maximum.

| Path | p50 | p95 | p99 | Max | Sustained rate | Successes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Numeric KDL solve, in process | 53.918 µs | 65.372 µs | 71.238 µs | 81.016 µs | — | 2,000/2,000 |
| `SolveIk`, localhost SHM | 86.324 µs | 99.175 µs | 104.763 µs | 125.437 µs | 11,360.945 calls/s | 2,000/2,000 |

## Measured round-trip accuracy at the shipped defaults

⚠ **Session A must size its Cartesian-drag expectations against this table.** This is what a caller
actually gets at the *shipped node defaults* — `position_tolerance = 1.0e-4 m`,
`orientation_tolerance = 1.0e-3 rad`, `numeric_max_iterations = 40` — with an unmodified request
(both tolerance fields left at `0.0`). Independently measured during post-MVP verification against
a from-scratch forward kinematics that shares no code with this package, 250 poses per noise level:
each target pose is produced by FK from a known joint vector, the seed is that vector plus noise,
and the returned solution is re-run through FK and compared to the target.

| Seed noise | position err p50 | position err max | orientation err p50 | orientation err max |
| ---: | ---: | ---: | ---: | ---: |
| 0.0 rad (exact seed) | 0.0 | **0.0** — bit-exact | 0.0 | **0.0** — bit-exact |
| 0.01 rad | 3.0e-8 m | 5.3e-7 m | 4.9e-5 rad | **9.9e-5 rad** |
| 0.1 rad | 9.8e-8 m | 6.8e-7 m | 2.9e-5 rad | **1.4e-4 rad** |

**In one line: expect ~1e-4 rad of orientation error and ~7e-7 m of position error at perturbed
seeds, and bit-exact reproduction at an exact seed.**

Two consequences a consumer must plan for:

- **Do not size a drag loop against `1e-9`.** `SESSION_B_IK_PLAN.md` §8.2 asks for a round trip
  reproducing the original pose to `1e-9 m / 1e-9 rad`. That bar was authored expecting an analytic
  backend; the numeric-only v1 does not meet it at perturbed seeds and never claimed to. Accordingly
  `numeric_backend_round_trip_test` asserts 1e-9 **only** for the exact-seed case; for perturbed
  seeds it tightens the *request* tolerances to `1e-6` and asserts against those instead.
- **Contract invariant 3 still holds everywhere.** Every measured error is inside the service's own
  default acceptance tolerance — roughly 7x margin in orientation and 140x in position — so a
  returned solution is always FK-verified against the tolerances actually in force. The gap is
  between the plan's aspirational number and reality, not between the contract and the code.

## Measured solver limitations

V1 is KDL-only; no analytic solver was vendored. Fixed-q7 round trips from exact seeds passed
6,700/6,700 reachable witnesses.

**Read the next two sentences with their measurement regime attached.** The success rates below
were measured with the **request tolerances tightened to `1e-6`** — that is, roughly 100x stricter
in position and 1000x stricter in orientation than the shipped defaults — and are therefore a
deliberately pessimistic lower bound, not the behaviour a default caller sees. With deterministic
seed perturbations *under that tightened regime*, the measured success rates were 99.8209% at
0.01 rad (6,688/6,700), 99.88% at 0.1 rad, and 96.36% at 0.5 rad.

At the **shipped defaults** the independently measured success rates are higher: **250/250 at each
of 0.0, 0.01 and 0.1 rad**, plus **3,100/3,100 corpus single poses** and **4,000/4,000 corpus drag
steps** (worst joint step 0.0023 rad against the 0.15 rad bound), all through the live service, with
zero out-of-limit solutions returned.

The larger perturbations are deliberately harsh, and the reduced fixed-q7 numeric solve can fail to
find an acceptable solution even for a pose known to be reachable. Callers should feed each
accepted solution back as the next seed for drag continuity and must handle
`RESULT_NO_ACCEPTABLE_SOLUTION` without treating it as proof of global infeasibility.

## What this does not do

- It does not perform self-collision or environment-collision checking.
- It does not generate, interpolate, time-parameterise, or validate trajectories.
- It does not read joint state, TF, controller state, or robot state; the caller supplies the
  seed and frame.
- It does not publish commands, call controller services, recover hardware, or contact a robot.
- It does not enforce a downstream controller's stricter operator fence unless the caller selects
  a sufficient `joint_limit_margin`.
- It is never a safety mechanism. A returned kinematic solution is not authorization to move and
  does not establish that motion is safe.

For the interface source copied exactly from the `.msg` and `.srv` files, see
[`doc/CONTRACT.md`](doc/CONTRACT.md).
