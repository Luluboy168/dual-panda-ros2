# franka_workspace_model

A non-real-time workspace and collision model for the dual-Panda cell: where the
two arms are, where they are allowed to be, and whether a configuration, a path
or a jog is allowed.

The cell already had a geometric model — it is hard-coded into the robot
description, it drives the forward kinematics the controllers and
`robot_state_publisher` use, and nothing checked a configuration against the
room it stands in. The only position fence was a per-joint box inside the
reviewed controller, with no Cartesian, self-collision or cross-arm awareness at
all. This package is the layer that sits *above* that fence, and never inside
it.

**It may only ever reject.** It never widens, relaxes, overrides or substitutes
for any check the controller performs. See `doc/CONTRACT.md`, "The layering
invariant".

---

## What is in here

| Path | What it is |
| --- | --- |
| `franka_workspace_model/model.py` | The whole public surface: `CellModel`, `Contact`, `CheckResult`, `JogResult`, `AllowedVolume`, `WorkspaceModelError`, `result_to_json`. Imports numpy and pyyaml, and no ROS. |
| `franka_workspace_model/geometry.py` | The four closed-form distance primitives. numpy only; no collision library. |
| `franka_workspace_model/generate_link_geometry.py` | The generator that derives `link_geometry_v1.yaml` from the robot description. |
| `franka_workspace_model/ros/` | ROS-facing adapters. The core does not know they exist. |
| `cell/cell_model_v1.yaml` | **This lab's measured cell.** Hand-written, reviewed like code, installed as the default. |
| `cell/link_geometry_v1.yaml` | Derived from the description, committed, regenerated and byte-compared in CI. |
| `doc/CONTRACT.md` | The contract. Every error message this package emits points here. |
| `doc/cell_frame_top_view.svg` | The cell frame, seen from above, with how to measure it. |
| `test/corpus/` | The validation corpus: hand-derived expectations, with the derivation attached to each. |

## Using it

```python
from franka_workspace_model.model import CellModel, default_cell_model_path

model = CellModel.load(default_cell_model_path(), profile='dual')

q = {'panda1': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
     'panda2': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]}

result = model.check_configuration(q)
if not result.ok:
    worst = result.contacts[0]
    print(worst.arm_id, worst.kind, worst.a, worst.b, worst.distance)

jog = model.check_jog('panda1', q, joint_index=1, delta=0.0349)
if jog.allowed and jog.clamped:
    print('jogged less than you asked for:', jog.limiting.kind)
```

`q` carries **every** arm the model declares, not just the one you are moving: a
jog is only safe relative to where the other arm actually is.

Load once, at session start, and hold the object. It is immutable afterwards and
safe to call from several threads without a lock.

Interlock it against the robot you are actually talking to:

```python
from franka_workspace_model.ros.description_interlock import (
    read_running_description, verify_description)

verify_description(model, read_running_description(node))
```

On a mismatch that raises, and the correct response is to start with jogging
disabled and a visible banner naming both digests — not to check anyway.

## The cell file is the thing you edit

`cell/cell_model_v1.yaml` is the single source of truth for everything about the
cell that is not in the robot description, the SRDF or the joint-limit policy.
It records what was measured, by whom, on what date, to what tolerance, and
against which version of every file it was derived from — and the model refuses
to load if any of those files has changed underneath it.

A different cell needs a different file, not an edit to the checker.

Two things in the shipped file are honestly marked as placeholders rather than
quietly presented as derived: the five margins, and the base-pose tolerances.
`doc/CONTRACT.md`, "Margins", says what would replace them.

## Running the tests

```sh
colcon build --symlink-install --packages-select franka_workspace_model
colcon test --packages-select franka_workspace_model
```

or, for the fast loop, `python3 -m pytest test` from the package directory. The
whole suite is offline: no robot, no running ROS graph, no launch files.

What it proves, beyond the obvious: that the derived geometry regenerates byte
for byte from the current description; that each derived volume really contains
the primitives it replaces, by sampling their surfaces rather than by assertion;
that every corpus verdict matches a number derived by hand or by an independent
kinematic chain; that every malformed cell file is refused with its own distinct
message; that no approved jog target lies outside the controller's own joint
box, over ten thousand fixed-seed random jogs; and that importing the core in a
clean subprocess pulls in no ROS module at all.

One test is skipped on purpose, with its entry condition attached: the
cross-validation against MoveIt, which is not installed here.

## What this does not do

It does not plan, does not solve inverse kinematics, does not repair a
configuration, and does not render anything. It carries no payload or dynamics
model. And it fences server-mediated motion only: an arm driven by a node that
publishes to the controller directly is outside this model's reach, and the
consequences of that are spelled out in `doc/CONTRACT.md`, "What this model does
not fence". Do not let a user interface imply otherwise.
