# dual-panda-ros2

Two Franka Emika Panda arms, one browser page.

This is the control stack for a dual-Panda cell: ROS 2 Jazzy on Ubuntu 24.04
with a PREEMPT_RT kernel, driving two real arms whose bases stand 1.000 m apart.
It is not a simulation that might one day meet hardware — the bringup path, the
impedance controller and the console in this repository are the ones the arms in
this lab actually run.

The front door is **`franka_web`**, the operator console. One command starts it,
one page drives it: pick the arms, pick a mode, press Start, and the page walks
the whole bringup by itself and shows every step as it happens. You do not have
to learn the launch files, the controller names or the topic layout to bring both
arms up and move them.

**The physical stop buttons are the only real stop.**

---

## 1. Install (once)

Before any of this you need:

* **Ubuntu 24.04** and **ROS 2 Jazzy**.
* A **PREEMPT_RT kernel** — for the real arms. The fake stack runs on any machine.
* **libfranka 0.9.2**, built somewhere on the machine. Note the path to its
  `build` directory: the next step wants it, and so does the real-time preflight
  the console runs before it powers anything.

Then four steps:

```bash
git clone https://github.com/luluboy168/dual-panda-ros2.git
cd dual-panda-ros2
colcon build --symlink-install \
  --cmake-args -DFranka_DIR=/path/to/libfranka/build \
  --packages-skip multi_mode_controller multi_mode_controller_impl panda_motion_generators
source install/setup.bash
```

That builds 23 packages and is the whole install. **No Node.js, no npm, no
frontend build step, and no environment variables to set** — the console is
plain static files served by the server, fonts included, so it works on a machine
with no internet access.

**About the three skipped packages.** They are part of the inherited multi-mode
controller (see [Lineage](#7-lineage)). Their source uses an Eigen indexing form
that stopped compiling at Eigen 3.4, which is the version Ubuntu 24.04 ships, so
they do not build here. Nothing on the supported path uses them and their
real-robot launch files are deliberately not installed, so leaving them out costs
you nothing. Everything else in the repository builds clean.

One package wants a system library beyond stock Jazzy: `franka_robotiq` needs
`python3-serial` to open its USB-RS485 adapter.

---

## 2. Every day

There is one step nothing can automate, and it comes first: **on the robots
themselves, release the brakes and activate FCI in Desk.** Nothing the console
does replaces it.

Then, from the repository root:

```bash
./start.sh
```

and open **http://localhost:8765**.

That is the whole daily loop. `start.sh` sources this workspace and runs the
server; if you have the workspace sourced somewhere else already,
`ros2 run franka_web franka_web_server` is the same thing. The server prints the
address to open, plus a second lab-network address when the page is reachable
from other machines — so you can drive the cell from a laptop or a tablet
instead of from the machine under the bench.

Configuration is optional and lives in one file,
`~/.config/franka_web/config.yaml`. No file at all means the defaults, silently.
Angles in it are in degrees, and everything on screen is degrees too.

Everything else about the console — the three modes, the startup checklist, the
per-arm source switch, the operator badge, session recordings, and a table of
what to do when something goes wrong — is in
[`franka_web/README.md`](./franka_web/README.md). Read that one; it is short.

---

## 3. Safety

**The physical stop buttons are the only real stop.** Nothing on the page, and
nothing in this repository, is a safety device. The console can refuse to start a
session and it can tear one down cleanly, but when an arm is doing something you
do not want, you hit the button on the wall.

**Compliance is a feature, not a fault.** The arms run a joint impedance
controller, so they give when you push them, they sag a little under load, and
they do not hold a pose to the millimetre. An arm you can move by hand is
behaving exactly as designed. How firmly each joint holds its pose is a per-joint
stiffness dial in the config file — a comfort choice, not a safety one, because
the torque ceilings, which the springs can never exceed, are the safety bound.

**The settling gate.** Before Motion hands an arm back to you the console
measures the resting pose with the controller paused, then watches all seven
joints until they are genuinely still — drift within 2 degrees of that measured
pose, movement under 0.05 degrees across the window, speed under 1 degree per
second, held for a full second — and refuses to activate, with the arm
untouched, if they are not.

**The collision model.** `franka_workspace_model` carries this lab's measured
cell and both arms as capsules and answers whether a configuration, a path or a
jog is allowed; by construction it may only ever reject, never widen or override
a check the controller already makes. It is a library today — the checker and
this cell's measured file ship, and no console or controller path calls it yet.

---

## 4. What is in here

| Package | What it is |
| --- | --- |
| [`franka_web`](./franka_web) | The operator console, and the front door to everything else: one command, one page, both arms. |
| `franka_bringup` | Launch files and runtime configuration. The `launch/operator/` profiles — fake or production, single or dual, state-only or guarded-motion — are the supported entry points; several inherited real-robot launches stay in the tree but are deliberately not installed, so `ros2 launch` cannot reach them. |
| `franka_hardware` | The `ros2_control` hardware interface that talks to the real arms through libfranka at 1 kHz. Its MuJoCo counterpart is present but off by default in this port. |
| `franka_description` | URDF/xacro and meshes for one arm, two arms and the cell. The dual model puts the two bases 1.000 m apart, measured. |
| `franka_example_controllers` | The controllers. The reviewed dual-arm joint impedance, hold and velocity controllers are what the console loads; the rest are examples, and the joint-position one is kept off the real arms. |
| `franka_msgs` | Messages, services and actions specific to the Franka arms. |
| `franka_robot_state_broadcaster` | Publishes `FrankaState` — the per-arm state the console's health view reads. |
| `franka_semantic_components` | The typed state and model accessors a controller uses to read an arm. |
| `franka_gripper` | Action and service interface for the Franka Hand. |
| [`franka_robotiq`](./franka_robotiq) | Per-arm drivers for two Robotiq 2F-85 grippers over Modbus RTU, one node per arm and nothing global. **Hardware arriving** — the driver, a protocol-faithful fake and the mounting runbook are in; the grippers are not on the arms yet. |
| [`franka_workspace_model`](./franka_workspace_model) | The workspace and collision model: this lab's measured cell, both arms as capsules, and the checker that answers whether a configuration, a path or a jog is allowed. A library so far, not yet on any live path. |
| [`franka_ik`](./franka_ik) | A deterministic inverse-kinematics service for one to four Panda chains: a URDF and a request in, repeatable solutions out. It reads no live state, publishes no command and never contacts a robot. |
| `franka_ik_interfaces` | The two services (`SolveIk`, `GetChainInfo`) and messages `franka_ik` exposes. |
| [`franka_ghost`](./franka_ghost) | Assets and scaffolding for an in-browser 3D "ghost" of the arms — a standalone, fake-hardware-only prototype whose browser module is meant to land in the console later. |
| `franka_control2` | A small `controller_manager` node that runs its control loop at real-time priority. |
| `franka_simple_publishers` | Three tiny Python publishers — joint position, joint velocity, Cartesian pose — handy for driving an arm from a terminal. |
| `franka_moveit_config` | MoveIt 2 configuration for the single, dual and simulated arms. Not on the console path. |
| `franka_multi_mode_controller/` | Five inherited packages (`multi_mode_controller`, `multi_mode_controller_impl`, `multi_mode_control_msgs`, `panda_motion_generator_msgs`, `panda_motion_generators`) implementing the upstream multi-mode controller. Three do not compile against Eigen 3.4 and are skipped at build; the real-robot launches are not installed. |
| `garmi_packages/` | Four inherited packages (bringup, controllers, description, MoveIt config) for GARMI, the mobile manipulator with two Panda arms. Simulation only, carried along from upstream unchanged. |
| `mujoco_ros_pkgs/` | A submodule pointer to the MuJoCo ROS fork the inherited simulation needs. Empty unless you ask for it; MuJoCo support is off in this port. |
| `docs/` | The inherited per-package documentation, written for the Humble version. |
| `tools/` | The inherited Docker one-click environment (`tools/setup_env`, then `./run`). Humble-era, and not how the arms in this lab are run. |

---

## 5. Documentation

* [`franka_web/README.md`](./franka_web/README.md) — the console, end to end. Start here.
* [`franka_robotiq/README.md`](./franka_robotiq/README.md) — the grippers, plus mounting, serial binding and units notes under `franka_robotiq/doc/`.
* [`franka_workspace_model/README.md`](./franka_workspace_model/README.md) — the cell model and its contract.
* [`franka_ik/README.md`](./franka_ik/README.md) — the IK service.
* [`docs/main.md`](./docs/main.md) — the inherited framework documentation: `franka_bringup`, `franka_description`, `franka_hardware`, the multi-mode controller and GARMI. Written for Humble, still the best description of the layers underneath.

---

## 6. Known issues

* **The multi-mode controller does not build on Ubuntu 24.04.** Eigen 3.4 rejects
  an indexing form its source relies on. It is skipped at build time (§1) and its
  real-robot launch files are not installed.
* **MuJoCo simulation is off in this port.** The inherited simulation hardware
  interface is behind a build option that defaults to off, and the
  `mujoco_ros_pkgs` submodule is empty by default. The console's Simulate mode
  does not use MuJoCo at all — it runs the whole stack on `ros2_control`'s fake
  hardware, which needs nothing extra.
* **The joint position controller can produce rough motor behaviour.** Inherited
  and still true; torque or velocity is the safer choice. It is not offered on
  the real-arm path.
* **`franka_moveit_config` uses `warehouse_ros_sqlite`**, having moved off the
  deprecated `warehouse_ros_mongo`.
* **The CI workflow is stale** — `.github/workflows/ci.yml` still builds against
  Humble in Docker and does not describe this port.

---

## 7. Lineage

This repository is a fork and a Jazzy port of
[**`multipanda_ros2`**](https://github.com/tenfoldpaper/multipanda_ros2) by
tenfoldpaper — a `ros2_control` framework for the Franka Emika Panda that
implemented most of `franka_ros` on ROS 2 Humble and Ubuntu 22.04, added
multi-arm support, and kept the Panda alive after Franka Emika dropped it from
`franka_ros2`. That work in turn began as
[mcbed's Humble port](https://github.com/mcbed/franka_ros2/tree/humble) of
Franka Emika's own `franka_ros2`. Nearly everything below the console — the
hardware interface, the descriptions, the state broadcaster, the semantic
components, the controllers, GARMI — came from there, and the debt is a real one.

What this fork adds is the dual-Panda cell as it is actually operated: the
`franka_web` console, the operator launch profiles and their gating, the settling
gate, the measured 1.000 m base separation, the workspace and collision model,
the Robotiq grippers, the IK service, and the move to ROS 2 Jazzy on Ubuntu
24.04.

**What changed from upstream, in short.** Upstream targets Humble on 22.04 and
leans on MuJoCo for simulation through
[its fork of `mujoco_ros_pkgs`](https://github.com/tenfoldpaper/mujoco_ros_pkgs)
— which is what the `mujoco_ros_pkgs` submodule here points at, and which you
would need to populate and build to use the inherited sim launches. This port
targets Jazzy on 24.04, builds MuJoCo support off by default, and simulates with
fake hardware instead. Upstream also documents a longer manual install (Eigen
3.3.9 rather than 3.4, MuJoCo 3.2.0 from source, `dq-robotics` for the motion
generators, and library paths exported into your shell); none of that is needed
for the four steps in §1, and the upstream README remains the reference if you
want the MuJoCo or multi-mode paths. Upstream work on FR3 support continues
there, not here.

The inherited simulation, when it is built, looks like this:

<img src="docs/images/single_sim.png" alt="single Panda in MuJoCo" height="200">
<img src="docs/images/dual_sim.png" alt="dual Panda in MuJoCo" height="200">
<img src="docs/images/garmi_sim.png" alt="GARMI in MuJoCo" height="200">

### Citing the framework

If the `multipanda_ros2` framework helps your research, please cite the upstream
paper:

**[Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems](https://arxiv.org/abs/2602.02269)**

```bibtex
@misc{škerlj2026multipanda_ros2,
      title={Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems},
      author={Jon Škerlj and Seongjin Bien and Abdeldjallil Naceri and Sami Haddadin},
      year={2026},
      eprint={2602.02269},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2602.02269},
}
```

---

## License

All packages are licensed under the
[Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html),
following `franka_ros2`. See [`LICENSE`](./LICENSE) and [`NOTICE`](./NOTICE).
