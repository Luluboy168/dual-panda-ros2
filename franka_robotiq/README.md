# franka_robotiq — Robotiq 2F-85 grippers on the Panda arms

One node per arm, one gripper per node, and nothing global. Each node owns one
USB-RS485 adapter, speaks Modbus RTU to one gripper, and publishes that
gripper's state under its own namespace. There is no "both grippers"
interface — a caller that wants both calls both, exactly as it does for the
arms. The nodes run on their own, with or without the web console.

**Activating a gripper moves its fingers.** Activation runs an
auto-calibration: the fingers travel to both stops. Nobody's hand is between
them when `~/reactivate` is called, and nobody's hand is between them when a
gripper is first powered.

---

## 1. Install

There is one dependency beyond stock ROS 2 Jazzy, `python3-serial`, and rosdep
installs it:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select franka_robotiq
source install/setup.bash
```

Your user needs to be in the `dialout` group to open the adapter; see
`doc/SERIAL_BINDING.md` section 8 if it is not.

---

## 2. Try it with no hardware

The emulator speaks the real protocol over a pty — real Modbus framing, real
CRCs, the real activation state machine, real fault codes, and a simulated
unplug. The whole ROS surface works against it:

```bash
ros2 launch franka_robotiq dual_robotiq.launch.py use_fake:=true
```

Add `fake_object_mm:=30.0` to put a 30 mm object between the fingers, so a
close reports a real grip. `fake_object_mm:=0` is the empty-hand path.

This is how the interface is demonstrated and reviewed, and it is the first
thing to run on the day the hardware arrives — before the hardware can be
blamed for anything.

---

## 3. Every day

One arm:

```bash
ros2 launch franka_robotiq robotiq.launch.py arm_id:=panda1 serial_id:=<panda1's by-id name>
ros2 launch franka_robotiq robotiq.launch.py arm_id:=panda2 serial_id:=<panda2's by-id name>
```

Both at once:

```bash
ros2 launch franka_robotiq dual_robotiq.launch.py \
  panda1_serial_id:=<panda1's by-id name> panda2_serial_id:=<panda2's by-id name>
```

`ls -l /dev/serial/by-id/` prints the names. **Do not use `/dev/ttyUSB0`** —
`doc/SERIAL_BINDING.md` explains why in one paragraph, and it is the paragraph
that keeps one arm's command off the other arm's gripper.

The gripper is not activated when the node starts. Nothing moves until a human
calls `~/reactivate`.

---

## 4. The ROS 2 interface

Everything is namespaced under the node: `/panda1_robotiq/…`,
`/panda2_robotiq/…`.

| Kind | Name | Type | Notes |
|---|---|---|---|
| Action | `~/gripper_action` | `control_msgs/action/GripperCommand` | the primary control surface |
| Service | `~/open` | `std_srvs/srv/Trigger` | go to the configured open width at the configured speed and force |
| Service | `~/close` | `std_srvs/srv/Trigger` | go to the configured close width at the configured speed and force |
| Service | `~/stop` | `std_srvs/srv/Trigger` | fingers hold where they are; cancels any active goal |
| Service | `~/reactivate` | `std_srvs/srv/Trigger` | the activation cycle, and the documented recovery from a major fault |
| Topic (pub) | `~/joint_states` | `sensor_msgs/msg/JointState` | two finger joints, half-width in metres each |
| Topic (pub) | `~/status` | `diagnostic_msgs/msg/DiagnosticStatus` | position, object detection, fault, link health |

There is deliberately **no** `~/homing`, `~/move` or `~/grasp`. The Robotiq
protocol has exactly one motion primitive — go to a requested position — and
three names for one primitive would be three fake knobs. `~/reactivate` exists
because the protocol genuinely requires a distinct activation edge and no other
surface can produce one.

Close a gripper:

```bash
ros2 action send_goal /panda1_robotiq/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
ros2 action send_goal /panda2_robotiq/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
```

Watch one:

```bash
ros2 topic echo /panda1_robotiq/status
ros2 topic echo /panda2_robotiq/status
```

**The goal position is half-width in metres — and so are the feedback and the
result.** `0.0` is fully closed, `0.0425` is fully open. `franka_gripper`
interprets its goal as half-width but reports feedback and result as full
width; this package does not copy that asymmetry, so a goal → result round trip
is consistent here. `doc/UNITS.md` has the full table.

A `max_effort` of `0.0` or less means "use the configured force", which is what
a bare `send_goal` with an unset field asks for.

---

## 5. Configuration

Every key below is a ROS parameter on the node, and the same key under
`grippers.<arm>` in the console's `~/.config/franka_web/config.yaml`. Widths
are in millimetres, speeds in mm/s, forces in newtons — the same units the
whole package uses.

| Key | Default | Range | Why that default |
|---|---|---|---|
| `serial_id` | `""` | a basename under `/dev/serial/by-id/` | required when the gripper is enabled |
| `usb_path` | `""` | a basename under `/dev/serial/by-path/` | the fallback for an adapter with no unique serial; mutually exclusive with `serial_id` |
| `speed_mm_s` | **85.0** | 20–150 | the midpoint of the documented 20–150 mm/s range |
| `force_n` | **74.0** | 20–235 | see below |
| `open_width_mm` | **85.0** | 0–85 | the mechanical stop |
| `close_width_mm` | **0.0** | 0–85, below `open_width_mm` | the other mechanical stop |
| `poll_rate_hz` | **20.0** | 1–100 | far under the protocol's ceiling, far over its spacing floor |
| `auto_activate` | **true** | — | re-activate after a power loss, when no goal is running |
| `motion_timeout_s` | **5.0** | 0.5–30 | 85 mm at the minimum 20 mm/s is 4.25 s, plus margin |
| `activation_timeout_s` | **10.0** | 1–60 | provisional; the manual never states the duration, so bring-up measures it and this key raises it without a code change |
| `reconnect_interval_s` | **2.0** | 0.5–30 | the retry period after the link drops |
| `joint_names` | `[<arm>_robotiq_finger_joint1, …2]` | two distinct names | they do not collide with the Franka Hand's, so both nodes can run |

**Why 74 N and not the midpoint.** The manual splits force into bands: count 0
is the lowest force with re-grasp off; 1–127 is low torque with re-grasp on,
described as the mode for "solid & fragile objects"; 128–255 is high torque.
74 N is count 64 — the middle of the low-torque band. The *numeric* midpoint of
20–235 N is 127.5 N, which maps to count 128, exactly the high-torque boundary.
For a lab picking unknown things, gentle by default and raiseable in the config
is the right way round.

Runtime-settable: `speed_mm_s`, `force_n`, `open_width_mm`, `close_width_mm`,
`auto_activate`. The rest are read at startup, and a set attempt is rejected
with a message saying so. `serial_id` and `usb_path` are **never**
runtime-settable — re-binding a live gripper to a different adapter through a
parameter set is precisely the swap this package exists to prevent.

---

## 6. From the web console

The console shows a gripper row on each arm card when a gripper is configured
and its node is reachable. **The row is convenience; the ROS surface above is
the product.**

The gripper nodes are **standing nodes**: you start them with the launch file
in section 3, and they keep running with no web session at all. The server
**connects to** them, monitors them and offers the row — it never starts,
supervises or stops them.

So when the row says

> No gripper node is running for panda1. Start it with
> "ros2 launch franka_robotiq dual_robotiq.launch.py".

the fix is to start one, and that is the line to run. Stopping a session does
not stop the grippers, and a gripper still holding a workpiece across a server
restart is intended behaviour rather than a leak.

The row is absent — not disabled, absent — when no gripper is configured for
that arm, and in Simulate. Simulate observes; a gripper that exists only inside
the server's own process would be a demonstration of the server, not of the
cell.

---

## 7. When something goes wrong

| Situation | What to do |
|---|---|
| `link: down` and "no gripper at …" | The configured adapter is not present. The message lists the adapters that *are*; put the right name in the configuration. `doc/SERIAL_BINDING.md` section 4. |
| "One adapter cannot drive two grippers" and neither node starts | Both arms are configured with the same adapter name. Give each arm its own. `doc/SERIAL_BINDING.md` section 4. |
| "Refusing to drive panda1's gripper through panda2's adapter" | The by-id symlink resolves to a device whose serial disagrees with the name. Nothing was commanded. Re-run `ls -l /dev/serial/by-id/` and fix the configuration, or the symlink. |
| "the adapter opened but the gripper never answered" | The gripper was reconfigured away from Robotiq's factory serial settings. Set it back with Robotiq's own tool. `doc/SERIAL_BINDING.md` section 7. |
| A major fault, and `~/status` says so | Clear the hand, then call `~/reactivate`. A major fault clears only on an activation edge; that is what the service is for. |
| "Gripper is too hot" | Nothing to do. It is a minor fault and it resumes by itself once it cools. |
| "Gripper is not activated" | Call `~/reactivate`. Fingers move when you do. |
| A goal is rejected as out of range | The action takes **half-width in metres**, so the largest legal value is `0.0425`, not `0.085`. `doc/UNITS.md` section 3. |
| "Another gripper goal is running; cancel it first" | One serial link, one physical motion. Cancel the running goal, or call `~/stop`, then send the new one. Goals are never silently preempted. |
| Permission denied opening the adapter | Your user is not in `dialout`. `doc/SERIAL_BINDING.md` section 8, and then log out and back in. |

---

## 8. Mounting a gripper

`doc/MOUNTING.md` — unbox to working, per arm, with a rollback for every step
and a log table to fill in. Read it before you pick up a gripper, not while you
are holding one.

---

## 9. Notes

* `doc/UNITS.md` — what every number means, which conversions the manufacturer
  documents and which two are interpolations, and the `at_position` trap.
* `doc/SERIAL_BINDING.md` — binding a gripper to its adapter, the three
  refusals word for word, and the one mis-binding no rule catches.
* `doc/CAPSULE.md` — the enclosing collision volume for the workspace model,
  its derivation and its containment proof.
* `udev/99-franka-robotiq.rules` — optional convenience: group access, and
  keeping ModemManager off the link.
