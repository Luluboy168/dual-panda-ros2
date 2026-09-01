# Mounting a Robotiq 2F-85 — the day's runbook

Unbox to working, per arm, with a rollback for every step. Written for a person
standing at the robot holding a gripper.

Every step has an **expected observation** and a **what to do if it does not do
that**. Work one arm at a time, and finish the whole runbook on panda 1 before
starting panda 2 — except where a step says otherwise.

**Two places in this file deliberately name one arm only**, and neither is an
oversight:

* **Quoted refusal messages** name one arm because the software printed it that
  way. Balancing the arm names would distort a verbatim quote.
* **Section 8's bring-up table**, where step 8.9 says "repeat 8.2–8.8 for
  panda2" rather than reprinting eight rows. Its commands stay **inline in the
  table cells** for that reason — see the note at the table.

Everywhere else, both arms are written out.

---

## 0. Before the day

In the room, on the bench, before anyone touches a robot:

- [ ] Two Robotiq 2F-85 grippers.
- [ ] Two couplings, **part number recorded as they arrived**. The Panda flange
      is DIN ISO 9409-1-A50, 4 × M6; the manual's own ISO 9409-1-50-4-M6
      coupling is `AGC-CPL-062-002` (Fig. 6-6, page 124). The coupling is
      **mandatory** — it carries the gripper's electronics and electrical
      contacts — so a bare adapter plate is not a substitute, at most something
      that goes *between* the flange and a Robotiq coupling.
- [ ] Fasteners, a torque wrench, and calipers.
- [ ] Two USB-RS485 adapters and their 24 V supplies.
- [ ] Cable clamps for the run along each arm.
- [ ] This document, printed or open on a second screen.

On the machine:

- [ ] The workspace is built and `franka_robotiq` is installed.
- [ ] Step 8.1 (the fake, no hardware) has passed **at least once** already.
      Proving the software before the hardware is in the way is what lets you
      blame the right thing later.

---

## 1. Safety

**The physical stop buttons are the only real stop.**

* Brakes engaged and FCI off for all mechanical work. Nothing in this runbook
  is done with a controller active except where a step says so explicitly.
* **Activating a Robotiq gripper is a motion.** The manual's own warning: the
  fingers move on the activation edge, running an auto-calibration. Nobody's
  hand is between the fingers when `~/reactivate` is called, and nobody's hand
  is between them when a gripper is first powered.
* One person at the robot, one person at the keyboard, and they can see each
  other.

---

## 2. Record the "before" state

This is the step that makes rollback possible rather than a guess. Do it first.

Start a **Watch** session, or bring the arm stack up with `ros2 launch`, and
read each arm's current load declaration **before** touching it.

**The topic name depends on how many arms the session brought up, so both are
printed here.** A two-arm session spawns per-arm broadcasters; a single-arm
session spawns the broadcaster under a fixed name that carries no arm id at
all. One command tells you which you have:

```bash
# which shape is this session?
ros2 topic list | grep robot_state

# two-arm session
ros2 topic echo /franka_panda1_robot_state_broadcaster/robot_state --once
ros2 topic echo /franka_panda2_robot_state_broadcaster/robot_state --once

# one-arm session -- the broadcaster name is fixed regardless of arm id
ros2 topic echo /franka_robot_state_broadcaster/robot_state --once
```

Without the `grep` line, an operator on a single-arm session types the two-arm
name, gets silence, and has no way to tell a wrong topic from a dead
broadcaster.

**Copy `m_load`, `f_x_cload` and `i_load` for each arm into the log table
(section 11), and record the pose the arm is in.** Section 9's settling
re-check has to be run at the same pose, and section 10's rollback restores
exactly these values.

*If the topic is silent on both shapes:* the broadcaster is not running.
Fix that before going further; a runbook that starts from an unknown "before"
has no rollback.

---

## 3. Mechanical mount, per arm

Brakes engaged. Flange clean and dry.

1. Offer the coupling to the flange, indexing pin located.
2. **Fit check, before torquing anything.** The coupling's robot-side recess
   must seat on the flange's spigot, flat, with no rock. Fig. 6-6 (page 124)
   gives `AGC-CPL-062-002` as ISO 9409-1-50-4-M6: 50 mm pitch circle diameter,
   4 × M6 clearance holes with counterbores, one M6 indexing pin at 45°, ⌀75
   outside, ⌀63 F8 robot-side recess, 3 mm robot-side step, 13.9 mm total
   thickness.
   **If it does not seat: STOP.** An adapter plate is needed and the mount is
   not a today job. Do not force it, and do not torque a coupling that is
   sitting on its pin.
3. Bolt the coupling, then the gripper, in a diagonal pattern.
   **Screw torque: ________ N·m** — take it from the Franka product manual for
   M6 into the flange and from Robotiq's coupling instructions. This document
   does not invent a torque figure.

Record three things in the log table, in these words:

* **The coupling part number** as it arrived.
* **The measured `t_c`** — the **assembled** height from the flange face to the
  gripper's mounting face, taken with calipers on the bolted stack. It is
  **not** the coupling's catalogue thickness: Fig. 6-6's 13.9 mm is the part,
  and the part seats *into* the gripper's ⌀71h8 recess, so the two numbers are
  not the same quantity. If they happen to agree, that is a measurement result,
  not an expectation that was met.
* **The clocking, in degrees** — the angle between the finger-opening direction
  and the `<arm_id>_link8` frame's `x` axis. Measure it and write down a
  number. **Not "to the nearest 90°".** `link8`'s own collision primitives sit
  at `xyz="0.0424 0.0424 …"`, a 45° diagonal, so the relationship between that
  frame's axes and the flange bolt pattern is not established in this phase,
  and a runbook offering only 0/90/180/270 would invite you to round away the
  one fact section 6 needs.

---

## 4. Electrical

* **24 V DC ±10 %**, absolute maximum **28 V DC**, peak current **1 A**,
  quiescent power **< 1 W** (§6.3, page 144).
* The coupling carries the gripper's electrical contacts, so the gripper is
  powered **through the coupling**, not by a separate lead into the body.
* Adapter to gripper: wire it per the adapter's own kit sheet. This document
  does not restate a wiring diagram it has not read.

*Expected:* on power-up the gripper's LED lights. A solid blue/red LED while
booting is normal.

---

## 5. Cable route — blocking

The cable runs along the arm. A cable that binds at J6 is a gripper problem
that presents as an arm fault, so this is checked before the first session and
the result is a line in the log table.

1. Clamp the cable along the arm, leaving service loops at the wrist.
2. With the arm **free** — brakes released, hand-guided — move slowly through
   the full range of J4 to J7 and confirm:
   * no tension at any pose;
   * no pinch at any joint;
   * no snag into the other arm's workspace;
   * the connector is not taking load at any point.

**Hand-guide it; do not try to jog it.** A Watch session exposes no motion
surface at all — the state-only profiles are declared with motion disallowed
and the console's motion controls are absent rather than merely greyed out — so
"jog it in Watch" is an instruction nobody can follow. Either hand-guide with
the brakes released, which is the intended path here and the one that lets you
*feel* a cable start to load, or run the sweep in a **Motion** session and say
so explicitly in the log.

*If the cable binds:* re-clamp and repeat. Do not proceed to section 6 with a
cable that loads the connector; the symptom later is an intermittent link and
it will be blamed on the adapter.

---

## 6. The load configuration

### The values

| Quantity | Value | Source |
|---|---|---|
| `mass` | **0.900 kg**, plus anything else riding on the flange — weigh it | §6.2.3 table, 2F-85 row (coupling included), page 140; agrees with the product sheet |
| `center_of_mass` | **(0.000, 0.000, 0.057) m** | same row, page 140 |
| `load_inertia` — **the printed default** | **diag(0.003149, 0.003149, 0.000564) kg·m²**, the orientation-independent form | derived from the row below by replacing the smaller lateral entry with the larger one; never below the true value on either lateral axis, whatever the clocking turns out to be |
| `load_inertia` — **the exact matrix, labelled alternative** | **diag(0.002768, 0.003149, 0.000564) kg·m²** | §6.2.3 **Fig. 6-20**, block "2-FINGER 85 OPTION", **page 142**; printed there as `diag(2768, 3149, 564) kg·mm²`, cross-checked against the figure's own `lb·in²` column and against the 2-FINGER 140 block on the same page |

**Why the conservative form is printed first.** The exact matrix is a *figure
read* — a transcription off a drawing. Until a second independent reader has
confirmed it, the runbook's default is the orientation-independent form and the
exact matrix is the labelled alternative. The remaining risk is not the
magnitudes, which two unit cross-checks confirm; it is the **clocking**, which
this phase does not establish. **Record in the log table which form you sent.**

### Four things about that matrix

1. **Column-major does not bite here.** The service's `load_inertia` field is a
   9-element column-major 3×3. Both matrices above are diagonal, so row-major
   and column-major produce the same nine numbers in the same order. That is
   worth knowing you checked rather than got lucky on.
2. **Clocking, in degrees.** `I_xx != I_yy`. The manual's `x` is the
   finger-opening direction (Fig. 6-1). A lateral rotation of the mount
   redistributes the two lateral entries, and because the
   `link8`-frame-to-bolt-pattern relationship is unverified, there is no safe
   90° quantum to round to. Unless section 3's measured angle is 0° give or
   take a few degrees **and** somebody has established what `link8`'s `x` axis
   points at, **send the conservative matrix**.
3. **Datum.** Fig. 6-20 does not state the reference point. `setLoad` expects
   the inertia about the **load's centre of mass**. If the figure is instead
   about the mounting face, then by the parallel-axis theorem
   `I_face = I_com + m·d² > I_com`, so using it as a COM inertia over-states —
   the safe direction. Recorded as a residual uncertainty rather than papered
   over.
4. **The end-effector transform is deliberately not set in this phase.** The
   2F-85's nominal tool centre point is (0.000, 0.000, 0.171) m (§6.2.3,
   page 140), printed here **as a reference value only**. This phase declares
   the *load*, not the end-effector transform: the tool centre point depends on
   the fitted coupling and on the fingertip option, and a wrong transform
   silently moves every Cartesian target. Its absence is a decision.

### Where it is set

```bash
ros2 service call /panda1_param_service_server/set_load franka_msgs/srv/SetLoad \
  "{mass: 0.900, center_of_mass: [0.0, 0.0, 0.057],
    load_inertia: [0.003149, 0.0, 0.0,  0.0, 0.003149, 0.0,  0.0, 0.0, 0.000564]}"
```

```bash
ros2 service call /panda2_param_service_server/set_load franka_msgs/srv/SetLoad \
  "{mass: 0.900, center_of_mass: [0.0, 0.0, 0.057],
    load_inertia: [0.003149, 0.0, 0.0,  0.0, 0.003149, 0.0,  0.0, 0.0, 0.000564]}"
```

Both lines are printed in full on purpose — the same reason section 2 prints
both state topics — and a later editor must not collapse the second into "and
the same line with panda2". To send the exact matrix instead, substitute
`0.002768` for the first `0.003149` and record that you did.

The request is `float64 mass`, `float64[3] center_of_mass`,
`float64[9] load_inertia`, returning `bool success` and `string error`. The
service is created on a node named `<robot_name>_param_service_server` and
reaches libfranka's `setLoad`.

### When it is called

**After the stack is up, and before any impedance controller is activated.**
The concrete way to be in that state is a **Watch** session: its checklist is
preflight → connect → health → baseline and it stops there. The controller and
settling steps exist only in Motion.

**And here is the consequence, because it is why section 9 has a gate.** A
Motion session is "ONE GO" — preflight → connect → health → baseline →
controller → settling — with no operator window between "stack up" and
"impedance controller active". So Watch is not merely *a* place this rule can
be honoured, it is the **only** one, and the load is therefore declared in a
**different session** from the settling re-check that depends on it.

### Verify the write; do not assume it

Immediately re-read the state topic from section 2 — the same one, two-arm or
one-arm name — and confirm `m_load`, `f_x_cload` and `i_load` now hold the
values you just sent. The service's `success` flag alone does not prove the
robot took them. Record **read-back confirmed: y/n** in the log table.

### Persistence is a measurement, not an assumption

Nothing in this repository re-applies the load on connect. Until the day shows
otherwise, treat the load declaration as **per-session**: stop the session,
start a new one, re-read the topic, and record whether the values survived.
That answer goes in the log table, gates section 9, and decides whether a later
phase needs an automatic re-apply.

`franka_robotiq` does **not** declare the load itself — a gripper driver
silently rewriting the arm's dynamics model is a surprise with real
consequences — and this document does not ask anyone to make it.

---

## 7. Serial identity capture

The how is in `franka_robotiq/doc/SERIAL_BINDING.md`. The day's procedure is
short, and the order matters:

1. Plug in **one** adapter — panda 1's.
2. `ls -l /dev/serial/by-id/` and write the new name against panda 1.
3. Plug in the second adapter.
4. `ls -l /dev/serial/by-id/` again and write the new name against panda 2.

One at a time is what makes the attribution certain. Both at once and you are
guessing which name belongs to which arm.

Then write the two names into `~/.config/franka_web/config.yaml` under
`grippers.panda1.serial_id` and `grippers.panda2.serial_id`, or pass them as
`serial_id:=` / `panda1_serial_id:=` / `panda2_serial_id:=` launch arguments on
the standalone path.

---

## 8. Bring-up tests, in order

**The commands in this table stay inline in their cells; they are not fenced
blocks, and that is deliberate.** Step 8.9 collapses the second arm into
"repeat 8.2–8.8 for panda2", so writing 8.2–8.8's commands as fenced blocks
would make them panda1-only in the eyes of the per-arm symmetry check, which
reads fenced command blocks only. If this table ever needs fenced commands, the
exemption has to move with them.

| # | Test | Expected |
|---|---|---|
| 8.1 | Fake first, no hardware: `ros2 launch franka_robotiq dual_robotiq.launch.py use_fake:=true fake_object_mm:=30.0` | both nodes up, `~/status` publishing, `link: up`. Proves the software before the hardware can be blamed. The launch argument places an object in the emulator, so 8.1 goes all the way to a gripped object with no hardware in the room; `fake_object_mm:=0` is the "nothing to grip" path. |
| 8.2 | One real arm: `ros2 launch franka_robotiq robotiq.launch.py arm_id:=panda1 serial_id:=<panda1's by-id name>` then `ros2 topic echo /panda1_robotiq/status --once` | `link: up`, `hardware_id` equal to the configured by-id name, `activated: false` |
| 8.3 | Activate: `ros2 service call /panda1_robotiq/reactivate std_srvs/srv/Trigger` — **the fingers move** | `activated: true`, `fault_code: 0x00`. **Time it** and record the duration; that is what decides whether `activation_timeout_s` stays at 10 s. |
| **8.4a** | **Anti-swap, rule 1 — a name that is not there.** Launch panda1 with a `serial_id` that does not exist (append `-nope` to the real one) | the "no gripper at … / Adapters present now: …" refusal, `link: down`, the node stays up and **nothing is commanded** |
| **8.4b** | **Anti-swap, rule 2 — one adapter, two arms.** `ros2 launch franka_robotiq dual_robotiq.launch.py panda1_serial_id:=X panda2_serial_id:=X` with the same real name for X | the **startup refusal**: "One adapter cannot drive two grippers." Neither node comes up. |
| **8.4c** | **Anti-swap, rule 3 — a symlink that lies.** A bench check: point the `by_id_root` parameter at a temporary directory holding a link named after panda1's adapter but resolving to panda2's device | the serial-mismatch refusal: "the adapter at that path reports serial …, but panda1 is bound to …. Refusing to drive panda1's gripper through panda2's adapter. Nothing was commanded." The port closes and stays down. |
| 8.5 | **Unplug under load**: pull panda1's adapter while `~/status` is echoing | `link: down`, ERROR, `~/joint_states` stops. On replug, one line saying the link is back. |
| 8.6 | Motion: `ros2 action send_goal /panda1_robotiq/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"` | closes; `reached_goal: true` |
| 8.7 | Grip: the same goal with an object between the fingers | `reached_goal: true`, **`stalled: true`**, and `~/status` shows `object: closed_on_object` |
| 8.8 | `ros2 topic echo /panda1_robotiq/joint_states --once` | two joints, **half-width in metres** each, `velocity` and `effort` zero (see `franka_robotiq/doc/UNITS.md`) |
| 8.9 | Repeat 8.2–8.8 for panda2 | symmetric |
| 8.10 | **Both at once**, and watch with your eyes which gripper moves | a panda1 goal moves the gripper on arm 1. **This is the step that covers the one mis-binding no rule refuses** — see the box below. |
| **8.11** | **Record the hardware transcript** | a JSON-lines file at the named path, and a replay run whose only differences are the four divergences named in advance. Details below. |
| **8.12** | **The web gripper row, beside a Watch session** | both rows reach `configured: true` / `available: true` with a node-composed status line; a close and an open drive `busy` true→false and the width down and back; the web end-to-end test's second half runs instead of skipping. Details below. |

> **Mis-configuring one arm with the *other arm's real* by-id name is refused by
> no rule.** Nothing in the software can tell those two names apart — both are
> real adapters, and the serial embedded in the name matches the serial the
> adapter reports. Rule 1 does not fire because the path exists; rule 3 does not
> fire because the two serials agree; rule 2 fires only when both arms are
> configured, which a single-arm launch never is. What catches it is **8.10:
> sending a panda1 goal and watching which gripper moves.** Do 8.10 with your
> eyes on the hardware, not on the terminal.

The parameter 8.4c points at a temporary directory is a **bench check**, and
that cell is the only place in this package's documents where it is named. It
exists so rule 3 can be proven under supervision; it is not an operating knob,
and the package README does not mention it at all.

### 8.11 — the transcript, in full

This is the step that closes the emulator's only falsifiable fidelity check.

1. With one real gripper up and activated, run the scripted session through the
   driver's serial-factory seam with the transcript recorder wrapping it:
   **open → activate → close on an object → inject a fault → reactivate**.
2. Save the JSON-lines file to
   `~/franka_robotiq_transcripts/<YYYY-MM-DD>-panda1.jsonl` (and the matching
   `-panda2.jsonl`), and write that path into the log table.
3. Replay it against the emulator:

   ```bash
   FRANKA_ROBOTIQ_TRANSCRIPT=~/franka_robotiq_transcripts/<date>-panda1.jsonl \
     python3 -m pytest franka_robotiq/test/test_fake.py -q -rs
   ```

   The test skips without that variable; with it, it runs. Record **ran or
   skipped**, and the outcome, in the log table.
4. **The expected divergences are named in advance**, so finding one is neither
   a surprise nor a reason to edit the emulator: the motor-current magnitude;
   the activation duration; motion timing; and every inference in the fault
   model — whether the gripper really latches fault `0x05` / `0x07`, and
   whether auto-release really runs `0x0B` → `0x0F`. The manual documents those
   codes as *meanings* only and never states the latching trigger, so **a
   divergence there is a finding to record, not a red test to fix.** Nobody
   changes the emulator on the strength of one session.
5. Until this step runs, the fidelity gate has only the manual's printed frames
   behind it. That is the honest state, and it is worth saying out loud rather
   than implying more coverage than exists.

### 8.12 — the web gripper row, in full

Deliberately shaped like 8.11, because it is the same problem: an assertion
that could not be made before the cell existed, deferred to the day it can be.

1. **Why it lives here.** Every Watch profile requires real robot addresses,
   and Simulate is given no gripper row at all, so a Watch session is the first
   place the row can render — and a Watch session needs the cell. Nothing was
   weakened; the assertion was deferred.
2. With the standing gripper nodes up — real adapters, or `use_fake:=true` if
   the grippers are not yet mounted — start the web console and open a
   **Watch** session with both arms and both grippers enabled in
   `~/.config/franka_web/config.yaml`.
3. Confirm on the page, and record it: each arm card shows its gripper row;
   `configured: true`; `available: true`; a live width; and a status line that
   is the **node's own sentence**, not one the server composed. Then stop one
   standing node and watch that row go grey and read *"No gripper node is
   running for panda1. Start it with …"* — the sentence that teaches the fix.
   Restart it and watch the row come back.
4. Drive one close and one open, from the page's buttons if the session allows
   it, otherwise from `ros2 action send_goal`, and watch `busy` go true then
   false while the width falls and returns. Watch is observe-only, so the row's
   buttons are disabled; that is expected, and the command comes from outside
   the page.
5. Re-run the web end-to-end test with the real-cell environment variable set,
   so its second half runs instead of skipping:

   ```bash
   python3 -m pytest franka_web/test/e2e_fake_dual_gripper_row_test.py -q -rs
   ```

   Read the `-rs` summary and confirm the second half is no longer in it.
   Record **ran or skipped**, and the outcome, in the log table.
6. A failure here is a **finding about the web layer**, and it stops nothing
   else on the day: the gripper's own ROS surface was already proven by
   8.1–8.10, and the row is secondary by design.

---

## 9. The settling re-check — blocking

Bolting 0.9 kg onto each flange changes the gravity torque at every joint, and
the activation-settling gate measures exactly that.

### 0. THE GATE — prove the load is declared in the session that measures

The load was declared in section 6 in a **Watch** session. This check runs in a
**Motion** session, and nothing in this repository re-applies the load on
connect. So, after the Motion session reaches `running`, and **before recording
any settling number**, re-read the state topic:

```bash
# the same topic as section 2 -- the per-arm name for a two-arm session,
# the fixed franka_robot_state_broadcaster name for a one-arm session
ros2 topic echo /franka_panda1_robot_state_broadcaster/robot_state --once
ros2 topic echo /franka_panda2_robot_state_broadcaster/robot_state --once
```

Compare `m_load`, `f_x_cload` and `i_load` against what section 6 sent.

* **They match** → continue at step 1.
* **They do not match** → **STOP. The settling result is void; do not record a
  number.** Write "load not declared in the measuring session" in the log
  table. The settling re-check **cannot be performed under this stack**, and
  that is escalated as a real gap needing a re-apply-on-connect step in a later
  phase. It is **not** worked around by re-issuing the load declaration
  mid-session, which is outside this contract.

Why this is a gate and not a nicety: a clean settling pass with **no load
declared** proves nothing at all, and is more dangerous than a trip, because it
gets recorded as a pass.

### 1–5

1. Run a **Motion** session per arm with the grippers mounted, **at the pose
   recorded in section 2**.
2. The baseline is captured in-session, automatically, during startup — so a
   fresh session already carries a fresh baseline and there is no manual
   invalidation step to perform. **The same-pose condition still binds**:
   settling drift is pose-dependent, so a fresh baseline at a different pose is
   not a comparison. No ceremony, and no pose drift.
3. Record both arms' settling outcome in the log table. Panda 2 keeps its
   stiffened joint 2; the mounted load interacts with that asymmetry, so panda 2
   is *expected* to behave differently from panda 1 and a difference is not by
   itself a fault.
4. If settling trips, the message already names the joint, the measured value
   and the config key. **Raising the settling drift limit is the user's logged
   decision, never a builder's default.**
5. Do all of this **before** enabling the workspace model's end effector. Two
   changes at once and neither is attributable.

---

## 10. Rollback

The gripper nodes are **standing nodes**: they are launched by
`franka_robotiq`'s own launch file, they run with no web session at all, they
are never a session-group member and the web server never owns their lifetime.
The server connects to them, monitors them and offers the row. So "stop the
session" is not a rollback for the gripper nodes, and a gripper still holding a
workpiece across a web-server restart is intended behaviour, not a leak.

Reverse the order of the day. Each step stands on its own.

| Undo | How |
|---|---|
| The workspace model's end effector | set `end_effector.present: false` — the switch, one word |
| The web console's gripper row | delete the `grippers:` section from `~/.config/franka_web/config.yaml`, or set both arms' `enabled` to false. A missing section is grippers-off, silently. |
| The gripper nodes | **stop the launch you started them with** (Ctrl-C in that terminal). Stopping a web session does not stop them, and starting one does not start them. Nothing persists once they are down. |
| The load declaration | re-issue the load declaration with the **section 2** values for that arm, then re-read the state topic to confirm |
| The adapters | unplug them. The node reports `link: down` and retries; that is the correct behaviour, not an error to chase. |
| The gripper itself | brakes engaged, power off, unbolt. Record the removal in the log table. |
| The udev rule | `sudo rm /etc/udev/rules.d/99-franka-robotiq.rules && sudo udevadm control --reload-rules` |

---

## 11. The mounting-day log

Fill this in as you go, not afterwards. It is the provenance record for every
number this phase could not source from a document.

**Date: ____________  Operator: ____________**

| Field | panda1 | panda2 |
|---|---|---|
| Coupling part number, as it arrived | | |
| Measured `t_c` — assembled stack height, mm (section 3) | | |
| Screw torque used, N·m (section 3) | | |
| Clocking, **degrees** (section 3) | | |
| Weighed total end-effector mass, kg (section 6) | | |
| Inertia matrix sent — **exact** or **conservative** (section 6) | | |
| Read-back after the load declaration confirmed — y/n (section 6) | | |
| Load survived the Watch → Motion teardown — y/n (section 6) | | |
| Adapter by-id name (section 7) | | |
| Activation duration, seconds (8.3) | | |
| Cable-sweep result, and hand-guided or Motion (section 5) | | |
| Section 9 rule-0 gate — load present in the measuring session, y/n | | |
| Settling result (section 9) | | |
| Pose the settling was measured at (sections 2 and 9) | | |
| `r_cbl` — cable standoff, mm | | |
| Transcript path (8.11) | | |
| Transcript replay — ran or skipped, and divergences seen (8.11) | | |
| Web gripper row reached `available: true` — y/n (8.12) | | |
| Web end-to-end second half — ran or skipped (8.12) | | |
| Notes | | |

The depth of the gripper body is **not** a column here. It closed at the desk,
at 75 mm, with its page and figure — see `franka_robotiq/doc/CAPSULE.md`. A
blank column for a value that is already known is an invitation to re-measure
it badly.

---

## 12. The ordering, restated

Non-negotiable, and this is the last line of the runbook for a reason:

**mount → declare the load → verify the load is declared in the measuring
session (section 9's gate) → settling re-check → measure the coupling stack
height and the cable standoff → complete the capsule
(`franka_robotiq/doc/CAPSULE.md`) → flip `end_effector.present: true`.**

Flipping `present` before the settling re-check means an arm whose dynamics
nobody validated is being checked against geometry nobody measured.
