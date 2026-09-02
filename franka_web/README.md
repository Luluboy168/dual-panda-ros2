# franka_web — the operator console

`franka_web` is the operator console for the two Panda arms in this workspace:
one command starts it, one page drives it. Pick a mode and the page does the
rest — **Simulate** runs the whole console on a fake stack with no
hardware at all, **Watch** brings the real arms up and observes them with the
arm left free, and **Motion** brings them up ready to move.

**The physical stop buttons are the only real stop.**

---

## 1. Install (once)

1. Ubuntu 24.04, ROS 2 Jazzy, and libfranka 0.9.2 built somewhere on the machine.
2. `git clone https://github.com/luluboy168/dual-panda-ros2.git`
3. `colcon build --symlink-install --packages-skip multi_mode_controller multi_mode_controller_impl panda_motion_generators --cmake-args -DFranka_DIR=/path/to/libfranka/build`
   (the three skipped packages are inherited controllers that do not compile against
   Ubuntu 24.04's Eigen; nothing this console uses needs them — see the root README)
4. `source install/setup.bash`

Note the `-DFranka_DIR` path you used — on this machine the real-time preflight
may need it again (see §3).

No Node.js, no npm, no frontend build step, and no environment variables — the
console is plain static files served by the server, fonts included, so it works
on a machine with no internet access.

---

## 2. Every day

1. **On the robots** — release the brakes and activate FCI in Desk. This is
   physical and cannot be automated; nothing the console does replaces it.
2. **Start the server** — run `./start.sh` from the repository root, or
   `ros2 run franka_web franka_web_server` from any sourced workspace. It
   prints `open http://localhost:8765`, and, when the page is reachable from
   other machines, a second line with the lab-network address.
3. **Open the page** — on this machine, or from a laptop or tablet on the lab
   network using that second address.
4. **Pick arms and a mode, press Start** — panda 1, panda 2 or both. The page
   then walks the startup checklist by itself and shows every step as it
   happens: preflight, connect, health check, stack ready, controller
   paused, baseline captured, controller active, settling check. Motion pauses the
   impedance controller for a moment to measure where the arms are resting,
   then hands them straight back; the arms hold position and are briefly
   movable by hand while it does, and nothing is commanded. The pause lasts
   about a second: the driver needs a moment between two controller switches,
   and rushing it makes its control loop miss cycles and stop the arms.
5. **Drive an arm** — enable it and jog from the page, switch that arm's
   source to **External** and publish from your own node, or switch it to
   **Ghost** and apply a pose you drew in the 3D scene (§5). The External panel
   shows the exact topic name, a copyable message template, and the live
   incoming rate.
   Commands must keep arriving at 10 Hz or more; if they stop, the arm freezes
   within 0.1 s. That watchdog is the point of the rate display.
6. **Press Stop** — the whole stack is torn down cleanly and the session's
   recording is sealed into `~/franka_web_recordings/`.

---

## 3. Configuration (optional)

There is one optional file: `~/.config/franka_web/config.yaml` (if
`$XDG_CONFIG_HOME` is set, it wins). No file at all means the defaults below,
silently — no warning, nothing created, nothing to answer.

**Real robots need `directories.franka_dir` when libfranka is not
discoverable.** Watch and Motion run the real-time preflight before anything is
powered, and the preflight has to identify libfranka. On a machine where it
cannot — the usual case is libfranka built outside this workspace — every Watch
or Motion start is refused, cleanly and with the robots untouched, and the
message names the failing check. Fix it once:

```yaml
# ~/.config/franka_web/config.yaml
directories:
  franka_dir: "/path/to/libfranka/build"
```

the same path you passed to `colcon build --cmake-args -DFranka_DIR=…`.
Simulate needs nothing.

Angles in the file are in **degrees**, and everything on screen is degrees too.

Check a file without starting anything:

```bash
ros2 run franka_web franka_web_server --check-config
```

Exit 0 means the file is valid. Exit 2 prints one line saying which key, what
was found, and what is allowed. An invalid file stops the server rather than
half-applying it, so fix the line the message names and start again.

**The controller's watchdog timing is fixed and is not a key.** The impedance
controller's watchdog timing — a 0.1 s hold, a 1.0 s maximum message age and a
0.1 s future tolerance — is fixed by the controller's reviewed timing policy.
It is not settable from this file; writing `watchdog_timeout_s` under a profile
is an unknown-key error. The values in force are shown read-only by
`GET /api/config`.

**Torque ceilings are editable, but the default is the proven set.**
`profiles.<arm>.torque_limit_nm` may be lowered, or raised within the Panda
hardware ceiling. When a loaded profile differs from the proven values the
server logs one WARN line at startup saying so.

A commented example — every line is a comment, so pasting the whole block
changes nothing:

```yaml
# ~/.config/franka_web/config.yaml
# Every key is optional. Deleting this file restores the defaults shown here.
# Angles are in DEGREES; everything else is SI.

# port: 8765                  # 1024..65535
# bind: "0.0.0.0"             # "127.0.0.1" keeps the page on this machine only
# ros_domain_id: null         # null: use $ROS_DOMAIN_ID if it is usable, else 0

# robots:
#   panda1:
#     ip: "172.16.0.2"        # Franka's factory default for the first arm
#   panda2:
#     ip: "172.16.0.3"

# directories:
#   state: "~/.local/state/franka_web"
#   recordings: "~/franka_web_recordings"
#   franka_dir: null          # libfranka build directory for the real-time preflight
#   cell_model: null          # the workspace model's cell file, when it lives
#                             # outside the install space (see the 3D scene)

# recording:
#   enabled: true             # false: run without recording, the page hides the REC chip

# jog:
#   step_deg: 2.0             # one press of - / + , 0 < x <= 15

# settling:
#   drift_limit_deg: 2.0      # a single number, or a list of 7 (one per joint)
#   stable_window_s: 1.0
#   min_samples: 6
#   timeout_s: 5.0

# profiles:
#   panda2:
#     stiffness: [20, 60, 20, 20, 10, 10, 60]      # panda 2 ships with a stiffer joint 2
#     damping: [1.0, 2.0, 1.0, 1.0, 0.5, 0.5, 1.0]
```

Every key, with its default and its unit, is in the installed example:
`$(ros2 pkg prefix franka_web)/share/franka_web/config/config.example.yaml`.

---

## 4. What the page gives you

**Arms and modes.** A picker for panda 1, panda 2 or both, and the three modes.
Simulate needs no hardware; Watch and Motion need a real-time-ready host.

**The startup checklist.** Six steps, shown as they run and written to the log:
preflight → connect → health → baseline → controller → settling. If one fails,
the page says which and why instead of leaving you at a spinner.

**The per-arm source switch.** In Motion each arm takes its commands from one
of three places: **Jog** — the on-page controls; **External**, where the page
hands you the topic name, a copyable message template, and the live rate of the
messages actually arriving; or **Ghost**, where the arm travels to the pose you
drew in the 3D scene (§5).

**The operator badge.** One operator at a time, on a 15-second lease. The badge
always shows who holds it, and anyone can Take over — which resets every arm's
enable to off, deliberately.

**The next-step hint line.** Always visible, always says what to do next.

**The log drawer.** Docked at the bottom, collapsed to a slim bar with a
warn/error count badge. Expanded it holds the last 500 lines of the launched
stack's output, colored by level. Every fault banner has a **View logs** link
that opens it at the newest line.

**Recordings.** One per session — Simulate included — sealed when you press
Stop.

---

## 5. The 3D scene

A panel beside the arm cards draws both arms live, at their measured poses,
and the measured cell they stand in: the table surface and the box the arms
are meant to stay inside. It is available in every mode, and with no session
at all — an idle console shows the empty cell.

The scene is drawn from files the build generates: the same robot
description the robot runs, expanded once and converted into a browser mesh
format. The first page load fetches about 9.5 MB of that and then caches it
forever; every load after it fetches a few tens of kilobytes. Nothing is
downloaded from the internet, at build time or at run time.

### Authoring a pose

Show an arm's ghost and drag its hand. Each drag asks the IK service for the
joint angles that reach the point you dragged to, and asks the workspace
model whether that pose is allowed. The ghost is a scratchpad: it lives in
your browser tab, it is never sent anywhere, and closing the page loses it.

The IK service is a standing node, started separately and running
independently of any session:

```bash
ros2 launch franka_ik franka_ik.launch.py
```

Without it the scene still draws both arms; the ghost controls are disabled
and the panel says the one line above.

**Copy** is the product. It puts the seven joint angles on your clipboard —
in degrees for reading, in radians for your code — inside a ready-to-paste
`JointTrajectory` message, addressed at the topic your session's arm
selection is using. Your own node is the consumer; this console never sends
it anywhere.

One rule the snippet states and it is worth repeating here: the impedance
controller ignores a target whose header stamp is zero, more than a second
old, or more than 0.1 s in the future, and it wants an empty `frame_id`.
Stamp each message with the time you send it.

**About the check.** This check looks at the pose you drew. It does not
watch or limit anything the robot is doing. A pose the console calls clear
is a pose that is allowed to exist, not a promise about a motion to it.

### Applying a pose — drag and go

Drag the ghost where you want it, switch that arm's source to **Ghost** on its
card, and press **Apply**. The arm travels there, slowly, along a path that was
checked before the first message was sent.

Six things worth knowing, because they are what makes it safe rather than
merely convenient:

* **The straight line is in JOINT space, not in the air.** The seven joints
  interpolate together; the hand's path is whatever that produces. The card
  says so, because an operator who expects a straight line and gets an arc
  will not trust the console again.
* **The whole line is checked, not its two ends.** The check is handed three
  poses — where the arm is now, where it is currently commanded, and where you
  want it — and the cell model resamples between them. A refusal says how far
  along the way the trouble is.
* **Nothing new commands the robot.** A travel changes the value of the same
  held target the jog buttons change, and it leaves through the same 20 Hz
  publisher under the same guards: enable, operator lock, watchdog, torque
  ceilings.
* **Cancel is the fastest stop this console has.** It never queues behind
  anything, it is never greyed out while a travel runs, and it holds the arm
  at the last checked point on the line. The physical stop buttons are still
  the only real stop.
* **The travel stops itself** if the other arm moves more than a degree from
  the pose the check was given, or if this arm falls more than twelve degrees
  behind its commanded pose — something in its way, most likely. Both say so
  in words.
* **No cell model, no Apply.** The ghost still draws and still edits without
  the workspace model, because a ghost commands nothing. Apply refuses, in the
  checker's own words, because Apply commands everything.

One travel at a time, session-wide: two independently timed checked paths do
not compose, so the console refuses the second rather than pretending they do.

If the workspace model is not installed the scene still draws the arms and
the ghost is still editable — the panel says, in one sentence, that poses are
not being collision-checked and the cell is not drawn. Its cell file is
found automatically where that package installs it; a cell file kept
somewhere else is named by `directories.cell_model` (§3).

---

## 6. When something goes wrong

| Situation | What to do |
|---|---|
| `start.sh` says the workspace is not built | Run the `colcon build` line from step 3, or source the workspace where it is already built and run `ros2 run franka_web franka_web_server`. |
| The page does not open from another machine | The server prints a second "or from the lab network" line with the address to use; both machines must be on the lab network, and `bind` must not be set to `127.0.0.1`. |
| "an external stop button is pressed" | Release the stop button on the robot, then press Recover. |
| "stopped itself: a protective limit was reached" | Check nothing is obstructing the arm, then press Recover. |
| "communication with the robot failed" | Check that nobody pressed a stop, press Recover; if it fails again, check the robot's Desk page and that FCI is still active. |
| "Your control expired" | Press Reclaim, then Recover. |
| "The session stopped and cannot continue" | Press Stop, start a new session, and open the log drawer to see what failed. |
| The server exits 2 at startup with a config message | The line names the key, what was found and what is allowed. Fix that line, or delete the file to fall back to defaults. |
| A session refuses to record and quotes a permissions message | The recorder checks its own directory and its sentence is shown verbatim; run the `chmod 700 <path>` line the server prints next to it. The state and recording directories are created at mode 0700 when they are missing, so this only happens to a directory that already existed with wider permissions. |
| A Watch or Motion start is refused at preflight, naming libfranka | Set `directories.franka_dir` (§3). Simulate is unaffected. |
| A Watch or Motion session refuses to start on preflight | The host is not real-time-ready. Simulate still runs anywhere; production modes need the PREEMPT_RT kernel and limits the preflight checks. |

---

## 7. Notes

The launched stack's full logs land on disk in `~/.ros/log`, alongside the
per-session server files under the state directory
(`~/.local/state/franka_web/` by default). The drawer answers "what just
happened"; the disk is for digging. Recordings live in
`~/franka_web_recordings/`, one sealed bag per session — every mode records,
Simulate included.

There is no login, no TLS and no account. Only people on the lab network can
reach the page, and that is deliberate: this is a lab tool for known people,
not a product.
