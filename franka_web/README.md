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
5. **Drive an arm** — enable it and jog from the page, or switch that arm's
   source to **External** and publish from your own node. The page shows the
   exact topic name, a copyable message template, and the live incoming rate.
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

**Recordings are capped, and the cap is one number.**
`recordings.max_total_gb` bounds the TOTAL size of everything under the
recordings directory, and defaults to **50** GB (one GB is 1 000 000 000
bytes). The recorder writes about 4 MB a second — roughly 14 GB per hour — so
without a bound a busy week fills the disk. When the total is over the cap,
whole sessions are removed **oldest first**, by the timestamp in the directory
name, until it is back under; the pass runs at startup and again each time a
session's bag is sealed, and writes one plain line per removal to the log
drawer. Two kinds of directory are never removed. The first is the session
being recorded right now and every earlier segment of its chain — including,
at the moment you press Stop, the one just sealed, so a stop never deletes
what it has just recorded; if that session alone is bigger than the cap, the
summary line says the total is still above it. That protection ends with the
session: at the next server start it is a sealed recording like any other,
and the cap applies to it oldest-first. The second is any session directory with no
`metadata.yaml` — a crashed session's bag is evidence, so it is kept,
counted, and named in the summary line. Anything in that directory the server
did not write is left alone entirely. `0` is refused — as is any value so
small it comes to less than one byte, which is the same thing — and the
refusal says to write `max_total_gb: unlimited` if keeping every recording
for ever is what was meant.

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

# recording:
#   enabled: true             # false: run without recording, the page hides the REC chip

# recordings:
#   max_total_gb: 50          # total kept on disk; > 0, or "unlimited"

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

**The per-arm source switch.** In Motion each arm is either **Jog** — the
on-page controls — or **External**, where the page hands you the topic name, a
copyable message template, and the live rate of the messages actually arriving.

**The operator badge.** One operator at a time, on a 15-second lease. The badge
always shows who holds it, and anyone can Take over — which resets every arm's
enable to off, deliberately.

**The next-step hint line.** Always visible, always says what to do next.

**The log drawer.** Docked at the bottom, collapsed to a slim bar with a
warn/error count badge. Expanded it holds the last 500 lines of the launched
stack's output, colored by level. Every fault banner has a **View logs** link
that opens it at the newest line.

**Recordings.** One per session — Simulate included — sealed when you press
Stop, and kept until the total reaches the size cap (`recordings.max_total_gb`,
50 GB by default), at which point the oldest sealed sessions are removed and
the log drawer says which.

---

## 5. When something goes wrong

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
| An old recording is gone | The size cap removed it. The total is held at or under `recordings.max_total_gb` (50 GB by default) by removing whole sessions oldest first; every removal is one line in the log drawer, naming the session and the new total. Raise the number, or set it to `unlimited`, and copy anything you need to keep out of the recordings directory. |
| A Watch or Motion start is refused at preflight, naming libfranka | Set `directories.franka_dir` (§3). Simulate is unaffected. |
| A Watch or Motion session refuses to start on preflight | The host is not real-time-ready. Simulate still runs anywhere; production modes need the PREEMPT_RT kernel and limits the preflight checks. |

---

## 6. Notes

The launched stack's full logs land on disk in `~/.ros/log`, alongside the
per-session server files under the state directory
(`~/.local/state/franka_web/` by default). The drawer answers "what just
happened"; the disk is for digging. Recordings live in
`~/franka_web_recordings/`, one sealed bag per session — every mode records,
Simulate included. They do not grow without bound: `recordings.max_total_gb`
(§3) keeps the total at or under 50 GB by default, removing whole sessions
oldest first and logging each removal.

There is no login, no TLS and no account. Only people on the lab network can
reach the page, and that is deliberate: this is a lab tool for known people,
not a product.
