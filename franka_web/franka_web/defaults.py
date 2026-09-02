# Copyright 2026 The multipanda_ros2 Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The single home of every baked-in franka_web number, in SI units.

This module is pure data: no imports, no functions, no classes, no I/O. It is
importable in a bare ``python3`` with no ROS environment. Every value here is
either a reviewed fact of the stack the server talks to (the Panda factory
joint-limit policy, the impedance controller's reviewed timing) or a
live-proven operating value, and the provenance is stated in prose next to it.

The config file speaks DEGREES; this module stores SI (radians, seconds,
newton-metres). A key absent from the config file uses the SI value here
bit-exactly -- no degree round-trip is ever performed on a default.
"""

# --- identity (contract section 4.4; the ONE home for these three names, and
# --- for MOTION_CONTROLLER in the next block -- four in all) -----------------

SERVER_NAME = 'franka_web'
SERVER_VERSION = '2.0.0'
# Version 5 adds the always-present `arms.<id>.motion.apply` block and the
# third value `"ghost"` of `arms.<id>.motion.source`, so a consumer written
# against version 4 is missing a required key AND would render a source it has
# never heard of. (Version 4 added the always-present `arms.<id>.gripper`
# block, for the same kind of reason.)
SCHEMA_VERSION = 5

# --- the one motion controller the web surface offers ------------------------

MOTION_CONTROLLER = 'dual_arm_joint_impedance_controller'

# --- arms and joints ---------------------------------------------------------

JOINT_COUNT = 7
ARM_IDS = ('panda1', 'panda2')

# --- network binding ---------------------------------------------------------

DEFAULT_BIND = '0.0.0.0'
DEFAULT_PORT = 8765
PORT_MINIMUM = 1024   # below this is privileged; the server never runs as root
PORT_MAXIMUM = 65535

# --- default paths (expanded at load time, never at import time) -------------

DEFAULT_STATE_DIR = '~/.local/state/franka_web'
DEFAULT_RECORDING_ROOT = '~/franka_web_recordings'

# --- per-arm robot addresses (Franka factory defaults; not secrets) ----------

DEFAULT_ROBOT_IPS = {'panda1': '172.16.0.2', 'panda2': '172.16.0.3'}

# --- recording ---------------------------------------------------------------

DEFAULT_RECORDING_ENABLED = True

# --- the Panda factory joint-limit policy ------------------------------------
#
# Copied from the reviewed Panda joint-limits policy (schema_version 1,
# libfranka 0.9.2): position_lower, position_upper, effort_ceiling and
# urdf_velocity_ceiling, plus the reviewed timing of the impedance controller.

POLICY_POSITION_LOWER_RAD = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
POLICY_POSITION_UPPER_RAD = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
POLICY_EFFORT_CEILING_NM = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
POLICY_VELOCITY_CEILING_RAD_S = (2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)

# The impedance controller's reviewed timing policy. These are NOT config keys
# (ledger D4): the reviewed controller-config validator requires exact equality
# with these values, so a key that only ever accepts one value would be a fake
# knob. They are carried on MotionProfile for the renderer and are reported
# read-only in GET /api/config.
REVIEWED_TIMING_S = {'watchdog_timeout': 0.1, 'max_header_age': 1.0,
                     'future_tolerance': 0.1}

# --- motion profiles ---------------------------------------------------------
#
# panda1 -- the standard profile: the user-approved Phase 10 jog set, flown
# live on both robots. Joint-7 K=60/D=1.0 was approved after the K=5 jog
# stalled in joint friction; the torque ceilings are the safety bound and were
# never raised with the stiffness.

STANDARD_PROFILE = {
    'k_gains': (20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 60.0),
    'd_gains': (1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 1.0),
    'max_effort_nm': (10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0),
    'max_target_velocity_rad_s': (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
    'watchdog_timeout_s': 0.1,
    'max_header_age_s': 1.0,
    'future_tolerance_s': 0.1,
}

# panda2 -- identical to the standard profile except joint 2 (K 20->60,
# D 1.0->2.0), adopted after the live activation settle on panda 2 was
# dominated by a J2 residual; predicted settle ~1.5 deg at the observed
# ~1.6 N.m residual. TORQUE CEILINGS UNCHANGED.

STIFF_J2_PROFILE = {
    'k_gains': (20.0, 60.0, 20.0, 20.0, 10.0, 10.0, 60.0),
    'd_gains': (1.0, 2.0, 1.0, 1.0, 0.5, 0.5, 1.0),
    'max_effort_nm': (10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0),
    'max_target_velocity_rad_s': (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
    'watchdog_timeout_s': 0.1,
    'max_header_age_s': 1.0,
    'future_tolerance_s': 0.1,
}

DEFAULT_PROFILES = {'panda1': STANDARD_PROFILE, 'panda2': STIFF_J2_PROFILE}

# --- activation settling defaults --------------------------------------------
#
# DO NOT "FIX" THE TRUNCATED LITERALS. span_limit_rad, velocity_limit_rad_s and
# fence_margin_rad are the live-proven policy numbers and are deliberately not
# math.radians(0.05) / math.radians(1.0) / math.radians(5.0) to full double
# precision. They are normative and must stay character for character; an
# "obvious cleanup" here silently changes a live-proven policy and its digest.

DEFAULT_SETTLING = {
    'drift_limit_rad': (0.03490658503988659,) * 7,   # 2.0 deg  (spec section 3)
    'span_limit_rad': (0.000872664626,) * 7,         # 0.05 deg (live-proven)
    'velocity_limit_rad_s': (0.01745329252,) * 7,    # 1.0 deg/s (live-proven)
    'fence_margin_rad': (0.0872664626,) * 7,         # 5.0 deg  (live-proven)
    'stable_window_s': 1.0,                          # live-proven
    'min_samples': 6,                                # live-proven
    'timeout_s': 5.0,                                # live-proven
}

# --- the jog / motion contract (pinned to the impedance controller) ----------

JOG_STEP_RAD = 0.03490658503988659   # 2 degrees, the one fixed step of the UI
JOG_STREAM_HZ = 20.0                 # 2x the 10 Hz floor of the 0.1 s watchdog

# --- apply (executing a ghost pose on the real arm) --------------------------
#
# An Apply is one bounded joint-space travel, streamed through the same 20 Hz
# producer as the jog, after CellModel.check_path approved the whole line.
# Nothing here is a config key: every value is a policy the G3 plan argues for,
# and a key that only ever takes one value would be a fake knob.

# Fraction of the profile's own max_target_velocity_rad_s the travel uses. The
# 20% headroom is not decoration: at 1.0 a single late tick would make the
# controller's per-joint slew limiter clamp some joints and not others, and the
# executed path would leave the line check_path approved.
APPLY_SPEED_FRACTION = 0.8

# A travel longer than this is refused. Two minutes of continuous motion from
# one button press is a program, not a pose.
APPLY_MAX_DURATION_S = 120.0

# Total joint-space excursion, summed over joints and over BOTH checked
# segments (measured -> held, then held -> goal), above which a travel is
# refused. It bounds check_path's resampled sample count (~400 at the cell
# model's proposed policy.max_joint_step_rad of 0.0175 rad) and therefore the
# one supervisor-thread stall an Apply can cause.
APPLY_MAX_PATH_RAD = 7.0

# The floor on the interval between two advances of a travel. A stalled ROS
# executor delivers ticks in a burst when it catches up, and a burst is the one
# schedule that would let the commanded waypoint run further ahead of the
# controller's ramp than one step -- which is exactly the width of the tube the
# executed path is proved to stay inside. 90% of the nominal 1/JOG_STREAM_HZ,
# so ordinary jitter never drops a step. A floor can only make a travel longer.
APPLY_MIN_ADVANCE_PERIOD_S = 0.045

# Below this the ghost is where the arm already is; there is nothing to apply.
APPLY_MIN_TRAVEL_RAD = 0.0087266462          # 0.5 deg

# The held target and the measured pose must agree this closely, per joint,
# before a travel is planned. This is a STALENESS gate, not a geometric one:
# the measured pose is itself the first waypoint of the check, so the gap is
# checked rather than assumed. What this refuses is a held target that has
# stopped describing the arm -- pushed by hand, still finishing a jog, fighting
# an obstruction. One jog step, so the operator already knows how big it is.
APPLY_START_ALIGN_RAD = 0.0349065850         # 2.0 deg

# How far the OTHER arm may drift from the pose check_path was given before the
# travel stops. One resampling step of the checking policy: the smallest
# displacement the swept check could not have been blind to. The web server
# cannot read policy.max_joint_step_rad from the model's public surface today,
# so this is a documented duplicate of that value's proposal.
APPLY_CO_ARM_DRIFT_RAD = 0.0174532925        # 1.0 deg

# How far the arm may lag its own commanded target before the travel stops. The
# torque ceilings are the real bound; this is an earlier one that can explain
# itself. 2.6x the worst steady-state impedance residual this stack has
# recorded (~4.6 deg at K=20 under the observed 1.6 N.m).
APPLY_LAG_LIMIT_RAD = 0.2094395102           # 12.0 deg

# The budget one whole plan_travel -- including check_path over the longest
# legal path -- may spend on the supervisor thread. Asserted as a budget, not
# measured as a performance figure: its purpose is that the supervisor stall an
# Apply can cause is a known quantity rather than a surprise.
APPLY_CHECK_BUDGET_S = 0.25

# --- state fan-out -----------------------------------------------------------

STATE_FRAME_HZ = 5.0              # SSE `state` event cadence
SSE_PING_INTERVAL_S = 10.0        # SSE `ping` event cadence

# The Broker constructor's BARE default. The production server constructs its
# broker with queue_depth=64 (server.py, PART2). Nobody edits this constant to
# fix a log-eviction problem.
SSE_QUEUE_DEPTH = 4

# --- operator lock -----------------------------------------------------------

OPERATOR_LOCK_TTL_S = 15.0
OPERATOR_HEARTBEAT_INTERVAL_S = 5.0

# --- supervisor timing -------------------------------------------------------

SUPERVISOR_TICK_S = 0.1
PREFLIGHT_TIMEOUT_S = 30.0
STARTING_TIMEOUT_S = 60.0
# An independent post-readiness budget. A Motion session may spend up to
# STARTING_TIMEOUT_S reaching a ready graph and then enter this separately
# bounded, command-closed activation observation state.
ACTIVATION_SETTLING_MAX_TIMEOUT_S = 60.0
SERVICE_CALL_TIMEOUT_S = 5.0

# --- freshness and fault thresholds ------------------------------------------

ENABLE_JOINT_STATE_MAX_AGE_S = 0.2   # enable refuses on older samples
JOINT_STATE_STALE_FAULT_S = 1.0      # fault rule F6
CCSR_FAULT_THRESHOLD = 0.95          # fault rule F4 (Phase 10 stop procedure)
CCSR_FAULT_SUSTAIN_S = 5.0

# --- recording ---------------------------------------------------------------

RECORDING_SEGMENT_DURATION_S = 3600  # franka_record's hard --duration cap

# --- child stop escalation (same ladder as franka_bringup's recorder) --------

STOP_SIGINT_WAIT_S = 10.0
STOP_SIGTERM_WAIT_S = 5.0
STOP_SIGKILL_WAIT_S = 5.0

# The recorder child gets a longer first stage: after SIGINT, franka_record
# legitimately runs its own bounded ladder against `ros2 bag record` (up to
# ~25 s worst case). Escalating past SIGTERM kills it mid-seal and produces
# the unsealed, reindex-required bag it exists to prevent.

RECORDER_STOP_SIGINT_WAIT_S = 30.0
RECORDER_STOP_SIGTERM_WAIT_S = 10.0
RECORDER_STOP_SIGKILL_WAIT_S = 5.0

# --- ROS domain --------------------------------------------------------------

ROS_DOMAIN_ID_MAXIMUM = 232          # Fast-DDS port-arithmetic hard ceiling

# --- fixed operator-facing strings -------------------------------------------

STOP_ADVISORY = 'The physical stop buttons are the only real stop.'

# --- Robotiq 2F-85 grippers --------------------------------------------------
#
# The stroke and the two adjustable ranges are the manufacturer's documented
# figures for the 2F-85 (85 mm opening; 20-150 mm/s finger speed; 20-235 N grip
# force). They are duplicated from the franka_robotiq driver's unit table on
# purpose: franka_web must build and run on a workspace where franka_robotiq
# was never built, so it cannot import them.  e2e_fake_dual_gripper_row_test
# asserts the two copies are identical whenever franka_robotiq IS present.

GRIPPER_STROKE_MM = 85.0
GRIPPER_SPEED_RANGE_MM_S = (20.0, 150.0)
GRIPPER_FORCE_RANGE_N = (20.0, 235.0)

GRIPPER_ACTIONS = ('open', 'close', 'width', 'stop', 'reactivate')
GRIPPER_NODE_SUFFIX = '_robotiq'

# The gripper nodes are STANDING nodes: the operator starts them, this server
# never does.  These two names exist only so the "no gripper node is running"
# sentence can teach the exact command that fixes it, with one owner for the
# spelling.
GRIPPER_LAUNCH_PACKAGE = 'franka_robotiq'
GRIPPER_DUAL_LAUNCH_FILE = 'dual_robotiq.launch.py'

# Per-arm defaults; every one is a contract value with its reason recorded in
# the shipped config example.  THIRTEEN keys, matching the config schema:
# joint_names is the thirteenth and cannot live here, because its default
# depends on the arm id -- GRIPPER_JOINT_NAME_TEMPLATE below is what
# config.py formats per arm.
DEFAULT_GRIPPER = {
    'enabled': False,
    'serial_id': '',
    'usb_path': '',
    'speed_mm_s': 85.0,
    'force_n': 74.0,
    'open_width_mm': 85.0,
    'close_width_mm': 0.0,
    'poll_rate_hz': 20.0,
    'auto_activate': True,
    'motion_timeout_s': 5.0,
    'activation_timeout_s': 10.0,
    'reconnect_interval_s': 2.0,
}

# '{arm_id}' is filled by config.py.  These names deliberately do NOT collide
# with franka_gripper's <arm_id>_finger_joint1/2, so both nodes can run.
GRIPPER_JOINT_NAME_TEMPLATE = ('{arm_id}_robotiq_finger_joint1',
                               '{arm_id}_robotiq_finger_joint2')

GRIPPER_POLL_RATE_RANGE_HZ = (1.0, 100.0)
GRIPPER_MOTION_TIMEOUT_RANGE_S = (0.5, 30.0)
GRIPPER_ACTIVATION_TIMEOUT_RANGE_S = (1.0, 60.0)
GRIPPER_RECONNECT_INTERVAL_RANGE_S = (0.5, 30.0)

# A status sample older than this makes the arm's gripper unavailable, the same
# treatment `positions_stale` gets on the joint stream.
GRIPPER_STATUS_STALE_S = 2.0

# How long a gripper request may hold the supervisor thread waiting for the
# node to ACCEPT it. Motion is never waited for.
GRIPPER_REQUEST_TIMEOUT_S = 1.0

# The busy-flag watchdog.  The server does not launch the gripper nodes and so
# cannot know their motion_timeout_s; this is the contract's own ceiling for
# that key (30 s) plus one second, so a crashed node cannot wedge the row.
GRIPPER_BUSY_MAX_S = 31.0

# --- ghost (the 3D scene and pose authoring) ---------------------------------
#
# The ghost solves a hand pose into joint angles and checks it; it commands
# nothing. These are the interaction budgets that keeps a drag honest.

# An interactive drag: five seconds of SERVICE_CALL_TIMEOUT_S here would
# freeze a pointer. The IK service measures p99 105 us over localhost shared
# memory, so 0.25 s is roughly 2000x its own worst case and really bounds a
# wedged node rather than the solver.
GHOST_SOLVE_TIMEOUT_S = 0.25
# The deadline for the WHOLE batch of redundancy samples, not for one call:
# 25 sequential calls at a per-call budget would be a 37 s request.
GHOST_REDUNDANCY_TIMEOUT_S = 1.5
GHOST_RATE_CAPACITY = 60
GHOST_RATE_REFILL_HZ = 30.0     # 1.5x JOG_STREAM_HZ, the interactive rate
GHOST_REDUNDANCY_SAMPLES = 25
GHOST_REDUNDANCY_SAMPLES_MIN = 9
GHOST_REDUNDANCY_SAMPLES_MAX = 33
GHOST_SESSION_VIEW_TTL_S = 0.2  # = 1 / STATE_FRAME_HZ
# Where the generated scene assets are served from, relative to the static
# root. The trailing slash is part of the value: it is prefixed onto a served
# path, and a missing slash would match `ghost/assetsfoo` too.
GHOST_ASSET_PREFIX = 'ghost/assets/'

# --- config-schema bounds that nothing else owns -----------------------------

JOG_STEP_MAXIMUM_DEG = 15.0          # config section 4.2 bound on jog.step_deg
SETTLING_DRIFT_MAXIMUM_DEG = 30.0    # config section 4.2 bound
CONFIG_FILE_MAXIMUM_BYTES = 1048576  # a config is dozens of lines, not a MiB
