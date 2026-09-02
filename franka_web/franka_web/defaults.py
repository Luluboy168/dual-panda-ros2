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
SCHEMA_VERSION = 3

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

# --- config-schema bounds that nothing else owns -----------------------------

JOG_STEP_MAXIMUM_DEG = 15.0          # config section 4.2 bound on jog.step_deg
SETTLING_DRIFT_MAXIMUM_DEG = 30.0    # config section 4.2 bound
CONFIG_FILE_MAXIMUM_BYTES = 1048576  # a config is dozens of lines, not a MiB
