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
Test double for ``franka_web.defaults`` (see ``part1_stub/__init__.py``).

Every constant carries the value the build contract pins for it. The real
module is authored in a parallel change and replaces this one wholesale; the
values here exist so the backend can be exercised in the meantime, and so a
drift between the two is caught the moment the real module lands.
"""

# --- identity ---------------------------------------------------------------

SERVER_NAME = 'franka_web'
SERVER_VERSION = '2.0.0'
#: Bumped from 2: the frame gained hint, logs, session.steps,
#: recording.disabled, the fault cause block and the per-arm
#: source/rate/topic/template keys, and lost controller_name/gains_sha256.
SCHEMA_VERSION = 3
MOTION_CONTROLLER = 'dual_arm_joint_impedance_controller'

# --- the jog / motion contract ----------------------------------------------

JOINT_COUNT = 7
JOG_STEP_RAD = 0.03490658503988659      # 2 degrees
JOG_STREAM_HZ = 20.0

#: The reviewed impedance controller's timing policy. NOT config keys: the
#: reviewed validator requires exact equality with these values.
REVIEWED_TIMING_S = {
    'watchdog_timeout': 0.1,
    'max_header_age': 1.0,
    'future_tolerance': 0.1,
}

# --- state fan-out ----------------------------------------------------------

STATE_FRAME_HZ = 5.0
SSE_PING_INTERVAL_S = 10.0
#: The Broker constructor's bare default. The production broker is built with
#: queue_depth=64; nobody edits this to fix a log-eviction problem.
SSE_QUEUE_DEPTH = 4

# --- operator lock ----------------------------------------------------------

OPERATOR_LOCK_TTL_S = 15.0
OPERATOR_HEARTBEAT_INTERVAL_S = 5.0

# --- supervisor timing ------------------------------------------------------

SUPERVISOR_TICK_S = 0.1
PREFLIGHT_TIMEOUT_S = 30.0
STARTING_TIMEOUT_S = 60.0
ACTIVATION_SETTLING_MAX_TIMEOUT_S = 60.0
SERVICE_CALL_TIMEOUT_S = 5.0

# --- freshness and fault thresholds -----------------------------------------

ENABLE_JOINT_STATE_MAX_AGE_S = 0.2
JOINT_STATE_STALE_FAULT_S = 1.0
CCSR_FAULT_THRESHOLD = 0.95
CCSR_FAULT_SUSTAIN_S = 5.0

# --- recording --------------------------------------------------------------

RECORDING_SEGMENT_DURATION_S = 3600

# --- child stop escalation --------------------------------------------------

STOP_SIGINT_WAIT_S = 10.0
STOP_SIGTERM_WAIT_S = 5.0
STOP_SIGKILL_WAIT_S = 5.0
RECORDER_STOP_SIGINT_WAIT_S = 30.0
RECORDER_STOP_SIGTERM_WAIT_S = 10.0
RECORDER_STOP_SIGKILL_WAIT_S = 5.0

# --- ROS domain -------------------------------------------------------------

ROS_DOMAIN_ID_MAXIMUM = 232

# --- fixed operator-facing strings ------------------------------------------

STOP_ADVISORY = 'The physical stop buttons are the only real stop.'

# --- defaults the config file may override ----------------------------------

DEFAULT_PORT = 8765
DEFAULT_BIND = '0.0.0.0'
DEFAULT_ROBOT_IPS = {'panda1': '172.16.0.2', 'panda2': '172.16.0.3'}
DEFAULT_STATE_DIR = '~/.local/state/franka_web'
DEFAULT_RECORDING_ROOT = '~/franka_web_recordings'

# --- the Panda factory policy limits ----------------------------------------

POLICY_POSITION_LOWER_RAD = (
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
POLICY_POSITION_UPPER_RAD = (
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
POLICY_EFFORT_CEILING_NM = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
POLICY_VELOCITY_CEILING_RAD_S = (2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)

# --- the live-proven motion profiles ----------------------------------------

STANDARD_PROFILE = {
    'k_gains': (20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 60.0),
    'd_gains': (1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 1.0),
    'max_effort_nm': (10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0),
    'max_target_velocity_rad_s': (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
    'watchdog_timeout_s': 0.1,
    'max_header_age_s': 1.0,
    'future_tolerance_s': 0.1,
}

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

DEFAULT_SETTLING = {
    'drift_limit_rad': (0.03490658503988659,) * 7,
    'span_limit_rad': (0.000872664626,) * 7,
    'velocity_limit_rad_s': (0.01745329252,) * 7,
    'fence_margin_rad': (0.0872664626,) * 7,
    'stable_window_s': 1.0,
    'min_samples': 6,
    'timeout_s': 5.0,
}
