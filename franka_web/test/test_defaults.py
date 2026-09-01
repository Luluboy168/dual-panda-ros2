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

"""Pin every baked-in constant in ``franka_web.defaults`` to its reviewed source."""

import ast
import math
import os
from pathlib import Path
import re
import subprocess
import sys

from franka_web import defaults
import pytest


MODULE_PATH = Path(defaults.__file__)
PACKAGE_DIR = MODULE_PATH.parent


def _module_source():
    """Return the text of defaults.py."""
    return MODULE_PATH.read_text(encoding='utf-8')


class TestModuleShape:
    """The module is pure data and imports nothing."""

    def test_module_has_no_imports(self):
        """No import statement of any kind appears in defaults.py."""
        tree = ast.parse(_module_source())
        offenders = [node for node in ast.walk(tree)
                     if isinstance(node, (ast.Import, ast.ImportFrom))]
        assert offenders == []

    def test_module_has_no_functions_or_classes(self):
        """The module declares data only -- no callables, no classes."""
        tree = ast.parse(_module_source())
        offenders = [node for node in ast.walk(tree)
                     if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                                          ast.ClassDef))]
        assert offenders == []

    def test_importable_without_ros(self):
        """The module imports in a bare interpreter with no ROS environment."""
        result = subprocess.run(
            [sys.executable, '-c',
             'from franka_web import defaults; print(defaults.SERVER_VERSION)'],
            env={'PATH': '/usr/bin:/bin', 'PYTHONPATH': str(PACKAGE_DIR.parent)},
            capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == defaults.SERVER_VERSION


class TestIdentity:
    """The four identity constants live here and nowhere else."""

    def test_identity_constants(self):
        """The server name, version and schema version are the v2 values."""
        assert defaults.SERVER_NAME == 'franka_web'
        assert defaults.SERVER_VERSION == '2.0.0'
        assert defaults.SCHEMA_VERSION == 3

    def test_motion_controller_is_the_impedance_controller(self):
        """The one controller the web surface offers is the impedance controller."""
        assert defaults.MOTION_CONTROLLER == 'dual_arm_joint_impedance_controller'

    def test_identity_constants_are_defined_exactly_once_in_the_tree(self):
        """Only defaults.py ASSIGNS SERVER_VERSION anywhere in the package."""
        pattern = re.compile(r'^SERVER_VERSION\s*=', re.MULTILINE)
        assigners = []
        for path in sorted(PACKAGE_DIR.glob('*.py')):
            try:
                text = path.read_text(encoding='utf-8')
            except OSError:  # pragma: no cover - installed tree without sources
                continue
            if pattern.search(text):
                assigners.append(path.name)
        assert assigners == ['defaults.py']


class TestJointVectors:
    """Every joint vector is seven long and inside the factory policy."""

    @pytest.mark.parametrize('name', [
        'POLICY_POSITION_LOWER_RAD', 'POLICY_POSITION_UPPER_RAD',
        'POLICY_EFFORT_CEILING_NM', 'POLICY_VELOCITY_CEILING_RAD_S'])
    def test_every_policy_vector_is_length_seven(self, name):
        """Each policy vector carries exactly seven joints."""
        assert len(getattr(defaults, name)) == defaults.JOINT_COUNT

    @pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
    @pytest.mark.parametrize('key', ['k_gains', 'd_gains', 'max_effort_nm',
                                     'max_target_velocity_rad_s'])
    def test_every_profile_vector_is_length_seven(self, arm_id, key):
        """Each profile vector carries exactly seven joints."""
        assert len(defaults.DEFAULT_PROFILES[arm_id][key]) == defaults.JOINT_COUNT

    @pytest.mark.parametrize('key', ['drift_limit_rad', 'span_limit_rad',
                                     'velocity_limit_rad_s', 'fence_margin_rad'])
    def test_every_settling_vector_is_length_seven(self, key):
        """Each settling vector carries exactly seven joints."""
        assert len(defaults.DEFAULT_SETTLING[key]) == defaults.JOINT_COUNT

    def test_position_policy_lower_is_below_upper_on_every_joint(self):
        """The factory box is non-degenerate on all seven joints."""
        for lower, upper in zip(defaults.POLICY_POSITION_LOWER_RAD,
                                defaults.POLICY_POSITION_UPPER_RAD):
            assert lower < upper


class TestProfiles:
    """The two live-proven motion profiles."""

    def test_standard_profile_matches_the_contract(self):
        """panda1's profile is the user-approved Phase 10 jog set, verbatim."""
        assert defaults.STANDARD_PROFILE == {
            'k_gains': (20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 60.0),
            'd_gains': (1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 1.0),
            'max_effort_nm': (10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0),
            'max_target_velocity_rad_s': (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
            'watchdog_timeout_s': 0.1,
            'max_header_age_s': 1.0,
            'future_tolerance_s': 0.1,
        }

    def test_stiff_j2_profile_differs_from_standard_only_in_joint_two(self):
        """panda2 stiffens joint 2 and changes nothing else."""
        standard = defaults.STANDARD_PROFILE
        stiff = defaults.STIFF_J2_PROFILE
        assert stiff['k_gains'][1] == 60.0 and standard['k_gains'][1] == 20.0
        assert stiff['d_gains'][1] == 2.0 and standard['d_gains'][1] == 1.0
        for index in range(defaults.JOINT_COUNT):
            if index == 1:
                continue
            assert stiff['k_gains'][index] == standard['k_gains'][index]
            assert stiff['d_gains'][index] == standard['d_gains'][index]
        for key in ('max_effort_nm', 'max_target_velocity_rad_s',
                    'watchdog_timeout_s', 'max_header_age_s', 'future_tolerance_s'):
            assert stiff[key] == standard[key]

    def test_torque_ceilings_are_identical_across_profiles(self):
        """Stiffening joint 2 never raised a torque ceiling."""
        assert (defaults.STIFF_J2_PROFILE['max_effort_nm']
                == defaults.STANDARD_PROFILE['max_effort_nm'])

    @pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
    def test_profile_efforts_are_within_the_policy_effort_ceiling(self, arm_id):
        """No default torque limit exceeds the Panda hardware ceiling."""
        for value, ceiling in zip(defaults.DEFAULT_PROFILES[arm_id]['max_effort_nm'],
                                  defaults.POLICY_EFFORT_CEILING_NM):
            assert 0.0 < value <= ceiling

    @pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
    def test_profile_velocities_are_within_the_policy_velocity_ceiling(self, arm_id):
        """No default speed limit exceeds the factory URDF velocity ceiling."""
        profile = defaults.DEFAULT_PROFILES[arm_id]
        for value, ceiling in zip(profile['max_target_velocity_rad_s'],
                                  defaults.POLICY_VELOCITY_CEILING_RAD_S):
            assert 0.0 < value <= ceiling

    def test_reviewed_timing_is_the_impedance_controller_timing(self):
        """The reviewed timing table carries the controller's three fixed values."""
        assert defaults.REVIEWED_TIMING_S == {
            'watchdog_timeout': 0.1, 'max_header_age': 1.0, 'future_tolerance': 0.1}
        for arm_id in defaults.ARM_IDS:
            profile = defaults.DEFAULT_PROFILES[arm_id]
            assert profile['watchdog_timeout_s'] == defaults.REVIEWED_TIMING_S[
                'watchdog_timeout']
            assert profile['max_header_age_s'] == defaults.REVIEWED_TIMING_S[
                'max_header_age']
            assert profile['future_tolerance_s'] == defaults.REVIEWED_TIMING_S[
                'future_tolerance']


class TestSettlingDefaults:
    """The live-proven activation-settling numbers."""

    def test_default_settling_literals_are_the_contract_literals(self):
        """The four vectors are the exact literals, truncated digits included."""
        assert defaults.DEFAULT_SETTLING['drift_limit_rad'] == (0.03490658503988659,) * 7
        assert defaults.DEFAULT_SETTLING['span_limit_rad'] == (0.000872664626,) * 7
        assert defaults.DEFAULT_SETTLING['velocity_limit_rad_s'] == (0.01745329252,) * 7
        assert defaults.DEFAULT_SETTLING['fence_margin_rad'] == (0.0872664626,) * 7
        assert defaults.DEFAULT_SETTLING['stable_window_s'] == 1.0
        assert defaults.DEFAULT_SETTLING['min_samples'] == 6
        assert defaults.DEFAULT_SETTLING['timeout_s'] == 5.0

    @pytest.mark.parametrize('key,degrees', [
        ('drift_limit_rad', 2.0), ('span_limit_rad', 0.05),
        ('velocity_limit_rad_s', 1.0), ('fence_margin_rad', 5.0)])
    def test_default_settling_literals_agree_with_degrees_within_1e_9(self, key, degrees):
        """
        Show the truncated literals are the intended degree values, not typos.

        This documents intent. It must NOT become an exact-equality test: three
        of the four are deliberately truncated live-proven numbers.
        """
        assert abs(defaults.DEFAULT_SETTLING[key][0] - math.radians(degrees)) < 1e-9

    def test_default_settling_is_feasible_at_the_supervisor_cadence(self):
        """The shipped settling policy fits inside its own timeout."""
        tick_ns = int(defaults.SUPERVISOR_TICK_S * 1e9)
        window_ns = math.ceil(defaults.DEFAULT_SETTLING['stable_window_s'] * 1e9)
        stable_span_ns = ((window_ns + tick_ns - 1) // tick_ns) * tick_ns
        sample_span_ns = (defaults.DEFAULT_SETTLING['min_samples'] - 1) * tick_ns
        minimum_total_ns = (2 * tick_ns) + max(stable_span_ns, sample_span_ns)
        assert minimum_total_ns < math.ceil(
            defaults.DEFAULT_SETTLING['timeout_s'] * 1e9)


class TestOtherConstants:
    """The remaining pinned numbers and the constants that must be gone."""

    def test_jog_step_is_exactly_two_degrees(self):
        """The one fixed jog step is exactly two degrees in radians."""
        assert defaults.JOG_STEP_RAD == math.radians(2.0)

    def test_jog_stream_is_at_least_twice_the_watchdog_floor(self):
        """One jog stream period is well inside the controller's watchdog window."""
        assert 1.0 / defaults.JOG_STREAM_HZ < defaults.REVIEWED_TIMING_S['watchdog_timeout']

    def test_default_robot_ips_are_the_factory_addresses(self):
        """Both arms default to their Franka factory addresses."""
        assert defaults.DEFAULT_ROBOT_IPS == {'panda1': '172.16.0.2',
                                              'panda2': '172.16.0.3'}

    def test_default_bind_and_port(self):
        """The server defaults to the lab-reachable bind and the v2 port."""
        assert defaults.DEFAULT_BIND == '0.0.0.0'
        assert defaults.DEFAULT_PORT == 8765

    def test_sse_queue_depth_is_the_bare_constructor_default(self):
        """The depth stays 4 and says so, so nobody repurposes it."""
        assert defaults.SSE_QUEUE_DEPTH == 4
        assert 'queue_depth=64' in _module_source()
        # That line is a signpost in a comment. The number that actually runs
        # lives in server.py, so pin it where it is (test_sse.py's
        # TestFramePump proves what it does).
        server_source = (PACKAGE_DIR / 'server.py').read_text(encoding='utf-8')
        assert '_PRODUCTION_QUEUE_DEPTH = 64' in server_source
        assert 'Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)' in server_source

    @pytest.mark.parametrize('name', [
        'MAX_GAINS_BYTES', 'GAINS_DIR_NAME', 'ALLOWED_BIND', 'WEB_CONTROLLERS',
        'JOG_CONTROLLERS', 'POSE_CACHE_TTL_S',
        'WATCHDOG_TIMEOUT_S', 'MAX_HEADER_AGE_S', 'FUTURE_TOLERANCE_S'])
    def test_deleted_v1_constants_are_absent(self, name):
        """Every deleted or de-duplicated v1 constant is gone from the module."""
        assert not hasattr(defaults, name)


class TestReviewedPolicyAgreement:
    """The copied policy constants still agree with the reviewed policy file."""

    def test_policy_matches_the_reviewed_panda_limit_policy_file(self):
        """The four policy vectors and the timing equal the reviewed policy file."""
        packages = pytest.importorskip('ament_index_python.packages')
        yaml = pytest.importorskip('yaml')
        try:
            share = packages.get_package_share_directory('franka_example_controllers')
        except Exception:  # pragma: no cover - package not built in this workspace
            pytest.skip('franka_example_controllers share directory not found')
        path = os.path.join(share, 'config', 'panda_joint_limits_v1.yaml')
        if not os.path.exists(path):  # pragma: no cover - policy file not installed
            pytest.skip('reviewed panda joint-limits policy file not installed')
        with open(path, 'r', encoding='utf-8') as handle:
            policy = yaml.safe_load(handle)
        assert tuple(policy['position_lower']) == defaults.POLICY_POSITION_LOWER_RAD
        assert tuple(policy['position_upper']) == defaults.POLICY_POSITION_UPPER_RAD
        assert tuple(policy['effort_ceiling']) == defaults.POLICY_EFFORT_CEILING_NM
        assert (tuple(policy['urdf_velocity_ceiling'])
                == defaults.POLICY_VELOCITY_CEILING_RAD_S)
        assert (policy['reviewed_timing_seconds'][defaults.MOTION_CONTROLLER]
                == defaults.REVIEWED_TIMING_S)
