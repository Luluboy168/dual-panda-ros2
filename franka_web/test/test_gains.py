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
Tests for franka_web.gains: render from config, validate, content-address.

There is no upload surface any more. A Motion session's controller profile is
RENDERED from the operator's configuration, handed to the reviewed validator
in process, and written under its own content hash. These tests pin the
rendered document byte-for-byte where the launch depends on it, prove the
exact doubles survive the round trip, and keep every store-integrity case the
upload path used to carry.

Nothing here initializes rclpy, starts a node or touches DDS; the validator is
a pure function over text, which is why the whole path is unit-testable.
"""

import hashlib
import os
import stat

from franka_bringup import controller_config_validator as validator
from franka_web import defaults
from franka_web.config import MotionProfile
from franka_web.gains import (
    ARM_SELECTIONS, FENCE_KEYS, PROFILE_DIR_NAME, ProfileStore,
    ProfileStoreError, render_profile_yaml, StoredProfile)
import pytest

IMPEDANCE = defaults.MOTION_CONTROLLER
IMPEDANCE_TYPE = 'franka_example_controllers/DualArmJointImpedanceController'


def make_profile(arm_id, **overrides):
    """Build one MotionProfile from the baked-in defaults plus overrides."""
    baked = defaults.DEFAULT_PROFILES[arm_id]
    values = {
        'arm_id': arm_id,
        'k_gains': tuple(baked['k_gains']),
        'd_gains': tuple(baked['d_gains']),
        'max_effort_nm': tuple(baked['max_effort_nm']),
        'max_target_velocity_rad_s': tuple(baked['max_target_velocity_rad_s']),
        'watchdog_timeout_s': defaults.REVIEWED_TIMING_S['watchdog_timeout'],
        'max_header_age_s': defaults.REVIEWED_TIMING_S['max_header_age'],
        'future_tolerance_s': defaults.REVIEWED_TIMING_S['future_tolerance'],
        'position_lower_rad': tuple(defaults.POLICY_POSITION_LOWER_RAD),
        'position_upper_rad': tuple(defaults.POLICY_POSITION_UPPER_RAD),
        'fence_enabled': False,
        'from_file': False,
    }
    values.update(overrides)
    return MotionProfile(**values)


def both_profiles(**overrides):
    """Return the two-arm profile mapping."""
    return {arm_id: make_profile(arm_id, **overrides)
            for arm_id in ('panda1', 'panda2')}


@pytest.fixture()
def store(tmp_path):
    """Return a ProfileStore over a private 0700 state directory."""
    state_dir = tmp_path / 'state'
    state_dir.mkdir(mode=0o700)
    os.chmod(str(tmp_path), 0o700)
    return ProfileStore(str(state_dir))


# ----------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------


def test_render_produces_the_dual_layout_the_validator_accepts():
    """The two-arm document validates and carries no arm_count key."""
    text = render_profile_yaml(('panda1', 'panda2'), both_profiles())
    assert 'arm_count' not in text
    validated = validator.validate_controller_config_text(text, IMPEDANCE)
    assert validated.controller_type == IMPEDANCE_TYPE


def test_render_produces_the_single_layout_with_arm_count_one():
    """The one-arm document carries arm_count: 1 and one arm block."""
    text = render_profile_yaml(('panda1',), {'panda1': make_profile('panda1')})
    assert '    arm_count: 1\n' in text
    assert 'arm_2' not in text
    validated = validator.validate_single_controller_config_text(
        text, IMPEDANCE, 'panda1')
    assert validated.controller_name == IMPEDANCE


def test_render_puts_the_selected_arm_in_arm_1_for_a_single_session():
    """A panda2-only session names panda2 in the single arm_1 block."""
    text = render_profile_yaml(('panda2',), {'panda2': make_profile('panda2')})
    assert '      arm_id: panda2\n' in text
    assert 'panda2_joint7' in text
    validator.validate_single_controller_config_text(text, IMPEDANCE, 'panda2')


def test_rendered_numbers_round_trip_to_the_exact_doubles():
    """
    The fence the jog model clamps to must be the fence the file carries.

    A printed-and-reparsed bound would be a different double from the one
    JogTargetModel.step clamps to and the controller's accept() checks.
    """
    awkward = tuple(-2.8973 + index * 1e-9 for index in range(7))
    text = render_profile_yaml(
        ('panda1', 'panda2'), both_profiles(position_lower_rad=awkward))
    parsed = validator.load_strict_yaml(text)[
        '/' + IMPEDANCE]['ros__parameters']['arm_1']['position_lower']
    assert tuple(float(value) for value in parsed) == awkward


def test_render_uses_the_profile_timings_verbatim():
    """The reviewed timing triple is what the profile carries."""
    text = render_profile_yaml(('panda1', 'panda2'), both_profiles())
    parameters = validator.load_strict_yaml(text)['/' + IMPEDANCE]['ros__parameters']
    assert parameters['watchdog_timeout'] == defaults.REVIEWED_TIMING_S['watchdog_timeout']
    assert parameters['max_header_age'] == defaults.REVIEWED_TIMING_S['max_header_age']
    assert parameters['future_tolerance'] == defaults.REVIEWED_TIMING_S['future_tolerance']


def test_render_uses_the_profile_position_bounds_verbatim():
    """gains.py does no fence policy of its own; the profile decides."""
    lower = tuple(value + 0.1 for value in defaults.POLICY_POSITION_LOWER_RAD)
    upper = tuple(value - 0.1 for value in defaults.POLICY_POSITION_UPPER_RAD)
    profiles = both_profiles(position_lower_rad=lower, position_upper_rad=upper,
                             fence_enabled=True)
    parameters = validator.load_strict_yaml(
        render_profile_yaml(('panda1', 'panda2'), profiles))[
            '/' + IMPEDANCE]['ros__parameters']
    assert tuple(parameters['arm_2']['position_upper']) == upper


def test_render_carries_the_stiffened_j2_default_for_panda2():
    """panda2's baked-in profile is the stiffened-J2 set."""
    text = render_profile_yaml(('panda1', 'panda2'), both_profiles())
    parameters = validator.load_strict_yaml(text)['/' + IMPEDANCE]['ros__parameters']
    assert parameters['arm_1']['k_gains'][1] == 20.0
    assert parameters['arm_2']['k_gains'][1] == 60.0
    assert parameters['arm_2']['d_gains'][1] == 2.0
    # Torque ceilings are the safety bound and were never raised with it.
    assert parameters['arm_1']['max_effort'] == parameters['arm_2']['max_effort']


# ----------------------------------------------------------------------
# The store
# ----------------------------------------------------------------------


def test_materialize_content_addresses_by_sha256(store):
    """The file name is the validator's own hash of the rendered bytes."""
    record = store.materialize('both', both_profiles())
    assert isinstance(record, StoredProfile)
    assert os.path.basename(record.path) == record.config_sha256 + '.yaml'
    with open(record.path, 'rb') as handle:
        assert hashlib.sha256(handle.read()).hexdigest() == record.config_sha256
    assert record.arms == ('panda1', 'panda2')
    assert set(record.fence) == {'panda1', 'panda2'}
    assert set(record.fence['panda1']) == set(FENCE_KEYS)


def test_materialize_is_idempotent_for_identical_profiles(store):
    """The same profile always lands on the same path and record."""
    first = store.materialize('both', both_profiles())
    second = store.materialize('both', both_profiles())
    assert first is second
    assert first.path == second.path


def test_materialize_writes_the_single_layout_for_a_one_arm_session(store):
    """A single-arm session materializes the one-arm document."""
    record = store.materialize('panda2', {'panda2': make_profile('panda2')})
    assert record.arms == ('panda2',)
    with open(record.path, encoding='utf-8') as handle:
        assert 'arm_count: 1' in handle.read()


def test_materialize_raises_profile_invalid_with_the_validators_own_sentence(store):
    """An impossible profile carries the validator's exact sentence."""
    over_ceiling = tuple(
        value + 1.0 for value in defaults.POLICY_EFFORT_CEILING_NM)
    with pytest.raises(ProfileStoreError) as excinfo:
        store.materialize('both', both_profiles(max_effort_nm=over_ceiling))
    assert excinfo.value.code == 'profile_invalid'
    text = render_profile_yaml(
        ('panda1', 'panda2'), both_profiles(max_effort_nm=over_ceiling))
    with pytest.raises(validator.ControllerConfigError) as direct:
        validator.validate_controller_config_text(text, IMPEDANCE)
    assert excinfo.value.detail == str(direct.value)


def test_materialize_refuses_an_unknown_arm_selection(store):
    """`arms` is a closed set here too."""
    with pytest.raises(ProfileStoreError):
        store.materialize('panda3', both_profiles())


def test_a_squatted_content_addressed_name_raises_oserror_not_a_refusal(store):
    """
    Store corruption is a 500, never an operator-correctable refusal.

    The profile was fine, the filesystem is not, and nobody should edit
    their configuration in response to an attack on the state directory.
    """
    record = store.materialize('both', both_profiles())
    with open(record.path, 'w', encoding='utf-8') as handle:
        handle.write('not the bytes this name promises\n')
    with pytest.raises(OSError):
        store.materialize('both', both_profiles())


def test_verify_detects_a_file_whose_content_changed_after_materialize(store):
    """The last web-owned boundary re-proves the hash before the launch."""
    record = store.materialize('both', both_profiles())
    store.verify(record)
    with open(record.path, 'a', encoding='utf-8') as handle:
        handle.write('\n# edited\n')
    with pytest.raises(OSError):
        store.verify(record)


def test_the_store_refuses_a_profiles_directory_with_group_bits(store, tmp_path):
    """A directory mode nobody sane set means something else has been here."""
    profile_dir = os.path.join(str(tmp_path / 'state'), PROFILE_DIR_NAME)
    os.mkdir(profile_dir, 0o750)
    os.chmod(profile_dir, 0o750)
    with pytest.raises(OSError):
        store.materialize('both', both_profiles())
    assert stat.S_IMODE(os.stat(profile_dir).st_mode) == 0o750


def test_the_arm_selection_table_is_the_three_legal_values():
    """`both` is the two-arm layout; the other two are one-arm."""
    assert dict(ARM_SELECTIONS) == {
        'panda1': ('panda1',),
        'panda2': ('panda2',),
        'both': ('panda1', 'panda2'),
    }
