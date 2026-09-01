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
Tests for franka_web.gains: validate in-process, store content-addressed.

Every config in ``test/support/sample_gains/`` is a real file on disk, and each
invalid one is here to fail for its own distinct reason -- YAML size, depth,
scalar count, anchors, aliases, merge keys, explicit tags, nulls, booleans,
non-finite numbers, duplicate keys, non-string keys, the wrong controller's
root key, missing or unknown keys at every level, non-canonical joint names,
gains that are short/negative/non-numeric/overflowing, efforts and target
velocities outside the pinned Panda policy, an inverted fence, timing values
that do not equal the reviewed policy, ``arm_2`` present in a one-arm config,
``arm_count`` missing or not 1, and a one-arm config aimed at the wrong arm.

Two things every invalid case asserts: the section 6.5 code is ``gains_invalid``
and ``detail`` is the validator's own sentence VERBATIM -- pinned twice, once
against a literal in the table below (so the fixture provably fails for the
reason it claims to) and once against the message the validator raises when
called directly on the same bytes (so nothing here wraps, prefixes or truncates
it).

Nothing in this file initializes rclpy, starts a node or touches DDS; the
validator is a pure function over text, which is exactly why the upload path
can be proven at unit level.
"""

import ast
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
import re
import stat

from franka_bringup import controller_config_validator as validator
from franka_web import config
from franka_web.gains import ARM_SELECTIONS, FENCE_KEYS, GainsError, GainsStore, StoredGains
import pytest

SAMPLE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'support', 'sample_gains')

IMPEDANCE = 'dual_arm_joint_impedance_controller'
HOLD = 'dual_arm_joint_hold_controller'
VELOCITY = 'dual_arm_joint_velocity_controller'

IMPEDANCE_TYPE = 'franka_example_controllers/DualArmJointImpedanceController'
HOLD_TYPE = 'franka_example_controllers/DualArmJointHoldController'

# The gains in the fixture corpus, so the fence assertions read as data.
K_GAINS = (20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0)
D_GAINS = (1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.25)
MAX_EFFORT = (10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0)
POSITION_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
POSITION_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
MAX_TARGET_VELOCITY = (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

# (fixture, controller_name, arms) for each config the validator accepts.
VALID_CASES = (
    ('valid_dual_impedance.yaml', IMPEDANCE, 'both'),
    ('valid_dual_hold.yaml', HOLD, 'both'),
    ('valid_single_impedance_panda1.yaml', IMPEDANCE, 'panda1'),
    ('valid_single_impedance_panda2.yaml', IMPEDANCE, 'panda2'),
    ('valid_single_hold_panda1.yaml', HOLD, 'panda1'),
    ('valid_single_hold_panda2.yaml', HOLD, 'panda2'),
)

# The one fixture that never reaches the validator at all: its bytes are not
# UTF-8, so `detail` is franka_web's own sentence rather than a validator one.
NOT_UTF8_FIXTURE = 'invalid_not_utf8.yaml'
NOT_UTF8_DETAIL = 'configuration is not valid UTF-8'

# (fixture, controller_name, arms, the validator's verbatim message).
INVALID_CASES = (
    # --- structural: refused by load_strict_yaml before any schema check ----
    ('invalid_empty.yaml', IMPEDANCE, 'both',
     'YAML size is outside the fixed limit'),
    ('invalid_nul_byte.yaml', IMPEDANCE, 'both',
     'NUL bytes are forbidden'),
    ('invalid_multiple_documents.yaml', IMPEDANCE, 'both',
     'multiple YAML documents are forbidden'),
    ('invalid_alias.yaml', IMPEDANCE, 'both',
     'YAML aliases are forbidden'),
    ('invalid_anchor.yaml', IMPEDANCE, 'both',
     'YAML anchors are forbidden'),
    ('invalid_explicit_tag.yaml', IMPEDANCE, 'both',
     'explicit YAML tags are forbidden'),
    ('invalid_merge_key.yaml', IMPEDANCE, 'both',
     'YAML merge keys are forbidden'),
    ('invalid_deep_nesting.yaml', IMPEDANCE, 'both',
     'YAML nesting exceeds the fixed limit'),
    ('invalid_scalar_flood.yaml', IMPEDANCE, 'both',
     'YAML scalar count exceeds the fixed limit'),
    ('invalid_malformed.yaml', IMPEDANCE, 'both',
     'malformed YAML'),
    ('invalid_duplicate_key.yaml', IMPEDANCE, 'both',
     'duplicate YAML key: k_gains'),
    ('invalid_non_string_key.yaml', IMPEDANCE, 'both',
     'every YAML mapping key must be a string'),
    ('invalid_null_value.yaml', IMPEDANCE, 'both',
     'null YAML values are forbidden'),
    ('invalid_boolean_value.yaml', IMPEDANCE, 'both',
     'boolean YAML values are forbidden'),
    ('invalid_non_finite.yaml', IMPEDANCE, 'both',
     'non-finite YAML numbers are forbidden'),
    ('invalid_unsupported_value.yaml', IMPEDANCE, 'both',
     'unsupported YAML value'),
    ('invalid_root_sequence.yaml', IMPEDANCE, 'both',
     'controller configuration must be a mapping'),
    # --- the wrong document, or the wrong controller's document ------------
    ('invalid_root_key.yaml', IMPEDANCE, 'both',
     'controller configuration keys differ; '
     "missing=['/dual_arm_joint_impedance_controller'] "
     "unknown=['/other_controller']"),
    ('invalid_extra_root_key.yaml', IMPEDANCE, 'both',
     "controller configuration keys differ; missing=[] unknown=['extra_root']"),
    ('invalid_hold_config_as_impedance.yaml', IMPEDANCE, 'both',
     'controller configuration keys differ; '
     "missing=['/dual_arm_joint_impedance_controller'] "
     "unknown=['/dual_arm_joint_hold_controller']"),
    ('invalid_velocity_config_as_impedance.yaml', IMPEDANCE, 'both',
     'controller configuration keys differ; '
     "missing=['/dual_arm_joint_impedance_controller'] "
     "unknown=['/dual_arm_joint_velocity_controller']"),
    ('invalid_node_key.yaml', IMPEDANCE, 'both',
     '/dual_arm_joint_impedance_controller keys differ; '
     "missing=['ros__parameters'] unknown=['params']"),
    # --- key sets, dual impedance -------------------------------------------
    ('invalid_missing_arm_2.yaml', IMPEDANCE, 'both',
     "ros__parameters keys differ; missing=['arm_2'] unknown=[]"),
    ('invalid_unknown_parameter.yaml', IMPEDANCE, 'both',
     "ros__parameters keys differ; missing=[] unknown=['extra']"),
    ('invalid_missing_k_gains.yaml', IMPEDANCE, 'both',
     "arm_1 keys differ; missing=['k_gains'] unknown=[]"),
    ('invalid_arm_1_not_mapping.yaml', IMPEDANCE, 'both',
     'arm_1 must be a mapping'),
    # --- arm identity and joint names ---------------------------------------
    ('invalid_arm_2_arm_id.yaml', IMPEDANCE, 'both',
     'arm_2 arm_id must be panda2'),
    ('invalid_joint_names_not_canonical.yaml', IMPEDANCE, 'both',
     'arm_1 joint_names are not canonical'),
    ('invalid_joint_names_short.yaml', IMPEDANCE, 'both',
     'joint_names must be a 7-element string list'),
    # --- number lists --------------------------------------------------------
    ('invalid_k_gains_six.yaml', IMPEDANCE, 'both',
     'k_gains must contain exactly seven numbers'),
    ('invalid_k_gains_string.yaml', IMPEDANCE, 'both',
     'k_gains must contain only numbers'),
    ('invalid_k_gains_overflow.yaml', IMPEDANCE, 'both',
     'k_gains must contain only finite numbers'),
    ('invalid_k_gains_negative.yaml', IMPEDANCE, 'both',
     'k_gains must contain finite nonnegative values'),
    ('invalid_d_gains_negative.yaml', IMPEDANCE, 'both',
     'd_gains must contain finite nonnegative values'),
    # --- bounds against the pinned Panda limit policy ------------------------
    ('invalid_max_effort_over_ceiling.yaml', IMPEDANCE, 'both',
     'max_effort must be positive and no greater than policy'),
    ('invalid_position_lower_under_policy.yaml', IMPEDANCE, 'both',
     'position bounds must be inside the Panda policy'),
    ('invalid_fence_inverted.yaml', IMPEDANCE, 'both',
     'position bounds must be inside the Panda policy'),
    ('invalid_max_target_velocity_over_ceiling.yaml', IMPEDANCE, 'both',
     'max_target_velocity must be positive and no greater than policy'),
    ('invalid_max_target_velocity_zero.yaml', IMPEDANCE, 'both',
     'max_target_velocity must be positive and no greater than policy'),
    # --- reviewed timing ------------------------------------------------------
    ('invalid_watchdog_timeout.yaml', IMPEDANCE, 'both',
     'watchdog_timeout must equal the reviewed policy value'),
    ('invalid_max_header_age.yaml', IMPEDANCE, 'both',
     'max_header_age must equal the reviewed policy value'),
    ('invalid_future_tolerance.yaml', IMPEDANCE, 'both',
     'future_tolerance must equal the reviewed policy value'),
    ('invalid_watchdog_not_numeric.yaml', IMPEDANCE, 'both',
     'watchdog_timeout must be numeric'),
    # --- the hold controller's narrower key set ------------------------------
    ('invalid_hold_joint_names_present.yaml', HOLD, 'both',
     "arm_1 keys differ; missing=[] unknown=['joint_names']"),
    ('invalid_hold_max_effort_zero.yaml', HOLD, 'both',
     'max_effort must be positive and no greater than policy'),
    ('invalid_single_hold_missing_d_gains.yaml', HOLD, 'panda1',
     "arm_1 keys differ; missing=['d_gains'] unknown=[]"),
    # --- the one-arm layout ---------------------------------------------------
    ('invalid_single_missing_arm_count.yaml', IMPEDANCE, 'panda1',
     "ros__parameters keys differ; missing=['arm_count'] unknown=[]"),
    ('invalid_single_arm_2_present.yaml', IMPEDANCE, 'panda1',
     "ros__parameters keys differ; missing=[] unknown=['arm_2']"),
    ('invalid_single_arm_count_two.yaml', IMPEDANCE, 'panda1',
     'arm_count must be exactly 1'),
    ('invalid_single_arm_id_mismatch.yaml', IMPEDANCE, 'panda2',
     'arm_1 arm_id must be panda2'),
)

FIRST_MOMENT = datetime(2026, 8, 30, 14, 12, 0, 500000, tzinfo=timezone.utc)


def read_fixture(name):
    """Return the raw bytes of one fixture in ``test/support/sample_gains``."""
    with open(os.path.join(SAMPLE_DIR, name), 'rb') as handle:
        return handle.read()


class StepClock:
    """A UTC clock that advances one second per reading, for stable ordering."""

    def __init__(self, start=FIRST_MOMENT):
        """Start at ``start``."""
        self._now = start

    def __call__(self):
        """Return the current moment, then advance one second."""
        moment = self._now
        self._now = self._now + timedelta(seconds=1)
        return moment


@pytest.fixture
def store(tmp_path):
    """Build a store rooted at ``tmp_path`` with a deterministic upload clock."""
    return GainsStore(str(tmp_path), utcnow=StepClock())


@pytest.fixture
def strict_umask():
    """Pin the process umask to 0022 for mode assertions, then restore it."""
    previous = os.umask(0o022)
    try:
        yield
    finally:
        os.umask(previous)


def make_gains_dir(tmp_path, mode=0o700):
    """Pre-create the store's gains directory with an explicit mode."""
    path = tmp_path / config.GAINS_DIR_NAME
    path.mkdir()
    os.chmod(str(path), mode)
    return path


# --------------------------------------------------------------------------
# the fixture corpus itself
# --------------------------------------------------------------------------


def test_fixture_corpus_is_complete_and_covers_at_least_twenty_reasons():
    """Every fixture file is claimed by a table, and the invalid set is broad."""
    on_disk = set(os.listdir(SAMPLE_DIR))
    claimed = ({name for name, _, _ in VALID_CASES} |
               {name for name, _, _, _ in INVALID_CASES} |
               {NOT_UTF8_FIXTURE})
    assert on_disk == claimed
    assert len(INVALID_CASES) >= 20
    assert len({name for name, _, _, _ in INVALID_CASES}) == len(INVALID_CASES)
    # "a different reason each" is the point of the corpus; the only repeated
    # sentences are the pairs where one validator sentence covers two distinct
    # branches (over-ceiling vs non-positive, out-of-policy vs inverted fence).
    assert len({detail for _, _, _, detail in INVALID_CASES}) >= 20


def test_every_fixture_is_within_the_upload_size_cap():
    """No fixture is refused for size before its own reason can fire."""
    oversize = {
        name for name in os.listdir(SAMPLE_DIR)
        if os.path.getsize(os.path.join(SAMPLE_DIR, name)) > config.MAX_GAINS_BYTES
    }
    assert oversize == set()


# --------------------------------------------------------------------------
# valid uploads
# --------------------------------------------------------------------------


def test_valid_dual_impedance_upload_matches_section_6_5(store):
    """The dual impedance response carries every section 6.5 field and fence."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    body = record.response()
    assert set(body) == {
        'config_sha256', 'controller_name', 'controller_type', 'path', 'uploaded_at',
        'arms', 'command_interfaces', 'state_interfaces', 'fence',
    }
    assert body['controller_name'] == IMPEDANCE
    assert body['controller_type'] == IMPEDANCE_TYPE
    assert body['arms'] == ['panda1', 'panda2']
    assert body['uploaded_at'] == '2026-08-30T14:12:00.500000Z'
    assert len(body['command_interfaces']) == 14
    assert body['command_interfaces'][0] == 'panda1_joint1/effort'
    assert len(body['state_interfaces']) == 32
    assert 'panda2/robot_model' in body['state_interfaces']
    assert set(body['fence']) == {'panda1', 'panda2'}
    for arm_id in ('panda1', 'panda2'):
        assert body['fence'][arm_id] == {
            'position_lower': list(POSITION_LOWER),
            'position_upper': list(POSITION_UPPER),
            'max_target_velocity': list(MAX_TARGET_VELOCITY),
            'k_gains': list(K_GAINS),
            'd_gains': list(D_GAINS),
            'max_effort': list(MAX_EFFORT),
        }


def test_valid_dual_hold_upload_reports_null_position_and_velocity(store):
    """The hold controller configures no fence, so section 6.5 reports nulls."""
    record = store.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    body = record.response()
    assert body['controller_type'] == HOLD_TYPE
    assert body['arms'] == ['panda1', 'panda2']
    assert len(body['command_interfaces']) == 14
    assert len(body['state_interfaces']) == 32
    for arm_id in ('panda1', 'panda2'):
        assert body['fence'][arm_id] == {
            'position_lower': None,
            'position_upper': None,
            'max_target_velocity': None,
            'k_gains': list(K_GAINS),
            'd_gains': list(D_GAINS),
            'max_effort': list(MAX_EFFORT),
        }


@pytest.mark.parametrize('name,controller_name,arms', VALID_CASES)
def test_valid_fixtures_upload_and_are_stored_under_their_hash(
        store, name, controller_name, arms):
    """Each valid fixture validates, lands on disk, and reports its own hash."""
    raw = read_fixture(name)
    record = store.upload(raw, controller_name, arms)
    assert isinstance(record, StoredGains)
    assert record.controller_name == controller_name
    assert record.arms == ARM_SELECTIONS[arms]
    assert record.config_sha256 == hashlib.sha256(raw).hexdigest()
    assert record.path == os.path.join(store.gains_dir, record.config_sha256 + '.yaml')
    with open(record.path, 'rb') as handle:
        assert handle.read() == raw
    assert set(record.fence) == set(ARM_SELECTIONS[arms])
    for limits in record.fence.values():
        assert tuple(limits) == FENCE_KEYS


@pytest.mark.parametrize('name,controller_name,arms', [
    case for case in VALID_CASES if case[2] != 'both'])
def test_single_arm_uploads_report_only_the_requested_arm(
        store, name, controller_name, arms):
    """A one-arm config's arm_1 block is the CHOSEN arm, not always panda1."""
    record = store.upload(read_fixture(name), controller_name, arms)
    assert record.arms == (arms,)
    assert set(record.fence) == {arms}
    assert len(record.command_interfaces) == 7
    assert all(interface.startswith(arms + '_') for interface in record.command_interfaces)
    assert record.state_interfaces[-2:] == (arms + '/robot_state', arms + '/robot_model')


@pytest.mark.parametrize('name,controller_name,arms', VALID_CASES)
def test_fence_arms_follow_the_configs_own_arm_ids(store, name, controller_name, arms):
    """
    Each fence entry is keyed by the arm_id its own config block declares.

    Slot-to-arm is the one mapping a Motion card cannot get wrong quietly: a
    fence attached to the wrong arm would clamp jogs against the other robot's
    limits, so it is checked against the file rather than assumed positional.
    """
    raw = read_fixture(name)
    document = validator.load_strict_yaml(raw.decode('utf-8'))
    parameters = document['/' + controller_name]['ros__parameters']
    slots = ('arm_1', 'arm_2') if arms == 'both' else ('arm_1',)

    record = store.upload(raw, controller_name, arms)
    assert set(record.fence) == {parameters[slot]['arm_id'] for slot in slots}
    for slot in slots:
        arm = parameters[slot]
        limits = record.fence[arm['arm_id']]
        assert limits['k_gains'] == tuple(arm['k_gains'])
        assert limits['d_gains'] == tuple(arm['d_gains'])
        assert limits['max_effort'] == tuple(arm['max_effort'])
        for key in ('position_lower', 'position_upper', 'max_target_velocity'):
            expected = arm.get(key)
            assert limits[key] == (None if expected is None else tuple(expected))


def test_max_target_velocity_is_read_from_the_config_not_pinned(store):
    """Nothing pins max_target_velocity at 0.1; the fence reports what was sent."""
    raw = read_fixture('valid_dual_impedance.yaml').replace(
        b'max_target_velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]',
        b'max_target_velocity: [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]')
    record = store.upload(raw, IMPEDANCE, 'both')
    assert record.fence['panda1']['max_target_velocity'] == (2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5)
    assert record.fence['panda2']['max_target_velocity'] == (2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5)


# --------------------------------------------------------------------------
# the single-arm path the CLI cannot reach (plan section 0.7, hazard 3)
# --------------------------------------------------------------------------


def test_single_arm_config_needs_the_in_process_validator(store, tmp_path, capsys):
    """
    Prove the CLI cannot validate a one-arm config, and that this store can.

    ``franka_validate_controller_config`` has no ``--arm-id`` argument at all,
    so every file it is given goes through the two-arm entry point and a
    one-arm config is rejected for the ``arm_count``/``arm_2`` mismatch. The
    same bytes validate in-process when the chosen arm is supplied.
    """
    raw = read_fixture('valid_single_impedance_panda1.yaml')
    path = tmp_path / 'single_arm_config.yaml'
    path.write_bytes(raw)

    with pytest.raises(SystemExit):
        validator.main([str(path), '--controller-name', IMPEDANCE, '--arm-id', 'panda1'])
    capsys.readouterr()

    assert validator.main([str(path), '--controller-name', IMPEDANCE]) == 2
    reported = json.loads(capsys.readouterr().err)
    assert reported == {
        'ok': False,
        'error': "ros__parameters keys differ; missing=['arm_2'] unknown=['arm_count']",
    }

    record = store.upload(raw, IMPEDANCE, 'panda1')
    assert record.arms == ('panda1',)
    assert record.controller_type == IMPEDANCE_TYPE


def test_gains_module_never_shells_out_to_the_validator():
    """
    The module's whole import surface, pinned (hazard 3).

    Shelling out to ``franka_validate_controller_config`` would silently drop
    every one-arm session's config on the floor, so the absence of any process
    machinery here is asserted structurally rather than by grepping prose.
    """
    import franka_web.gains as gains_module
    with open(gains_module.__file__, 'r', encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    imported = {
        alias.name.split('.')[0]
        for node in ast.walk(tree) if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported |= {
        node.module.split('.')[0]
        for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module
    }
    assert imported == {
        'dataclasses', 'datetime', 'errno', 'hashlib', 'os', 'stat', 'types',
        'franka_bringup', 'franka_web',
    }
    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert not attributes & {
        'system', 'popen', 'Popen', 'run', 'call', 'check_output', 'execv', 'fork', 'spawnv'}
    assert not hasattr(gains_module, 'subprocess')


# --------------------------------------------------------------------------
# every invalid fixture -> gains_invalid, with the validator's own sentence
# --------------------------------------------------------------------------


@pytest.mark.parametrize('name,controller_name,arms,detail', INVALID_CASES)
def test_invalid_fixture_is_refused_with_the_validator_message(
        store, name, controller_name, arms, detail):
    """The code is gains_invalid and the detail is the validator's, verbatim."""
    raw = read_fixture(name)
    with pytest.raises(GainsError) as refusal:
        store.upload(raw, controller_name, arms)
    assert refusal.value.code == 'gains_invalid'
    assert refusal.value.detail == detail

    # ...and verbatim really means verbatim: the same sentence the validator
    # itself raises on the same bytes, with nothing added around it.
    text = raw.decode('utf-8')
    with pytest.raises(validator.ControllerConfigError) as direct:
        if arms == 'both':
            validator.validate_controller_config_text(text, controller_name)
        else:
            validator.validate_single_controller_config_text(text, controller_name, arms)
    assert refusal.value.detail == str(direct.value)
    assert str(refusal.value) == detail


@pytest.mark.parametrize('name,controller_name,arms,detail', INVALID_CASES)
def test_invalid_fixture_stores_nothing(tmp_path, name, controller_name, arms, detail):
    """A refused upload leaves no file and no listing entry behind."""
    fresh = GainsStore(str(tmp_path), utcnow=StepClock())
    with pytest.raises(GainsError):
        fresh.upload(read_fixture(name), controller_name, arms)
    assert fresh.entries() == []
    assert not os.path.exists(fresh.gains_dir) or os.listdir(fresh.gains_dir) == []


def test_non_utf8_body_is_gains_invalid_before_the_validator_sees_it(store):
    """A body that is not UTF-8 is refused with franka_web's own sentence."""
    raw = read_fixture(NOT_UTF8_FIXTURE)
    with pytest.raises(UnicodeDecodeError):
        raw.decode('utf-8')
    with pytest.raises(GainsError) as refusal:
        store.upload(raw, IMPEDANCE, 'both')
    assert refusal.value.code == 'gains_invalid'
    assert refusal.value.detail == NOT_UTF8_DETAIL
    assert store.entries() == []


# --------------------------------------------------------------------------
# the cheap refusals, in their fixed order
# --------------------------------------------------------------------------


def test_size_cap_is_the_validators_own_and_the_boundary_is_exact(store):
    """Exactly MAX_GAINS_BYTES is accepted; one byte more is gains_too_large."""
    assert config.MAX_GAINS_BYTES == validator.MAXIMUM_CONFIG_BYTES
    base = read_fixture('valid_dual_impedance.yaml')
    padding = config.MAX_GAINS_BYTES - len(base) - 2
    at_cap = base + b'#' + b'p' * padding + b'\n'
    assert len(at_cap) == config.MAX_GAINS_BYTES

    record = store.upload(at_cap, IMPEDANCE, 'both')
    assert record.config_sha256 == hashlib.sha256(at_cap).hexdigest()

    with pytest.raises(GainsError) as refusal:
        store.upload(at_cap + b'\n', IMPEDANCE, 'both')
    assert refusal.value.code == 'gains_too_large'
    assert str(config.MAX_GAINS_BYTES) in refusal.value.detail
    assert store.entries() == [record.summary()]


def test_size_is_checked_before_anything_else(store):
    """An oversized body is never decoded, so its content cannot matter."""
    with pytest.raises(GainsError) as refusal:
        store.upload(b'\xff' * (config.MAX_GAINS_BYTES + 1), 'not_a_controller', 'nonsense')
    assert refusal.value.code == 'gains_too_large'


def test_velocity_controller_is_not_reviewed_for_the_web(store):
    """The reviewed-but-excluded velocity controller is refused by name."""
    assert VELOCITY in validator.REVIEWED_CONTROLLERS
    assert VELOCITY not in config.WEB_CONTROLLERS
    with pytest.raises(GainsError) as refusal:
        store.upload(read_fixture('valid_dual_impedance.yaml'), VELOCITY, 'both')
    assert refusal.value.code == 'controller_not_reviewed'
    assert VELOCITY in refusal.value.detail


@pytest.mark.parametrize('controller_name', ['', 'joint_state_broadcaster', IMPEDANCE.upper()])
def test_unknown_controller_names_are_refused(store, controller_name):
    """Only the two web controllers are accepted, spelled exactly."""
    with pytest.raises(GainsError) as refusal:
        store.upload(read_fixture('valid_dual_impedance.yaml'), controller_name, 'both')
    assert refusal.value.code == 'controller_not_reviewed'


@pytest.mark.parametrize('arms', ['', 'panda3', 'BOTH', 'panda1,panda2', 'all'])
def test_invalid_arm_selections_are_refused(store, arms):
    """`arms` is exactly panda1, panda2 or both."""
    with pytest.raises(GainsError) as refusal:
        store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, arms)
    assert refusal.value.code == 'invalid_arms'


def test_controller_allowlist_is_checked_before_the_arm_selection(store):
    """The refusal order is fixed, so the error an operator sees is stable."""
    with pytest.raises(GainsError) as refusal:
        store.upload(read_fixture('valid_dual_impedance.yaml'), VELOCITY, 'panda3')
    assert refusal.value.code == 'controller_not_reviewed'


def test_a_text_body_is_a_programming_error_not_a_gains_error(store):
    """Bodies are raw bytes; a str is the caller's bug, not the operator's."""
    with pytest.raises(TypeError):
        store.upload(read_fixture('valid_dual_impedance.yaml').decode('utf-8'), IMPEDANCE, 'both')


def test_error_codes_stay_inside_the_closed_section_6_14_set(store):
    """Every GainsError this module can raise carries a contract code."""
    allowed = {
        'gains_too_large', 'gains_invalid', 'controller_not_reviewed', 'invalid_arms',
        'gains_unknown', 'gains_controller_mismatch', 'gains_arms_mismatch',
    }
    seen = set()
    attempts = (
        (b'x' * (config.MAX_GAINS_BYTES + 1), IMPEDANCE, 'both'),
        (read_fixture(NOT_UTF8_FIXTURE), IMPEDANCE, 'both'),
        (read_fixture('valid_dual_impedance.yaml'), VELOCITY, 'both'),
        (read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'panda3'),
        (read_fixture('invalid_empty.yaml'), IMPEDANCE, 'both'),
    )
    for raw, controller_name, arms in attempts:
        with pytest.raises(GainsError) as refusal:
            store.upload(raw, controller_name, arms)
        seen.add(refusal.value.code)
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    for sha, controller_name, arms in (
            ('0' * 64, IMPEDANCE, 'both'),
            (record.config_sha256, HOLD, 'both'),
            (record.config_sha256, IMPEDANCE, 'panda1'),
            (record.config_sha256, IMPEDANCE, 'nonsense')):
        with pytest.raises(GainsError) as refusal:
            store.match(sha, controller_name, arms)
        seen.add(refusal.value.code)
    assert seen == allowed


# --------------------------------------------------------------------------
# the store on disk
# --------------------------------------------------------------------------


def test_store_creates_a_private_directory_and_private_files(store, strict_umask):
    """The gains directory is 0700 and every stored config is 0600."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert stat.S_IMODE(os.stat(store.gains_dir).st_mode) == 0o700
    assert stat.S_IMODE(os.stat(record.path).st_mode) == 0o600
    assert os.path.basename(store.gains_dir) == config.GAINS_DIR_NAME


@pytest.mark.parametrize('umask', [0o022, 0o077, 0o777])
def test_modes_hold_under_any_umask(tmp_path, umask):
    """A hostile umask cannot leave the store unreadable or world-readable."""
    previous = os.umask(umask)
    try:
        fresh = GainsStore(str(tmp_path), utcnow=StepClock())
        record = fresh.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    finally:
        os.umask(previous)
    assert stat.S_IMODE(os.stat(fresh.gains_dir).st_mode) == 0o700
    assert stat.S_IMODE(os.stat(record.path).st_mode) == 0o600


def test_stored_path_is_absolute_normalized_and_symlink_free(store):
    """The path handed to controller_param_file is safe to pass straight on."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert os.path.isabs(record.path)
    assert os.path.normpath(record.path) == record.path
    assert os.path.realpath(record.path) == record.path
    assert not os.path.islink(record.path)
    assert re.fullmatch(r'[0-9a-f]{64}\.yaml', os.path.basename(record.path))


@pytest.mark.parametrize('state_dir', ['relative/state', 'state', '/a/../b', ''])
def test_a_relative_or_unnormalized_state_dir_is_refused(state_dir):
    """The store never resolves a path it was not given in normalized form."""
    with pytest.raises(ValueError):
        GainsStore(state_dir)


def test_double_upload_of_the_same_bytes_is_idempotent(store):
    """Same bytes, same hash, same record -- one file, one listing entry."""
    raw = read_fixture('valid_dual_impedance.yaml')
    first = store.upload(raw, IMPEDANCE, 'both')
    second = store.upload(raw, IMPEDANCE, 'both')
    assert second is first
    assert second.uploaded_at == first.uploaded_at
    assert os.listdir(store.gains_dir) == [first.config_sha256 + '.yaml']
    assert store.entries() == [first.summary()]


def test_a_second_store_over_the_same_directory_adopts_the_existing_file(tmp_path):
    """A restart re-uploading the same config finds its own bytes and agrees."""
    first = GainsStore(str(tmp_path), utcnow=StepClock())
    raw = read_fixture('valid_single_impedance_panda2.yaml')
    original = first.upload(raw, IMPEDANCE, 'panda2')

    second = GainsStore(str(tmp_path), utcnow=StepClock())
    adopted = second.upload(raw, IMPEDANCE, 'panda2')
    assert adopted.path == original.path
    assert adopted.config_sha256 == original.config_sha256
    assert os.listdir(tmp_path / config.GAINS_DIR_NAME) == [original.config_sha256 + '.yaml']


def test_a_symlinked_gains_directory_is_refused(tmp_path):
    """The gains directory is opened with O_NOFOLLOW: a symlink is not a door."""
    elsewhere = tmp_path / 'elsewhere'
    elsewhere.mkdir()
    os.symlink(str(elsewhere), str(tmp_path / config.GAINS_DIR_NAME))
    fresh = GainsStore(str(tmp_path), utcnow=StepClock())
    with pytest.raises(OSError):
        fresh.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert os.listdir(str(elsewhere)) == []


def test_a_symlink_squatting_the_target_path_is_refused(tmp_path, store):
    """O_EXCL|O_NOFOLLOW never writes through a planted symlink."""
    raw = read_fixture('valid_dual_impedance.yaml')
    sha = hashlib.sha256(raw).hexdigest()
    target = tmp_path / 'victim.txt'
    target.write_bytes(b'untouched\n')
    make_gains_dir(tmp_path)
    os.symlink(str(target), os.path.join(str(tmp_path), config.GAINS_DIR_NAME, sha + '.yaml'))

    with pytest.raises(OSError):
        store.upload(raw, IMPEDANCE, 'both')
    assert target.read_bytes() == b'untouched\n'
    assert store.entries() == []


def test_foreign_content_under_a_content_addressed_name_is_refused(tmp_path, store):
    """A name that does not hold the bytes it names is never trusted or fixed."""
    raw = read_fixture('valid_dual_impedance.yaml')
    sha = hashlib.sha256(raw).hexdigest()
    gains_dir = make_gains_dir(tmp_path)
    squatter = gains_dir / (sha + '.yaml')
    squatter.write_bytes(b'not the config that hashes to this name\n')

    with pytest.raises(OSError):
        store.upload(raw, IMPEDANCE, 'both')
    assert squatter.read_bytes() == b'not the config that hashes to this name\n'
    assert store.entries() == []


def test_a_shared_gains_directory_is_refused(tmp_path, store):
    """Group or other permission bits mean somebody else has been writing here."""
    make_gains_dir(tmp_path, mode=0o777)
    with pytest.raises(OSError):
        store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')


def test_an_unsearchable_gains_directory_is_refused(tmp_path, store):
    """A pre-existing directory missing an owner bit is refused, not repaired."""
    gains_dir = make_gains_dir(tmp_path, mode=0o600)
    try:
        with pytest.raises(OSError):
            store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    finally:
        os.chmod(str(gains_dir), 0o700)


# --------------------------------------------------------------------------
# listing, lookup and matching
# --------------------------------------------------------------------------


def test_entries_are_summaries_newest_first(store):
    """GET /api/gains lists the section 6.6 fields, most recent upload first."""
    first = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    second = store.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    third = store.upload(read_fixture('valid_single_hold_panda1.yaml'), HOLD, 'panda1')

    entries = store.entries()
    assert [entry['config_sha256'] for entry in entries] == [
        third.config_sha256, second.config_sha256, first.config_sha256]
    assert entries[0] == {
        'config_sha256': third.config_sha256,
        'controller_name': HOLD,
        'arms': ['panda1'],
        'uploaded_at': third.uploaded_at,
        'path': third.path,
    }
    assert [entry['uploaded_at'] for entry in entries] == [
        '2026-08-30T14:12:02.500000Z',
        '2026-08-30T14:12:01.500000Z',
        '2026-08-30T14:12:00.500000Z',
    ]


def test_re_uploading_does_not_reorder_the_listing(store):
    """An idempotent re-upload is a no-op, including for ordering."""
    first = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    second = store.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert [entry['config_sha256'] for entry in store.entries()] == [
        second.config_sha256, first.config_sha256]


def test_entries_are_empty_before_any_upload(store):
    """A fresh store lists nothing and creates nothing."""
    assert store.entries() == []
    assert not os.path.exists(store.gains_dir)


@pytest.mark.parametrize('sha', ['', '0' * 64, 'not-a-hash', None, 42, ['a']])
def test_get_returns_none_for_anything_unknown(store, sha):
    """`get` never raises, whatever the page sent as a sha256."""
    store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert store.get(sha) is None


def test_get_returns_the_record_for_a_known_hash(store):
    """A known hash returns the very record the upload produced."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert store.get(record.config_sha256) is record


def test_match_accepts_the_configuration_it_validated(store):
    """The happy path: same hash, same controller, same arm layout."""
    record = store.upload(read_fixture('valid_single_hold_panda2.yaml'), HOLD, 'panda2')
    assert store.match(record.config_sha256, HOLD, 'panda2') is record


def test_match_refuses_a_stored_object_mutated_after_upload(store):
    """A cached hash cannot authorize different bytes placed under its name."""
    record = store.upload(
        read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    replacement = read_fixture('valid_dual_impedance.yaml').replace(
        b'max_target_velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]',
        b'max_target_velocity: [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]')
    assert hashlib.sha256(replacement).hexdigest() != record.config_sha256
    with open(record.path, 'wb') as handle:
        handle.write(replacement)

    with pytest.raises(OSError):
        store.match(record.config_sha256, IMPEDANCE, 'both')


def test_match_refuses_a_stored_object_removed_after_upload(store):
    """A record whose content-addressed file vanished fails loud and closed."""
    record = store.upload(
        read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    os.unlink(record.path)

    with pytest.raises(OSError):
        store.match(record.config_sha256, IMPEDANCE, 'both')


def test_match_refuses_a_symlink_replacing_the_stored_object(store, tmp_path):
    """Match reopens with O_NOFOLLOW and never follows a replacement symlink."""
    record = store.upload(
        read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    target = tmp_path / 'replacement.yaml'
    target.write_bytes(read_fixture('valid_dual_impedance.yaml'))
    os.unlink(record.path)
    os.symlink(str(target), record.path)

    with pytest.raises(OSError):
        store.match(record.config_sha256, IMPEDANCE, 'both')


def test_match_refuses_an_oversized_stored_object_after_upload(store):
    """Even matching-prefix content cannot bypass the fixed stored-object cap."""
    record = store.upload(
        read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with open(record.path, 'wb') as handle:
        handle.write(b'x' * (config.MAX_GAINS_BYTES + 1))

    with pytest.raises(OSError):
        store.match(record.config_sha256, IMPEDANCE, 'both')


def test_match_refuses_an_unknown_hash(store):
    """A stale page's sha256 is gains_unknown, not a silent miss."""
    store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with pytest.raises(GainsError) as refusal:
        store.match('f' * 64, IMPEDANCE, 'both')
    assert refusal.value.code == 'gains_unknown'


def test_match_refuses_the_other_controllers_configuration(store):
    """A hold config cannot start an impedance session, or the reverse."""
    hold = store.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    impedance = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with pytest.raises(GainsError) as refusal:
        store.match(hold.config_sha256, IMPEDANCE, 'both')
    assert refusal.value.code == 'gains_controller_mismatch'
    assert HOLD in refusal.value.detail
    with pytest.raises(GainsError) as reverse:
        store.match(impedance.config_sha256, HOLD, 'both')
    assert reverse.value.code == 'gains_controller_mismatch'


@pytest.mark.parametrize('name,uploaded_for,requested', [
    ('valid_dual_impedance.yaml', 'both', 'panda1'),
    ('valid_dual_impedance.yaml', 'both', 'panda2'),
    ('valid_single_impedance_panda1.yaml', 'panda1', 'both'),
    ('valid_single_impedance_panda1.yaml', 'panda1', 'panda2'),
    ('valid_single_impedance_panda2.yaml', 'panda2', 'panda1'),
])
def test_match_refuses_a_configuration_built_for_other_arms(
        store, name, uploaded_for, requested):
    """A dual config aimed at a one-arm session (and back) is gains_arms_mismatch."""
    record = store.upload(read_fixture(name), IMPEDANCE, uploaded_for)
    with pytest.raises(GainsError) as refusal:
        store.match(record.config_sha256, IMPEDANCE, requested)
    assert refusal.value.code == 'gains_arms_mismatch'
    assert '+'.join(ARM_SELECTIONS[uploaded_for]) in refusal.value.detail


def test_match_refuses_an_illegal_arm_selection(store):
    """An `arms` value outside the closed set is invalid_arms, not a mismatch."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with pytest.raises(GainsError) as refusal:
        store.match(record.config_sha256, IMPEDANCE, 'panda3')
    assert refusal.value.code == 'invalid_arms'


def test_match_checks_the_hash_before_the_controller_and_arms(store):
    """An unknown hash is reported as unknown even when everything else is wrong."""
    store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with pytest.raises(GainsError) as refusal:
        store.match('a' * 64, HOLD, 'panda1')
    assert refusal.value.code == 'gains_unknown'


# --------------------------------------------------------------------------
# the record itself
# --------------------------------------------------------------------------


def test_summary_is_exactly_the_section_6_6_entry(store):
    """GET /api/gains carries five fields and no fence or interface lists."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert set(record.summary()) == {
        'config_sha256', 'controller_name', 'arms', 'uploaded_at', 'path'}
    assert record.summary()['arms'] == ['panda1', 'panda2']


def test_response_is_the_summary_plus_the_section_6_5_extras(store):
    """POST /api/gains adds the type, the interface lists and the fence."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    body = record.response()
    summary = record.summary()
    assert set(body) - set(summary) == {
        'controller_type', 'command_interfaces', 'state_interfaces', 'fence'}
    for key, value in summary.items():
        assert body[key] == value
    assert 'ok' not in body


def test_the_response_is_json_serializable_and_stable(store):
    """The body goes out as JSON; nothing in it is a tuple or a proxy."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    encoded = json.dumps(record.response(), sort_keys=True, allow_nan=False)
    assert json.loads(encoded) == record.response()
    assert json.dumps(record.response(), sort_keys=True) == encoded


def test_a_stored_record_cannot_be_edited_in_place(store):
    """The record is frozen all the way down, fence included."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    with pytest.raises(Exception):
        record.path = '/tmp/elsewhere.yaml'
    with pytest.raises(TypeError):
        record.fence['panda1'] = {}
    with pytest.raises(TypeError):
        record.fence['panda1']['k_gains'] = (0.0,) * 7


def test_mutating_a_response_does_not_reach_the_store(store):
    """Every caller gets its own copy of the wire body."""
    record = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    body = record.response()
    body['fence']['panda1']['k_gains'][0] = 999.0
    body['arms'].append('panda3')
    body['command_interfaces'].clear()
    assert record.fence['panda1']['k_gains'] == K_GAINS
    assert record.arms == ('panda1', 'panda2')
    assert record.response()['fence']['panda1']['k_gains'] == list(K_GAINS)
    assert record.response()['arms'] == ['panda1', 'panda2']
    assert len(record.response()['command_interfaces']) == 14


def test_uploaded_at_is_rfc3339_utc_with_microseconds(tmp_path):
    """The timestamp format is the one section 6.5 shows, to the microsecond."""
    fresh = GainsStore(str(tmp_path))
    record = fresh.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{6}Z', record.uploaded_at)
    parsed = datetime.strptime(record.uploaded_at, '%Y-%m-%dT%H:%M:%S.%fZ')
    parsed = parsed.replace(tzinfo=timezone.utc)
    assert abs((parsed - datetime.now(timezone.utc)).total_seconds()) < 120.0


def test_the_fence_key_set_is_the_section_6_5_one(store):
    """Six per-arm vectors, in a fixed order, present for every controller."""
    assert FENCE_KEYS == (
        'position_lower', 'position_upper', 'max_target_velocity',
        'k_gains', 'd_gains', 'max_effort')
    impedance = store.upload(read_fixture('valid_dual_impedance.yaml'), IMPEDANCE, 'both')
    hold = store.upload(read_fixture('valid_dual_hold.yaml'), HOLD, 'both')
    for record in (impedance, hold):
        for limits in record.fence.values():
            assert tuple(limits) == FENCE_KEYS
            for values in limits.values():
                assert values is None or len(values) == config.JOINT_COUNT


def test_the_arm_selection_table_is_the_section_6_5_one():
    """`arms` is a closed set of three, and `both` is ordered panda1, panda2."""
    assert dict(ARM_SELECTIONS) == {
        'panda1': ('panda1',),
        'panda2': ('panda2',),
        'both': ('panda1', 'panda2'),
    }
    with pytest.raises(TypeError):
        ARM_SELECTIONS['panda3'] = ('panda3',)
