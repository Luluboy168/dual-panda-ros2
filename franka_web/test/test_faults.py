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
Tests for franka_web.faults: rules F1-F8, recoverability, the F4 window.

Every snapshot here is hand-built in the shape ``health.project_arm`` produces
(plan section 6.11's frame), so these tests pin the fault contract without
depending on the projection module. No robot address appears anywhere: the
rules read health fields only, and none of them carries one.
"""

import dataclasses

from franka_web import config
from franka_web.faults import (
    FAULT_CODES,
    FaultEngine,
    FaultReason,
    FaultSnapshot,
    MODE_MOTION,
    MODE_SIMULATE,
    MODE_WATCH,
    PRODUCTION_MODES,
    RECOVERABLE_FAULT_CODES,
)
import pytest
from support.fake_clock import FakeClock

ALL_MODES = (MODE_SIMULATE, MODE_WATCH, MODE_MOTION)
CONTROLLER = 'dual_arm_joint_impedance_controller'


def make_arm(arm_id='panda1'):
    """Build one healthy arm projection of the health.project_arm shape."""
    return {
        'arm_id': arm_id,
        'status': 'ok',
        'status_line': 'backend state is healthy',
        'joint_names': ['{}_joint{}'.format(arm_id, index) for index in range(1, 8)],
        'positions': [0.0] * 7,
        'velocities': [0.0] * 7,
        'efforts': [0.0] * 7,
        'positions_age_s': 0.031,
        'positions_stale': False,
        'robot_state': {
            'available': True,
            'age_s': 0.041,
            'control_command_success_rate': 0.998,
            'robot_mode': 2,
            'robot_mode_label': 'move',
            'current_errors': [],
            'last_motion_errors': [],
        },
        'diagnostic': {
            'available': True,
            'level': 0,
            'level_label': 'ok',
            'message': 'backend state is healthy',
            'age_s': 0.6,
            'values': {'arm_id': arm_id},
        },
    }


def make_simulated_arm(arm_id='panda1'):
    """Build the degraded arm projection that ``simulate`` really produces."""
    arm = make_arm(arm_id)
    arm['robot_state'] = {
        'available': False,
        'age_s': None,
        'control_command_success_rate': None,
        'robot_mode': None,
        'robot_mode_label': None,
        'current_errors': [],
        'last_motion_errors': [],
    }
    arm['diagnostic'] = {
        'available': False,
        'level': None,
        'level_label': None,
        'message': None,
        'age_s': None,
        'values': {},
    }
    arm['status_line'] = 'simulated hardware; no Franka diagnostics'
    return arm


def make_snapshot(
    mode=MODE_WATCH,
    arms=None,
    controller_name=None,
    controller_states=None,
    hardware_available=True,
    hardware_lifecycle_label='active',
    launch_alive=True,
):
    """Build a snapshot that fires nothing unless a caller poisons a field."""
    if arms is None:
        arms = {'panda1': make_arm('panda1')}
    if controller_states is None:
        controller_states = {'joint_state_broadcaster': 'active'}
        if controller_name:
            controller_states[controller_name] = 'active'
    return FaultSnapshot(
        mode=mode,
        arms=arms,
        controller_name=controller_name,
        controller_states=controller_states,
        hardware_available=hardware_available,
        hardware_lifecycle_label=hardware_lifecycle_label,
        launch_alive=launch_alive,
    )


def codes(reasons):
    """Return the codes of ``reasons`` in order."""
    return [reason.code for reason in reasons]


def only(reasons):
    """Assert exactly one reason fired and return it."""
    assert len(reasons) == 1, codes(reasons)
    return reasons[0]


@pytest.fixture()
def clock():
    """Return a deterministic monotonic clock."""
    return FakeClock()


@pytest.fixture()
def engine(clock):
    """Return an engine whose F4 window is driven by the fake clock."""
    return FaultEngine(monotonic=clock.monotonic)


class TestCodeSet:
    """The code set is closed and its recoverable subset is the plan's."""

    def test_fault_codes_are_exactly_the_eight_rules(self):
        """F1-F8 each own one code and nothing else is in the set."""
        assert FAULT_CODES == frozenset({
            'diagnostic_error',
            'robot_mode_fault',
            'robot_errors',
            'ccsr_low',
            'controller_deactivated',
            'joint_state_stale',
            'launch_exited',
            'hardware_inactive',
        })

    def test_recoverable_codes_are_f1_to_f6_and_f8(self):
        """Full recovery addresses F1-F6 and F8; only F7 needs restart."""
        assert RECOVERABLE_FAULT_CODES == frozenset({
            'diagnostic_error',
            'robot_mode_fault',
            'robot_errors',
            'ccsr_low',
            'controller_deactivated',
            'joint_state_stale',
            'hardware_inactive',
        })
        assert RECOVERABLE_FAULT_CODES < FAULT_CODES

    def test_production_modes_exclude_simulate(self):
        """Only watch and motion talk to real hardware."""
        assert PRODUCTION_MODES == (MODE_WATCH, MODE_MOTION)
        assert MODE_SIMULATE not in PRODUCTION_MODES


class TestHealthySnapshot:
    """A healthy stack fires nothing, in any mode."""

    @pytest.mark.parametrize('mode', ALL_MODES)
    def test_nothing_fires(self, engine, mode):
        """No rule fires on a clean snapshot."""
        arms = {'panda1': make_simulated_arm() if mode == MODE_SIMULATE else make_arm()}
        controller = CONTROLLER if mode == MODE_MOTION else None
        snapshot = make_snapshot(mode=mode, arms=arms, controller_name=controller)
        assert engine.evaluate(snapshot) == []

    def test_default_clock_is_usable(self):
        """A default-constructed engine works without an injected clock."""
        assert FaultEngine().evaluate(make_snapshot()) == []


class TestF1DiagnosticError:
    """F1: any canonical diagnostic at level >= 2 (plan section 0.5)."""

    @pytest.mark.parametrize('mode', PRODUCTION_MODES)
    def test_level_two_fires_per_arm(self, engine, mode):
        """Level 2 on one arm fires exactly one reason, naming that arm."""
        arms = {'panda1': make_arm('panda1'), 'panda2': make_arm('panda2')}
        arms['panda2']['diagnostic']['level'] = 2
        arms['panda2']['diagnostic']['message'] = 'global fault latched'
        reason = only(engine.evaluate(make_snapshot(
            mode=mode, arms=arms, controller_name=CONTROLLER if mode == MODE_MOTION else None)))
        assert reason.code == 'diagnostic_error'
        assert reason.arm_id == 'panda2'
        assert 'global fault latched' in reason.detail

    def test_level_above_error_also_fires(self, engine):
        """The rule is a threshold, not an equality: level 3 fires too."""
        arms = {'panda1': make_arm()}
        arms['panda1']['diagnostic']['level'] = 3
        assert codes(engine.evaluate(make_snapshot(arms=arms))) == ['diagnostic_error']

    @pytest.mark.parametrize('level', [0, 1])
    def test_ok_and_warn_do_not_fire(self, engine, level):
        """OK(0) and WARN(1) are below the franka_status threshold."""
        arms = {'panda1': make_arm()}
        arms['panda1']['diagnostic']['level'] = level
        assert engine.evaluate(make_snapshot(arms=arms)) == []

    def test_absent_level_does_not_fire(self, engine):
        """No canonical status seen is not the same as a healthy arm."""
        arms = {'panda1': make_arm()}
        arms['panda1']['diagnostic'] = {'available': False, 'level': None, 'values': {}}
        assert engine.evaluate(make_snapshot(arms=arms)) == []

    def test_missing_message_still_produces_a_detail(self, engine):
        """A level with no summary string still yields a readable detail."""
        arms = {'panda1': make_arm()}
        arms['panda1']['diagnostic']['level'] = 2
        arms['panda1']['diagnostic']['message'] = None
        reason = only(engine.evaluate(make_snapshot(arms=arms)))
        assert 'no diagnostic summary reported' in reason.detail


class TestF2RobotModeFault:
    """F2: robot_mode in {REFLEX(4), USER_STOPPED(5)} (plan section 0.4)."""

    @pytest.mark.parametrize('robot_mode,label', [(4, 'reflex'), (5, 'user_stopped')])
    def test_fault_modes_fire_and_name_the_mode(self, engine, robot_mode, label):
        """The detail names the mode the operator has to act on."""
        arms = {'panda1': make_arm()}
        arms['panda1']['robot_state']['robot_mode'] = robot_mode
        reason = only(engine.evaluate(make_snapshot(arms=arms)))
        assert reason.code == 'robot_mode_fault'
        assert reason.arm_id == 'panda1'
        assert label in reason.detail
        assert str(robot_mode) in reason.detail

    @pytest.mark.parametrize('robot_mode', [0, 1, 2, 3, 6, None])
    def test_other_modes_do_not_fire(self, engine, robot_mode):
        """Every non-fault robot_mode, including AUTOMATIC_ERROR_RECOVERY."""
        arms = {'panda1': make_arm()}
        arms['panda1']['robot_state']['robot_mode'] = robot_mode
        assert engine.evaluate(make_snapshot(arms=arms)) == []

    def test_fires_once_per_arm(self, engine):
        """Both arms in reflex produce one reason each, in snapshot order."""
        arms = {'panda1': make_arm('panda1'), 'panda2': make_arm('panda2')}
        for arm in arms.values():
            arm['robot_state']['robot_mode'] = 4
        reasons = engine.evaluate(make_snapshot(arms=arms))
        assert codes(reasons) == ['robot_mode_fault', 'robot_mode_fault']
        assert [reason.arm_id for reason in reasons] == ['panda1', 'panda2']


class TestF3RobotErrors:
    """F3: any franka_msgs/Errors field true on the arm."""

    def test_current_errors_fire_and_are_listed(self, engine):
        """The detail lists the error names the projection extracted."""
        arms = {'panda1': make_arm()}
        arms['panda1']['robot_state']['current_errors'] = [
            'joint_position_limits_violation', 'cartesian_reflex']
        reason = only(engine.evaluate(make_snapshot(arms=arms)))
        assert reason.code == 'robot_errors'
        assert reason.arm_id == 'panda1'
        assert 'joint_position_limits_violation' in reason.detail
        assert 'cartesian_reflex' in reason.detail

    def test_empty_current_errors_do_not_fire(self, engine):
        """An empty list is the healthy case, not a fault."""
        arms = {'panda1': make_arm()}
        arms['panda1']['robot_state']['current_errors'] = []
        assert engine.evaluate(make_snapshot(arms=arms)) == []

    def test_last_motion_errors_alone_do_not_fire(self, engine):
        """History is not a live fault; only current_errors fires F3."""
        arms = {'panda1': make_arm()}
        arms['panda1']['robot_state']['last_motion_errors'] = ['joint_reflex']
        assert engine.evaluate(make_snapshot(arms=arms)) == []


class TestF4CcsrWindow:
    """F4: ccsr below the signed gate, sustained past the signed window."""

    def poison(self, ccsr, arm_id='panda1', mode=MODE_MOTION):
        """Build a one-arm snapshot whose ccsr is ``ccsr``."""
        arm = make_arm(arm_id)
        arm['robot_state']['control_command_success_rate'] = ccsr
        return make_snapshot(
            mode=mode, arms={arm_id: arm},
            controller_name=CONTROLLER if mode == MODE_MOTION else None)

    def test_the_gate_is_the_signed_one(self):
        """F4 uses config's user-signed threshold and window, not its own."""
        assert config.CCSR_FAULT_THRESHOLD == 0.95
        assert config.CCSR_FAULT_SUSTAIN_S == 5.0

    def test_brief_dip_does_not_fire(self, engine, clock):
        """A dip shorter than the window is not a fault."""
        low = self.poison(0.90)
        assert engine.evaluate(low) == []
        clock.advance(4.9)
        assert engine.evaluate(low) == []

    def test_exactly_the_window_does_not_fire(self, engine, clock):
        """The rule is 'sustained > 5.0 s'; 5.0 s exactly is not yet a fault."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(config.CCSR_FAULT_SUSTAIN_S)
        assert engine.evaluate(low) == []

    def test_sustained_dip_fires(self, engine, clock):
        """Past the window the rule fires and the detail carries the numbers."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(config.CCSR_FAULT_SUSTAIN_S + 0.1)
        reason = only(engine.evaluate(low))
        assert reason.code == 'ccsr_low'
        assert reason.arm_id == 'panda1'
        assert '0.900' in reason.detail
        assert '0.95' in reason.detail

    def test_watch_zero_is_state_only_while_identical_motion_zero_fires(
            self, engine, clock):
        """Mutation pair: Watch has no command-quality stream; Motion does."""
        watch = self.poison(0.0, mode=MODE_WATCH)
        assert engine.evaluate(watch) == []
        clock.advance(config.CCSR_FAULT_SUSTAIN_S + 0.1)
        assert engine.evaluate(watch) == []

        motion = self.poison(0.0, mode=MODE_MOTION)
        assert engine.evaluate(motion) == []
        clock.advance(config.CCSR_FAULT_SUSTAIN_S + 0.1)
        assert codes(engine.evaluate(motion)) == ['ccsr_low']

    def test_it_keeps_firing_while_the_rate_stays_low(self, engine, clock):
        """The fault is level-triggered, not edge-triggered."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(6.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']
        clock.advance(1.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']

    def test_a_good_sample_resets_the_window(self, engine, clock):
        """Recovery above the threshold closes the window completely."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(4.0)
        assert engine.evaluate(self.poison(0.99)) == []
        clock.advance(4.0)
        assert engine.evaluate(low) == []
        clock.advance(6.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']

    def test_the_threshold_itself_is_not_low(self, engine, clock):
        """A rate equal to the threshold is acceptable and resets."""
        at_gate = self.poison(config.CCSR_FAULT_THRESHOLD)
        engine.evaluate(at_gate)
        clock.advance(60.0)
        assert engine.evaluate(at_gate) == []

    def test_unavailable_robot_state_does_not_fire(self, engine, clock):
        """No FrankaState means no sample, and no sample never fires F4."""
        arm = make_simulated_arm()
        snapshot = make_snapshot(
            mode=MODE_MOTION, arms={'panda1': arm}, controller_name=CONTROLLER)
        engine.evaluate(snapshot)
        clock.advance(60.0)
        assert engine.evaluate(snapshot) == []

    def test_a_gap_in_samples_holds_the_window_open(self, engine, clock):
        """Silence is not evidence of recovery: only a good sample resets."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(3.0)
        assert engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={'panda1': make_simulated_arm()},
            controller_name=CONTROLLER)) == []
        clock.advance(3.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']

    def test_windows_are_per_arm(self, engine, clock):
        """One arm's dip cannot mature another arm's window."""
        first = make_arm('panda1')
        first['robot_state']['control_command_success_rate'] = 0.90
        second = make_arm('panda2')
        engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={'panda1': first, 'panda2': second},
            controller_name=CONTROLLER))
        clock.advance(4.0)
        second['robot_state']['control_command_success_rate'] = 0.90
        engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={'panda1': first, 'panda2': second},
            controller_name=CONTROLLER))
        clock.advance(2.0)
        reasons = engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={'panda1': first, 'panda2': second},
            controller_name=CONTROLLER))
        assert codes(reasons) == ['ccsr_low']
        assert reasons[0].arm_id == 'panda1'
        clock.advance(4.0)
        assert [r.arm_id for r in engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={'panda1': first, 'panda2': second},
            controller_name=CONTROLLER))] == ['panda1', 'panda2']

    def test_reset_clears_the_window(self, engine, clock):
        """A new session starts with no accumulated dip."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(10.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']
        engine.reset()
        assert engine.evaluate(low) == []
        clock.advance(4.0)
        assert engine.evaluate(low) == []

    def test_an_arm_leaving_the_snapshot_drops_its_window(self, engine, clock):
        """A restarted arm is not judged on the window of its predecessor."""
        low = self.poison(0.90)
        engine.evaluate(low)
        clock.advance(10.0)
        assert codes(engine.evaluate(low)) == ['ccsr_low']
        assert engine.evaluate(make_snapshot(
            mode=MODE_MOTION, arms={}, controller_name=CONTROLLER)) == []
        assert engine.evaluate(low) == []

    @pytest.mark.parametrize('ccsr', [None, 'nan', float('nan'), float('inf'), True])
    def test_non_numeric_samples_never_fire(self, engine, clock, ccsr):
        """Nothing that is not a finite number counts as a sample."""
        snapshot = self.poison(ccsr)
        engine.evaluate(snapshot)
        clock.advance(60.0)
        assert engine.evaluate(snapshot) == []


class TestF5ControllerDeactivated:
    """F5: the session controller leaves ``active`` while in motion."""

    @pytest.mark.parametrize('state', ['inactive', 'unconfigured', 'finalized'])
    def test_non_active_controller_fires(self, engine, state):
        """Any non-active lifecycle string is a fault in motion."""
        snapshot = make_snapshot(
            mode=MODE_MOTION, controller_name=CONTROLLER,
            controller_states={'joint_state_broadcaster': 'active', CONTROLLER: state})
        reason = only(engine.evaluate(snapshot))
        assert reason.code == 'controller_deactivated'
        assert reason.arm_id is None
        assert CONTROLLER in reason.detail
        assert state in reason.detail

    def test_a_missing_controller_fires(self, engine):
        """A controller that unloaded entirely is not active either."""
        snapshot = make_snapshot(
            mode=MODE_MOTION, controller_name=CONTROLLER,
            controller_states={'joint_state_broadcaster': 'active'})
        reason = only(engine.evaluate(snapshot))
        assert reason.code == 'controller_deactivated'
        assert 'not loaded' in reason.detail

    def test_active_controller_does_not_fire(self, engine):
        """The healthy motion case."""
        snapshot = make_snapshot(mode=MODE_MOTION, controller_name=CONTROLLER)
        assert engine.evaluate(snapshot) == []

    @pytest.mark.parametrize('mode', [MODE_SIMULATE, MODE_WATCH])
    def test_non_motion_modes_never_fire(self, engine, mode):
        """Watch and simulate run no session controller, so F5 cannot fire."""
        arms = {'panda1': make_simulated_arm() if mode == MODE_SIMULATE else make_arm()}
        snapshot = make_snapshot(
            mode=mode, arms=arms, controller_name=CONTROLLER,
            controller_states={CONTROLLER: 'inactive'})
        assert engine.evaluate(snapshot) == []

    def test_no_session_controller_is_not_evaluated(self, engine):
        """With no controller named there is nothing for the rule to read."""
        snapshot = make_snapshot(
            mode=MODE_MOTION, controller_name=None,
            controller_states={'joint_state_broadcaster': 'active'})
        assert engine.evaluate(snapshot) == []


class TestF6JointStateStale:
    """F6: the joint stream went stale (all modes)."""

    @pytest.mark.parametrize('mode', ALL_MODES)
    def test_stale_fires_in_every_mode(self, engine, mode):
        """The joint stream exists in simulate too, so F6 is not mode-gated."""
        arm = make_simulated_arm() if mode == MODE_SIMULATE else make_arm()
        arm['positions_stale'] = True
        arm['positions_age_s'] = 2.5
        controller = CONTROLLER if mode == MODE_MOTION else None
        reason = only(engine.evaluate(make_snapshot(
            mode=mode, arms={'panda1': arm}, controller_name=controller)))
        assert reason.code == 'joint_state_stale'
        assert reason.arm_id == 'panda1'
        assert '2.50' in reason.detail

    def test_fires_only_for_the_stale_arm(self, engine):
        """One stale arm does not implicate its healthy neighbour."""
        arms = {'panda1': make_arm('panda1'), 'panda2': make_arm('panda2')}
        arms['panda2']['positions_stale'] = True
        reason = only(engine.evaluate(make_snapshot(arms=arms)))
        assert reason.arm_id == 'panda2'

    def test_never_seen_sample_still_produces_a_detail(self, engine):
        """A sample that never arrived has no age, and the detail says so."""
        arm = make_arm()
        arm['positions_stale'] = True
        arm['positions_age_s'] = None
        reason = only(engine.evaluate(make_snapshot(arms={'panda1': arm})))
        assert 'no sample within' in reason.detail
        assert str(config.JOINT_STATE_STALE_FAULT_S) in reason.detail

    def test_fresh_positions_do_not_fire(self, engine):
        """A fresh stream is the healthy case."""
        assert engine.evaluate(make_snapshot()) == []


class TestF7LaunchExited:
    """F7: the ros2 launch child exited while the session was running."""

    @pytest.mark.parametrize('mode', ALL_MODES)
    def test_dead_launch_fires_in_every_mode(self, engine, mode):
        """No mode survives losing its launch child."""
        arms = {'panda1': make_simulated_arm() if mode == MODE_SIMULATE else make_arm()}
        controller = CONTROLLER if mode == MODE_MOTION else None
        reason = only(engine.evaluate(make_snapshot(
            mode=mode, arms=arms, controller_name=controller, launch_alive=False)))
        assert reason.code == 'launch_exited'
        assert reason.arm_id is None
        assert reason.detail

    def test_live_launch_does_not_fire(self, engine):
        """The healthy case."""
        assert engine.evaluate(make_snapshot(launch_alive=True)) == []


class TestF8HardwareInactive:
    """F8: the hardware component is missing or not ``active``."""

    @pytest.mark.parametrize('mode', PRODUCTION_MODES)
    def test_unavailable_hardware_fires(self, engine, mode):
        """A component absent from list_hardware_components is a fault."""
        controller = CONTROLLER if mode == MODE_MOTION else None
        reason = only(engine.evaluate(make_snapshot(
            mode=mode, controller_name=controller,
            hardware_available=False, hardware_lifecycle_label=None)))
        assert reason.code == 'hardware_inactive'
        assert reason.arm_id is None
        assert 'not present' in reason.detail

    @pytest.mark.parametrize('label', ['inactive', 'unconfigured', 'finalized', None])
    def test_non_active_lifecycle_fires(self, engine, label):
        """Any lifecycle other than active is a fault in a production mode."""
        reason = only(engine.evaluate(make_snapshot(hardware_lifecycle_label=label)))
        assert reason.code == 'hardware_inactive'
        assert (label or 'unknown') in reason.detail

    def test_active_hardware_does_not_fire(self, engine):
        """The healthy case."""
        assert engine.evaluate(make_snapshot(hardware_lifecycle_label='active')) == []

    def test_simulate_never_fires(self, engine):
        """Mock hardware is not judged against the real lifecycle rule."""
        snapshot = make_snapshot(
            mode=MODE_SIMULATE, arms={'panda1': make_simulated_arm()},
            hardware_available=False, hardware_lifecycle_label='unconfigured')
        assert engine.evaluate(snapshot) == []


class TestSimulateIsStructurallyGated:
    """In simulate, F1-F4 and F8 cannot fire even on poisoned data."""

    def poisoned_arm(self, arm_id='panda1'):
        """Build an arm carrying data mock hardware could never produce."""
        arm = make_arm(arm_id)
        arm['diagnostic']['level'] = 2
        arm['diagnostic']['message'] = 'global fault latched'
        arm['robot_state']['robot_mode'] = 4
        arm['robot_state']['current_errors'] = ['joint_position_limits_violation']
        arm['robot_state']['control_command_success_rate'] = 0.0
        return arm

    def test_poisoned_simulate_snapshot_fires_nothing(self, engine, clock):
        """Every production rule is gated by mode, not by the data."""
        snapshot = make_snapshot(
            mode=MODE_SIMULATE, arms={'panda1': self.poisoned_arm()},
            hardware_available=False, hardware_lifecycle_label='unconfigured')
        assert engine.evaluate(snapshot) == []
        clock.advance(60.0)
        assert engine.evaluate(snapshot) == []

    def test_the_same_data_in_watch_fires_only_state_health_rules(
            self, engine, clock):
        """Watch evaluates state health, but never Motion-only CCSR."""
        snapshot = make_snapshot(
            mode=MODE_WATCH, arms={'panda1': self.poisoned_arm()},
            hardware_available=False, hardware_lifecycle_label='unconfigured')
        engine.evaluate(snapshot)
        clock.advance(60.0)
        assert codes(engine.evaluate(snapshot)) == [
            'diagnostic_error',
            'robot_mode_fault',
            'robot_errors',
            'hardware_inactive',
        ]

    def test_simulate_still_fires_f6_and_f7(self, engine):
        """The rules that do not need Franka data are unaffected."""
        arm = make_simulated_arm()
        arm['positions_stale'] = True
        snapshot = make_snapshot(
            mode=MODE_SIMULATE, arms={'panda1': arm}, launch_alive=False)
        assert codes(engine.evaluate(snapshot)) == ['joint_state_stale', 'launch_exited']


class TestCombinations:
    """Several rules firing at once: one reason per rule per arm, in order."""

    def test_every_rule_at_once(self, engine, clock):
        """All eight rules fire together and each contributes its own reason."""
        arm = make_arm('panda1')
        arm['diagnostic']['level'] = 2
        arm['robot_state']['robot_mode'] = 5
        arm['robot_state']['current_errors'] = ['cartesian_reflex']
        arm['robot_state']['control_command_success_rate'] = 0.1
        arm['positions_stale'] = True
        snapshot = make_snapshot(
            mode=MODE_MOTION, arms={'panda1': arm}, controller_name=CONTROLLER,
            controller_states={CONTROLLER: 'inactive'},
            hardware_available=True, hardware_lifecycle_label='inactive',
            launch_alive=False)
        engine.evaluate(snapshot)
        clock.advance(config.CCSR_FAULT_SUSTAIN_S + 0.1)
        reasons = engine.evaluate(snapshot)
        assert codes(reasons) == [
            'diagnostic_error',
            'robot_mode_fault',
            'robot_errors',
            'ccsr_low',
            'controller_deactivated',
            'joint_state_stale',
            'launch_exited',
            'hardware_inactive',
        ]
        assert set(codes(reasons)) == FAULT_CODES

    def test_per_arm_rules_multiply_by_arm(self, engine):
        """Two faulted arms yield two reasons per per-arm rule."""
        arms = {'panda1': make_arm('panda1'), 'panda2': make_arm('panda2')}
        for arm in arms.values():
            arm['diagnostic']['level'] = 2
            arm['positions_stale'] = True
        reasons = engine.evaluate(make_snapshot(arms=arms))
        assert [(r.code, r.arm_id) for r in reasons] == [
            ('diagnostic_error', 'panda1'),
            ('diagnostic_error', 'panda2'),
            ('joint_state_stale', 'panda1'),
            ('joint_state_stale', 'panda2'),
        ]

    def test_every_reason_is_well_formed(self, engine):
        """Codes stay inside the closed set and details are never empty."""
        arm = make_arm()
        arm['diagnostic']['level'] = 2
        arm['robot_state']['robot_mode'] = 4
        arm['robot_state']['current_errors'] = ['cartesian_reflex']
        arm['positions_stale'] = True
        reasons = engine.evaluate(make_snapshot(
            arms={'panda1': arm}, hardware_available=False, launch_alive=False))
        assert reasons
        for reason in reasons:
            assert reason.code in FAULT_CODES
            assert isinstance(reason.detail, str) and reason.detail


class TestRecoverable:
    """The Recover button is offered only where recovery can help."""

    @pytest.mark.parametrize('code', sorted(FAULT_CODES))
    @pytest.mark.parametrize('mode', ALL_MODES)
    def test_truth_table(self, mode, code):
        """One row per (mode, code): production mode AND a recoverable code."""
        reasons = [FaultReason(code=code, arm_id='panda1', detail='x')]
        expected = (mode in PRODUCTION_MODES
                    and code in RECOVERABLE_FAULT_CODES
                    and not (mode == MODE_WATCH
                             and code in ('ccsr_low', 'controller_deactivated')))
        assert FaultEngine.recoverable(mode, reasons) is expected

    @pytest.mark.parametrize('mode', ALL_MODES)
    def test_no_reasons_is_not_recoverable(self, mode):
        """An empty fault list offers nothing to recover."""
        assert FaultEngine.recoverable(mode, []) is False

    def test_f2_plus_f5_physical_stop_shape_is_recoverable(self):
        """A stopped backend plus inactive impedance controller is addressed."""
        reasons = [
            FaultReason(code='robot_mode_fault', arm_id='panda1', detail='x'),
            FaultReason(code='controller_deactivated', arm_id=None, detail='x'),
        ]
        assert FaultEngine.recoverable(MODE_MOTION, reasons) is True

    def test_f1_plus_f7_is_blocked_by_the_dead_launch(self):
        """One addressed reason cannot hide a mixed dead-launch reason."""
        reasons = [
            FaultReason(code='diagnostic_error', arm_id='panda1', detail='x'),
            FaultReason(code='launch_exited', arm_id=None, detail='x'),
        ]
        assert FaultEngine.recoverable(MODE_WATCH, reasons) is False

    def test_f7_blocks_every_mixed_recovery(self):
        """Every firing reason must be covered by the restore sequence."""
        reasons = [
            FaultReason(code='controller_deactivated', arm_id=None, detail='x'),
            FaultReason(code='launch_exited', arm_id=None, detail='x'),
        ]
        assert FaultEngine.recoverable(MODE_MOTION, reasons) is False

    def test_dict_reasons_are_accepted(self):
        """The frame stores reasons as dicts; classification still works."""
        reasons = [FaultReason(code='ccsr_low', arm_id='panda1', detail='x').as_dict()]
        assert FaultEngine.recoverable(MODE_MOTION, reasons) is True

    def test_poisoned_watch_ccsr_reason_is_not_recoverable(self):
        """A structurally impossible Watch F4 reason never opens recovery."""
        reasons = [FaultReason(code='ccsr_low', arm_id='panda1', detail='x')]
        assert FaultEngine.recoverable(MODE_WATCH, reasons) is False

    def test_unknown_mode_is_not_recoverable(self):
        """Only the two production modes ever offer Recover."""
        reasons = [FaultReason(code='ccsr_low', arm_id='panda1', detail='x')]
        assert FaultEngine.recoverable('bootstrapping', reasons) is False

    def test_evaluated_faults_classify(self, engine):
        """End to end: a real evaluation feeds the classification unchanged."""
        arm = make_arm()
        arm['robot_state']['robot_mode'] = 4
        reasons = engine.evaluate(make_snapshot(arms={'panda1': arm}))
        assert FaultEngine.recoverable(MODE_WATCH, reasons) is True


class TestFaultReasonShape:
    """FaultReason matches the plan section 7.1 wire shape exactly."""

    def test_as_dict_keys_and_values(self):
        """as_dict is {'code', 'arm_id', 'detail'} and nothing more."""
        reason = FaultReason(
            code='diagnostic_error', arm_id='panda1', detail='global fault latched')
        assert reason.as_dict() == {
            'code': 'diagnostic_error',
            'arm_id': 'panda1',
            'detail': 'global fault latched',
        }

    def test_arm_id_is_null_for_session_wide_rules(self):
        """Session-wide rules carry arm_id None, which serializes to null."""
        reason = FaultReason(code='launch_exited', arm_id=None, detail='gone')
        assert reason.as_dict()['arm_id'] is None

    def test_reasons_are_immutable(self):
        """A published reason cannot be edited after the fact."""
        reason = FaultReason(code='ccsr_low', arm_id='panda1', detail='x')
        with pytest.raises(dataclasses.FrozenInstanceError):
            reason.code = 'robot_errors'


class TestMalformedSnapshots:
    """A missing or malformed section degrades to 'no input', never a crash."""

    def test_empty_arm_dicts(self, engine):
        """An arm with no sections at all fires nothing and does not raise."""
        assert engine.evaluate(make_snapshot(arms={'panda1': {}})) == []

    def test_none_sections(self, engine):
        """None sections are read as absent, not as zeros."""
        arm = {'arm_id': 'panda1', 'robot_state': None, 'diagnostic': None}
        assert engine.evaluate(make_snapshot(arms={'panda1': arm})) == []

    def test_no_arms_at_all(self, engine):
        """A stopped-out arms map is evaluable; session-wide rules still fire."""
        assert engine.evaluate(make_snapshot(arms={})) == []
        assert codes(engine.evaluate(make_snapshot(arms={}, launch_alive=False))) == [
            'launch_exited']

    def test_current_errors_of_the_wrong_type(self, engine):
        """A non-list current_errors is ignored rather than split into letters."""
        arm = make_arm()
        arm['robot_state']['current_errors'] = 'joint_reflex'
        assert engine.evaluate(make_snapshot(arms={'panda1': arm})) == []
