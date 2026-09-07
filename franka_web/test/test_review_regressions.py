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
Regression pins for the adversarial-review findings, and the package scans.

Each behavioural test names the finding it pins. The two package-tree scans
at the end are new in v2: they walk every shipped file of this package and
assert that the deleted environment contract and the notes tree appear
nowhere, with no allowance for any file. Both are exported as module-level
helpers, because the end-to-end console battery calls them too.
"""

import hashlib
import json
import math
import os
import re
import threading
from types import SimpleNamespace
import xml.etree.ElementTree

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from franka_msgs.msg import FrankaState
from franka_web import defaults, health, sse
from franka_web.launcher import LauncherError
from franka_web.recording import RecordingError, RecordingSupervisor
from franka_web.ros_bridge import FrankaWebBridge
from franka_web.session import _Command, _STEP_LABELS, SessionRequest
import pytest
from sensor_msgs.msg import JointState
from support.fake_launcher import FakeRecording
from test_session_state_machine import Harness


class BridgeCacheHarness(FrankaWebBridge):
    """The cache-only attributes used by FrankaWebBridge callback methods."""

    def __init__(self):
        """Build a bridge-shaped object without constructing an rclpy node."""
        self._cache_lock = threading.Lock()
        self._joint_callback_condition = threading.Condition()
        self._joint_callback_boundary_lock = threading.Lock()
        self._joint_callbacks_inflight = 0
        self._joint_callback_entry_closed = False
        self._session_epoch = 7
        self._session_subs = []
        self._arm_ids = ('panda1',)
        self._activation_capture = None
        self._activation_capture_generation = 0
        self._joint = None
        self._robot_states = {}
        self._diagnostics = {}
        self._controller_states = {}
        self._controller_types = {}
        self._hardware = None
        self._lifecycle_gen = 3
        self._list_controllers = object()
        self._list_hardware = object()
        self.created_subscriptions = []
        self.destroyed_subscriptions = []
        self.refresh_epochs = []

    def create_subscription(self, message_type, topic, callback, qos):
        """Record one subscription without constructing an rclpy node."""
        subscription = SimpleNamespace(
            message_type=message_type, topic=topic, callback=callback, qos=qos)
        self.created_subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        """Record a successful subscription destroy."""
        self.destroyed_subscriptions.append(subscription)
        return True

    def _refresh_details(self, epoch):
        """Record the epoch an activity callback asks to refresh."""
        self.refresh_epochs.append(epoch)


class PauseFirstLock:
    """Pause the first entrant immediately before taking the real lock."""

    def __init__(self, entered, resume):
        """Wrap one ordinary lock with a deterministic first-entry barrier."""
        self._lock = threading.Lock()
        self._entered = entered
        self._resume = resume
        self._pause_first = True

    def __enter__(self):
        """Hold the first caller outside the lock until the test releases it."""
        if self._pause_first:
            self._pause_first = False
            self._entered.set()
            assert self._resume.wait(timeout=2.0)
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Release the wrapped lock."""
        self._lock.release()


def lifecycle_activity(controllers=(), hardware=()):
    """Build a cache-test ControllerManagerActivity-shaped value."""
    return SimpleNamespace(
        controllers=[
            SimpleNamespace(
                name=name, state=SimpleNamespace(id=state_id, label=label))
            for name, state_id, label in controllers
        ],
        hardware_components=[
            SimpleNamespace(
                name=name, state=SimpleNamespace(id=state_id, label=label))
            for name, state_id, label in hardware
        ],
    )


class TestBridgeSessionEpoch:
    """An old subscription callback cannot repopulate a cleared cache."""

    @staticmethod
    def blocking_clock(monkeypatch):
        """Pause a callback after it starts but before it takes the cache lock."""
        entered = threading.Event()
        resume = threading.Event()

        def monotonic_ns():
            entered.set()
            assert resume.wait(timeout=2.0)
            return 123456789

        monkeypatch.setattr('franka_web.ros_bridge.time.monotonic_ns', monotonic_ns)
        return entered, resume

    def test_old_joint_callback_crossing_clear_is_discarded(self, monkeypatch):
        """A destroyed joint subscription cannot restore its old sample."""
        bridge = BridgeCacheHarness()
        callback = FrankaWebBridge._joint_callback(
            bridge, bridge._session_epoch)
        entered = threading.Event()
        resume = threading.Event()
        bridge._cache_lock = PauseFirstLock(entered, resume)
        clock_calls = []
        monkeypatch.setattr(
            'franka_web.ros_bridge.time.monotonic_ns',
            lambda: clock_calls.append(True) or 123456789)
        worker = threading.Thread(target=callback, args=(JointState(),), daemon=True)
        worker.start()
        assert entered.wait(timeout=2.0)

        FrankaWebBridge.clear_session(bridge)
        resume.set()
        worker.join(timeout=2.0)
        assert worker.is_alive() is False
        assert bridge._joint is None
        assert clock_calls == []

    def test_old_robot_state_callback_crossing_clear_is_discarded(
            self, monkeypatch):
        """A destroyed per-arm subscription cannot restore its old state."""
        bridge = BridgeCacheHarness()
        callback = FrankaWebBridge._robot_state_callback(
            bridge, 'panda1', bridge._session_epoch)
        entered, resume = self.blocking_clock(monkeypatch)
        worker = threading.Thread(target=callback, args=(FrankaState(),), daemon=True)
        worker.start()
        assert entered.wait(timeout=2.0)

        FrankaWebBridge.clear_session(bridge)
        resume.set()
        worker.join(timeout=2.0)
        assert worker.is_alive() is False
        assert bridge._robot_states == {}

    def test_old_diagnostics_crossing_same_arm_reconfigure_is_discarded(
            self, monkeypatch):
        """Epoch, not arm-name equality, owns a diagnostic sample."""
        bridge = BridgeCacheHarness()
        callback = FrankaWebBridge._diagnostics_callback(
            bridge, bridge._session_epoch)
        message = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = health.canonical_diagnostic_name('panda1')
        message.status.append(status)
        entered, resume = self.blocking_clock(monkeypatch)
        worker = threading.Thread(target=callback, args=(message,), daemon=True)
        worker.start()
        assert entered.wait(timeout=2.0)

        FrankaWebBridge.clear_session(bridge)
        with bridge._cache_lock:
            bridge._arm_ids = ('panda1',)  # next session selects the same arm
        resume.set()
        worker.join(timeout=2.0)
        assert worker.is_alive() is False
        assert bridge._diagnostics == {}

    def test_current_epoch_callbacks_store_normally(self, monkeypatch):
        """The epoch guard rejects only retired callbacks, not live traffic."""
        bridge = BridgeCacheHarness()
        monkeypatch.setattr(
            'franka_web.ros_bridge.time.monotonic_ns', lambda: 987654321)
        joint = JointState()
        state = FrankaState()
        diagnostic = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = health.canonical_diagnostic_name('panda1')
        diagnostic.status.append(status)

        FrankaWebBridge._joint_callback(
            bridge, bridge._session_epoch)(joint)
        FrankaWebBridge._robot_state_callback(
            bridge, 'panda1', bridge._session_epoch)(state)
        FrankaWebBridge._diagnostics_callback(
            bridge, bridge._session_epoch)(diagnostic)

        assert bridge._joint == (987654321, joint)
        assert bridge._robot_states == {'panda1': (987654321, state)}
        assert bridge._diagnostics == {'panda1': (987654321, status)}

    def test_activation_capture_cannot_be_rearmed_while_live(self):
        """A reentrant begin cannot discard unseen activation extrema."""
        bridge = BridgeCacheHarness()
        first_generation = bridge.begin_activation_capture(('panda1',))
        capture = bridge._activation_capture

        with pytest.raises(RuntimeError, match='already armed'):
            bridge.begin_activation_capture(('panda1',))

        assert bridge._activation_capture is capture
        assert bridge._activation_capture_generation == first_generation

    def test_old_async_lifecycle_details_cannot_cross_session_epoch(self):
        """An old list response cannot refill controller/hardware caches."""
        bridge = BridgeCacheHarness()
        old_epoch = bridge._session_epoch
        old_generation = bridge._lifecycle_gen
        FrankaWebBridge.clear_session(bridge)
        controller = SimpleNamespace(name='old_controller', type='Old/Type',
                                     state='active')
        controller_future = SimpleNamespace(
            result=lambda: SimpleNamespace(controller=[controller]))
        lifecycle = SimpleNamespace(id=3, label='active')
        hardware = SimpleNamespace(name='old_hardware', plugin_name='Old/Plugin',
                                   state=lifecycle)
        hardware_future = SimpleNamespace(
            result=lambda: SimpleNamespace(component=[hardware]))

        FrankaWebBridge._on_list_controllers(
            bridge, controller_future, old_generation, old_epoch)
        FrankaWebBridge._on_list_hardware(
            bridge, hardware_future, old_generation, old_epoch)

        assert bridge._controller_states == {}
        assert bridge._controller_types == {}
        assert bridge._hardware is None

    def test_activity_subscription_is_recreated_and_old_epoch_is_inert(self):
        """An activity dequeued from session A cannot populate session B."""
        bridge = BridgeCacheHarness()
        bridge.configure_session(('panda1',), 'single')
        old_epoch = bridge._session_epoch
        old_activity = next(
            sub.callback for sub in bridge.created_subscriptions
            if sub.topic == '/controller_manager/activity')

        bridge.configure_session(('panda1',), 'single')
        current_epoch = bridge._session_epoch
        current_activity = next(
            sub.callback for sub in reversed(bridge.created_subscriptions)
            if sub.topic == '/controller_manager/activity')
        message = lifecycle_activity(
            controllers=(('joint_state_broadcaster', 3, 'active'),),
            hardware=(('FrankaMultiHardwareInterface', 3, 'active'),))

        old_activity(message)
        assert bridge._controller_states == {}
        assert bridge._hardware is None
        assert bridge.refresh_epochs == [old_epoch, current_epoch]

        current_activity(message)
        assert bridge._controller_states == {'joint_state_broadcaster': 'active'}
        assert bridge._hardware['lifecycle_label'] == 'active'
        assert bridge.refresh_epochs == [old_epoch, current_epoch, current_epoch]

    def test_activity_replaces_removed_controllers_and_empty_hardware(self):
        """An authoritative activity snapshot removes names no longer present."""
        bridge = BridgeCacheHarness()
        bridge._controller_states = {
            'kept_controller': 'active', 'removed_controller': 'active'}
        bridge._controller_types = {
            'kept_controller': 'Kept/Type', 'removed_controller': 'Old/Type'}
        bridge._hardware = {
            'name': 'FrankaMultiHardwareInterface', 'plugin_name': 'Old/Plugin',
            'lifecycle_id': 3, 'lifecycle_label': 'active'}
        callback = bridge._activity_callback(bridge._session_epoch)
        old_generation = bridge._lifecycle_gen

        callback(lifecycle_activity(
            controllers=(('kept_controller', 2, 'inactive'),)))
        stale_hardware = SimpleNamespace(
            result=lambda: SimpleNamespace(component=[SimpleNamespace(
                name='FrankaMultiHardwareInterface', plugin_name='Old/Plugin',
                state=SimpleNamespace(id=3, label='active'))]))
        FrankaWebBridge._on_list_hardware(
            bridge, stale_hardware, old_generation, bridge._session_epoch)

        assert bridge._controller_states == {'kept_controller': 'inactive'}
        assert bridge._controller_types == {'kept_controller': 'Kept/Type'}
        assert bridge._hardware is None
        assert bridge.refresh_epochs == [bridge._session_epoch]

    def test_clear_retires_evidence_and_attempts_every_destroy_on_errors(self):
        """Destroy failures cannot skip retirement, clearing, or later handles."""
        bridge = BridgeCacheHarness()
        bridge._session_subs = ['first', 'second', 'third']
        bridge._joint = object()
        bridge._robot_states = {'panda1': object()}
        bridge._diagnostics = {'panda1': object()}
        bridge._controller_states = {'old': 'active'}
        bridge._controller_types = {'old': 'Old/Type'}
        bridge._hardware = {'name': 'old'}
        old_epoch = bridge._session_epoch
        attempted = []

        def fail_two(subscription):
            # rclpy destruction must never run under the callback cache lock.
            assert bridge._cache_lock.acquire(blocking=False) is True
            bridge._cache_lock.release()
            attempted.append(subscription)
            if subscription != 'second':
                raise RuntimeError('scripted {}'.format(subscription))
            return True

        bridge.destroy_subscription = fail_two
        with pytest.raises(RuntimeError, match='2 session subscription') as excinfo:
            bridge.clear_session()

        assert attempted == ['first', 'second', 'third']
        assert 'scripted first' in str(excinfo.value)
        assert 'scripted third' in str(excinfo.value)
        assert bridge._session_epoch == old_epoch + 1
        assert bridge._session_subs == []
        assert bridge._joint is None
        assert bridge._robot_states == {}
        assert bridge._diagnostics == {}
        assert bridge._controller_states == {}
        assert bridge._controller_types == {}
        assert bridge._hardware is None
        assert bridge._arm_ids == ()

    def test_newer_activity_wins_over_in_flight_controller_query(self):
        """A stale service response cannot overwrite or escape past activity."""
        bridge = BridgeCacheHarness()
        stale = SimpleNamespace(controller=[SimpleNamespace(
            name='live_controller', state='inactive', type='Live/Type'),
            SimpleNamespace(
                name='removed_controller', state='active', type='Old/Type')])

        def activity_during_call(_client, _request, _timeout_s):
            bridge._activity_callback(bridge._session_epoch)(
                lifecycle_activity(
                    controllers=(('live_controller', 3, 'active'),)))
            return stale

        bridge._bounded_call = activity_during_call
        observed = bridge.query_controller_states()

        assert observed == {'live_controller': 'active'}
        assert bridge._controller_states == {'live_controller': 'active'}
        assert bridge._controller_types == {'live_controller': 'Live/Type'}

    def test_newer_activity_wins_over_in_flight_hardware_query(self):
        """A stale hardware service response cannot replace newer activity."""
        bridge = BridgeCacheHarness()
        stale = SimpleNamespace(component=[SimpleNamespace(
            name='old_hardware', plugin_name='Old/Plugin',
            state=SimpleNamespace(id=2, label='inactive'))])

        def activity_during_call(_client, _request, _timeout_s):
            bridge._activity_callback(bridge._session_epoch)(
                lifecycle_activity(hardware=(
                    ('FrankaMultiHardwareInterface', 3, 'active'),)))
            return stale

        bridge._bounded_call = activity_during_call
        observed = bridge.query_hardware_component()

        assert observed['name'] == 'FrankaMultiHardwareInterface'
        assert observed['lifecycle_label'] == 'active'
        assert bridge._hardware == observed


class TestActivationCaptureAtomicity:
    """The production bridge makes callback capture closure one boundary."""

    @staticmethod
    def complete_joint_state():
        """Build one complete, finite Panda joint-state callback payload."""
        message = JointState()
        message.name = list(health.joint_names_for('panda1'))
        message.position = [0.0] * 7
        message.velocity = [0.0] * 7
        message.effort = [0.0] * 7
        return message

    def test_callback_waits_until_ready_observer_commits_and_disarms(self):
        """A callback concurrent with finalization linearizes after Running."""
        bridge = BridgeCacheHarness()
        bridge.begin_activation_capture(('panda1',))
        state = {'value': 'settling'}
        observer_entered = threading.Event()
        release_observer = threading.Event()
        callback_started = threading.Event()
        callback_done = threading.Event()
        outcome = {}

        def observer(capture):
            assert capture['sample_count'] == 0
            assert bridge._activation_capture is not None
            observer_entered.set()
            assert release_observer.wait(timeout=2.0)
            state['value'] = 'running'
            return SimpleNamespace(status='ready')

        def finalize():
            try:
                outcome['verdict'] = bridge.finalize_activation_capture(observer)
            except BaseException as error:  # surfaced in the test thread
                outcome['error'] = error

        finalizer = threading.Thread(target=finalize, daemon=True)
        finalizer.start()
        assert observer_entered.wait(timeout=2.0)

        callback = bridge._joint_callback(bridge._session_epoch)

        def deliver():
            callback_started.set()
            callback(self.complete_joint_state())
            callback_done.set()

        delivery = threading.Thread(target=deliver, daemon=True)
        delivery.start()
        assert callback_started.wait(timeout=2.0)
        assert callback_done.wait(timeout=0.05) is False

        release_observer.set()
        finalizer.join(timeout=2.0)
        delivery.join(timeout=2.0)
        assert finalizer.is_alive() is False
        assert delivery.is_alive() is False
        assert 'error' not in outcome
        assert outcome['verdict'].status == 'ready'
        assert state['value'] == 'running'
        assert bridge._activation_capture is None
        assert bridge._joint is not None

    @pytest.mark.parametrize('boundary_method', [
        'finalize_activation_capture', 'close_activation_capture'])
    def test_callback_admitted_before_finalizer_is_included(
            self, boundary_method):
        """Finalization waits for an admitted callback and sees its extrema."""
        bridge = BridgeCacheHarness()
        bridge.begin_activation_capture(('panda1',))
        entered_cache = threading.Event()
        release_cache = threading.Event()
        bridge._cache_lock = PauseFirstLock(entered_cache, release_cache)
        callback = bridge._joint_callback(bridge._session_epoch)
        message = self.complete_joint_state()
        message.position[0] = 0.111
        callback_done = threading.Event()

        def deliver():
            callback(message)
            callback_done.set()

        delivery = threading.Thread(target=deliver, daemon=True)
        delivery.start()
        assert entered_cache.wait(timeout=2.0)

        observer_entered = threading.Event()
        outcome = {}

        def observer(capture):
            observer_entered.set()
            outcome['capture'] = capture
            status = ('failed' if capture['arms']['panda1'][
                'max_position_rad'][0] > 0.1 else 'ready')
            return SimpleNamespace(status=status)

        def finalize():
            outcome['verdict'] = getattr(bridge, boundary_method)(observer)

        finalizer = threading.Thread(target=finalize, daemon=True)
        finalizer.start()
        assert observer_entered.wait(timeout=0.05) is False

        release_cache.set()
        delivery.join(timeout=2.0)
        finalizer.join(timeout=2.0)
        assert delivery.is_alive() is False
        assert finalizer.is_alive() is False
        assert callback_done.is_set()
        assert observer_entered.is_set()
        assert outcome['capture']['sample_count'] == 1
        assert outcome['verdict'].status == 'failed'
        assert bridge._activation_capture is None

    def test_settling_finalizer_retains_capture_and_reopens_admission(self):
        """A non-ready final drain retains one continuous capture generation."""
        bridge = BridgeCacheHarness()
        generation = bridge.begin_activation_capture(('panda1',))
        capture_object = bridge._activation_capture

        first = bridge.finalize_activation_capture(
            lambda capture: SimpleNamespace(status='settling'))
        assert first.status == 'settling'
        assert bridge._activation_capture is capture_object
        assert bridge._activation_capture.generation == generation
        assert bridge._joint_callback_entry_closed is False

        message = self.complete_joint_state()
        message.position[0] = 0.05
        bridge._joint_callback(bridge._session_epoch)(message)
        observed = {}

        def ready(capture):
            observed.update(capture)
            return SimpleNamespace(status='ready')

        second = bridge.finalize_activation_capture(ready)
        assert second.status == 'ready'
        assert observed['generation'] == generation
        assert observed['sample_count'] == 1
        assert observed['arms']['panda1']['max_position_rad'][0] == 0.05
        assert bridge._activation_capture is None

    def test_observer_exception_reopens_admission_without_losing_capture(self):
        """An observer exception cannot strand callbacks behind the boundary."""
        bridge = BridgeCacheHarness()
        bridge.begin_activation_capture(('panda1',))
        capture_object = bridge._activation_capture

        def explode(_capture):
            raise RuntimeError('scripted observer failure')

        with pytest.raises(RuntimeError, match='scripted observer failure'):
            bridge.finalize_activation_capture(explode)
        assert bridge._joint_callback_entry_closed is False
        assert bridge._activation_capture is capture_object

        bridge._joint_callback(bridge._session_epoch)(
            self.complete_joint_state())
        final = bridge.close_activation_capture(
            lambda capture: SimpleNamespace(
                status='ready', sample_count=capture['sample_count']))
        assert final.sample_count == 1
        assert bridge._activation_capture is None


class TestStaleCommandCancellation:
    """R1/R2/R4: a timed-out command must be discarded, never executed late."""

    def test_abandoned_start_is_skipped(self, tmp_path):
        """An abandoned start command spawns nothing and changes no state."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        command = _Command(kind='start',
                           request=SessionRequest(arms='both', mode='simulate'))
        harness.supervisor._commands.put(command)
        assert command.abandon() is True
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        assert harness.spawner.spawned == []
        assert not command.done.is_set()

    def test_begun_command_cannot_be_abandoned(self):
        """Once execution begins the waiter must wait for the real answer."""
        command = _Command(kind='stop')
        assert command.try_begin() is True
        assert command.abandon() is False

    def test_abandoned_command_cannot_begin(self):
        """The reverse ordering also holds."""
        command = _Command(kind='stop')
        assert command.abandon() is True
        assert command.try_begin() is False


class TestNonFiniteSanitization:
    """R3: NaN/Inf must never reach the wire as invalid JSON."""

    def test_nan_position_projects_to_null_and_stale(self):
        """A NaN joint sample is reported absent, not forwarded."""
        message = JointState()
        for joint in range(1, 8):
            message.name.append('panda1_joint{}'.format(joint))
            message.position.append(0.1 * joint)
            message.velocity.append(0.0)
            message.effort.append(0.0)
        message.position[3] = float('nan')
        message.velocity[2] = float('inf')
        joints = health.extract_joints('panda1', message)
        assert joints['positions'][3] is None
        assert joints['velocities'][2] is None
        assert joints['complete'] is False
        projection = health.project_arm('panda1', 10**9, (10**9, message), None, None)
        assert projection['positions_stale'] is True
        assert projection['status'] in ('warn', 'unknown')

    def test_encode_event_never_emits_nan_tokens(self):
        """Even an unsanitized NaN encodes as strictly valid JSON null."""
        payload = {'a': float('nan'), 'b': [1.0, float('inf')], 'c': {'d': float('-inf')}}
        encoded = sse.encode_event('state', payload).decode('utf-8')
        body = encoded.split('data: ', 1)[1].rstrip('\n')
        parsed = json.loads(body)   # raises if a bare NaN token were present
        assert parsed == {'a': None, 'b': [1.0, None], 'c': {'d': None}}

    def test_safe_json_dumps_finite_passthrough(self):
        """Ordinary finite payloads are unchanged by the belt."""
        parsed = json.loads(sse.safe_json_dumps({'x': 1.5}))
        assert parsed == {'x': 1.5} and math.isfinite(parsed['x'])


class TestRecorderSpawnFailureContainment:
    """R5: a LauncherError under the recorder is a refusal, not a crash."""

    def test_launcher_error_becomes_recording_error(self, tmp_path):
        """RecordingSupervisor.start wraps a failed spawn."""
        harness = Harness(tmp_path)

        def failing_spawn(argv, env, name, **kwargs):
            raise LauncherError('scripted spawn failure')
        supervisor = RecordingSupervisor(harness.settings, failing_spawn)
        with pytest.raises(RecordingError):
            supervisor.start('web-20260829-000000', 'dual', {})

    def test_session_refuses_with_recording_failed(self, tmp_path):
        """A recorder whose start raises LauncherError refuses the session."""
        class LauncherErrorRecording(FakeRecording):
            def start(self, base_name, arm_mode, env):
                raise LauncherError('scripted spawn failure')
        harness = Harness(tmp_path, recorder=LauncherErrorRecording())
        harness.make_ready_simulate()
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        frame = harness.supervisor.frame()
        assert frame['session']['last_error']['code'] == 'recording_failed'


class TestTickExceptionContainment:
    """R6/R8: a recorder failure mid-session stops the session, not the server."""

    def test_recording_error_in_tick_stops_session(self, tmp_path):
        """A RecordingError from tick() drives running -> stopping -> stopped."""
        class BrokenChainRecording(FakeRecording):
            def __init__(self, events=None):
                super().__init__(events=events)
                self.fail_ticks = False

            def tick(self, env):
                super().tick(env)
                if self.fail_ticks:
                    raise RecordingError('scripted rollover failure')
        recorder = BrokenChainRecording()
        harness = Harness(tmp_path, recorder=recorder)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        recorder.fail_ticks = True
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        frame = harness.supervisor.frame()
        assert frame['session']['last_error']['code'] == 'recording_failed'


class TestStage2ReviewPins:
    """Pins for the Stage 2 review findings (S-numbers in the session log)."""

    def test_revocation_without_enables_queues_nothing(self, tmp_path):
        """S2: release with nothing enabled must not enqueue disable work."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(6):
            harness.supervisor.tick()
        before = harness.supervisor._commands.qsize()
        for _ in range(10):
            harness.supervisor.revoke_operator_authorization()
        assert harness.supervisor._commands.qsize() == before

    def test_arm_not_enabled_maps_to_409(self):
        """S5: the §6.13 jog refusal must be 409, never 500."""
        from franka_web.http_api import ApiError
        assert ApiError('arm_not_enabled', 'x').status == 409

    def test_the_frame_carries_no_controller_or_gains_identity_at_all(
            self, tmp_path):
        """
        S9, restated for v2: both keys are GONE, not merely null.

        The controller is no longer a request field and there is no uploaded
        configuration to identify, so the session block does not carry either
        key in any mode.
        """
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        command = _Command(kind='start', request=SessionRequest(
            arms='both', mode='simulate'),
            operator_lease=harness.operator_lease)
        harness.supervisor._commands.put(command)
        for _ in range(6):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        session = harness.supervisor.frame()['session']
        assert 'controller_name' not in session
        assert 'gains_sha256' not in session


class TestStoppedUptimeFrozen:
    """R19: a stopped session's uptime must not keep counting."""

    def test_uptime_freezes_at_stop(self, tmp_path):
        """After stopped, advancing the clock does not grow uptime_s."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        first = harness.supervisor.frame()['session']['uptime_s']
        harness.clock.advance(1000.0)
        second = harness.supervisor.frame()['session']['uptime_s']
        assert first == second


# ----------------------------------------------------------------------
# Package-tree scans (new in v2)
# ----------------------------------------------------------------------

_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Directories that are build output, not shipped source.
_SKIPPED_DIRECTORIES = ('__pycache__', 'build', 'install', '.git')

#: Scanned as bytes rather than decoded as text.
_BINARY_SUFFIXES = ('.woff2', '.png', '.ico', '.svg.gz')

#: This file assembles both needles at runtime, so it must skip itself or the
#: scan fails on its own source.
_SELF = os.path.relpath(os.path.abspath(__file__), _PACKAGE_ROOT)


#: Shipped files that sit at the package root rather than under a scanned
#: directory. README.md is INSTALLED (CMakeLists.txt installs it to
#: share/franka_web/README.md) and is the largest body of hand-written
#: operator prose in the package, so the scans must see it; the two build
#: files are scanned because they are shipped source even though they are not
#: themselves installed. `test_the_walk_covers_every_installed_file_named_in_
#: cmake` fails if a future `install(FILES ...)` adds a root file here and
#: forgets this tuple.
_ROOT_FILES = ('CMakeLists.txt', 'package.xml', 'README.md')


def installed_files_from_cmake():
    """Return every relative path named by an ``install(FILES ...)`` block."""
    with open(os.path.join(_PACKAGE_ROOT, 'CMakeLists.txt'),
              encoding='utf-8') as handle:
        text = handle.read()
    paths = []
    for block in re.findall(r'install\s*\((.*?)\)', text, re.DOTALL):
        match = re.search(r'\bFILES\b(.*?)\b(?:DESTINATION|RENAME|PATTERN)\b',
                          block, re.DOTALL)
        if match is not None:
            paths.extend(match.group(1).split())
    return paths


def walk_package_files():
    """Yield ``(relative_path, text_or_None)`` for every shipped package file."""
    roots = ('franka_web', 'scripts', 'static', 'test', 'config')
    files = [os.path.join(_PACKAGE_ROOT, name) for name in _ROOT_FILES]
    for root in roots:
        base = os.path.join(_PACKAGE_ROOT, root)
        for directory, subdirectories, names in os.walk(base):
            subdirectories[:] = [name for name in subdirectories
                                 if name not in _SKIPPED_DIRECTORIES]
            files.extend(os.path.join(directory, name) for name in names)
    for path in sorted(files):
        if not os.path.isfile(path):
            continue
        relative = os.path.relpath(path, _PACKAGE_ROOT)
        if relative == _SELF:
            continue
        if relative.endswith(_BINARY_SUFFIXES):
            yield relative, None
            continue
        try:
            with open(path, encoding='utf-8') as handle:
                yield relative, handle.read()
        except (OSError, UnicodeDecodeError):
            yield relative, None


def assert_no_legacy_environment_prefix():
    """Fail if any shipped file carries the v1 prefix. No allowance."""
    needle = 'FRANKA_WEB' + '_'
    offenders = []
    for relative, text in walk_package_files():
        if text is None or needle not in text:
            continue
        offenders.append(relative)
    assert not offenders, (
        'the deleted environment contract survives in: {}'.format(offenders))


def assert_no_notes_tree_reference():
    """Fail if any shipped file names the notes tree. No allowance."""
    needle = 'multipanda_ros2' + '_jazzy_notes'
    offenders = [relative for relative, text in walk_package_files()
                 if text is not None and needle in text]
    assert not offenders, (
        'a notes-tree path survives in: {}'.format(offenders))


class TestPackageScans:
    """Two whole-package walks that also run in the console battery."""

    def test_no_source_file_mentions_the_old_environment_prefix(self):
        """
        Nothing under this package reads or names a FRANKA_WEB_* variable.

        The build files carried the last one -- CMakeLists.txt's
        node-executable option -- and it went with the Node block, so this
        scan now runs with no allowance at all.
        """
        assert_no_legacy_environment_prefix()

    def test_no_python_file_reintroduces_the_prefix(self):
        """The Python half of the scan, stated separately so it cannot rot."""
        needle = 'FRANKA_WEB' + '_'
        python_offenders = [
            relative for relative, text in walk_package_files()
            if text is not None and needle in text and relative.endswith('.py')]
        assert python_offenders == []

    def test_no_source_file_mentions_the_notes_tree(self):
        """A future user will not have the notes tree; nothing may name it."""
        assert_no_notes_tree_reference()

    def test_the_walk_actually_visits_the_package(self):
        """A scan that walks nothing would pass forever."""
        visited = {relative for relative, _text in walk_package_files()}
        assert 'franka_web/session.py' in visited
        assert 'package.xml' in visited
        assert 'CMakeLists.txt' in visited
        assert 'README.md' in visited
        assert any(relative.startswith('static/') for relative in visited)
        assert _SELF not in visited

    def test_the_walk_covers_every_installed_file_named_in_cmake(self):
        """
        Every installed file is scanned, the README included.

        The README is the file most likely to gain a copy-pasted notes-tree
        path in a future edit, and it is installed to share/franka_web. This
        derives the list from CMakeLists.txt rather than restating it, so
        installing a new root file without adding it to the walk fails here
        instead of quietly widening the hole.
        """
        installed = installed_files_from_cmake()
        assert 'README.md' in installed
        assert 'config/config.example.yaml' in installed
        visited = {relative for relative, _text in walk_package_files()}
        missing = [path for path in installed if path not in visited]
        assert not missing, (
            'installed but never scanned: {}'.format(missing))

    def test_the_readme_reaches_the_scans_as_readable_text(self):
        """
        A file the walk yields as ``None`` is walked but never scanned.

        Both assertion helpers skip a ``None`` payload, so "the README is in
        the list" is not the same property as "the README's prose is
        actually searched". This pins the second one, and that the prose the
        operator reads is real content rather than an empty file.
        """
        walked = dict(walk_package_files())
        assert 'README.md' in walked
        text = walked['README.md']
        assert text is not None, 'the README is walked but never read'
        assert len(text) > 1000
        assert ('multipanda_ros2' + '_jazzy_notes') not in text
        assert ('FRANKA_WEB' + '_') not in text


class TestReadmeTeaching:
    """The installed operator doc answers the two things the live day needed."""

    @staticmethod
    def readme():
        """Return the installed README's prose."""
        with open(os.path.join(_PACKAGE_ROOT, 'README.md'),
                  encoding='utf-8') as handle:
            return handle.read()

    def test_the_readme_documents_the_franka_dir_caveat(self):
        """
        The one zero-config caveat a real robot hits is written down.

        A machine whose libfranka the preflight cannot identify refuses every
        Watch and Motion start. The fix is one key, and the operator doc is
        where an operator can find it without reading the source.
        """
        text = self.readme()
        paragraphs = [block for block in text.split('\n\n')
                      if 'directories.franka_dir' in block
                      or 'franka_dir:' in block]
        assert paragraphs, 'the README never names directories.franka_dir'
        joined = '\n\n'.join(paragraphs)
        assert 'Watch' in joined and 'Motion' in joined, joined
        assert 'Simulate' in joined, joined

    def test_the_readme_checklist_matches_the_motion_step_labels(self):
        """
        The checklist the README describes is the one the server publishes.

        This is exactly the drift the old README already had: it listed five
        steps while Motion published seven. Deriving the expectation from
        `_STEP_LABELS` makes a future step insertion fail here rather than
        quietly leave the doc wrong.
        """
        text = self.readme()
        marker = '**Pick arms and a mode, press Start**'
        assert marker in text, 'the README lost its Start step'
        # Whitespace-collapsed: a label may be split across a wrapped line.
        step_four = ' '.join(
            text.split(marker, 1)[1].split('\n5. ', 1)[0].lower().split())
        motion_steps = ('preflight', 'health', 'stack_ready', 'controller_pause',
                        'baseline', 'controller', 'settling')
        for step_id in motion_steps:
            label = _STEP_LABELS[step_id].lower()
            assert label in step_four, (
                '{!r} is not in the README checklist sentence'.format(label))


class TestPackageIdentity:
    """One version string, with an equality test so it cannot drift."""

    def test_package_xml_version_matches_the_defaults_module(self):
        """
        Two copies of a version string with no equality test WILL drift.

        `package.xml` belongs to the packaging change, which lands last and
        bumps it; until then this arms itself rather than failing on a file
        this change may not edit. Confirm it is RUNNING, not skipping, once
        that change is in.
        """
        tree = xml.etree.ElementTree.parse(
            os.path.join(_PACKAGE_ROOT, 'package.xml'))
        declared = tree.getroot().findtext('version')
        if declared == '0.1.0':
            pytest.skip(
                'package.xml still declares the v1 version; the packaging '
                'change bumps it and arms this assertion')
        assert declared == defaults.SERVER_VERSION

    def test_the_package_module_reports_the_same_version(self):
        """`__init__` carries `__version__` and defines no identity of its own."""
        import franka_web
        assert franka_web.__version__ == defaults.SERVER_VERSION
        for name in ('SERVER_NAME', 'SERVER_VERSION', 'SCHEMA_VERSION'):
            assert not hasattr(franka_web, name)


class TestRevocationHookDiscipline:
    """The hook runs inside the operator lock's own mutex."""

    def test_takeover_and_expiry_both_reach_the_same_revocation_hook(
            self, tmp_path):
        """One hook, both paths, so both leave a new operator with nothing on."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_enabled['panda1'] = True
        harness.supervisor._arm_source['panda1'] = 'external'
        harness.lock.takeover()
        assert harness.supervisor._arm_enabled['panda1'] is False
        assert harness.supervisor._arm_source['panda1'] == 'jog'

        harness.supervisor._arm_enabled['panda2'] = True
        harness.supervisor._arm_source['panda2'] = 'external'
        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 1.0)
        harness.lock.state()                       # lazy expiry runs the hook
        assert harness.supervisor._arm_enabled['panda2'] is False
        assert harness.supervisor._arm_source['panda2'] == 'jog'

    def test_the_revocation_hook_never_calls_ros(self, tmp_path):
        """
        Its whole effect is flag mutation plus at most one queue put.

        It runs inside the lock's mutex, so anything that blocks or calls
        back into the lock is a deadlock waiting for a scheduler.
        """
        class ExplodingBridge:
            """Any call at all is a failure of the hook's contract."""

            def __getattr__(self, name):
                raise AssertionError(
                    'the revocation hook called the bridge: ' + name)

        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_enabled['panda1'] = True
        harness.supervisor._bridge = ExplodingBridge()
        before = harness.supervisor._commands.qsize()
        harness.supervisor.revoke_operator_authorization()
        assert harness.supervisor._commands.qsize() == before + 1
        assert harness.supervisor._arm_enabled['panda1'] is False

    def test_the_revocation_hook_acquires_no_supervisor_lock(self, tmp_path):
        """
        Holding `_state_lock` elsewhere must not stall the hook.

        A `_state_lock`-taking source reset here would create the lock-order
        inversion OperatorLock._mutex -> _state_lock, opposite to the
        supervisor's own order: any future code reading the lock while
        holding `_state_lock` would deadlock the whole server, and even today
        it would stall every heartbeat behind a supervisor critical section.
        """
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_source['panda1'] = 'external'
        harness.supervisor._arm_source['panda2'] = 'external'

        held = threading.Event()
        release = threading.Event()

        def hold_state_lock():
            with harness.supervisor._state_lock:
                held.set()
                release.wait(5.0)

        holder = threading.Thread(target=hold_state_lock, daemon=True)
        holder.start()
        assert held.wait(5.0)
        try:
            done = threading.Event()

            def revoke():
                harness.supervisor.revoke_operator_authorization()
                done.set()

            worker = threading.Thread(target=revoke, daemon=True)
            worker.start()
            assert done.wait(1.0), (
                'the revocation hook blocked on a supervisor lock')
            assert harness.supervisor._arm_source == {
                'panda1': 'jog', 'panda2': 'jog'}
        finally:
            release.set()
            holder.join(timeout=5.0)


# ----------------------------------------------------------------------
# The 3D scene's shipped surface
# ----------------------------------------------------------------------

#: The eight modules the scene is built from, plus its two vendored files.
#: Named here so that a ninth appearing without a decision fails, and so
#: that a missing one is not mistaken for a passing scan.
SCENE_MODULES = ('cell.js', 'ghost.js', 'ghost_state.js', 'hand_drag.js',
                 'kinematics.js', 'meshes.js', 'scene.js', 'urdf.js')
SCENE_VENDOR = ('three.r111.min.js', 'three-license.txt')

#: The vendored renderer, pinned by size and digest. It is a distribution
#: package's own build, committed rather than copied at build time, so the
#: only thing standing between it and a silent substitution is this pair.
THREE_BYTES = 850490
THREE_SHA256 = 'd4c5322f72bc86b8ffe7e2a3d1652c0999e4c449342770418cf08b43dc66fbce'

#: The scene's own JavaScript budget: 192 KiB raw for the eight modules. The
#: renderer and the generated assets are not in it; this is the code the
#: build actually writes.
#:
#: Adjudicated twice. 2026-09-02: the plan's original 90 KB was unmeetable
#: (code alone, stripped of every comment and licence header, measured 95,770
#: bytes) and the bound became 128 KiB, which the position-only ghost met with
#: headroom. AMENDED 2026-09-03 to 192 KiB: three rounds the operator asked
#: for -- the six-degree-of-freedom rotation rings, Apply, and then the
#: translate arrows that gave the third axis a handle instead of a hidden
#: modifier -- spent that headroom down to five bytes. What this number guards
#: is dependency bloat and dead code, and neither has appeared: no library
#: joined the scene and nothing here is unreached. It does not guard against
#: features that were asked for and built. The refusal to minify or strip
#: comments to fit under the old figure is deliberate: a budget met by
#: deleting the explanations is a budget that has started lying.
SCENE_JS_MAX_BYTES = 196608

#: Words that would mean the scene knows about ROS, or about this server's
#: API, or is building markup by hand.
SCENE_FORBIDDEN = ('impedance', 'rclpy', '/api/', 'topic', 'service',
                   '/dual_arm', 'innerHTML', 'style="', 'sendBeacon')


def scene_directory():
    """Return the shipped scene directory, or None when it has not landed."""
    directory = os.path.join(_PACKAGE_ROOT, 'static', 'ghost')
    return directory if os.path.isdir(directory) else None


def installed_share():
    """Return the installed share directory, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('franka_web')
    except Exception:                    # noqa: BLE001 - not installed is fine
        return None


def licence_header_lines(text):
    """Return the line numbers of the Apache header block, one-based."""
    lines = text.splitlines()
    header = set()
    for index, line in enumerate(lines, start=1):
        stripped = line.strip()
        if index > 20:
            break
        if (stripped.startswith('//') or stripped.startswith('*')
                or stripped.startswith('/*') or stripped.startswith('#')
                or stripped.startswith('<!--') or not stripped):
            header.add(index)
            continue
        break
    return header


class TestSceneStaticSurface:
    """
    What the build installs under static/, exactly.

    Half of this can only run once the scene's JavaScript has landed; those
    cases skip rather than fail on a checkout that has only the backend, so
    the gate says "not yet" instead of "broken".
    """

    def test_the_generated_assets_are_installed(self):
        """
        Sixteen mesh files, a manifest and a description, under one root.

        The scene is unusable without them and the page says so plainly, so
        an absent tree is a state -- but an INSTALLED tree that is missing
        half its meshes is a broken build, and that is what this catches.
        """
        share = installed_share()
        if share is None:
            pytest.skip('franka_web is not installed in this workspace')
        assets = os.path.join(share, 'static', 'ghost', 'assets')
        if not os.path.isdir(assets):
            pytest.skip('the scene assets have not been generated yet')
        assert os.path.isfile(os.path.join(assets, 'manifest.json'))
        assert os.path.isfile(os.path.join(assets, 'model.urdf'))
        meshes = sorted(os.listdir(os.path.join(assets, 'meshes')))
        assert len(meshes) == 16, meshes
        assert all(re.match(r'^link[0-7]\.[0-9a-f]{12}\.ghostmesh\.(json|bin)$',
                            name) for name in meshes), meshes

    def test_the_installed_static_surface_is_exactly_the_shipped_one(self):
        """
        The whole served tree, enumerated.

        The console serves this directory to a browser; a file that arrives
        here without a decision is a file nobody chose to publish.
        """
        share = installed_share()
        if share is None:
            pytest.skip('franka_web is not installed in this workspace')
        static = os.path.join(share, 'static')
        if not os.path.isdir(static):
            pytest.skip('the static tree is not installed')
        present = set()
        for parent, _directories, names in os.walk(static):
            for name in names:
                present.add(os.path.relpath(os.path.join(parent, name), static))
        expected = {'index.html', 'app.css', 'app.js',
                    os.path.join('fonts', 'OFL.txt')}
        assert expected <= present, sorted(expected - present)
        fonts = {name for name in present if name.startswith('fonts' + os.sep)}
        assert len([name for name in fonts if name.endswith('.woff2')]) == 3
        if scene_directory() is None:
            pytest.skip('the scene JavaScript has not landed yet')
        for name in SCENE_MODULES:
            assert os.path.join('ghost', name) in present, name
        for name in SCENE_VENDOR:
            assert os.path.join('ghost', 'vendor', name) in present, name
        stray = {name for name in present
                 if name.startswith('ghost' + os.sep)
                 and not name.startswith(os.path.join('ghost', 'assets'))
                 and os.path.basename(name) not in SCENE_MODULES + SCENE_VENDOR}
        assert stray == set(), sorted(stray)

    def test_the_scene_javascript_stays_inside_its_budget(self, capsys):
        """
        The adjudicated bound for the whole scene, renderer excluded.

        Every byte here is parsed on a phone before the panel opens. The
        measurement is printed so the headroom is a number in the log rather
        than something a reader has to go and take for themselves.
        """
        directory = scene_directory()
        if directory is None:
            pytest.skip('the scene JavaScript has not landed yet')
        total = sum(os.path.getsize(os.path.join(directory, name))
                    for name in SCENE_MODULES
                    if os.path.isfile(os.path.join(directory, name)))
        with capsys.disabled():
            print('\n  scene JavaScript: {:,} B of {:,} B ({:,} B spare)'.format(
                total, SCENE_JS_MAX_BYTES, SCENE_JS_MAX_BYTES - total))
        assert total <= SCENE_JS_MAX_BYTES, total

    def test_the_vendored_renderer_is_the_build_that_was_reviewed(self):
        """A substituted renderer is a substituted dependency."""
        directory = scene_directory()
        if directory is None:
            pytest.skip('the scene JavaScript has not landed yet')
        path = os.path.join(directory, 'vendor', 'three.r111.min.js')
        if not os.path.isfile(path):
            pytest.skip('the vendored renderer has not landed yet')
        with open(path, 'rb') as handle:
            payload = handle.read()
        assert len(payload) == THREE_BYTES
        assert hashlib.sha256(payload).hexdigest() == THREE_SHA256
        assert os.path.isfile(os.path.join(directory, 'vendor',
                                           'three-license.txt'))


#: A CSS value the browser will actually paint with, in the driver's own
#: spelling: ``readScenePalette`` in ``app.js`` drops anything else rather
#: than handing the module a string it cannot parse.
COLOUR_VALUE = re.compile(r'^(#[0-9a-fA-F]{3,8}|rgba?\(|hsla?\()')


def static_file(name):
    """Return the text of one shipped static file, or None."""
    path = os.path.join(_PACKAGE_ROOT, 'static', name)
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def driver_palette_tokens():
    """Return the CSS token -> module key map the console's driver reads."""
    text = static_file('app.js')
    if text is None:
        return None
    block = re.search(r'var SCENE_PALETTE_KEYS = \{(.*?)\};', text, re.S)
    assert block is not None, 'app.js no longer names its palette tokens'
    return dict(re.findall(r"'(--[a-z0-9-]+)':\s*'([A-Za-z0-9]+)'",
                           block.group(1)))


def theme_blocks(css, tokens):
    """
    Return one (selector, declarations) pair per THEME block of the sheet.

    A theme block is a ``:root`` rule that carries palette tokens; the sheet
    opens with another ``:root`` holding the font stacks and the dock
    measurements, and that one is not a theme.
    """
    found = []
    for match in re.finditer(r'(?m)^\s*(:root[^{\n]*)\{', css):
        opened = css.index('{', match.start())
        depth = 0
        for index in range(opened, len(css)):
            if css[index] == '{':
                depth += 1
            elif css[index] == '}':
                depth -= 1
                if depth == 0:
                    body = css[opened + 1:index]
                    break
        declared = dict(re.findall(r'(--[a-z0-9-]+)\s*:\s*([^;}]+)', body))
        if any(token in declared for token in tokens):
            found.append((match.group(1).strip(),
                          {name: value.strip()
                           for name, value in declared.items()}))
    return found


def resolve(declared, name, seen=None):
    """Follow one token through this block's ``var()`` chain to a value."""
    seen = seen or set()
    if name in seen or name not in declared:
        return None
    value = declared[name]
    indirect = re.match(r'^var\((--[a-z0-9-]+)\)$', value)
    if indirect:
        return resolve(declared, indirect.group(1), seen | {name})
    return value


class TestTheStylesheetOwnsEveryColourTheSceneDraws:
    """
    The colours the console actually paints the 3D view with.

    The module carries a fallback palette and the browser suite measures
    THAT: its harness page loads no stylesheet, so a drag case mounting the
    scene reads ``BUILTIN_PALETTE``. In production the driver reads the CSS
    custom properties instead and overrides every one of them, so the values
    the operator sees are the stylesheet's -- and an edit to the stylesheet
    alone would sail past every case in the browser suite. This is the gate
    that stands where that edit lands.
    """

    def tokens(self):
        """Return the driver's token map, or skip on a backend-only tree."""
        tokens = driver_palette_tokens()
        if tokens is None or static_file('app.css') is None:
            pytest.skip('the console static tree has not landed yet')
        return tokens

    def themes(self):
        """Return the sheet's theme blocks, asserting there are three."""
        tokens = self.tokens()
        blocks = theme_blocks(static_file('app.css'), tokens)
        assert len(blocks) == 3, [selector for selector, _ in blocks]
        return tokens, blocks

    def test_the_driver_asks_for_exactly_the_keys_the_module_offers(self):
        """
        Otherwise a token can be renamed on one side and go quiet.

        A key the module does not know is dropped on the floor; a key the
        driver never sends leaves the module on its fallback, which is a
        colour nobody chose and which no theme change will ever move.
        """
        scene = static_file(os.path.join('ghost', 'scene.js'))
        if scene is None:
            pytest.skip('the scene JavaScript has not landed yet')
        light = re.search(r'light:\s*\{(.*?)\n  \},', scene, re.S)
        assert light is not None, 'scene.js no longer holds a light fallback'
        offered = set(re.findall(r'(?m)([A-Za-z][A-Za-z0-9]*):\s*"#',
                                 light.group(1)))
        assert set(self.tokens().values()) == offered

    def test_every_scene_token_resolves_to_a_colour_in_every_theme(self):
        """
        In all three blocks, not just the light one.

        Two of the three are dark: the media query for the reader who never
        chose, and the explicit ``data-theme`` for the reader who did. A
        token defined in one and forgotten in another leaves the scene on
        its fallback in exactly one theme, which is the hardest kind of
        wrong colour to notice.
        """
        tokens, blocks = self.themes()
        missing = []
        for selector, declared in blocks:
            for token in sorted(tokens):
                value = resolve(declared, token)
                if value is None or not COLOUR_VALUE.match(value):
                    missing.append((selector, token, value))
        assert missing == [], missing

    def test_the_elbow_is_not_wearing_an_axis_colour_in_any_theme(self):
        """
        The defect B18 was reported for, at the file that decides it.

        "Why does the arm ring control the rotation of the hand mount" began
        with a fourth ring the same blue as the world-Z one. The browser
        case that measures the difference measures the MODULE's fallback;
        this measures the sheet the console paints from.
        """
        tokens, blocks = self.themes()
        axes = [token for token in tokens if token.startswith('--axis-')]
        clashes = []
        for selector, declared in blocks:
            for token in ('--elbow', '--elbow-active'):
                mine = (resolve(declared, token) or '').lower()
                for axis in axes:
                    if mine and mine == (resolve(declared, axis) or '').lower():
                        clashes.append((selector, token, axis, mine))
        assert clashes == [], clashes

    def test_the_elbow_says_which_state_it_is_in(self):
        """Held and idle must differ, or the highlight says nothing."""
        _tokens, blocks = self.themes()
        for selector, declared in blocks:
            idle = (resolve(declared, '--elbow') or '').lower()
            held = (resolve(declared, '--elbow-active') or '').lower()
            assert idle and held and idle != held, selector


class TestNothingOnTheSceneTestingSurfaceIsUnreached:
    """
    Every key a scene module exposes for tests is read by a case.

    The scene's byte budget guards two things, dependency bloat and dead
    code, and a testing surface is where dead code hides best: it costs the
    budget, it reads as proof, and nothing complains when the case that
    justified it goes away. Worse, an entry that RESTATES a value instead of
    reading it -- an opacity written as a literal beside the material it is
    meant to describe -- keeps saying the old thing after the material
    changes.
    """

    def surface_keys(self, text):
        """Return the top-level keys of one module's ``testing`` object."""
        opened = text.index('{', text.index('    testing: {'))
        depth = 0
        body = None
        for index in range(opened, len(text)):
            if text[index] == '{':
                depth += 1
            elif text[index] == '}':
                depth -= 1
                if depth == 0:
                    body = text[opened + 1:index]
                    break
        assert body is not None, 'the testing object is not closed'
        body = re.sub(r'/\*.*?\*/', ' ', body, flags=re.S)
        body = re.sub(r'(?m)//[^\n]*', ' ', body)
        keys = []
        depth = 0
        piece = ''
        for character in body + ',':
            if character in '{[(':
                depth += 1
            elif character in '}])':
                depth -= 1
            if character == ',' and depth == 0:
                named = re.match(r'\s*(?:get\s+|set\s+)?([A-Za-z_]\w*)', piece)
                if named:
                    keys.append(named.group(1))
                piece = ''
                continue
            piece += character
        return keys

    def test_every_testing_key_the_scene_exposes_is_read_by_a_case(self):
        """A key no case names is a key that proves nothing."""
        directory = scene_directory()
        if directory is None:
            pytest.skip('the scene JavaScript has not landed yet')
        corpus = ''
        for parent, _directories, names in os.walk(
                os.path.join(_PACKAGE_ROOT, 'test')):
            if '__pycache__' in parent or '.pytest_cache' in parent:
                continue
            for name in sorted(names):
                if name.endswith(('.js', '.py', '.html')):
                    with open(os.path.join(parent, name), encoding='utf-8',
                              errors='replace') as handle:
                        corpus += handle.read()
        unread = []
        for name in sorted(os.listdir(directory)):
            if not name.endswith('.js'):
                continue
            with open(os.path.join(directory, name), encoding='utf-8') as fh:
                text = fh.read()
            if '    testing: {' not in text:
                continue
            keys = self.surface_keys(text)
            assert keys, (name, 'no testing keys were parsed at all')
            unread += [(name, key) for key in keys
                       if not re.search(r'\.' + key + r'\b', corpus)]
        assert unread == [], unread


class TestScenePurity:
    """The scene knows about geometry, and about nothing else."""

    def scene_files(self):
        """Return every shipped scene module as (name, text)."""
        directory = scene_directory()
        if directory is None:
            pytest.skip('the scene JavaScript has not landed yet')
        found = []
        for name in sorted(os.listdir(directory)):
            path = os.path.join(directory, name)
            if os.path.isfile(path) and name.endswith('.js'):
                with open(path, encoding='utf-8') as handle:
                    found.append((name, handle.read()))
        assert found, 'the scene directory holds no modules at all'
        return found

    def test_no_scene_module_knows_about_ros_or_this_api(self):
        """
        The seam is the point: the page passes it a callback, not a URL.

        A module that named an endpoint could not be reused, could not be
        tested without a server, and would be a second place where the wire
        shape is written down.
        """
        offenders = []
        for name, text in self.scene_files():
            for word in SCENE_FORBIDDEN:
                if word in text:
                    offenders.append((name, word))
        assert offenders == [], offenders

    def test_no_scene_module_fetches_anything_off_this_origin(self):
        """
        Every URL outside the licence header is a failure.

        The header block carries the Apache licence URL, which every shipped
        file in this package has and which nothing ever fetches.
        """
        offenders = []
        for name, text in self.scene_files():
            header = licence_header_lines(text)
            for number, line in enumerate(text.splitlines(), start=1):
                if number in header:
                    continue
                if 'http://' in line or 'https://' in line:
                    offenders.append((name, number, line.strip()))
        assert offenders == [], offenders

    def test_every_scene_import_resolves_inside_the_scene(self):
        """The scene may import from itself and its vendor, and nowhere else."""
        pattern = re.compile(
            "(?:^|\\s)(?:import|export)[^'\"\\n]*from\\s+['\"]([^'\"]+)['\"]")
        offenders = []
        for name, text in self.scene_files():
            for target in pattern.findall(text):
                if not (target.startswith('./') or target.startswith('../')):
                    offenders.append((name, target))
        assert offenders == [], offenders


class TestNoPrototypePathSurvives:
    """
    Nothing shipped may name the prototype package or the notes tree.

    The prototype stays in the repository as the record of how this was
    worked out; a shipped file that pointed at it would break the moment it
    was cleaned up, and a reader with only this package would be reading a
    dangling reference either way.
    """

    def test_no_shipped_file_reaches_the_prototype_package(self):
        """
        No PATH into it, and no import of it.

        The bare name still appears in one place and legitimately so: the
        domain-allocation table names every package that holds an id,
        including the ones this package never touches. What must not exist
        is a file that reads from that tree or imports out of it, because
        that tree is frozen and will one day be cleaned up.
        """
        name = 'franka' + '_ghost'
        reach = re.compile(r'{0}/|import\s+{0}|from\s+{0}'.format(name))
        offenders = [relative for relative, text in walk_package_files()
                     if text is not None and reach.search(text)]
        assert offenders == [], offenders

    def test_no_shipped_file_names_a_planning_document(self):
        """
        A reader with the repository alone must never meet a dead reference.

        The notes-tree scan alone does not catch this: a comment naming a
        contract by filename carries no path and is just as dangling.
        """
        needles = ('GHOST_CONTRACT', 'GHOST_DESIGN', 'V2_CONTRACT', 'V2_SPEC',
                   'DECISIONS' + '.md')
        offenders = []
        for relative, text in walk_package_files():
            if text is None:
                continue
            for needle in needles:
                if needle in text:
                    offenders.append((relative, needle))
        assert offenders == [], offenders

    def test_the_scene_modules_reach_the_scans_as_readable_text(self):
        """A file the walk yields as None is walked but never scanned."""
        if scene_directory() is None:
            pytest.skip('the scene JavaScript has not landed yet')
        walked = dict(walk_package_files())
        scene = {relative for relative in walked
                 if relative.startswith(os.path.join('static', 'ghost'))}
        assert scene, 'the walk never visited the scene directory'
        assert all(walked[relative] is not None for relative in scene
                   if relative.endswith('.js'))
