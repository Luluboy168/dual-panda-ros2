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
Regression pins for the Stage 1 adversarial-review findings.

Each test names the finding it pins (R-numbers from the review workflow
`wf_e9c7334b-314`, triaged in the session log).
"""

import json
import math
import threading
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from franka_msgs.msg import FrankaState
from franka_web import health, sse
from franka_web.launcher import LauncherError
from franka_web.recording import RecordingError, RecordingSupervisor
from franka_web.ros_bridge import FrankaWebBridge
from franka_web.session import _Command, SessionRequest
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

    def test_simulate_never_fills_the_pose_cache(self, tmp_path):
        """S4: a simulated pose must not satisfy the §5.4 fence gate."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(6):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        assert harness.supervisor._pose_cache == {}

    def test_operator_released_without_enables_queues_nothing(self, tmp_path):
        """S2: release with nothing enabled must not enqueue disable work."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(6):
            harness.supervisor.tick()
        before = harness.supervisor._commands.qsize()
        for _ in range(10):
            harness.supervisor.operator_released()
        assert harness.supervisor._commands.qsize() == before

    def test_arm_not_enabled_maps_to_409(self):
        """S5: the §6.13 jog refusal must be 409, never 500."""
        from franka_web.http_api import ApiError
        assert ApiError('arm_not_enabled', 'x').status == 409

    def test_non_motion_session_carries_null_controller_fields(self, tmp_path):
        """S9: a simulate frame never carries stray controller/gains identity."""
        harness = Harness(tmp_path)
        harness.make_ready_simulate()
        command = _Command(kind='start', request=SessionRequest(
            arms='both', mode='simulate',
            controller_name='dual_arm_joint_impedance_controller',
            gains_sha256='deadbeef'),
            operator_lease=harness.operator_lease)
        harness.supervisor._commands.put(command)
        for _ in range(6):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        session = harness.supervisor.frame()['session']
        assert session['controller_name'] is None
        assert session['gains_sha256'] is None


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
