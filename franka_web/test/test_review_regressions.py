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

from franka_web import health, sse
from franka_web.launcher import LauncherError
from franka_web.recording import RecordingError, RecordingSupervisor
from franka_web.session import _Command, SessionRequest
import pytest
from sensor_msgs.msg import JointState
from support.fake_launcher import FakeRecording
from test_session_state_machine import Harness


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
