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

"""Fake child processes, spawners, bridge, broker and friends for tests."""

from franka_web.health import extract_joints
from franka_web.settling import ActivationSampleCapture


class FakeChild:
    """A controllable stand-in for launcher.ChildProcess."""

    def __init__(self, name='child', pid=4242, on_line=None):
        """Start out alive with no recorded signals."""
        self.name = name
        self.pid = pid
        self.on_line = on_line
        self._alive = True
        self._returncode = None
        self.signals = []
        self.stop_calls = []
        self.lines = []
        self.stopped_by = 'sigint'

    def die(self, returncode=0):
        """Simulate the child exiting on its own."""
        self._alive = False
        self._returncode = returncode

    def alive(self):
        """Return the controllable liveness flag."""
        return self._alive

    def returncode(self):
        """Return the recorded exit status, if any."""
        return self._returncode

    def output_tail(self, limit=50):
        """Return the lines this child has emitted, newest last."""
        return self.lines[-limit:] if limit > 0 else []

    def emit(self, text):
        """
        Simulate the child printing one line of merged output.

        The line goes into this child's ring AND, if the spawn site passed
        one, into the log-bus sink -- which is what proves the wiring at that
        spawn site is really there.
        """
        self.lines.append(text)
        if self.on_line is not None:
            self.on_line(text)

    def send_signal(self, signum):
        """Record a per-pid signal."""
        self.signals.append(('pid', signum))

    def signal_group(self, signum):
        """Record a process-group signal."""
        self.signals.append(('group', signum))

    def stop(self, sigint_wait_s, sigterm_wait_s, sigkill_wait_s):
        """Record the stop call and mark the child dead."""
        self.stop_calls.append((sigint_wait_s, sigterm_wait_s, sigkill_wait_s))
        self._alive = False
        self._returncode = 0
        return self.stopped_by

    def wait_exited(self, timeout_s):
        """Report whether the child has exited."""
        return not self._alive


class FakeSpawner:
    """Records spawns and hands out preset FakeChild objects."""

    def __init__(self):
        """Start with an empty script of children to hand out."""
        self.children = []
        self.spawned = []
        self.fail_names = set()

    def queue_child(self, child):
        """Queue the next child to hand out."""
        self.children.append(child)

    def __call__(self, argv, env, name, on_line=None, **kwargs):
        """
        Spawn: record the call (``on_line`` included), fail if scripted.

        ``on_line`` is accepted and RECORDED rather than swallowed: both the
        session's launch spawn and the recording supervisor's spawn pass one,
        and a fake that could not take the keyword would be the one thing
        able to break that wiring silently.
        """
        from franka_web.launcher import LauncherError
        options = dict(kwargs)
        options['on_line'] = on_line
        self.spawned.append({'argv': tuple(argv), 'env': dict(env), 'name': name,
                             'options': options})
        if name in self.fail_names:
            raise LauncherError('scripted spawn failure for {}'.format(name))
        if self.children:
            child = self.children.pop(0)
        else:
            child = FakeChild(name=name)
        child.on_line = on_line
        return child


class FakeRecording:
    """A scripted stand-in for recording.RecordingSupervisor."""

    def __init__(self, events=None, fail_start=False):
        """Optionally share an ordered ``events`` list with other fakes."""
        self.events = events if events is not None else []
        self.fail_start = fail_start
        self.started = None
        self.ticks = 0
        self.stopped = False

    @property
    def active(self):
        """Return True between start and stop."""
        return self.started is not None and not self.stopped

    def start(self, base_name, arm_mode, env):
        """Record the start; raise RecordingError when scripted to."""
        from franka_web.recording import RecordingError
        if self.fail_start:
            raise RecordingError('scripted recorder failure')
        self.started = (base_name, arm_mode)
        self.events.append('recorder-start')

    def tick(self, env):
        """Count supervisor ticks."""
        self.ticks += 1

    def stop(self):
        """Record the seal ordering."""
        self.stopped = True
        self.events.append('recorder-stop')
        return 'sigint'

    def frame(self, topics):
        """Return a recording frame with the real name/path pairing."""
        if self.started is None:
            return {'active': False, 'name': None, 'sequence': 0,
                    'path': None, 'arm_mode': None, 'topics': []}
        return {
            'active': self.active,
            'name': self.started[0],
            'sequence': 1,
            'path': '/recordings/{}'.format(self.started[0]),
            'arm_mode': self.started[1],
            'topics': list(topics),
        }


class FakeBridge:
    """A settable stand-in for the ros_bridge interface session.py consumes."""

    def __init__(self):
        """Start with an empty, not-ready graph view."""
        self.controllers = {}
        self.types = {}
        self.joint = None
        self.robot_states = {}
        self.diagnostics = {}
        self.hardware = None
        self.configured = None
        self.cleared = 0
        self._activation_capture = None
        self._activation_capture_generation = 0

    def configure_session(self, arm_ids, arm_mode):
        """Record the session wiring request."""
        self.configured = (tuple(arm_ids), arm_mode)

    def clear_session(self):
        """Record the teardown."""
        self.cleared += 1
        self._activation_capture = None

    def controller_states(self):
        """Return the scripted controller lifecycle map."""
        return dict(self.controllers)

    def query_controller_states(self, timeout_s=5.0):
        """Return the scripted fresh controller-manager view."""
        return getattr(self, 'controller_query_response', dict(self.controllers))

    def controller_types(self):
        """Return the scripted controller type map."""
        return dict(self.types)

    def joint_sample(self):
        """Return the scripted (mono_ns, JointState) sample."""
        return self.joint

    def set_joint_sample(self, stamp, message):
        """Store a fake callback and feed any armed activation capture."""
        self.joint = (int(stamp), message)
        if self._activation_capture is not None:
            self._activation_capture.add(int(stamp), {
                arm_id: extract_joints(arm_id, message)
                for arm_id in self._activation_capture.arm_ids
            })

    def begin_activation_capture(self, arm_ids):
        """Arm the same bounded capture surface as the production bridge."""
        arm_ids = tuple(arm_ids)
        if not arm_ids or self.configured is None or arm_ids != self.configured[0]:
            raise RuntimeError(
                'activation capture arms do not match the configured session')
        if self._activation_capture is not None:
            raise RuntimeError('activation capture is already armed')
        self._activation_capture_generation += 1
        self._activation_capture = ActivationSampleCapture(
            arm_ids, self._activation_capture_generation)
        return self._activation_capture_generation

    def drain_activation_capture(self):
        """Return unseen fake extrema while retaining capture."""
        if self._activation_capture is None:
            return None
        return self._activation_capture.drain()

    def end_activation_capture(self):
        """Return final unseen fake extrema and disarm capture."""
        if self._activation_capture is None:
            return None
        capture = self._activation_capture
        self._activation_capture = None
        return capture.drain()

    def finalize_activation_capture(self, observer):
        """Apply a final fake interval without a close/re-arm observation gap."""
        capture = (self._activation_capture.drain()
                   if self._activation_capture is not None else None)
        verdict = observer(capture)
        if (self._activation_capture is not None
                and verdict.status in ('ready', 'failed')):
            self._activation_capture = None
        return verdict

    def close_activation_capture(self, observer):
        """Apply a final fake interval and always disarm the capture."""
        capture = (self._activation_capture.drain()
                   if self._activation_capture is not None else None)
        verdict = observer(capture)
        self._activation_capture = None
        return verdict

    def robot_state_sample(self, arm_id):
        """Return the scripted per-arm FrankaState sample."""
        return self.robot_states.get(arm_id)

    def diagnostic_sample(self, arm_id):
        """Return the scripted per-arm DiagnosticStatus sample."""
        return self.diagnostics.get(arm_id)

    def hardware_component(self):
        """Return the scripted hardware component dict."""
        return self.hardware

    def query_hardware_component(self, timeout_s=5.0):
        """Return the scripted fresh hardware view; empty means unobserved."""
        if hasattr(self, 'hardware_query_response'):
            return self.hardware_query_response
        return dict(self.hardware) if self.hardware is not None else {}

    # -- motion surface (Stage 2) --------------------------------------

    def configure_motion(self, arm_ids, controller_name):
        """Record the motion wiring request."""
        self.motion_configured = (tuple(arm_ids), controller_name)

    def clear_motion(self):
        """Record the motion teardown."""
        self.motion_cleared = getattr(self, 'motion_cleared', 0) + 1

    def now_msg(self):
        """
        Return a monotonically advancing builtin_interfaces Time.

        Never zero: a zero stamp is exactly what the jog model (and the
        controller's InvalidStamp rule) refuse, so a zeroed fake would make
        stream publishing untestable through this bridge.
        """
        from builtin_interfaces.msg import Time
        self._now_calls = getattr(self, '_now_calls', 0) + 1
        return Time(sec=1000 + self._now_calls, nanosec=1)

    def publish_target(self, slot, message):
        """Record a published joint target."""
        self.published_targets = getattr(self, 'published_targets', [])
        self.published_targets.append((slot, message))

    def enable_service_ready(self, slot):
        """Return the scripted readiness (default True)."""
        return getattr(self, 'enable_ready', True)

    def call_enable(self, slot, enabled, timeout_s=5.0):
        """Return the scripted enable response (default success)."""
        self.enable_calls = getattr(self, 'enable_calls', [])
        self.enable_calls.append((slot, enabled))
        return getattr(self, 'enable_response',
                       {'success': True, 'message': 'enabled; measured target '
                                                    'retained while awaiting a '
                                                    'fresh valid target'})

    def call_error_recovery(self, arm_id, timeout_s=5.0):
        """Return the scripted recovery response (default success)."""
        responses = getattr(self, 'recovery_responses', {})
        if arm_id in responses:
            return responses[arm_id]
        return getattr(self, 'recovery_response', {'success': True, 'error': ''})

    def call_switch_activate(self, controllers, timeout_s=5.0):
        """Return the scripted switch response (default ok)."""
        response = getattr(self, 'switch_response', {'ok': True})
        if response is not None and response['ok']:
            for controller in controllers:
                self.controllers[controller] = 'active'
        return response

    def call_switch_deactivate(self, controllers, timeout_s=5.0):
        """Return the scripted deactivate response (default ok)."""
        response = getattr(self, 'deactivate_response', {'ok': True})
        if response is not None and response['ok']:
            for controller in controllers:
                self.controllers[controller] = 'inactive'
        return response

    def call_hardware_active(self, name, timeout_s=5.0):
        """Return the scripted hardware-activation response (default ok)."""
        response = getattr(self, 'hardware_response', {'ok': True})
        if response is not None and response['ok']:
            self.hardware = {
                'name': name,
                'plugin_name': 'franka_hardware/FrankaMultiHardwareInterface',
                'lifecycle_id': 3,
                'lifecycle_label': 'active',
            }
        return response


class FakeBroker:
    """Records every published SSE event."""

    def __init__(self):
        """Start with no published events."""
        self.events = []

    def publish(self, event, data):
        """Record the event name and payload."""
        self.events.append((event, data))


class FakeLock:
    """A minimal operator-lock stand-in for frame assembly."""

    def state(self):
        """Report an unheld lock."""
        return {'locked': False, 'expires_in_s': None}


class FakePreflightResult:
    """Duck-typed PreflightResult for scripting session behaviour."""

    def __init__(self, overall='PASS', passed=True, blocking=False, error=None):
        """Script the verdict, and optionally the invocation-level reason."""
        self.overall = overall
        self.passed = passed
        self.blocking = blocking
        # Mirrors PreflightResult.error: set only for an ERROR verdict, where
        # it is the only account of WHY the run could not be made or
        # understood. The supervisor puts it in last_error (finding F-2).
        self.error = error

    def blocks_start(self):
        """Mirror preflight.PreflightResult.blocks_start."""
        return self.blocking and not self.passed

    def frame(self):
        """Mirror the §6.11 preflight block."""
        return {'ran_at': '2026-08-29T00:00:00.000000Z', 'overall': self.overall,
                'blocking': self.blocking, 'failed_checks': []}
