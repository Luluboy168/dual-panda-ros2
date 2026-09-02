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
The single rclpy node behind the web server.

``FrankaWebBridge`` lives for the whole server process; per-session
subscriptions are (re)built by :meth:`configure_session`. Every subscription
callback does bounded O(1) work: normally it stores only
``(monotonic_ns, message)`` in a latest-value slot; during torque activation it
also folds the fixed 2x7 joint set into bounded extrema. There are no queues or
per-message forwards (plan §5.1; ``/franka/joint_states`` is a 1 kHz stream).

Controller and hardware lifecycle comes primarily from
``/controller_manager/activity`` (transient-local, published on change), so
steady-state operation makes no service calls; the on-change callback kicks
one async ``list_controllers`` / ``list_hardware_components`` round to keep
the type/plugin details fresh.
"""

import collections
import threading
import time

from control_msgs.action import GripperCommand
from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState,
    SwitchController)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from franka_msgs.msg import FrankaState
from franka_msgs.srv import ErrorRecovery
from franka_web import defaults
from franka_web.ghost import IkReply
from franka_web.health import canonical_diagnostic_name, extract_joints
from franka_web.settling import ActivationSampleCapture
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

try:
    from franka_ik_interfaces.srv import SolveIk
except ImportError:
    # The console must still boot on a workspace built with
    # --packages-select franka_web. Without the interfaces the IK client is
    # None for the process's life, the scene reports the service as absent,
    # and the panel teaches the one line that fixes it.
    SolveIk = None

#: The standing IK service, named by its own contract.
SOLVE_IK_SERVICE = '/franka_ik_service/solve_ik'

_LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

_ACTIVITY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

_DIAGNOSTICS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

# The counting subscription used while an arm's command source is External.
# Depth 50 so a burst is counted rather than dropped at the middleware.
_EXTERNAL_TARGET_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

#: The window the incoming external rate is measured over.
EXTERNAL_RATE_WINDOW_S = 2.0

#: Bounded stamp ring: 2 s at 20 Hz is 40 entries; the cap makes even a 1 kHz
#: publisher cost O(1) memory and still report a correct (saturating) rate.
_EXTERNAL_STAMP_CAP = 4096

# TRANSIENT_LOCAL, mirroring the gripper node's publisher exactly: a
# mid-session subscribe is filled immediately instead of waiting a poll, which
# is what makes the gripper row correct the moment a session starts.
_GRIPPER_STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: The four Trigger services every gripper node serves.
GRIPPER_TRIGGERS = ('open', 'close', 'stop', 'reactivate')


class FrankaWebBridge(Node):
    """Latest-value view of the robot stack for the session supervisor."""

    def __init__(self):
        """Create the node and its session-independent wiring."""
        super().__init__('franka_web_bridge')
        self._cache_lock = threading.Lock()
        self._joint_callback_condition = threading.Condition()
        self._joint_callback_boundary_lock = threading.Lock()
        self._joint_callbacks_inflight = 0
        self._joint_callback_entry_closed = False
        self._joint = None
        self._robot_states = {}
        self._diagnostics = {}
        self._controller_states = {}
        self._controller_types = {}
        self._hardware = None
        self._lifecycle_gen = 0
        # Every configure/clear boundary retires the callbacks and async
        # detail queries created for the previous session.  Destroying an
        # rclpy subscription does not cancel a callback already executing.
        self._session_epoch = 0
        self._session_subs = []
        self._arm_ids = ()
        self._activation_capture = None
        self._activation_capture_generation = 0
        self._list_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self._list_hardware = self.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        self._switch_controller = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self._set_hardware_state = self.create_client(
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state')
        # Motion wiring (per session): slot -> publisher/client, arm -> client.
        self._target_publishers = {}
        self._enable_clients = {}
        self._recovery_clients = {}
        # External-source counting: arm -> subscription / slot / stamp ring.
        # Present only while that arm's command source is `external`, which is
        # exactly when the server publishes nothing on the topic -- so every
        # message counted is the operator's own.
        self._external_subs = {}
        self._external_slots = {}
        self._external_stamps = {}
        self._jog_callback = None
        # Gripper OBSERVATION and command, per session. These endpoints watch
        # and command STANDING nodes this server never launched: creating a
        # subscription to a topic nobody publishes is free, and the row stays
        # "no node is running" until one appears.
        self._gripper_subs = {}
        self._gripper_status = {}
        self._gripper_triggers = {}
        self._gripper_goals = {}
        # arm_id -> (token, force_clear_deadline_mono). One token per
        # dispatch, so a stale done-callback cannot clear a newer request.
        self._gripper_busy = {}
        self._gripper_token = 0
        # The IK service is a STANDING node with its own launch file, the
        # same pattern the grippers use. This client is created once and
        # never enters the session wiring: an IK service commands nothing,
        # so it has no drop hazard and no guardian role.
        self._solve_ik = (self.create_client(SolveIk, SOLVE_IK_SERVICE)
                          if SolveIk is not None else None)
        # The jog timer runs for the server's whole life at the contract
        # cadence; the callback slot decides whether anything is published.
        self._jog_timer = self.create_timer(
            1.0 / defaults.JOG_STREAM_HZ, self._on_jog_timer)

    # ------------------------------------------------------------------
    # Session wiring (called from the supervisor thread)
    # ------------------------------------------------------------------

    def configure_session(self, arm_ids, arm_mode, gripper_arm_ids=()):
        """
        (Re)build the per-session subscriptions for these arms.

        ``gripper_arm_ids`` are the session's arms that have a configured
        gripper, and it is EMPTY in Simulate, which gets no gripper surface at
        all. It is a DEFAULTED keyword so every existing caller and every mock
        harness that calls ``configure_session(arm_ids, arm_mode)`` keeps
        working unchanged.
        """
        self.clear_session()
        with self._cache_lock:
            self._arm_ids = tuple(arm_ids)
            epoch = self._session_epoch
        # Activity is transient-local, so a new subscription receives the
        # controller manager's current snapshot.  Making it session-owned is
        # what lets an already-running callback carry the old epoch and be
        # rejected after a teardown/reconfigure boundary.
        self._session_subs.append(self.create_subscription(
            ControllerManagerActivity, '/controller_manager/activity',
            self._activity_callback(epoch), _ACTIVITY_QOS))
        self._session_subs.append(self.create_subscription(
            JointState, '/franka/joint_states',
            self._joint_callback(epoch), _LATEST_QOS))
        for arm_id in arm_ids:
            if arm_mode == 'single':
                topic = '/franka_robot_state_broadcaster/robot_state'
            else:
                topic = '/franka_{}_robot_state_broadcaster/robot_state'.format(arm_id)
            self._session_subs.append(self.create_subscription(
                FrankaState, topic,
                self._robot_state_callback(arm_id, epoch), _LATEST_QOS))
        self._session_subs.append(self.create_subscription(
            DiagnosticArray, '/diagnostics',
            self._diagnostics_callback(epoch), _DIAGNOSTICS_QOS))
        self._configure_grippers(gripper_arm_ids, epoch)
        self._refresh_details(epoch)

    def _configure_grippers(self, gripper_arm_ids, epoch):
        """Subscribe to and hold clients for each configured gripper node."""
        subscriptions = {}
        triggers = {}
        goals = {}
        for arm_id in gripper_arm_ids:
            base = '/{}{}'.format(arm_id, defaults.GRIPPER_NODE_SUFFIX)
            subscriptions[arm_id] = self.create_subscription(
                DiagnosticStatus, base + '/status',
                self._gripper_status_callback(arm_id, epoch),
                _GRIPPER_STATUS_QOS)
            triggers[arm_id] = {
                name: self.create_client(Trigger, '{}/{}'.format(base, name))
                for name in GRIPPER_TRIGGERS}
            goals[arm_id] = ActionClient(self, GripperCommand,
                                         base + '/gripper_action')
        with self._cache_lock:
            self._gripper_subs = subscriptions
            self._gripper_triggers = triggers
            self._gripper_goals = goals

    def _gripper_status_callback(self, arm_id, epoch):
        """Build an epoch-guarded storer for one gripper node's ~/status."""
        def _store(msg):
            sample = (time.monotonic_ns(), msg)
            with self._cache_lock:
                if self._session_epoch == epoch:
                    self._gripper_status[arm_id] = sample
        return _store

    def clear_session(self):
        """Drop the per-session subscriptions and every cached sample."""
        with self._cache_lock:
            # Retire the evidence and detach every handle before calling into
            # rclpy.  A destroy may execute concurrently with an old callback
            # or even raise; either way the old epoch is already inert and no
            # stale sample remains visible.
            subscriptions = self._session_subs
            self._session_subs = []
            self._session_epoch += 1
            # Read defensively. A teardown must never be the thing that
            # fails, and this method is also driven by bridge-SHAPED test
            # harnesses that build only the caches the callback methods
            # touch -- a missing gripper cache means "there was nothing to
            # detach", which is exactly what these three defaults say.
            gripper_subs = list(getattr(self, '_gripper_subs', {}).values())
            gripper_clients = [
                client
                for clients in getattr(self, '_gripper_triggers', {}).values()
                for client in clients.values()]
            gripper_actions = list(getattr(self, '_gripper_goals', {}).values())
            self._gripper_subs = {}
            self._gripper_triggers = {}
            self._gripper_goals = {}
            self._gripper_status = {}
            self._gripper_busy = {}
            self._joint = None
            self._robot_states = {}
            self._diagnostics = {}
            self._controller_states = {}
            self._controller_types = {}
            self._hardware = None
            self._arm_ids = ()
            self._activation_capture = None
        try:
            self.set_external_counters({})
        except Exception:  # noqa: BLE001 - finish every teardown
            pass
        with self._cache_lock:
            self._external_stamps = {}
        failures = []
        for subscription in list(subscriptions) + gripper_subs:
            try:
                destroyed = self.destroy_subscription(subscription)
                if destroyed is False:
                    raise RuntimeError('destroy_subscription returned false')
            except Exception as error:  # noqa: BLE001 - finish every teardown
                failures.append('{}: {}'.format(type(error).__name__, error))
        for client in gripper_clients:
            try:
                self.destroy_client(client)
            except Exception as error:  # noqa: BLE001 - finish every teardown
                failures.append('{}: {}'.format(type(error).__name__, error))
        for action in gripper_actions:
            try:
                action.destroy()
            except Exception as error:  # noqa: BLE001 - finish every teardown
                failures.append('{}: {}'.format(type(error).__name__, error))
        if failures:
            raise RuntimeError(
                'failed to destroy {} session subscription(s): {}'.format(
                    len(failures), '; '.join(failures)))

    # ------------------------------------------------------------------
    # Reader surface (any thread)
    # ------------------------------------------------------------------

    def joint_sample(self):
        """Return the latest (mono_ns, JointState) or None."""
        with self._cache_lock:
            return self._joint

    def robot_state_sample(self, arm_id):
        """Return the latest (mono_ns, FrankaState) for the arm or None."""
        with self._cache_lock:
            return self._robot_states.get(arm_id)

    def diagnostic_sample(self, arm_id):
        """Return the latest (mono_ns, DiagnosticStatus) for the arm or None."""
        with self._cache_lock:
            return self._diagnostics.get(arm_id)

    def gripper_status_sample(self, arm_id):
        """Return the latest (mono_ns, DiagnosticStatus) for the arm, or None."""
        with self._cache_lock:
            return self._gripper_status.get(arm_id)

    def gripper_service_ready(self, arm_id, name):
        """Return True when that arm's named Trigger service is reachable."""
        with self._cache_lock:
            client = self._gripper_triggers.get(arm_id, {}).get(name)
        return bool(client is not None and client.service_is_ready())

    def gripper_busy(self, arm_id, now_mono=None):
        """
        Return True while a request of OURS is in flight for that arm.

        Set on dispatch, cleared by the future's done-callback, and
        force-cleared after ``defaults.GRIPPER_BUSY_MAX_S`` -- the contract's
        own ceiling for the node's ``motion_timeout_s`` plus one second, which
        this server cannot read because it does not launch the node -- so a
        crashed node cannot wedge the row. The frame ORs this with the
        projected ``moving``.
        """
        now = time.monotonic() if now_mono is None else float(now_mono)
        with self._cache_lock:
            entry = self._gripper_busy.get(arm_id)
            if entry is None:
                return False
            if now >= entry[1]:
                self._gripper_busy.pop(arm_id, None)
                return False
        return True

    def _mark_gripper_busy(self, arm_id):
        """Mark one arm busy and return the token that may clear it again."""
        with self._cache_lock:
            self._gripper_token += 1
            token = self._gripper_token
            self._gripper_busy[arm_id] = (
                token, time.monotonic() + defaults.GRIPPER_BUSY_MAX_S)
        return token

    def _clear_gripper_busy(self, arm_id, token):
        """Clear one arm's busy flag, but only for the dispatch that set it."""
        with self._cache_lock:
            entry = self._gripper_busy.get(arm_id)
            if entry is not None and entry[0] == token:
                self._gripper_busy.pop(arm_id, None)

    def call_gripper_trigger(self, arm_id, name,
                             timeout_s=defaults.GRIPPER_REQUEST_TIMEOUT_S):
        """
        Call one gripper Trigger service, bounded; None when unanswered.

        The wait happens on the CALLER's thread against an event the executor
        sets -- never inside a bridge callback, which would deadlock the
        single-threaded executor the server spins.
        """
        with self._cache_lock:
            client = self._gripper_triggers.get(arm_id, {}).get(name)
        token = self._mark_gripper_busy(arm_id)
        try:
            response = self._bounded_call(client, Trigger.Request(), timeout_s)
        finally:
            self._clear_gripper_busy(arm_id, token)
        if response is None:
            return None
        return {'success': bool(response.success), 'message': response.message}

    def send_gripper_trigger_async(self, arm_id, name, done=None):
        """
        Fire a Trigger without waiting; ``done`` runs on the executor thread.

        The reactivate path can take the node's whole ``activation_timeout_s``
        and no HTTP worker or supervisor tick may be held that long.
        """
        with self._cache_lock:
            client = self._gripper_triggers.get(arm_id, {}).get(name)
        if client is None or not client.service_is_ready():
            if done is not None:
                done(None)
            return False
        token = self._mark_gripper_busy(arm_id)
        future = client.call_async(Trigger.Request())

        def _finished(completed):
            """Clear the busy flag and hand the response on."""
            self._clear_gripper_busy(arm_id, token)
            try:
                response = completed.result()
            except Exception:  # noqa: BLE001 - a failed call is "no answer"
                response = None
            if done is not None:
                done(None if response is None else
                     {'success': bool(response.success),
                      'message': response.message})

        future.add_done_callback(_finished)
        return True

    def send_gripper_goal(self, arm_id, half_width_m, max_effort_n,
                          timeout_s=defaults.GRIPPER_REQUEST_TIMEOUT_S,
                          done=None):
        """
        Send one GripperCommand goal and wait ONLY for acceptance.

        Returns ``'accepted'``, ``'rejected'``, or ``None`` (client not ready,
        or no goal response within ``timeout_s``). ``done`` receives the
        RESULT when it arrives, on the executor thread, and is what clears the
        busy flag.

        ``max_effort_n`` is ALWAYS 0.0 from this server, which the action
        defines as "use the configured force_n" -- the STANDING NODE's
        force_n, which is the one actually in force. Sending this server's
        config copy instead would let a ``ros2 param set /panda1_robotiq
        force_n 40.0`` and the page disagree silently.
        """
        with self._cache_lock:
            client = self._gripper_goals.get(arm_id)
        if client is None or not client.server_is_ready():
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(half_width_m)
        goal.command.max_effort = float(max_effort_n)
        token = self._mark_gripper_busy(arm_id)
        accepted = threading.Event()
        sent = client.send_goal_async(goal)
        sent.add_done_callback(lambda _future: accepted.set())
        if not accepted.wait(timeout_s):
            self._clear_gripper_busy(arm_id, token)
            return None
        try:
            handle = sent.result()
        except Exception:  # noqa: BLE001 - a failed send is "no answer"
            handle = None
        if handle is None:
            self._clear_gripper_busy(arm_id, token)
            return None
        if not handle.accepted:
            self._clear_gripper_busy(arm_id, token)
            return 'rejected'

        def _finished(completed):
            """Clear the busy flag when the RESULT arrives, not the acceptance."""
            self._clear_gripper_busy(arm_id, token)
            if done is not None:
                try:
                    done(completed.result())
                except Exception:  # noqa: BLE001 - never kill the executor
                    done(None)

        handle.get_result_async().add_done_callback(_finished)
        return 'accepted'

    def controller_states(self):
        """Return {controller_name: lifecycle_label}."""
        with self._cache_lock:
            return dict(self._controller_states)

    def controller_types(self):
        """Return {controller_name: type_string} (from list_controllers)."""
        with self._cache_lock:
            return dict(self._controller_types)

    def hardware_component(self):
        """Return the primary hardware component dict or None."""
        with self._cache_lock:
            return dict(self._hardware) if self._hardware else None

    def begin_activation_capture(self, arm_ids):
        """Atomically arm fixed-size extrema capture before torque activation."""
        arm_ids = tuple(arm_ids)
        with self._joint_callback_boundary_lock:
            self._close_joint_callback_entry()
            try:
                with self._cache_lock:
                    if not arm_ids or arm_ids != self._arm_ids:
                        raise RuntimeError(
                            'activation capture arms do not match the configured session')
                    if self._activation_capture is not None:
                        raise RuntimeError('activation capture is already armed')
                    self._activation_capture_generation += 1
                    self._activation_capture = ActivationSampleCapture(
                        arm_ids, self._activation_capture_generation)
                    return self._activation_capture_generation
            finally:
                self._open_joint_callback_entry()

    def drain_activation_capture(self):
        """Return unseen activation extrema while leaving capture armed."""
        with self._cache_lock:
            if self._activation_capture is None:
                return None
            return self._activation_capture.drain()

    def end_activation_capture(self):
        """Atomically return final unseen extrema and disarm capture."""
        with self._joint_callback_boundary_lock:
            self._close_joint_callback_entry()
            try:
                with self._cache_lock:
                    if self._activation_capture is None:
                        return None
                    capture = self._activation_capture
                    self._activation_capture = None
                    return capture.drain()
            finally:
                self._open_joint_callback_entry()

    def finalize_activation_capture(self, observer):
        """Atomically validate the final interval and keep capture if not ready."""
        with self._joint_callback_boundary_lock:
            self._close_joint_callback_entry()
            try:
                with self._cache_lock:
                    capture = (self._activation_capture.drain()
                               if self._activation_capture is not None else None)
                    verdict = observer(capture)
                    if (self._activation_capture is not None
                            and verdict.status in ('ready', 'failed')):
                        self._activation_capture = None
                    return verdict
            finally:
                self._open_joint_callback_entry()

    def close_activation_capture(self, observer):
        """Atomically validate the final interval and always disarm capture."""
        with self._joint_callback_boundary_lock:
            self._close_joint_callback_entry()
            try:
                with self._cache_lock:
                    capture = (self._activation_capture.drain()
                               if self._activation_capture is not None else None)
                    verdict = observer(capture)
                    self._activation_capture = None
                    return verdict
            finally:
                self._open_joint_callback_entry()

    def _enter_joint_callback(self):
        """Admit one callback on exactly one side of an activation boundary."""
        with self._joint_callback_condition:
            while self._joint_callback_entry_closed:
                self._joint_callback_condition.wait()
            self._joint_callbacks_inflight += 1

    def _leave_joint_callback(self):
        """Release one admitted callback and wake a waiting boundary."""
        with self._joint_callback_condition:
            self._joint_callbacks_inflight -= 1
            if self._joint_callbacks_inflight == 0:
                self._joint_callback_condition.notify_all()

    def _close_joint_callback_entry(self):
        """Block new callbacks and wait until every admitted callback is stored."""
        with self._joint_callback_condition:
            self._joint_callback_entry_closed = True
            while self._joint_callbacks_inflight:
                self._joint_callback_condition.wait()

    def _open_joint_callback_entry(self):
        """Open callback admission after one activation boundary completes."""
        with self._joint_callback_condition:
            self._joint_callback_entry_closed = False
            self._joint_callback_condition.notify_all()

    # ------------------------------------------------------------------
    # Callbacks (executor thread; O(1) each)
    # ------------------------------------------------------------------

    def _joint_callback(self, epoch):
        """Build a joint-state storer bound to one subscription epoch."""
        def _store(msg):
            self._enter_joint_callback()
            try:
                with self._cache_lock:
                    if self._session_epoch == epoch:
                        # The entry barrier and this cache lock make the
                        # callback wholly before or wholly after capture
                        # finalization. No admitted callback can be omitted.
                        sample = (time.monotonic_ns(), msg)
                        self._joint = sample
                        if self._activation_capture is not None:
                            joints = {
                                arm_id: extract_joints(arm_id, msg)
                                for arm_id in self._activation_capture.arm_ids
                            }
                            self._activation_capture.add(sample[0], joints)
            finally:
                self._leave_joint_callback()
        return _store

    def _robot_state_callback(self, arm_id, epoch):
        """Build a per-arm FrankaState storer bound to one subscription epoch."""
        def _store(msg):
            sample = (time.monotonic_ns(), msg)
            with self._cache_lock:
                if self._session_epoch == epoch:
                    self._robot_states[arm_id] = sample
        return _store

    def _diagnostics_callback(self, epoch):
        """Build a diagnostic storer bound to one subscription epoch."""
        def _store(msg):
            now_ns = time.monotonic_ns()
            with self._cache_lock:
                if self._session_epoch != epoch:
                    return
                # Keep arm selection and writes in the same lock scope too:
                # the epoch rejects an old subscription after reconfigure;
                # this scope prevents clear/write overlap within an epoch.
                wanted = {canonical_diagnostic_name(arm_id): arm_id
                          for arm_id in self._arm_ids}
                for status in msg.status:
                    arm_id = wanted.get(status.name)
                    if arm_id is not None:
                        self._diagnostics[arm_id] = (now_ns, status)
        return _store

    def _activity_callback(self, epoch):
        """Build a controller-manager activity storer for one session epoch."""
        def _store(msg):
            with self._cache_lock:
                if self._session_epoch != epoch:
                    return
                # ControllerManagerActivity carries the complete current
                # controller/hardware state.  Replace rather than merge so an
                # unloaded controller/component cannot remain falsely active.
                self._lifecycle_gen += 1
                self._controller_states = {
                    entry.name: entry.state.label for entry in msg.controllers}
                self._controller_types = {
                    name: controller_type
                    for name, controller_type in self._controller_types.items()
                    if name in self._controller_states
                }
                hardware = None
                for entry in msg.hardware_components:
                    prior = self._hardware or {}
                    record = {
                        'name': entry.name,
                        'plugin_name': (prior.get('plugin_name')
                                        if prior.get('name') == entry.name
                                        else None),
                        'lifecycle_id': entry.state.id,
                        'lifecycle_label': entry.state.label,
                    }
                    if (hardware is None
                            or entry.name == 'FrankaMultiHardwareInterface'):
                        hardware = record
                self._hardware = hardware
            self._refresh_details(epoch)
        return _store

    def _refresh_details(self, epoch):
        """Kick async type/plugin refreshes for ``epoch``."""
        with self._cache_lock:
            if self._session_epoch != epoch:
                return
            generation = self._lifecycle_gen
        if self._list_controllers.service_is_ready():
            future = self._list_controllers.call_async(ListControllers.Request())
            future.add_done_callback(
                lambda done: self._on_list_controllers(done, generation, epoch))
        if self._list_hardware.service_is_ready():
            future = self._list_hardware.call_async(ListHardwareComponents.Request())
            future.add_done_callback(
                lambda done: self._on_list_hardware(done, generation, epoch))

    def _on_list_controllers(self, future, generation, epoch):
        """Fold a list_controllers response into the caches."""
        response = future.result()
        if response is None:
            return
        with self._cache_lock:
            if self._session_epoch != epoch:
                return
            current = self._lifecycle_gen == generation
            states = {entry.name: entry.state for entry in response.controller}
            types = {entry.name: entry.type for entry in response.controller}
            if current:
                self._controller_states = states
                self._controller_types = types
            else:
                # A newer activity event owns both membership and labels.
                # The old list may still fill a type for a controller that is
                # present now, but it cannot resurrect an absent name.
                self._controller_types = {
                    name: types.get(name, self._controller_types.get(name))
                    for name in self._controller_states
                    if types.get(name, self._controller_types.get(name)) is not None
                }

    def _on_list_hardware(self, future, generation, epoch):
        """Fold a list_hardware_components response into the cache."""
        response = future.result()
        if response is None:
            return
        with self._cache_lock:
            if self._session_epoch != epoch:
                return
            chosen = None
            for entry in response.component:
                record = {
                    'name': entry.name,
                    'plugin_name': entry.plugin_name,
                    'lifecycle_id': entry.state.id,
                    'lifecycle_label': entry.state.label,
                }
                if chosen is None or entry.name == 'FrankaMultiHardwareInterface':
                    chosen = record
            if chosen is None:
                if self._lifecycle_gen == generation:
                    self._hardware = None
                return
            if self._lifecycle_gen != generation:
                # A newer activity event owns identity, membership and the
                # lifecycle label.  Fill only the plugin detail when the old
                # response describes that exact still-present component.
                if (self._hardware is not None
                        and self._hardware.get('name') == chosen['name']):
                    self._hardware['plugin_name'] = chosen['plugin_name']
                return
            self._hardware = chosen

    # ------------------------------------------------------------------
    # Motion wiring (Stage 2; used only by motion sessions)
    # ------------------------------------------------------------------

    def configure_motion(self, arm_ids, controller_name):
        """
        Build the production-session endpoints.

        Per-arm error-recovery clients are created for EVERY production
        session (watch faults recover too). When ``controller_name`` names a
        jog controller, slot ``n`` (1-based) carries ``arm_ids[n-1]``: the
        controller names its endpoints ``~/arm_<n>/...`` and in one-arm mode
        creates only ``arm_1`` (plan §0.6). Publisher QoS matches the
        controller's subscription exactly: depth 1, reliable, volatile.
        """
        self.clear_motion()
        publishers = {}
        enables = {}
        recoveries = {}
        for slot, arm_id in enumerate(arm_ids, start=1):
            recoveries[arm_id] = self.create_client(
                ErrorRecovery,
                '/{}_error_recovery_service_server/error_recovery'.format(arm_id))
            if controller_name is not None:
                base = '/{}/arm_{}'.format(controller_name, slot)
                publishers[slot] = self.create_publisher(
                    JointTrajectory, base + '/joint_target', _LATEST_QOS)
                enables[slot] = self.create_client(SetBool, base + '/enable')
        with self._cache_lock:
            self._target_publishers = publishers
            self._enable_clients = enables
            self._recovery_clients = recoveries

    def set_external_counters(self, wanted):
        """
        Reconcile the counting subscriptions to exactly ``wanted``.

        ``wanted`` maps arm id to controller slot. Idempotent. Creating one
        resets that arm's stamp ring, so a rate is always measured from the
        moment the source was switched and never across a previous external
        interval.
        """
        wanted = dict(wanted or {})
        with self._cache_lock:
            epoch = self._session_epoch
            current = dict(self._external_slots)
            retired = [arm_id for arm_id, slot in current.items()
                       if wanted.get(arm_id) != slot]
            doomed = [self._external_subs.pop(arm_id) for arm_id in retired
                      if arm_id in self._external_subs]
            for arm_id in retired:
                self._external_slots.pop(arm_id, None)
                self._external_stamps.pop(arm_id, None)
            created = [(arm_id, slot) for arm_id, slot in wanted.items()
                       if self._external_slots.get(arm_id) != slot]
            for arm_id, _slot in created:
                self._external_stamps[arm_id] = collections.deque(
                    maxlen=_EXTERNAL_STAMP_CAP)
        for subscription in doomed:
            self.destroy_subscription(subscription)
        for arm_id, slot in created:
            subscription = self.create_subscription(
                JointTrajectory,
                '/{}/arm_{}/joint_target'.format(defaults.MOTION_CONTROLLER, slot),
                self._external_callback(arm_id, epoch), _EXTERNAL_TARGET_QOS)
            with self._cache_lock:
                self._external_subs[arm_id] = subscription
                self._external_slots[arm_id] = slot

    def _external_callback(self, arm_id, epoch):
        """Return the O(1) callback that stamps one incoming external target."""
        def _store(_message):
            now_ns = time.monotonic_ns()
            with self._cache_lock:
                if self._session_epoch != epoch:
                    return
                stamps = self._external_stamps.get(arm_id)
                if stamps is None:
                    return
                stamps.append(now_ns)
        return _store

    def external_rate_hz(self, arm_id, now_ns=None,
                         window_s=EXTERNAL_RATE_WINDOW_S):
        """Return messages/second over the sliding window, or None if not counting."""
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        cutoff = now_ns - int(window_s * 1e9)
        with self._cache_lock:
            stamps = self._external_stamps.get(arm_id)
            if stamps is None:
                return None
            while stamps and stamps[0] < cutoff:
                stamps.popleft()
            return len(stamps) / float(window_s)

    def clear_motion(self):
        """Tear down the motion endpoints (idempotent)."""
        try:
            self.set_external_counters({})
        except Exception:  # noqa: BLE001 - finish every teardown
            pass
        with self._cache_lock:
            publishers = self._target_publishers
            enables = self._enable_clients
            recoveries = self._recovery_clients
            self._target_publishers = {}
            self._enable_clients = {}
            self._recovery_clients = {}
        for publisher in publishers.values():
            self.destroy_publisher(publisher)
        for client in list(enables.values()) + list(recoveries.values()):
            self.destroy_client(client)

    def now_msg(self):
        """Return the node clock's now as a builtin_interfaces Time message."""
        return self.get_clock().now().to_msg()

    def publish_target(self, slot, message):
        """Publish one joint_target message on the slot's publisher."""
        with self._cache_lock:
            publisher = self._target_publishers.get(slot)
        if publisher is not None:
            publisher.publish(message)

    def enable_service_ready(self, slot):
        """Return True when the slot's enable service is reachable."""
        with self._cache_lock:
            client = self._enable_clients.get(slot)
        return bool(client is not None and client.service_is_ready())

    def call_enable(self, slot, enabled, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """
        Call the slot's SetBool enable service, bounded.

        Returns ``None`` when the service is missing/unanswered, else
        ``{'success': bool, 'message': str}``.
        """
        with self._cache_lock:
            client = self._enable_clients.get(slot)
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._bounded_call(client, request, timeout_s)
        if response is None:
            return None
        return {'success': bool(response.success), 'message': response.message}

    def ik_service_ready(self):
        """Return True when the standing IK node is reachable right now."""
        return bool(self._solve_ik is not None
                    and self._solve_ik.service_is_ready())

    def call_solve_ik(self, call, timeout_s=defaults.GHOST_SOLVE_TIMEOUT_S):
        """
        Call the IK service once, bounded; None when unreachable or silent.

        ``call`` is a plain dataclass, so this is the ONLY place in the
        server that names an IK message type -- which is what lets the whole
        endpoint above it be driven from a unit test with no ROS installed.

        The copy below is mechanical but not trivial: fourteen fields in a
        fixed order, and no offline test above it can see a transposed axis.
        test_ros_bridge_ik_mapping.py drives it against a hand-built request
        and response with a fake client.

        Safe from several HTTP threads at once: rclpy takes its own lock
        around send_request and keys pending futures by sequence number, so
        nothing is serialised here -- two viewers dragging must not queue
        behind each other.
        """
        if self._solve_ik is None:
            return None
        request = SolveIk.Request()
        message = request.request
        message.frame_id = call.frame_id
        message.arm_id = call.arm_id
        message.tip_frame = call.tip_frame
        message.target_pose.position.x = float(call.position[0])
        message.target_pose.position.y = float(call.position[1])
        message.target_pose.position.z = float(call.position[2])
        message.target_pose.orientation.x = float(call.orientation[0])
        message.target_pose.orientation.y = float(call.orientation[1])
        message.target_pose.orientation.z = float(call.orientation[2])
        message.target_pose.orientation.w = float(call.orientation[3])
        message.seed_positions = [float(value) for value in call.seed_positions]
        message.redundancy_mode = call.redundancy_mode
        message.redundancy_value = float(call.redundancy_value)
        message.max_solutions = call.max_solutions
        message.solver = call.solver
        message.position_tolerance = float(call.position_tolerance)
        message.orientation_tolerance = float(call.orientation_tolerance)
        message.joint_limit_margin = float(call.joint_limit_margin)
        response = self._bounded_call(self._solve_ik, request, timeout_s)
        if response is None:
            return None
        result = response.result
        solution = result.solutions[0] if result.solutions else None
        return IkReply(
            result=int(result.result),
            message=str(result.message),
            positions=tuple(solution.positions) if solution else (),
            redundancy_value=(float(solution.redundancy_value)
                              if solution else 0.0),
            position_error=float(solution.position_error) if solution else 0.0,
            orientation_error=(float(solution.orientation_error)
                               if solution else 0.0))

    def call_error_recovery(self, arm_id, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """
        Call the arm's ErrorRecovery service, bounded.

        Returns ``None`` when unreachable, else ``{'success','error'}``. A
        ``success=false, error='No errors'`` reply is informational (§0.8).
        """
        with self._cache_lock:
            client = self._recovery_clients.get(arm_id)
        response = self._bounded_call(client, ErrorRecovery.Request(), timeout_s)
        if response is None:
            return None
        return {'success': bool(response.success), 'error': response.error}

    def call_switch_activate(self, controllers, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Activate ``controllers`` via switch_controller (STRICT, asap)."""
        request = SwitchController.Request()
        request.activate_controllers = list(controllers)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        response = self._bounded_call(self._switch_controller, request, timeout_s)
        if response is None:
            return None
        return {'ok': bool(response.ok)}

    def call_switch_deactivate(self, controllers, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Deactivate ``controllers`` via switch_controller (STRICT, asap)."""
        request = SwitchController.Request()
        request.deactivate_controllers = list(controllers)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        response = self._bounded_call(self._switch_controller, request, timeout_s)
        if response is None:
            return None
        return {'ok': bool(response.ok)}

    def query_controller_states(self, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Return and cache a synchronous controller-manager view."""
        with self._cache_lock:
            generation = self._lifecycle_gen
            epoch = self._session_epoch
        response = self._bounded_call(
            self._list_controllers, ListControllers.Request(), timeout_s)
        if response is None:
            return None
        states = {entry.name: entry.state for entry in response.controller}
        types = {entry.name: entry.type for entry in response.controller}
        with self._cache_lock:
            if self._session_epoch != epoch:
                return None
            if self._lifecycle_gen != generation:
                # The activity subscription observed a later authoritative
                # state while this service request was in flight.  Never
                # overwrite or return the older service snapshot.
                self._controller_types = {
                    name: types.get(name, self._controller_types.get(name))
                    for name in self._controller_states
                    if types.get(name, self._controller_types.get(name)) is not None
                }
                return dict(self._controller_states)
            self._controller_states = dict(states)
            self._controller_types = dict(types)
        return states

    def query_hardware_component(self, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Return and cache the synchronous primary-hardware view."""
        with self._cache_lock:
            generation = self._lifecycle_gen
            epoch = self._session_epoch
        response = self._bounded_call(
            self._list_hardware, ListHardwareComponents.Request(), timeout_s)
        if response is None:
            return None
        chosen = None
        for entry in response.component:
            record = {
                'name': entry.name,
                'plugin_name': entry.plugin_name,
                'lifecycle_id': entry.state.id,
                'lifecycle_label': entry.state.label,
            }
            if chosen is None or entry.name == 'FrankaMultiHardwareInterface':
                chosen = record
        with self._cache_lock:
            if self._session_epoch != epoch:
                return None
            if self._lifecycle_gen != generation:
                # A newer activity event is the current view.  ``{}`` keeps
                # the method's existing "answered, no component" meaning.
                return dict(self._hardware) if self._hardware is not None else {}
            self._hardware = dict(chosen) if chosen is not None else None
        # Empty means the service answered but no component was observed;
        # None is reserved for an unavailable/unanswered service.
        return dict(chosen) if chosen is not None else {}

    def call_hardware_active(self, name, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Drive the named hardware component to the active lifecycle state."""
        request = SetHardwareComponentState.Request()
        request.name = name
        request.target_state = LifecycleState(
            id=LifecycleState.PRIMARY_STATE_ACTIVE, label='active')
        response = self._bounded_call(self._set_hardware_state, request, timeout_s)
        if response is None:
            return None
        return {'ok': bool(response.ok)}

    def set_jog_callback(self, callback):
        """Install (or clear, with None) the 20 Hz jog-timer callback."""
        self._jog_callback = callback

    def _on_jog_timer(self):
        """Run the installed jog callback; never let it kill the executor."""
        callback = self._jog_callback
        if callback is None:
            return
        try:
            callback()
        except Exception:  # noqa: BLE001 - the stream must outlive one bad tick
            pass

    def _bounded_call(self, client, request, timeout_s):
        """Async service call with a bounded wait; None on any failure."""
        if client is None or not client.service_is_ready():
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            future.cancel()
            return None
        try:
            return future.result()
        except Exception:
            return None
