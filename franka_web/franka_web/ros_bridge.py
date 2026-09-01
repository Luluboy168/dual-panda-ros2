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

import threading
import time

from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState,
    SwitchController)
from diagnostic_msgs.msg import DiagnosticArray
from franka_msgs.msg import FrankaState
from franka_msgs.srv import ErrorRecovery
from franka_web import config
from franka_web.health import canonical_diagnostic_name, extract_joints
from franka_web.settling import ActivationSampleCapture
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

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
        self._jog_callback = None
        # The jog timer runs for the server's whole life at the contract
        # cadence; the callback slot decides whether anything is published.
        self._jog_timer = self.create_timer(
            1.0 / config.JOG_STREAM_HZ, self._on_jog_timer)

    # ------------------------------------------------------------------
    # Session wiring (called from the supervisor thread)
    # ------------------------------------------------------------------

    def configure_session(self, arm_ids, arm_mode):
        """(Re)build the per-session subscriptions for these arms."""
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
        self._refresh_details(epoch)

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
            self._joint = None
            self._robot_states = {}
            self._diagnostics = {}
            self._controller_states = {}
            self._controller_types = {}
            self._hardware = None
            self._arm_ids = ()
            self._activation_capture = None
        failures = []
        for subscription in subscriptions:
            try:
                destroyed = self.destroy_subscription(subscription)
                if destroyed is False:
                    raise RuntimeError('destroy_subscription returned false')
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

    def clear_motion(self):
        """Tear down the motion endpoints (idempotent)."""
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

    def call_enable(self, slot, enabled, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
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

    def call_error_recovery(self, arm_id, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
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

    def call_switch_activate(self, controllers, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
        """Activate ``controllers`` via switch_controller (STRICT, asap)."""
        request = SwitchController.Request()
        request.activate_controllers = list(controllers)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        response = self._bounded_call(self._switch_controller, request, timeout_s)
        if response is None:
            return None
        return {'ok': bool(response.ok)}

    def call_switch_deactivate(self, controllers, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
        """Deactivate ``controllers`` via switch_controller (STRICT, asap)."""
        request = SwitchController.Request()
        request.deactivate_controllers = list(controllers)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        response = self._bounded_call(self._switch_controller, request, timeout_s)
        if response is None:
            return None
        return {'ok': bool(response.ok)}

    def query_controller_states(self, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
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

    def query_hardware_component(self, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
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

    def call_hardware_active(self, name, timeout_s=config.SERVICE_CALL_TIMEOUT_S):
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
