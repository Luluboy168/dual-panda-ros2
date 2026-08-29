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
callback does O(1) work: it stores ``(monotonic_ns, message)`` into a
latest-value slot under a lock — no queues, no per-message forwarding
(plan §5.1; ``/franka/joint_states`` is a 1 kHz stream).

Controller and hardware lifecycle comes primarily from
``/controller_manager/activity`` (transient-local, published on change), so
steady-state operation makes no service calls; the on-change callback kicks
one async ``list_controllers`` / ``list_hardware_components`` round to keep
the type/plugin details fresh.
"""

import threading
import time

from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray
from franka_msgs.msg import FrankaState
from franka_web.health import canonical_diagnostic_name
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState

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
        self._joint = None
        self._robot_states = {}
        self._diagnostics = {}
        self._controller_states = {}
        self._controller_types = {}
        self._hardware = None
        self._lifecycle_gen = 0
        self._session_subs = []
        self._arm_ids = ()
        self._activity_sub = self.create_subscription(
            ControllerManagerActivity, '/controller_manager/activity',
            self._on_activity, _ACTIVITY_QOS)
        self._list_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self._list_hardware = self.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')

    # ------------------------------------------------------------------
    # Session wiring (called from the supervisor thread)
    # ------------------------------------------------------------------

    def configure_session(self, arm_ids, arm_mode):
        """(Re)build the per-session subscriptions for these arms."""
        self.clear_session()
        with self._cache_lock:
            self._arm_ids = tuple(arm_ids)
        subs = [self.create_subscription(
            JointState, '/franka/joint_states', self._on_joint, _LATEST_QOS)]
        for arm_id in arm_ids:
            if arm_mode == 'single':
                topic = '/franka_robot_state_broadcaster/robot_state'
            else:
                topic = '/franka_{}_robot_state_broadcaster/robot_state'.format(arm_id)
            subs.append(self.create_subscription(
                FrankaState, topic, self._robot_state_callback(arm_id), _LATEST_QOS))
        subs.append(self.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, _DIAGNOSTICS_QOS))
        self._session_subs = subs
        self._refresh_details()

    def clear_session(self):
        """Drop the per-session subscriptions and every cached sample."""
        for sub in self._session_subs:
            self.destroy_subscription(sub)
        self._session_subs = []
        with self._cache_lock:
            self._joint = None
            self._robot_states = {}
            self._diagnostics = {}
            self._controller_states = {}
            self._controller_types = {}
            self._hardware = None
            self._arm_ids = ()

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

    # ------------------------------------------------------------------
    # Callbacks (executor thread; O(1) each)
    # ------------------------------------------------------------------

    def _on_joint(self, msg):
        """Store the newest joint sample, replacing the previous one."""
        sample = (time.monotonic_ns(), msg)
        with self._cache_lock:
            self._joint = sample

    def _robot_state_callback(self, arm_id):
        """Build a per-arm FrankaState storer."""
        def _store(msg):
            sample = (time.monotonic_ns(), msg)
            with self._cache_lock:
                self._robot_states[arm_id] = sample
        return _store

    def _on_diagnostics(self, msg):
        """Keep the newest canonical per-arm status from each array."""
        now_ns = time.monotonic_ns()
        with self._cache_lock:
            # One lock scope for read-arms-and-write: releasing between the
            # two would let a callback straddling clear_session() write a
            # stale sample back into the just-emptied cache.
            wanted = {canonical_diagnostic_name(arm_id): arm_id
                      for arm_id in self._arm_ids}
            for status in msg.status:
                arm_id = wanted.get(status.name)
                if arm_id is not None:
                    self._diagnostics[arm_id] = (now_ns, status)

    def _on_activity(self, msg):
        """Fold a lifecycle-change event into the caches; refresh details."""
        with self._cache_lock:
            # Every activity event advances the lifecycle generation, so an
            # in-flight service response captured before this event cannot
            # overwrite these fresher labels (activity is on-change only; a
            # stale overwrite would never be corrected).
            self._lifecycle_gen += 1
            for entry in msg.controllers:
                self._controller_states[entry.name] = entry.state.label
            hardware = None
            for entry in msg.hardware_components:
                record = {
                    'name': entry.name,
                    'plugin_name': (self._hardware or {}).get('plugin_name')
                    if (self._hardware or {}).get('name') == entry.name else None,
                    'lifecycle_id': entry.state.id,
                    'lifecycle_label': entry.state.label,
                }
                if hardware is None or entry.name == 'FrankaMultiHardwareInterface':
                    hardware = record
            if hardware is not None:
                self._hardware = hardware
        self._refresh_details()

    def _refresh_details(self):
        """Kick async type/plugin refreshes; results land in callbacks."""
        with self._cache_lock:
            generation = self._lifecycle_gen
        if self._list_controllers.service_is_ready():
            future = self._list_controllers.call_async(ListControllers.Request())
            future.add_done_callback(
                lambda done: self._on_list_controllers(done, generation))
        if self._list_hardware.service_is_ready():
            future = self._list_hardware.call_async(ListHardwareComponents.Request())
            future.add_done_callback(
                lambda done: self._on_list_hardware(done, generation))

    def _on_list_controllers(self, future, generation):
        """Fold a list_controllers response into the caches."""
        response = future.result()
        if response is None:
            return
        with self._cache_lock:
            current = self._lifecycle_gen == generation
            for entry in response.controller:
                # Types are stable facts; lifecycle labels only apply when no
                # newer activity event has arrived since this call was made.
                self._controller_types[entry.name] = entry.type
                if current:
                    self._controller_states[entry.name] = entry.state

    def _on_list_hardware(self, future, generation):
        """Fold a list_hardware_components response into the cache."""
        response = future.result()
        if response is None:
            return
        with self._cache_lock:
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
                return
            if self._lifecycle_gen != generation and self._hardware is not None:
                # A newer activity event owns the lifecycle label; take only
                # the identity details from this response.
                self._hardware['name'] = chosen['name']
                self._hardware['plugin_name'] = chosen['plugin_name']
            else:
                self._hardware = chosen
