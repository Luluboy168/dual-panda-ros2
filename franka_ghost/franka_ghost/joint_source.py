# [THROWAWAY] Session C standalone state sources; franka_web owns the merged stream.
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

"""Development joint-state sources for the standalone ghost prototype."""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Callable, Dict, Optional, Tuple


ROS_DOMAIN_ID = '82'
URDF_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
URDF_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
INITIAL_POSITIONS = (
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
)
CANONICAL_JOINT_NAMES = tuple(
    f'panda{arm}_joint{joint}' for arm in (1, 2) for joint in range(1, 8)
)


Sample = Tuple[Optional[int], Dict[str, float]]


def require_ros_domain_id(
    environment: Optional[dict] = None,
    required_domain_id: str = ROS_DOMAIN_ID,
) -> None:
    """Reject ROS mode unless the explicitly required DDS domain is selected."""
    env = os.environ if environment is None else environment
    actual = env.get('ROS_DOMAIN_ID')
    if actual != required_domain_id:
        shown = 'unset' if actual is None else repr(actual)
        raise RuntimeError(
            f'ROS source requires ROS_DOMAIN_ID={required_domain_id}; current value is {shown}'
        )


class DemoJointSource:
    """Generate deterministic, bounded motion without importing or running ROS."""

    source = 'demo'

    def __init__(
        self,
        clock_ns: Callable[[], int] = time.time_ns,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._clock_ns = clock_ns
        self._monotonic_ns = monotonic_ns
        self._started_ns = monotonic_ns()

    def latest(self) -> Sample:
        """Return all 14 joints, starting at the fake-hardware initial pose."""
        elapsed_s = max(0.0, (self._monotonic_ns() - self._started_ns) / 1.0e9)
        joints: Dict[str, float] = {}
        for arm in (1, 2):
            direction = 1.0 if arm == 1 else -1.0
            for offset, initial in enumerate(INITIAL_POSITIONS):
                clearance = min(initial - URDF_LOWER[offset], URDF_UPPER[offset] - initial)
                amplitude = 0.35 * clearance
                angular_speed = 0.13 + 0.017 * offset + 0.011 * (arm - 1)
                position = initial + direction * amplitude * math.sin(angular_speed * elapsed_s)
                joints[f'panda{arm}_joint{offset + 1}'] = position
        return int(self._clock_ns()), joints

    def close(self) -> None:
        """Match the ROS source lifecycle API."""


class RosJointSource:
    """Poll the newest by-name sample from a depth-one best-effort subscription."""

    source = 'ros'

    def __init__(
        self,
        topic: str = '/franka/joint_states',
        required_domain_id: str = ROS_DOMAIN_ID,
    ) -> None:
        # The production dev CLI never overrides this default (interactive domain 82).
        # Registered launch tests inject their separately allocated test domain instead.
        require_ros_domain_id(required_domain_id=required_domain_id)

        # Keep ROS imports out of demo-only development and unit-test processes.
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = rclpy.create_node('franka_ghost_joint_source', context=self._context)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._lock = threading.Lock()
        self._latest: Sample = (None, {})
        self._closed = False
        self._subscription = self._node.create_subscription(
            JointState, topic, self._on_joint_state, qos
        )

    def _on_joint_state(self, message: object) -> None:
        names = getattr(message, 'name', ())
        positions = getattr(message, 'position', ())
        joints: Dict[str, float] = {}
        canonical = set(CANONICAL_JOINT_NAMES)
        for name, position in zip(names, positions):
            value = float(position)
            if name in canonical and math.isfinite(value):
                joints[name] = value

        stamp = getattr(getattr(message, 'header', None), 'stamp', None)
        stamp_ns = 0
        if stamp is not None:
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(self._node.get_clock().now().nanoseconds)

        # The callback never queues application work: each message atomically replaces the last.
        with self._lock:
            self._latest = (stamp_ns, joints)

    def latest(self) -> Sample:
        """Copy the newest sample for rate-limited promotion by the HTTP bridge."""
        # SourcePump calls latest() at the configured browser output rate. This
        # is the same installed Jazzy take primitive used by rclpy's executor;
        # using it directly means the depth-one queue yields one newest sample
        # without making Python drain the ~1 kHz source between pump ticks.
        with self._subscription.handle:
            message_info = self._subscription.handle.take_message(
                self._subscription.msg_type, self._subscription.raw
            )
        if message_info is not None:
            self._on_joint_state(message_info[0])
        with self._lock:
            stamp_ns, joints = self._latest
            return stamp_ns, dict(joints)

    def close(self) -> None:
        """Stop the private ROS context and executor; safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        self._node.destroy_node()
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)


def create_joint_source(kind: str, topic: str = '/franka/joint_states') -> object:
    """Construct one of the two deliberately small development sources."""
    if kind == 'demo':
        return DemoJointSource()
    if kind == 'ros':
        return RosJointSource(topic=topic)
    raise ValueError(f"unknown joint source {kind!r}; expected 'demo' or 'ros'")
