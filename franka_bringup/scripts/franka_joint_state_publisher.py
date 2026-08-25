#!/usr/bin/env python3
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
"""Run joint_state_publisher with deterministic Python-owned SIGINT cleanup."""

import signal

from joint_state_publisher.joint_state_publisher import JointStatePublisher
import rclpy
from rclpy.signals import SignalHandlerOptions


def main(args=None):
    """Publish aggregate joint state while keeping SIGINT cleanup in the main thread."""
    stop_requested = False
    publisher = None

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        publisher = JointStatePublisher(None)
        while not stop_requested and rclpy.ok():
            rclpy.spin_once(publisher, timeout_sec=0.1)
    finally:
        if publisher is not None:
            publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
