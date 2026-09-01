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
The session-start interlock against the running robot description.

The checker is never used against a robot whose description it was not
generated from.  This adapter reads the description the graph is actually
running, hashes it, and compares it with what the loaded model records.  It is
the one place in this package that imports ROS, and the core never calls it.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient

from ..model import urdf_digest, WorkspaceModelError


DEFAULT_DESCRIPTION_NODE = 'robot_state_publisher'
DEFAULT_PARAMETER = 'robot_description'


def hash_description(text: str) -> str:
    """
    SHA-256 of the running URDF, normalised exactly as the model's was.

    The running description is expanded from the install space and the recorded
    one from a source checkout, so the two texts differ in xacro's banner - which
    names the absolute path - and in nothing else.  Both sides go through
    ``urdf_digest`` so the comparison is like for like; hashing the raw
    parameter string here would fail closed on every correct robot.
    """
    return urdf_digest(text)


def read_running_description(node: Node, description_node: str = None,
                             parameter: str = DEFAULT_PARAMETER,
                             timeout_seconds: float = 5.0) -> str:
    """
    Read the description parameter from the node that publishes it.

    The description is a node parameter, not a topic; there is no topic-based
    description anywhere in this repository.
    """
    target = description_node or DEFAULT_DESCRIPTION_NODE
    client = AsyncParameterClient(node, target)
    if not client.wait_for_services(timeout_sec=timeout_seconds):
        raise WorkspaceModelError(
            "the parameter services of node '{}' did not appear within {} s; the "
            'description interlock cannot be performed and jogging stays '
            'disabled'.format(target, timeout_seconds))
    future = client.get_parameters([parameter])
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_seconds)
    if not future.done() or future.result() is None:
        raise WorkspaceModelError(
            "node '{}' did not answer a request for parameter '{}'".format(
                target, parameter))
    values = future.result().values
    if not values or not values[0].string_value:
        raise WorkspaceModelError(
            "node '{}' reports no '{}' parameter".format(target, parameter))
    return values[0].string_value


def verify_description(model, description_text: str) -> None:
    """Fail closed when the running description is not the modelled one."""
    actual = hash_description(description_text)
    expected = model.urdf_sha256()
    if actual != expected:
        raise WorkspaceModelError(
            'the running robot description hashes to {} but the workspace model was '
            'generated against {}; the checker would compute clearances for a robot '
            'that is not the one moving, so jogging stays disabled. Regenerate the '
            'model against the current description, or launch the description the '
            'model was built from.'.format(actual, expected))
