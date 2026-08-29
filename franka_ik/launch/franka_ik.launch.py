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

"""Launch the standalone, URDF-only Franka IK service."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the robot-stack-free IK service launch description."""
    robot_description_file = LaunchConfiguration('robot_description_file')
    arm_ids = LaunchConfiguration('arm_ids')
    default_solver = LaunchConfiguration('default_solver')

    default_description = PathJoinSubstitution(
        [FindPackageShare('franka_ik'), 'urdf', 'panda_ik_dual.urdf.xacro']
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_description_file]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_description_file',
                default_value=default_description,
                description='URDF or xacro file rendered and parsed only by the IK node',
            ),
            DeclareLaunchArgument(
                'arm_ids',
                default_value='[panda1, panda2]',
                description='Configured arm-id string array',
            ),
            DeclareLaunchArgument(
                'default_solver',
                default_value='numeric',
                description='Default IK backend (numeric in the KDL-only v1 build)',
            ),
            Node(
                package='franka_ik',
                executable='franka_ik_service_node',
                name='franka_ik_service',
                namespace='/',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'arm_ids': ParameterValue(arm_ids, value_type=List[str]),
                        'default_solver': default_solver,
                    }
                ],
            ),
        ]
    )
