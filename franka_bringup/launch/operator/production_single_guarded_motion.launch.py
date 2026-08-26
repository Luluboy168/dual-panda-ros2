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

from franka_bringup.operator_launch import production_single_guarded_motion_setup
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            description=(
                'Required Panda address; this launch has no production address default.')),
        DeclareLaunchArgument(
            'arm_id',
            description=(
                "Arm ID of the single robot; must be 'panda1' or 'panda2'.")),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='Operational confirmation; only the literal true passes.'),
        DeclareLaunchArgument(
            'controller_name', default_value='',
            description='Exact reviewed controller name.'),
        DeclareLaunchArgument(
            'controller_param_file', default_value='',
            description=(
                'Absolute regular non-symlink full controller parameter file. Must set '
                'arm_count: 1 and only arm_1.* (arm_1.arm_id matching the arm_id argument).'),
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Visualize the guarded production system.'),
        OpaqueFunction(function=production_single_guarded_motion_setup),
    ])
