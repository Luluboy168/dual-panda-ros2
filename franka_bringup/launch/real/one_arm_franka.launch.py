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
One-arm-mode bringup: a single physical (or fake) Panda through the safe controllers.

This is a sibling of ``franka.launch.py``, not a replacement for it: ``franka.launch.py`` is
pinned to the fixed ``panda`` arm ID and the legacy ``single_controllers.yaml`` demo-controller
registration and must keep working unchanged for its existing callers. This file instead loads
``one_arm_controllers.yaml`` (the three reviewed safe controllers plus the state/model
broadcasters, registration only) against the same production single-arm description
(``panda_arm.urdf.xacro`` / ``panda_arm.ros2_control.xacro``, ``robot_count=1``,
``FrankaMultiHardwareInterface`` with ``mock_components/GenericSystem`` under fake hardware) and
accepts either production arm ID, ``panda1`` or ``panda2``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from franka_bringup.launch_validation import validate_one_arm_mode_arm_id
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    robot_ip_parameter_name = 'robot_ip'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    resolved_arm_id = arm_id.perform(context)
    validate_one_arm_mode_arm_id(resolved_arm_id)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'real',
        'panda_arm.urdf.xacro',
    )
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' arm_id:=', arm_id,
         ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            'real',
            'one_arm_controllers.yaml',
        ]
    )

    # franka_robot_state_broadcaster and franka_robot_model_broadcaster are registered under a
    # fixed instance name in one_arm_controllers.yaml (the arm ID is not known until launch time),
    # so their arm_id parameter is supplied here instead of in that file. launch_ros writes an
    # unqualified parameter dict to a '/**' wildcard params file, which applies process-wide to
    # every node ros2_control_node creates -- both broadcasters declare 'arm_id' (default
    # 'panda'), so this overrides both consistently. The three safe controllers declare
    # 'arm_1.arm_id'/'arm_2.arm_id' instead, so they are unaffected, and this launch spawns none
    # of them.
    broadcaster_arm_id_overrides = {'arm_id': arm_id, 'frequency': 30}

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'],
                 'rate': 30}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description}, franka_controllers,
                broadcaster_arm_id_overrides,
            ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--switch-asap'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster', '--switch-asap'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_model_broadcaster', '--switch-asap'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            description='Hostname or IP address of the single robot.'),
        DeclareLaunchArgument(
            'arm_id',
            description="Arm ID of the single robot; must be 'panda1' or 'panda2'."),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                'use_fake_hardware')),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        OpaqueFunction(function=_launch_setup),
    ])
