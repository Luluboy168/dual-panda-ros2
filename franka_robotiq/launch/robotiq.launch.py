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

r"""
Start one arm's Robotiq 2F-85 driver. This is how a gripper is started.

It runs standalone: no configuration file, no web server, no operator lock.
The gripper nodes are STANDING nodes -- you start them, and they keep running
across restarts of anything else.

    ros2 launch franka_robotiq robotiq.launch.py arm_id:=panda1 \\
         serial_id:=usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0

    ros2 launch franka_robotiq robotiq.launch.py arm_id:=panda1 use_fake:=true

Parameter precedence, in increasing order of authority:

    the node's own defaults  <  params_file  <  the launch arguments below

so a ``serial_id:=`` on the command line always wins over a parameters file,
and an argument left EMPTY contributes no parameter at all rather than
overwriting a bound adapter with a blank -- which is precisely the silent
misbinding this package exists to prevent.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Arguments that name the adapter. An empty one contributes NO parameter.
BINDING_ARGUMENTS = ('serial_id', 'usb_path')

ARGUMENTS = (
    # No default: a defaulted arm id is a wrong-arm command waiting to happen.
    DeclareLaunchArgument(
        'arm_id', description='the arm this gripper is mounted on'),
    DeclareLaunchArgument(
        'serial_id', default_value='',
        description='basename of the adapter entry under /dev/serial/by-id/'),
    DeclareLaunchArgument(
        'usb_path', default_value='',
        description='basename under /dev/serial/by-path/; the alternative '
                    'binding for adapters that report no unique serial'),
    DeclareLaunchArgument(
        'use_fake', default_value='false',
        description='drive a protocol-faithful fake gripper on a pty instead '
                    'of hardware'),
    DeclareLaunchArgument(
        'fake_object_mm', default_value='30.0',
        description='width of the object the fake gripper holds; 0 for none'),
    DeclareLaunchArgument(
        'params_file', default_value='',
        description='a ROS parameters file; the launch arguments above win '
                    'over it'),
    DeclareLaunchArgument(
        'node_name', default_value='',
        description='override the node name; defaults to <arm_id>_robotiq'),
)


def gripper_node(context, *args, **kwargs):
    """Build the one Node action, resolving the parameter precedence here."""
    arm_id = LaunchConfiguration('arm_id').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    node_name = LaunchConfiguration('node_name').perform(context).strip()
    overrides = {
        'arm_id': arm_id,
        'use_fake': LaunchConfiguration('use_fake').perform(context).lower() == 'true',
        'fake_object_mm': float(
            LaunchConfiguration('fake_object_mm').perform(context)),
    }
    for name in BINDING_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    parameters = ([params_file] if params_file else []) + [overrides]
    return [Node(package='franka_robotiq', executable='robotiq_node',
                 name=node_name or '{}_robotiq'.format(arm_id),
                 output='screen', parameters=parameters)]


def generate_launch_description():
    """Return the launch description for one arm's gripper driver."""
    return LaunchDescription(list(ARGUMENTS) + [OpaqueFunction(function=gripper_node)])
