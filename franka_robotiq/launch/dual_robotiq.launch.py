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
Start both arms' Robotiq 2F-85 drivers, each bound to its own adapter.

    ros2 launch franka_robotiq dual_robotiq.launch.py \\
         panda1_serial_id:=usb-... panda2_serial_id:=usb-...

    ros2 launch franka_robotiq dual_robotiq.launch.py use_fake:=true

There is no "both grippers" node and no aggregate topic: this file includes
the single-arm launch twice, and a caller that wants both calls both --
exactly as it does for the arms.

The cross-arm collision rule is enforced HERE as well as in the driver and in
the web config loader, because a dual launch is the one place two bindings
meet outside that config file. Two arms naming one adapter raise while the
launch description is being built, with ``discovery``'s own sentence: one
adapter cannot drive two grippers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from franka_robotiq import discovery
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARM_IDS = ('panda1', 'panda2')

#: The per-arm binding arguments, spelled ``<arm>_<key>``.
BINDING_KEYS = ('serial_id', 'usb_path')

_SHARED = (
    DeclareLaunchArgument(
        'use_fake', default_value='false',
        description='drive protocol-faithful fake grippers on ptys instead of '
                    'hardware'),
    DeclareLaunchArgument(
        'fake_object_mm', default_value='30.0',
        description='width of the object each fake gripper holds; 0 for none'),
    DeclareLaunchArgument(
        'params_file', default_value='',
        description='a ROS parameters file for both nodes'),
)


def _per_arm_arguments():
    """Declare ``<arm>_serial_id`` and ``<arm>_usb_path`` for both arms."""
    declared = []
    for arm_id in ARM_IDS:
        declared.append(DeclareLaunchArgument(
            '{}_serial_id'.format(arm_id), default_value='',
            description="basename of {}'s adapter under "
                        '/dev/serial/by-id/'.format(arm_id)))
        declared.append(DeclareLaunchArgument(
            '{}_usb_path'.format(arm_id), default_value='',
            description="basename of {}'s adapter under "
                        '/dev/serial/by-path/'.format(arm_id)))
    return declared


def both_grippers(context, *args, **kwargs):
    """Check the two bindings against each other, then include both launches."""
    bindings = {}
    for arm_id in ARM_IDS:
        bindings[arm_id] = tuple(
            LaunchConfiguration('{}_{}'.format(arm_id, key)).perform(context).strip()
            for key in BINDING_KEYS)
    if not LaunchConfiguration('use_fake').perform(context).lower() == 'true':
        # discovery.py owns this refusal's wording; this file does not restate
        # it. A BindingError here fails the launch before a single node
        # starts, which is the whole point: one adapter cannot drive two
        # grippers, and refusing beats silently leaving one arm unbound.
        discovery.check_cross_arm({arm_id: binding
                                   for arm_id, binding in bindings.items()
                                   if any(binding)})
    single = os.path.join(
        get_package_share_directory('franka_robotiq'), 'launch', 'robotiq.launch.py')
    included = []
    for arm_id in ARM_IDS:
        serial_id, usb_path = bindings[arm_id]
        included.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single),
            launch_arguments={
                'arm_id': arm_id,
                'serial_id': serial_id,
                'usb_path': usb_path,
                'use_fake': LaunchConfiguration('use_fake'),
                'fake_object_mm': LaunchConfiguration('fake_object_mm'),
                'params_file': LaunchConfiguration('params_file'),
            }.items()))
    return included


def generate_launch_description():
    """Return the launch description for both arms' gripper drivers."""
    return LaunchDescription(
        list(_SHARED) + _per_arm_arguments()
        + [OpaqueFunction(function=both_grippers)])
