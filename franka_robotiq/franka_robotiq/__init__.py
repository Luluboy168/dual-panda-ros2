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
Robotiq 2F-85 adaptive gripper support for the dual-Panda cell.

This package drives a Robotiq 2F-85 over Modbus RTU on an RS-485 USB adapter.
``node.py`` is the only module that imports ROS; every other module here is
plain Python that runs, and is tested, on a machine where ROS is not
installed. That separation is what lets the whole wire protocol be proved with
``pytest`` alone, against the pty-backed emulator in ``fake.py``.

There are deliberately no re-exports. Importing a convenience name such as
``RobotiqGripper`` from the package root would drag ``driver`` -- and with it
``pyserial`` -- into every import of the package, including runs that only
touch ``protocol``.
"""

__version__ = '0.1.0'
