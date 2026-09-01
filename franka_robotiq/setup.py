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

"""Packaging for franka_robotiq, the one owner of this package's colcon build."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'franka_robotiq'

# Every row naming a documentation or udev path is a glob() and never a file
# list. Those directories are written separately and land after this file
# does; a data_files entry naming a file that does not exist yet fails
# `colcon build` outright, while a glob that matches nothing installs nothing
# and builds clean. README.md therefore goes in through glob('*.md') too.
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'] + glob('*.md')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/doc', glob('doc/*.md')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luluboy168',
    maintainer_email='luluboy168@gmail.com',
    description='Per-arm ROS 2 drivers for two Robotiq 2F-85 grippers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['robotiq_node = franka_robotiq.node:main'],
    },
)
