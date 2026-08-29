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

"""Prove the robot-stack-free wrappers preserve the reviewed kinematic description."""

import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest


IK_SOURCE = Path(os.environ['FRANKA_IK_SOURCE_DIR'])
DESCRIPTION_SOURCE = Path(os.environ['FRANKA_DESCRIPTION_SOURCE_DIR'])


def _source_overlay(tmp_path):
    """Create a temporary ament-index entry resolving the exact source description."""
    prefix = tmp_path / 'source_overlay'
    marker = prefix / 'share/ament_index/resource_index/packages/franka_description'
    marker.parent.mkdir(parents=True)
    marker.write_text('', encoding='utf-8')
    share = prefix / 'share/franka_description'
    share.parent.mkdir(parents=True, exist_ok=True)
    share.symlink_to(DESCRIPTION_SOURCE, target_is_directory=True)
    return prefix


def _render(path, arguments, prefix):
    """Render one xacro in memory against the exact source-tree description."""
    executable = shutil.which('xacro')
    assert executable is not None
    environment = dict(os.environ)
    old_prefix = environment.get('AMENT_PREFIX_PATH', '')
    environment['AMENT_PREFIX_PATH'] = (
        str(prefix) if not old_prefix else f'{prefix}{os.pathsep}{old_prefix}'
    )
    result = subprocess.run(
        [executable, str(path), *arguments],
        check=True,
        capture_output=True,
        encoding='utf-8',
        env=environment,
        timeout=30,
    )
    return ET.fromstring(result.stdout)


def _attributes(element):
    """Return sorted attributes, or an empty tuple for an absent element."""
    return () if element is None else tuple(sorted(element.attrib.items()))


def _kinematic_projection(robot):
    """Project a URDF onto the complete link/joint kinematic contract."""
    links = frozenset(link.attrib['name'] for link in robot.findall('link'))
    joints = {}
    for joint in robot.findall('joint'):
        name = joint.attrib['name']
        joints[name] = (
            joint.attrib['type'],
            _attributes(joint.find('parent')),
            _attributes(joint.find('child')),
            _attributes(joint.find('origin')),
            _attributes(joint.find('axis')),
            _attributes(joint.find('limit')),
            _attributes(joint.find('mimic')),
        )
    child_links = {
        joint.find('child').attrib['link']
        for joint in robot.findall('joint')
        if joint.find('child') is not None
    }
    roots = links - child_links
    return roots, links, joints


@pytest.mark.parametrize('include_hand', [False, True])
def test_single_wrapper_matches_real_kinematics(tmp_path, include_hand):
    """The single wrapper matches the real fake-rendered description."""
    prefix = _source_overlay(tmp_path)
    hand = str(include_hand).lower()
    wrapper = _render(
        IK_SOURCE / 'urdf/panda_ik_single.urdf.xacro',
        [f'hand:={hand}'],
        prefix,
    )
    real = _render(
        DESCRIPTION_SOURCE / 'robots/real/panda_arm.urdf.xacro',
        ['robot_ip:=dont-care', 'use_fake_hardware:=true', f'hand:={hand}'],
        prefix,
    )
    assert _kinematic_projection(wrapper) == _kinematic_projection(real)
    _assert_stack_boundary(wrapper, real)


@pytest.mark.parametrize('include_hand', [False, True])
def test_dual_wrapper_matches_real_kinematics(tmp_path, include_hand):
    """The dual wrapper matches the real fake-rendered description."""
    prefix = _source_overlay(tmp_path)
    hand = str(include_hand).lower()
    wrapper = _render(
        IK_SOURCE / 'urdf/panda_ik_dual.urdf.xacro',
        [f'hand_1:={hand}', f'hand_2:={hand}'],
        prefix,
    )
    real = _render(
        DESCRIPTION_SOURCE / 'robots/real/dual_panda_arm.urdf.xacro',
        [
            'robot_ip_1:=dont-care',
            'robot_ip_2:=dont-care',
            'use_fake_hardware:=true',
            f'hand_1:={hand}',
            f'hand_2:={hand}',
        ],
        prefix,
    )
    assert _kinematic_projection(wrapper) == _kinematic_projection(real)
    _assert_stack_boundary(wrapper, real)


def _assert_stack_boundary(wrapper, real):
    """Assert only the real fixture contains ros2_control and its fake plugin."""
    assert wrapper.find('ros2_control') is None
    assert not wrapper.findall('.//plugin')
    real_control = real.find('ros2_control')
    assert real_control is not None
    plugins = [plugin.text.strip() for plugin in real_control.findall('.//plugin')]
    assert plugins == ['mock_components/GenericSystem']
