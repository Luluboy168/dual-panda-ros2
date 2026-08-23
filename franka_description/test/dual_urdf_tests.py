#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from os import path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

dual_xacro_file_name = path.join(get_package_share_directory('franka_description'), 'robots',
                                 'real', 'dual_panda_arm.urdf.xacro')

# Fixed, non-routable placeholder mappings only. Never insert a real robot IP here.
dual_mappings = {
    'arm_id_1': 'panda1',
    'arm_id_2': 'panda2',
    'hand_1': 'false',
    'hand_2': 'false',
    'use_fake_hardware': 'true',
    'robot_ip_1': 'dont-care',
    'robot_ip_2': 'dont-care',
}


def _render_dual_urdf():
    return xacro.process_file(dual_xacro_file_name, mappings=dual_mappings).toxml()


def _arm_joint_names(root):
    return [joint.get('name') for joint in root.findall('joint')
            if joint.get('type') == 'revolute']


def test_dual_xacro_renders_valid_xml():
    urdf = _render_dual_urdf()
    root = ET.fromstring(urdf)  # raises xml.etree.ElementTree.ParseError if not valid XML
    assert root.tag == 'robot'


def test_dual_arm_joint_count_is_fourteen():
    root = ET.fromstring(_render_dual_urdf())
    assert len(_arm_joint_names(root)) == 14


def test_dual_arm_joint_names_are_prefixed_by_arm_id():
    root = ET.fromstring(_render_dual_urdf())
    names = _arm_joint_names(root)
    assert len(names) > 0
    for name in names:
        assert name.startswith('panda1_') or name.startswith('panda2_'), name


def test_dual_arm_joint_names_have_no_duplicates():
    root = ET.fromstring(_render_dual_urdf())
    names = _arm_joint_names(root)
    assert len(names) == len(set(names))


def test_dual_fake_hardware_uses_mock_components_generic_system_plugin():
    root = ET.fromstring(_render_dual_urdf())
    plugins = [plugin.text for plugin in root.iter('plugin')]
    assert plugins == ['mock_components/GenericSystem']


if __name__ == '__main__':
    pass
