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
The zero-ROS rule, proved rather than asserted.

``node.py`` is the only module in this package allowed to import ROS. Every
other module must run on a machine where ROS is not installed, which is what
lets the whole wire protocol be tested with plain pytest. Two independent
checks enforce it: an AST walk, which sees every static import, and a
subprocess import, which catches a dynamic one the AST cannot see.
"""

import ast
import os
import pathlib
import subprocess
import sys

import franka_robotiq

import pytest

#: Root module names that mean ROS is in the import graph.
DENY_ROOTS = {'rclpy', 'rclpy_action', 'rcl_interfaces', 'ament_index_python',
              'rosidl_runtime_py', 'launch', 'launch_ros', 'launch_testing',
              'action_msgs', 'builtin_interfaces', 'rosbag2_py', 'tf2_ros',
              'std_msgs', 'std_srvs', 'sensor_msgs', 'geometry_msgs',
              'diagnostic_msgs', 'control_msgs', 'trajectory_msgs',
              'franka_msgs', 'controller_manager_msgs'}
#: Any future message or interface package, caught by shape.
DENY_PATTERNS = ('_msgs', '_srvs', '_interfaces')
DENY_PREFIXES = ('rcl', 'ros', 'ament_', 'launch')

#: The ONE exemption, and it is exactly one file with exactly one name.
EXCLUDED_FILES = {'node.py'}

#: protocol.py may import the intra-package registers module: the function
#: codes and bit masks have exactly one home, and duplicating them would
#: create the second home that module exists to prevent.
PROTOCOL_INTRAPACKAGE_ALLOW = {'franka_robotiq.registers', 'registers'}

PACKAGE_DIR = pathlib.Path(franka_robotiq.__file__).parent
CORE_MODULES = ('registers', 'protocol', 'units', 'driver', 'discovery',
                'fake')


def _core_files():
    """
    Every shipped module except the one named exemption. A glob, not a list.

    A seventh core module added later must be scanned automatically, or this
    check rots into decoration.
    """
    return sorted(path for path in PACKAGE_DIR.glob('*.py')
                  if path.name not in EXCLUDED_FILES)


def _imported_roots(path):
    """Every root module name imported anywhere in ``path``."""
    tree = ast.parse(path.read_text())
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                continue
            if node.module:
                roots.add(node.module.split('.')[0])
    return roots


def _is_ros(name):
    """Return True when a root module name belongs to the ROS ecosystem."""
    if name in DENY_ROOTS:
        return True
    if name.startswith(DENY_PREFIXES):
        return True
    return any(pattern in name for pattern in DENY_PATTERNS)


def test_core_modules_import_no_ros():
    """No shipped module but node.py may reach ROS, statically."""
    files = _core_files()
    assert len(files) >= 7, 'the glob found nothing; the check would be empty'
    offenders = []
    for path in files:
        for name in _imported_roots(path):
            if _is_ros(name):
                offenders.append('{}: {}'.format(path.name, name))
    assert offenders == []


def test_node_py_is_excluded_by_name_and_only_by_name():
    """One exemption, spelled out, so a second cannot be added quietly."""
    assert EXCLUDED_FILES == {'node.py'}


def test_the_glob_covers_every_module_this_part_ships():
    """The scan is a glob over the package, not a hand-maintained list."""
    scanned = {path.stem for path in _core_files()}
    assert set(CORE_MODULES) <= scanned
    assert '__init__' in scanned


def test_importing_the_core_modules_does_not_load_rclpy():
    """
    The AST cannot see a dynamic import. A subprocess can.

    The child runs with the package's parent directory on the path and
    imports every core module, then asserts rclpy never entered sys.modules.
    """
    script = (
        'import sys\n'
        'import franka_robotiq\n'
        + ''.join('from franka_robotiq import {}\n'.format(name)
                  for name in CORE_MODULES)
        + 'bad = [name for name in sys.modules\n'
          '       if name.split(".")[0] in {"rclpy", "rcl_interfaces",\n'
          '                                 "ament_index_python"}]\n'
          'assert not bad, bad\n'
          'print("clean")\n')
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(PACKAGE_DIR.parent)
    result = subprocess.run([sys.executable, '-c', script],
                            capture_output=True, text=True, timeout=60,
                            env=environment)
    assert result.returncode == 0, result.stderr
    assert 'clean' in result.stdout


@pytest.mark.parametrize('module_name', ['protocol', 'units', 'registers'])
def test_protocol_units_registers_import_only_the_standard_library(
        module_name):
    """
    The pure trio reaches for nothing but the standard library.

    ``protocol`` gets one named exemption -- the intra-package ``registers``
    module -- and no other. ``units`` gets none at all, which is what keeps
    physics and wire apart, and ``registers`` gets none, which is what keeps
    the import direction acyclic.
    """
    path = PACKAGE_DIR / (module_name + '.py')
    roots = _imported_roots(path)
    intrapackage = {name for name in roots if name == 'franka_robotiq'}
    stdlib = roots - intrapackage
    for name in stdlib:
        assert name in sys.stdlib_module_names, \
            '{} imports {}, which is not in the standard library'.format(
                module_name, name)

    tree = ast.parse(path.read_text())
    intra_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == 'franka_robotiq':
            intra_names.update('franka_robotiq.' + alias.name
                               for alias in node.names)
        elif isinstance(node, ast.Import):
            intra_names.update(alias.name for alias in node.names
                               if alias.name.startswith('franka_robotiq'))
    if module_name == 'protocol':
        assert intra_names <= PROTOCOL_INTRAPACKAGE_ALLOW
        assert len(PROTOCOL_INTRAPACKAGE_ALLOW & intra_names) == 1
    else:
        assert intra_names == set()


def test_the_protocol_exemption_is_exactly_one_name():
    """A one-name allow-set cannot be widened into a hole unnoticed."""
    assert PROTOCOL_INTRAPACKAGE_ALLOW == {'franka_robotiq.registers',
                                           'registers'}


def test_the_package_is_a_real_package_not_a_namespace_package():
    """
    A namespace-package shadow turns every failure into a missing module.

    The repository root holds a directory named ``franka_robotiq`` whose child
    is the package of the same name. Without the inner directory on the path,
    ``import franka_robotiq`` binds to the outer one as a namespace package
    and nothing under it resolves.
    """
    assert franka_robotiq.__file__ is not None, (
        'franka_robotiq resolved as a namespace package. Run pytest with '
        'PYTHONPATH=<repo>/franka_robotiq so the inner package is the one '
        'that is imported.')
    assert franka_robotiq.__version__ == '0.1.0'


def test_no_shipped_module_names_a_notes_tree_path():
    """Operator-facing text points at the installed docs, never at a notes tree."""
    # The banned string is built from two halves on purpose: this file is
    # itself inside the package the rule covers, so spelling it whole would
    # make the check its own first violation.
    banned = 'multipanda_ros2_jazzy' + '_notes'
    for path in PACKAGE_DIR.glob('*.py'):
        assert banned not in path.read_text(), path.name
