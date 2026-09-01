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

"""T14, T15 and T16: the budget, the deferred cross-validation, and the layering."""

import ast
import os
from pathlib import Path
import subprocess
import sys
import time

from conftest import CELL_MODEL_PATH, READY, SOURCE_DIR

import pytest


CORE_MODULES = ('model', 'geometry', 'strictyaml', 'generate_link_geometry')
ROS_PREFIXES = ('rclpy', 'rcl', 'rmw', 'ament_index_python', 'builtin_interfaces',
                'std_msgs', 'geometry_msgs', 'sensor_msgs', 'shape_msgs',
                'trajectory_msgs', 'moveit', 'launch', 'rosidl')
BOTH_READY = {'panda1': list(READY), 'panda2': list(READY)}
# Section 8.6's budget for a two-degree jog, which is three samples at
# policy.max_joint_step_rad = 0.0175.  Generous headroom on the p99 so that a
# loaded build machine does not make this flaky.
CONFIGURATION_BUDGET_MS = 2.0
JOG_BUDGET_P99_MS = 10.0
TWO_DEGREES = 0.0349


@pytest.mark.parametrize('module', CORE_MODULES)
def test_no_core_module_imports_ros_statically(module):
    """T16, the static half: the core does not know the adapters exist."""
    source = (SOURCE_DIR / 'franka_workspace_model' / '{}.py'.format(module)).read_text()
    tree = ast.parse(source)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module or '')
    for name in imported:
        assert not any(name.split('.')[0] == prefix for prefix in ROS_PREFIXES), name
        assert 'ros' not in name.split('.'), name


def test_importing_the_core_in_a_clean_subprocess_pulls_in_no_ros():
    """T16, the dynamic half: run it with the ROS environment stripped."""
    script = (
        'import sys\n'
        'import franka_workspace_model.model as model\n'
        'model.CellModel\n'
        'bad = [name for name in sys.modules\n'
        "       if name.split('.')[0] in {}]\n".format(repr(list(ROS_PREFIXES))) +
        "print('LEAKED:' + ','.join(sorted(bad)))\n"
    )
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith(('ROS_', 'AMENT_', 'RMW_', 'COLCON_'))}
    environment['PYTHONPATH'] = str(SOURCE_DIR)
    completed = subprocess.run([sys.executable, '-c', script], capture_output=True,
                               encoding='utf-8', env=environment, timeout=120)
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == 'LEAKED:'


def test_the_adapter_imports_the_core_and_not_the_other_way_round():
    adapter = (SOURCE_DIR / 'franka_workspace_model' / 'ros'
               / 'description_interlock.py').read_text()
    assert 'import rclpy' in adapter
    assert 'from ..model import' in adapter
    for module in CORE_MODULES:
        source = (SOURCE_DIR / 'franka_workspace_model'
                  / '{}.py'.format(module)).read_text()
        assert '.ros' not in source.replace('franka_workspace_model.ros', 'X')


def test_check_configuration_stays_inside_the_budget(cell_model):
    """T14: the fence costs a click, not a stream."""
    for _ in range(20):
        cell_model.check_configuration(BOTH_READY)
    samples = []
    for _ in range(1000):
        started = time.perf_counter()
        cell_model.check_configuration(BOTH_READY)
        samples.append((time.perf_counter() - started) * 1000.0)
    samples.sort()
    typical = samples[len(samples) // 2]
    assert typical < CONFIGURATION_BUDGET_MS, 'median {:.3f} ms'.format(typical)


def test_a_two_degree_jog_stays_inside_the_budget(cell_model):
    for _ in range(20):
        cell_model.check_jog('panda1', BOTH_READY, 1, TWO_DEGREES)
    samples = []
    for _ in range(1000):
        started = time.perf_counter()
        result = cell_model.check_jog('panda1', BOTH_READY, 1, TWO_DEGREES)
        samples.append((time.perf_counter() - started) * 1000.0)
    assert result.result.samples_evaluated == 3
    samples.sort()
    percentile_99 = samples[int(0.99 * len(samples))]
    assert percentile_99 < JOG_BUDGET_P99_MS, 'p99 {:.3f} ms'.format(percentile_99)


def _moveit_is_available():
    try:
        import moveit_msgs  # noqa: F401
    except ImportError:
        return False
    return True


@pytest.mark.skipif(not _moveit_is_available(),
                    reason='T15 is deferred: MoveIt is not installed and '
                           'franka_moveit_config is not built into the workspace, so '
                           "the cross-validation's entry condition does not hold. It "
                           'is skipped explicitly rather than silently.')
def test_moveit_agrees_with_the_checker_on_every_corpus_entry(cell_model):
    """T15: cross-validation against an independent implementation."""
    raise AssertionError(
        'MoveIt is now installed: implement the cross-validation rather than '
        'leaving this test asserting nothing')


def test_the_installed_cell_file_is_discoverable_without_configuration():
    """R5: a consumer must be able to find a cell model without being configured."""
    from franka_workspace_model.model import default_cell_model_path
    found = default_cell_model_path()
    assert found is not None
    assert Path(found).resolve() == CELL_MODEL_PATH.resolve()
