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

"""Shared fixtures.  The whole suite is offline: no robot, no ROS graph."""

import os
from pathlib import Path
import sys

import pytest


def package_source_dir() -> Path:
    """Return the package source directory, however the suite was started."""
    from_environment = os.environ.get('FRANKA_WSM_SOURCE_DIR')
    if from_environment:
        return Path(from_environment)
    return Path(__file__).resolve().parent.parent


SOURCE_DIR = package_source_dir()
if str(SOURCE_DIR) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIR))

CELL_MODEL_PATH = SOURCE_DIR / 'cell' / 'cell_model_v1.yaml'
LINK_GEOMETRY_PATH = SOURCE_DIR / 'cell' / 'link_geometry_v1.yaml'
CORPUS_DIR = Path(__file__).resolve().parent / 'corpus'
REPOSITORY_ROOT = SOURCE_DIR.parent

READY = (0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854)


@pytest.fixture(scope='session')
def cell_model():
    """Load the shipped cell model once for the whole session."""
    from franka_workspace_model.model import CellModel
    return CellModel.load(CELL_MODEL_PATH, profile='dual')


def make_scratch_repository(tmp_path):
    """
    Mirror just enough of the repository for a cell model to load from tmp.

    The three repository-relative sources are symlinked, so their hashes are the
    real ones; only the cell file itself is a copy the test may edit.
    """
    root = tmp_path / 'repository'
    (root / 'cell').mkdir(parents=True, exist_ok=True)
    for package in ('franka_description', 'franka_moveit_config',
                    'franka_example_controllers'):
        link = root / package
        if not link.exists():
            link.symlink_to(REPOSITORY_ROOT / package, target_is_directory=True)
    geometry = root / 'cell' / 'link_geometry_v1.yaml'
    if not geometry.exists():
        geometry.symlink_to(LINK_GEOMETRY_PATH)
    return root


def write_cell_model(tmp_path, mutate=None, text=None):
    """Write a cell model into a scratch repository and return its path."""
    import copy

    import yaml

    root = make_scratch_repository(tmp_path)
    target = root / 'cell' / 'cell_model_v1.yaml'
    if text is not None:
        target.write_text(text, encoding='utf-8')
        return target
    document = yaml.safe_load(CELL_MODEL_PATH.read_text(encoding='utf-8'))
    document = copy.deepcopy(document)
    if mutate is not None:
        mutate(document)
    target.write_text(
        yaml.safe_dump(document, default_flow_style=False, sort_keys=False,
                       allow_unicode=False),
        encoding='utf-8')
    return target


def load_mutated(tmp_path, mutate=None, text=None, profile='dual'):
    """Load a mutated copy of the shipped cell model."""
    from franka_workspace_model.model import CellModel
    return CellModel.load(write_cell_model(tmp_path, mutate, text), profile=profile)
