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
A cell-model fixture BUILDER, deliberately not a checked-in pair of files.

The cell model records the digests of the description it was measured
against, so a static fixture would be invalid the moment that description
changed -- including on the very merge this build depends on, the one that
moved the two arms a metre apart. Building the fixture at test time from the
files that are actually installed makes that whole class of staleness
unrepresentable.

The scratch copy is what a test may edit: the sources beside it are
symlinks, so their digests are the real ones.
"""

import os
from pathlib import Path
import shutil

#: The packages a cell model names as its sources.
SOURCE_PACKAGES = ('franka_description', 'franka_moveit_config',
                   'franka_example_controllers')

CELL_NAME = 'cell_model_v1.yaml'
GEOMETRY_NAME = 'link_geometry_v1.yaml'


def installed_cell_directory():
    """Return the model package's own cell directory, or None."""
    try:
        from franka_workspace_model import model
    except ImportError:
        return None
    package = Path(getattr(model, '__file__', '')).resolve().parent
    candidate = package.parent / 'cell'
    return candidate if (candidate / CELL_NAME).is_file() else None


def repository_root():
    """Return the checkout the source packages live in, or None."""
    cell = installed_cell_directory()
    if cell is None:
        return None
    root = cell.parent.parent
    return root if all((root / name).is_dir() for name in SOURCE_PACKAGES) else None


def write_sample_cell(directory, text=None):
    """
    Write an editable cell model into ``directory`` and return its path.

    Returns None when the workspace model or its sources are not present,
    which is the caller's cue to skip: a test that needs a real cell model
    cannot invent one.
    """
    root = repository_root()
    cell = installed_cell_directory()
    if root is None or cell is None:
        return None
    scratch = Path(directory) / 'repository'
    (scratch / 'cell').mkdir(parents=True, exist_ok=True)
    for name in SOURCE_PACKAGES:
        link = scratch / name
        if not link.exists():
            link.symlink_to(root / name, target_is_directory=True)
    # Every pinned artefact beside the cell file (the capsule geometry, the
    # mesh bodies, and whatever a later revision adds) rides along by link,
    # so the copy resolves its ``sources:`` exactly as the installed one does.
    for artefact in sorted(cell.glob('*.yaml')):
        if artefact.name == CELL_NAME:
            continue
        link = scratch / 'cell' / artefact.name
        if not link.exists():
            link.symlink_to(artefact)
    target = scratch / 'cell' / CELL_NAME
    if text is None:
        shutil.copyfile(str(cell / CELL_NAME), str(target))
    else:
        target.write_text(text, encoding='utf-8')
    return str(target)


def corpus_directory():
    """Return the model package's validation corpus, or None."""
    root = repository_root()
    if root is None:
        return None
    candidate = root / 'franka_workspace_model' / 'test' / 'corpus'
    return candidate if os.path.isdir(str(candidate)) else None
