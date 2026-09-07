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
The scene's generated assets: the build-time pipeline and the served summary.

This module is the seam between the two. The scene handler must not open
asset files, join asset paths or compute digests, so it asks here for one
small block of facts and passes them through untouched.

Nothing at import time reaches xacro, the description package or the
converter: the three modules beside this one are imported by the build
script that generates the assets, and by nothing that serves them.
"""

import json
import os
import threading

from franka_web import defaults

MANIFEST_NAME = 'manifest.json'
URDF_NAME = 'model.urdf'
MESH_DIRECTORY = 'meshes'

_CACHE = {}
_CACHE_LOCK = threading.Lock()


def asset_directory(static_root):
    """Return the directory the generated assets are installed into."""
    return os.path.join(static_root, *defaults.GHOST_ASSET_PREFIX.split('/')[:-1])


def installed_manifest_summary(static_root):
    """
    Return the scene payload's ``assets`` block, or None when there is none.

    The result is read once and cached: the tree is written by the build and
    never changes while the server runs. None is a designed state -- an
    unbuilt or partially installed workspace -- and the page renders it as
    "the 3D model files did not load", not as an error.

    ``asset_base`` ends in a slash because the renderer resolves every mesh
    path against it; without the slash each one would resolve a directory too
    high, and the whole scene would 404 in a way that looks like a missing
    build.
    """
    if not static_root:
        return None
    with _CACHE_LOCK:
        if static_root in _CACHE:
            return _CACHE[static_root]
    summary = _read_summary(asset_directory(static_root))
    with _CACHE_LOCK:
        _CACHE[static_root] = summary
    return summary


def forget_installed_manifest():
    """Drop the cached summary (the tests build trees at run time)."""
    with _CACHE_LOCK:
        _CACHE.clear()


def _read_summary(directory):
    """Read the manifest and measure the tree, or return None."""
    manifest_path = os.path.join(directory, MANIFEST_NAME)
    try:
        with open(manifest_path, encoding='utf-8') as handle:
            manifest = json.load(handle)
        digest = manifest['generated_from']['urdf_sha256']
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if not os.path.isfile(os.path.join(directory, URDF_NAME)):
        return None
    base = '/' + defaults.GHOST_ASSET_PREFIX
    return {
        'manifest_url': base + MANIFEST_NAME,
        'urdf_url': base + URDF_NAME,
        'asset_base': base,
        'urdf_sha256': str(digest),
        'total_bytes': _tree_bytes(directory),
    }


def _tree_bytes(directory):
    """Return the total size of the generated tree, in bytes."""
    total = 0
    for parent, _directories, names in os.walk(directory):
        for name in names:
            try:
                total += os.path.getsize(os.path.join(parent, name))
            except OSError:
                continue
    return total
