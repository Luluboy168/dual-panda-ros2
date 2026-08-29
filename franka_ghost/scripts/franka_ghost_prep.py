#!/usr/bin/env python3
# [THROWAWAY] Session C package CLI scaffold; franka_web owns the merged asset build.
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

"""Generate the browser assets used by the Franka ghost frontend."""

from __future__ import annotations

import argparse
from pathlib import Path

from franka_ghost.asset_prep import (
    default_package_root,
    prepare_assets,
    VENDOR_SOURCE,
)


def main() -> int:
    """Run the asset preparation command."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--package-root',
        type=Path,
        default=default_package_root(),
        help='franka_ghost package source root',
    )
    parser.add_argument(
        '--description-share',
        type=Path,
        help='override the resolved franka_description share directory',
    )
    parser.add_argument(
        '--three-js',
        type=Path,
        default=VENDOR_SOURCE,
        help='approved three.min.js source path',
    )
    arguments = parser.parse_args()
    result = prepare_assets(
        arguments.package_root,
        arguments.description_share,
        arguments.three_js,
    )
    action = 'regenerated' if result.regenerated else 'up to date'
    print(
        f'ghost assets {action}: {result.mesh_count} meshes, '
        f'{result.asset_bytes} bytes'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
