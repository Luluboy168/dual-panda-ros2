#!/usr/bin/env python3
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
Generate the 3D scene's assets into a build directory.

Run by the build, never at run time and never installed as a program. It
prints one line saying what it did, so a build log answers "were the meshes
regenerated?" without anyone opening the tree.
"""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from franka_web.ghost_assets.asset_prep import prepare_assets  # noqa: E402


def main(argv=None):
    """Generate the assets and report what happened, in one line."""
    parser = argparse.ArgumentParser(
        prog='franka_web_ghost_assets',
        description='Generate the ghost scene assets into a directory.')
    parser.add_argument('output', help='the directory to write the tree into')
    parser.add_argument(
        '--description-share', default=None,
        help='the franka_description share directory (default: the installed one)')
    args = parser.parse_args(argv)
    result = prepare_assets(Path(args.output), args.description_share)
    print('franka_web ghost assets: {} {} meshes, {} bytes, in {}'.format(
        'generated' if result.regenerated else 'reused',
        result.mesh_count, result.asset_bytes, args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
