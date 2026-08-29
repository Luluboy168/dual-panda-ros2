# [THROWAWAY] Session C browser-test wrapper; franka_web owns its browser harness.
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


"""Pytest entry point for URDF, FK, WebGL2 render, and joint-drag cases."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parents[1]
ASSETS = PACKAGE_ROOT / 'web' / 'assets'
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(HERE))

from franka_ghost.asset_prep import prepare_assets  # noqa: E402
from run_browser_tests import run_browser_suite  # noqa: E402


def test_browser_urdf_and_forward_kinematics() -> None:
    """Generate missing ignored assets, then require every browser case to pass."""
    if not (ASSETS / 'model.urdf').is_file() or not (ASSETS / 'manifest.json').is_file():
        prepare_assets(PACKAGE_ROOT)
    verdict = run_browser_suite()
    assert verdict.get('ok') is True, verdict
    assert verdict.get('tests', 0) >= 31, verdict
    assert verdict.get('failures') == [], verdict
    assert verdict.get('browser_problems') == [], verdict
