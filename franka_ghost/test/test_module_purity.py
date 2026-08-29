# [THROWAWAY] Session C package purity gate; franka_web owns the merged test suite.
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

"""Enforce the frozen Contract C3 module boundary and merge ownership."""

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GHOST_ROOT = PACKAGE_ROOT / 'web' / 'ghost'

# Contract C3 permits exactly this asset-loading fetch. Pinning both its line
# and complete source prevents a later transport fetch from borrowing a broad
# filename or function-level exception.
ASSET_FETCH_ALLOWLIST = {
    ('ghost.js', 153): 'return fetch(url, {signal}).then((response) => {',
}

FORBIDDEN_PATTERNS = {
    'fetch(': re.compile(r'\bfetch\s*\('),
    'XMLHttpRequest': re.compile(r'\bXMLHttpRequest\b'),
    'WebSocket': re.compile(r'\bWebSocket\b'),
    '/state.json': re.compile(r'/state\.json'),
    '/apply': re.compile(r'(?<!\.)/apply\b'),
    'impedance': re.compile(r'\bimpedance\b', re.IGNORECASE),
    'rclpy': re.compile(r'\brclpy\b'),
    'topic': re.compile(r'\btopic\b', re.IGNORECASE),
    'service': re.compile(r'\bservice\b', re.IGNORECASE),
    'dual_arm': re.compile(r'dual_arm', re.IGNORECASE),
    'franka_bringup': re.compile(r'franka_bringup', re.IGNORECASE),
    '172.16.': re.compile(r'172\.16\.'),
}

DURABLE_FILES = {
    'README.md',
    'franka_ghost/__init__.py',
    'franka_ghost/asset_prep.py',
    'franka_ghost/mesh_convert.py',
    'franka_ghost/urdf_export.py',
    'test/browser/cases/apply.js',
    'test/browser/cases/kinematics.js',
    'test/browser/cases/urdf.js',
    'test/test_mesh_convert.py',
    'test/test_urdf_export.py',
    'web/ghost/apply.js',
    'web/ghost/drag.js',
    'web/ghost/ghost.js',
    'web/ghost/kinematics.js',
    'web/ghost/meshes.js',
    'web/ghost/scene.js',
    'web/ghost/urdf.js',
}

THROWAWAY_FILES = {
    '.gitignore',
    'CMakeLists.txt',
    'franka_ghost/dev_server.py',
    'franka_ghost/joint_source.py',
    'package.xml',
    'scripts/franka_ghost_dev_server.py',
    'scripts/franka_ghost_prep.py',
    'test/browser/cases/drag.js',
    'test/browser/cases/scene.js',
    'test/browser/harness.html',
    'test/browser/run_browser_tests.py',
    'test/browser/test_browser.py',
    'test/live/test_live_state_bridge.py',
    'test/test_apply_payload.py',
    'test/test_dev_server.py',
    'test/test_module_purity.py',
    'web/index.html',
    'web/style.css',
    'web/transport.js',
}


def _ghost_sources():
    """Yield stable relative paths and source lines below ``web/ghost``."""
    for path in sorted(GHOST_ROOT.rglob('*')):
        if path.is_file():
            yield path.relative_to(GHOST_ROOT).as_posix(), path.read_text().splitlines()


def test_module_has_only_exact_asset_fetch_and_no_transport_or_ros_names():
    """Reject I/O, transport, ROS, controller, and real-address knowledge."""
    violations = []
    seen_allowlist = set()
    for relative, lines in _ghost_sources():
        for line_number, line in enumerate(lines, start=1):
            for label, pattern in FORBIDDEN_PATTERNS.items():
                if not pattern.search(line):
                    continue
                location = (relative, line_number)
                if label == 'fetch(' and location in ASSET_FETCH_ALLOWLIST:
                    expected = ASSET_FETCH_ALLOWLIST[location]
                    if line.strip() == expected:
                        seen_allowlist.add(location)
                        continue
                violations.append(f'{relative}:{line_number}: {label}: {line.strip()}')

    missing = set(ASSET_FETCH_ALLOWLIST) - seen_allowlist
    assert not missing, f'asset-fetch allowlist moved or changed: {sorted(missing)}'
    assert not violations, 'Contract C3 purity violations:\n' + '\n'.join(violations)


def test_module_imports_stay_inside_ghost_directory():
    """Allow local module imports and the single documented vendor seam only."""
    violations = []
    from_pattern = re.compile(r'\bfrom\s+["\']([^"\']+)["\']')
    bare_pattern = re.compile(r'^\s*import\s+["\']([^"\']+)["\']', re.MULTILINE)
    for relative, lines in _ghost_sources():
        source = '\n'.join(lines)
        imports = from_pattern.findall(source) + bare_pattern.findall(source)
        for specifier in imports:
            if not (specifier.startswith('./') or specifier == '../vendor/three.min.js'):
                violations.append(f'{relative}: import {specifier!r}')
    assert not violations, 'imports escape web/ghost:\n' + '\n'.join(violations)


def test_source_and_scaffold_files_have_merge_markers():
    """Keep the mechanical §8 durable/throwaway split machine-checkable."""
    for expected, paths in (
        ('[DURABLE]', DURABLE_FILES),
        ('[THROWAWAY]', THROWAWAY_FILES),
    ):
        for relative in sorted(paths):
            path = PACKAGE_ROOT / relative
            assert path.is_file(), f'merge inventory path is missing: {relative}'
            header = '\n'.join(path.read_text().splitlines()[:20])
            assert expected in header, f'{relative} lacks {expected} header marker'
            opposite = '[THROWAWAY]' if expected == '[DURABLE]' else '[DURABLE]'
            assert opposite not in header, f'{relative} has conflicting {opposite} marker'
