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
The pytest entry point for the browser suite.

It skips rather than fails on a machine with no Chromium, and on a checkout
where the harness page has not landed yet: the cases are JavaScript owned by
the scene work, and this wrapper is the Python that starts them.

The wrapper also holds two properties the JavaScript cannot assert about
itself -- that the harness really is served under the production policy, and
that the stub responder really is the server's own code.
"""

import json
import os
from pathlib import Path
import shutil
import sys

import pytest

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parents[1]
for path in (str(PACKAGE_ROOT), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

from run_browser_tests import (      # noqa: E402
    build_ghost_service, CONTENT_TYPES, harness_path, run_browser_suite)


def requirements():
    """Skip unless both the browser and the harness page are here."""
    if shutil.which('chromium') is None and shutil.which('chromium-browser') is None:
        pytest.skip('Chromium is not installed on this machine')
    if not harness_path().is_file():
        pytest.skip('the browser harness page is not present in this checkout')


def test_the_browser_suite_passes_every_case():
    """Every case green, no console problems, and no CSP violation."""
    requirements()
    verdict = run_browser_suite()
    assert verdict.get('ok') is True, verdict
    assert verdict.get('failures') == [], verdict
    assert verdict.get('browser_problems') == [], verdict
    assert verdict.get('tests', 0) > 0, verdict


def test_the_harness_is_served_under_the_production_policy():
    """
    The policy is imported from the server, never restated here.

    The suite's CSP story rests on a violation listener staying silent. A
    listener that can never fire, because the page carries no policy at all,
    proves nothing -- and a policy copied into this file would drift from
    the one that ships and test nothing about production.
    """
    from franka_web.http_api import _CSP
    from run_browser_tests import _CSP as harness_csp
    assert harness_csp is _CSP


def test_the_stub_responder_is_the_servers_own_code():
    """
    The payloads the page is built against cannot drift from the real ones.

    The stub is not a hand-written fixture: it is the production ghost
    service with a solver that returns the seed, so every field name and
    every shape comes from the module the Python gates assert against.
    """
    ghost = build_ghost_service()
    scene = ghost.scene({'manifest_url': '/ghost/assets/manifest.json',
                         'urdf_url': '/ghost/assets/model.urdf',
                         'asset_base': '/ghost/assets/',
                         'urdf_sha256': '0' * 64, 'total_bytes': 1})
    assert set(scene) == {'assets', 'cell', 'cell_source', 'cell_note', 'model',
                          'arms', 'ik', 'checker', 'ghost_available'}
    answer = ghost.solve({
        'arm_id': 'panda1',
        'seed': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
        'target': {'position': [0.3, 0.0, 0.5],
                   'orientation': [0.0, 1.0, 0.0, 0.0]},
        'redundancy': {'mode': 'from_seed'}})
    assert answer['solved'] is True
    assert answer['verdict']['status'] == 'clear'
    assert set(answer['copy']) == {'joints_deg', 'joints_rad', 'snippet'}
    table = ghost.redundancy({
        'arm_id': 'panda1',
        'seed': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
        'target': {'position': [0.3, 0.0, 0.5],
                   'orientation': [0.0, 1.0, 0.0, 0.0]},
        'samples': 9})
    assert table['samples'] == 9
    assert len(table['table']) == 9
    assert json.dumps(table)      # every value survives the JSON envelope


def test_the_harness_serves_javascript_as_javascript():
    """A module served as octet-stream is a module the browser refuses."""
    assert CONTENT_TYPES['.js'] == 'text/javascript; charset=utf-8'
    assert CONTENT_TYPES['.bin'] == 'application/octet-stream'
    assert CONTENT_TYPES['.urdf'] == 'application/xml'


def test_the_harness_serves_one_origin_with_the_production_layout():
    """
    Every path the page fetches is the path production serves.

    A case that passed against a different layout would prove nothing about
    the console, and the asset base is exactly where such a difference would
    hide.
    """
    from franka_web import defaults
    from run_browser_tests import HARNESS_URL_PREFIX, STATIC_ROOT
    assert defaults.GHOST_ASSET_PREFIX == 'ghost/assets/'
    assert os.path.basename(str(STATIC_ROOT)) == 'static'
    assert HARNESS_URL_PREFIX.startswith('/') and HARNESS_URL_PREFIX.endswith('/')
