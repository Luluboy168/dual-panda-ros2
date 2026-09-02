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
GET /api/scene: the static scene facts, over a real socket.

The endpoint's whole job is to be honest about three independent things --
the assets, the IK service and the checker -- so most of these cases are
about which combination of absences produces which sentence, and about the
one rule the renderer depends on absolutely: a cell box is present exactly
when the payload says its source is a cell model.
"""

import json
import os

from franka_web import defaults, workspace
from franka_web.ghost import GhostService
from franka_web.ghost_assets import (
    forget_installed_manifest, installed_manifest_summary)
from franka_web.server import scene_banner
import pytest
from support import fake_checker
from support.fake_checker import FakeChecker
from support.ghost_server import GhostServer
from support.stub_ik import StubSolver

ASSETS = {
    'manifest_url': '/ghost/assets/manifest.json',
    'urdf_url': '/ghost/assets/model.urdf',
    'asset_base': '/ghost/assets/',
    'urdf_sha256': 'b' * 64,
    'total_bytes': 9155771,
}


def build_ghost(checker=None, solver=None, arm_ids=(), ready=True):
    """Return a GhostService wired to doubles, with no ROS anywhere."""
    return GhostService(
        solver=solver or StubSolver(),
        checker=checker if checker is not None else FakeChecker(),
        session_view=lambda: {'arm_ids': list(arm_ids), 'command_topics': {}},
        ik_ready=lambda: ready)


@pytest.fixture()
def scene_server(tmp_path):
    """Serve one app whose ghost is wired to doubles."""
    running = GhostServer(tmp_path, build_ghost())
    yield running
    running.close()


class TestSceneOverTheSocket:
    """The payload, its headers, and the fact that no token is ever needed."""

    def test_scene_answers_without_a_token_in_every_session_state(
            self, scene_server):
        """A viewer with no lock may look at the scene; that is the point."""
        response = scene_server.request('GET', '/api/scene')
        assert response.status == 200
        body = response.json()
        assert body['ok'] is True
        assert body['arms'] == list(defaults.ARM_IDS)
        assert response.header('X-Content-Type-Options') == 'nosniff'
        assert response.header('Content-Security-Policy')

    def test_the_payload_carries_exactly_the_contract_keys(self, scene_server):
        """A new key is a deliberate change, not an accident."""
        body = scene_server.request('GET', '/api/scene').json()
        assert set(body) == {'ok', 'assets', 'cell', 'cell_source', 'cell_note',
                             'model', 'arms', 'ik', 'checker', 'ghost_available'}
        assert set(body['ik']) == {'available', 'arm_ids', 'tip_frame'}
        assert set(body['checker']) == {'available', 'profile', 'interlock',
                                        'note'}

    def test_the_operator_token_is_inert_here(self, scene_server):
        """The header may be attached by the page; the route never reads it."""
        plain = scene_server.request('GET', '/api/scene').json()
        with_token = scene_server.request(
            'GET', '/api/scene',
            headers={'X-Operator-Token': 'not-a-real-token'}).json()
        assert plain == with_token

    def test_the_response_is_never_cached(self, scene_server):
        """Scene availability is point-in-time; a cached copy would lie."""
        response = scene_server.request('GET', '/api/scene')
        assert response.header('Cache-Control') == 'no-store'


class TestScenePayload:
    """The four states, driven directly against the service object."""

    def test_all_well(self):
        """A loaded checker and a running IK service draw everything."""
        body = build_ghost().scene(ASSETS)
        assert body['cell_source'] == 'cell_model'
        assert body['cell']['id'] == 'work_area'
        assert body['cell_note'] is None
        assert body['checker']['available'] is True
        assert body['checker']['profile'] == 'dual'
        assert body['ghost_available'] is True
        assert body['ik']['available'] is True
        assert body['ik']['tip_frame'] == 'flange'
        assert body['model']['model_id'] == 'hcis_dual_panda_cell'

    def test_checker_absent(self):
        """No checker: no box, one sentence, and editing still allowed."""
        body = build_ghost(checker=FakeChecker(available=False)).scene(ASSETS)
        assert body['cell'] is None
        assert body['cell_source'] == 'unavailable'
        assert body['cell_note'] == workspace.NOTE_PACKAGE_ABSENT
        assert body['checker']['available'] is False
        # The checker degrades the verdict, never the editing.
        assert body['ghost_available'] is True

    def test_ik_absent(self):
        """No IK service: the scene still draws, but nothing may be authored."""
        body = build_ghost(ready=False).scene(ASSETS)
        assert body['ghost_available'] is False
        assert body['ik']['available'] is False
        assert body['cell'] is not None

    def test_no_session_keeps_the_dual_profile(self):
        """The cell is session-independent, so an idle console still draws it."""
        body = build_ghost(arm_ids=()).scene(ASSETS)
        assert body['checker']['profile'] == 'dual'
        assert body['cell'] is not None

    def test_a_single_arm_session_selects_the_single_profile(self):
        """One arm in the session means the one-arm cell model."""
        body = build_ghost(arm_ids=('panda1',)).scene(ASSETS)
        assert body['checker']['profile'] == 'single'

    def test_a_two_arm_session_selects_the_dual_profile(self):
        """Both arms means the two-arm cell model."""
        body = build_ghost(arm_ids=('panda1', 'panda2')).scene(ASSETS)
        assert body['checker']['profile'] == 'dual'

    def test_cell_is_null_exactly_when_the_source_is_unavailable(self):
        """The renderer keys its whole cell branch on this one equivalence."""
        for checker in (FakeChecker(), FakeChecker(available=False)):
            body = build_ghost(checker=checker).scene(ASSETS)
            assert (body['cell'] is None) == (body['cell_source'] == 'unavailable')
            assert (body['cell_note'] is not None) == (body['cell'] is None)

    def test_the_interlock_is_not_checked_by_default(self):
        """The ghost performs no interlock, and says so rather than claiming ok."""
        body = build_ghost().scene(ASSETS)
        assert body['checker']['interlock'] == 'not_checked'
        assert body['checker']['note'] is None

    def test_a_mismatched_interlock_carries_its_sentence(self):
        """This field is the sentence's ONE route to a screen."""
        checker = FakeChecker(interlock='mismatch')
        body = build_ghost(checker=checker).scene(ASSETS)
        assert body['checker']['interlock'] == 'mismatch'
        assert body['checker']['note'] == workspace.NOTE_INTERLOCK_MISMATCH
        # The cell still loads on a mismatch, so cell_note stays bound to
        # cell_source and cannot carry this sentence.
        assert body['cell'] is not None
        assert body['cell_note'] is None

    def test_assets_are_passed_through_untouched(self):
        """The handler hands this block over; the scene never rebuilds it."""
        assert build_ghost().scene(ASSETS)['assets'] == ASSETS

    def test_absent_assets_are_a_designed_state(self):
        """An unbuilt tree is null, not an error."""
        assert build_ghost().scene(None)['assets'] is None

    def test_the_asset_block_resolves_relative_paths_correctly(self):
        """
        asset_base ends in a slash and both URLs are absolute.

        The renderer resolves every mesh path against asset_base. Without
        the trailing slash each one resolves a directory too high and the
        whole scene 404s in a way that looks like a missing build.
        """
        assets = build_ghost().scene(ASSETS)['assets']
        assert assets['asset_base'].endswith('/')
        assert assets['asset_base'].startswith('/')
        assert assets['manifest_url'].startswith('/')
        assert assets['urdf_url'].startswith('/')
        assert '://' not in assets['manifest_url']
        assert '://' not in assets['urdf_url']


class TestInstalledManifestSummary:
    """The seam that reads the installed tree, exercised on a real one."""

    def build_tree(self, tmp_path, digest='c' * 64):
        """Write a minimal installed asset tree and return the static root."""
        forget_installed_manifest()
        assets = tmp_path / 'ghost' / 'assets'
        (assets / 'meshes').mkdir(parents=True)
        (assets / 'model.urdf').write_text('<robot name="x"/>')
        (assets / 'meshes' / 'link0.abc123.ghostmesh.bin').write_bytes(b'0' * 32)
        (assets / 'manifest.json').write_text(json.dumps(
            {'schema': 'franka.ghost.manifest/1',
             'generated_from': {'urdf_sha256': digest},
             'urdf': 'model.urdf', 'meshes': {}}))
        return str(tmp_path)

    def test_the_summary_matches_the_installed_tree(self, tmp_path):
        """Every field comes from the tree, not from a constant."""
        root = self.build_tree(tmp_path)
        summary = installed_manifest_summary(root)
        assert summary['urdf_sha256'] == 'c' * 64
        assert summary['asset_base'] == '/ghost/assets/'
        assert summary['manifest_url'] == '/ghost/assets/manifest.json'
        assert summary['urdf_url'] == '/ghost/assets/model.urdf'
        assert summary['total_bytes'] > 0
        assert summary['total_bytes'] == sum(
            os.path.getsize(os.path.join(parent, name))
            for parent, _dirs, names in os.walk(
                os.path.join(root, 'ghost', 'assets'))
            for name in names)

    def test_an_unbuilt_tree_is_none(self, tmp_path):
        """A missing tree is a state the page renders, not an exception."""
        forget_installed_manifest()
        assert installed_manifest_summary(str(tmp_path)) is None

    def test_a_manifest_without_its_urdf_is_none(self, tmp_path):
        """Half a tree is no tree: the renderer needs both files."""
        root = self.build_tree(tmp_path)
        os.unlink(os.path.join(root, 'ghost', 'assets', 'model.urdf'))
        forget_installed_manifest()
        assert installed_manifest_summary(root) is None


class TestAllowedVolumeFallback:
    """R1's accessor, and the documented YAML fallback beside it."""

    VOLUME = {'id': 'work_area', 'frame': 'cell',
              'x_min': -0.35, 'x_max': 0.9, 'y_min': -1.0, 'y_max': 1.0,
              'z_min': 0.0, 'z_max': 2.0}

    def write_cell(self, tmp_path):
        """Write a cell file carrying nothing but the allowed volume."""
        path = tmp_path / 'cell_model_v1.yaml'
        path.write_text(
            'allowed_volume:\n'
            + ''.join('  {}: {}\n'.format(key, value if not isinstance(value, str)
                                          else '"{}"'.format(value))
                      for key, value in self.VOLUME.items()))
        return str(path)

    def test_the_accessor_is_used_when_the_model_has_one(self, tmp_path):
        """The published accessor is the plan; the fallback is not."""
        volume = _Volume(**self.VOLUME)
        model = fake_checker.FakeCellModel(volume=volume)
        assert workspace._allowed_volume(model, None) == self.VOLUME

    def test_the_yaml_fallback_reads_the_same_six_floats(self, tmp_path):
        """
        A model package too old to answer still gets a drawn cell.

        This is the branch that ships if the accessor is ever withdrawn, and
        an untested fallback is not a prerequisite anyone can honestly tick.
        """
        path = self.write_cell(tmp_path)
        model = fake_checker.FakeCellModel()          # no accessor at all
        assert workspace._allowed_volume(model, path) == self.VOLUME

    def test_the_fallback_returns_none_rather_than_guessing(self, tmp_path):
        """No file, no numbers: the scene says so instead of drawing a box."""
        model = fake_checker.FakeCellModel()
        assert workspace._allowed_volume(model, None) is None


class _Volume:
    """The six bounds and their id, shaped like the model's own dataclass."""

    def __init__(self, **fields):
        """Store every field as an attribute."""
        self.__dict__.update(fields)


class TestSceneBanner:
    """Each shape of the one startup line, so no wording drifts unseen."""

    class _Bridge:
        def __init__(self, ready):
            self.ready = ready

        def ik_service_ready(self):
            return self.ready

    class _Checker:
        def __init__(self, text):
            self.text = text

        def banner(self):
            return self.text

    def test_both_present(self):
        """The line names what loaded and what is running."""
        line = scene_banner(self._Checker('cell model loaded (/cell.yaml)'),
                            self._Bridge(True))
        assert line == ('  scene: cell model loaded (/cell.yaml) · '
                        'IK service ready')

    def test_neither_present(self):
        """Both absences are named, in the same line, at startup."""
        line = scene_banner(self._Checker('no workspace model installed'),
                            self._Bridge(False))
        assert line == ('  scene: no workspace model installed · '
                        'IK service not running')

    def test_the_checker_authors_its_own_half(self):
        """Every wording of the checker half comes from one module."""
        checker = workspace.WorkspaceChecker(cell_path='/nowhere/cell.yaml')
        assert checker.banner() in (
            'no workspace model installed',
            'cell model not loaded (/nowhere/cell.yaml)')
