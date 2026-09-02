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
The static lane: which type each file answers with, and what may be cached.

Two rules, and both are about being believed. Under nosniff a browser is
FORBIDDEN from guessing a type, so a file served as octet-stream is a file
the page silently refuses to use. And a response may be cached forever only
when its name changes with its content -- which is exactly why the mesh
files carry a digest and the manifest does not.
"""

import json
import os

import pytest
from support.ghost_server import GhostServer
from support.stub_ik import StubSolver

from test_ghost_scene_api import build_ghost

IMMUTABLE = 'public, max-age=31536000, immutable'


@pytest.fixture()
def static_root(tmp_path):
    """Write a static tree shaped exactly like the installed one."""
    root = tmp_path / 'static'
    (root / 'fonts').mkdir(parents=True)
    (root / 'ghost' / 'assets' / 'meshes').mkdir(parents=True)
    (root / 'ghost' / 'vendor').mkdir(parents=True)
    (root / 'index.html').write_text('<!doctype html><title>console</title>')
    (root / 'app.css').write_text('body { color: black; }')
    (root / 'app.js').write_text('export const version = 2;')
    (root / 'fonts' / 'OFL.txt').write_text('the open font licence')
    (root / 'fonts' / 'archivo-var.woff2').write_bytes(b'wOF2fake')
    assets = root / 'ghost' / 'assets'
    (assets / 'manifest.json').write_text(json.dumps(
        {'schema': 'franka.ghost.manifest/1',
         'generated_from': {'urdf_sha256': 'd' * 64},
         'urdf': 'model.urdf', 'meshes': {}}))
    (assets / 'model.urdf').write_text('<robot name="dual_panda"/>')
    (assets / 'meshes' / 'link0.a15533ab3ab1.ghostmesh.bin').write_bytes(
        b'\x00\x01\x02\x03')
    (assets / 'meshes' / 'link0.a15533ab3ab1.ghostmesh.json').write_text('{}')
    (root / 'ghost' / 'vendor' / 'three.r111.min.js').write_text('// three')
    (root / 'ghost' / 'vendor' / 'three-license.txt').write_text('MIT')
    (root / 'ghost' / 'ghost.js').write_text('export function mount() {}')
    return root


@pytest.fixture()
def serving(tmp_path, static_root):
    """Serve that tree over the real HTTP surface."""
    server = GhostServer(tmp_path, build_ghost(solver=StubSolver()),
                         static_root=str(static_root))
    yield server
    server.close()


class TestContentTypes:
    """Every served extension answers with a type a browser will accept."""

    @pytest.mark.parametrize('path,content_type', [
        ('/index.html', 'text/html; charset=utf-8'),
        ('/app.css', 'text/css; charset=utf-8'),
        ('/app.js', 'text/javascript; charset=utf-8'),
        ('/fonts/archivo-var.woff2', 'font/woff2'),
        ('/fonts/OFL.txt', 'text/plain; charset=utf-8'),
        ('/ghost/ghost.js', 'text/javascript; charset=utf-8'),
        ('/ghost/vendor/three-license.txt', 'text/plain; charset=utf-8'),
        ('/ghost/assets/manifest.json', 'application/json; charset=utf-8'),
        ('/ghost/assets/model.urdf', 'application/xml'),
        ('/ghost/assets/meshes/link0.a15533ab3ab1.ghostmesh.bin',
         'application/octet-stream'),
    ])
    def test_each_file_answers_with_its_own_type(self, serving, path,
                                                 content_type):
        """Under nosniff the browser cannot guess, so the server must know."""
        response = serving.request('GET', path)
        assert response.status == 200, path
        assert response.header('Content-Type') == content_type

    def test_a_module_under_ghost_keeps_the_javascript_type(self, serving):
        """A dynamic import of an octet-stream body is refused outright."""
        response = serving.request('GET', '/ghost/ghost.js')
        assert response.header('Content-Type') == 'text/javascript; charset=utf-8'

    def test_every_static_response_carries_the_security_headers(self, serving):
        """The lane changes the cache header and nothing else."""
        for path in ('/index.html',
                     '/ghost/assets/meshes/link0.a15533ab3ab1.ghostmesh.bin'):
            response = serving.request('GET', path)
            assert response.header('X-Content-Type-Options') == 'nosniff'
            assert response.header('Content-Security-Policy')


class TestCacheLane:
    """What may be kept forever, and what must never be."""

    @pytest.mark.parametrize('path', [
        '/ghost/assets/meshes/link0.a15533ab3ab1.ghostmesh.bin',
        '/ghost/assets/meshes/link0.a15533ab3ab1.ghostmesh.json',
        '/ghost/vendor/three.r111.min.js',
        '/ghost/vendor/three-license.txt',
    ])
    def test_content_addressed_files_are_cached_forever(self, serving, path):
        """
        Safe because the name changes when the bytes do.

        Nine megabytes of mesh is a four-second first load on lab wifi and
        nothing at all on every load after it.
        """
        assert serving.request('GET', path).header('Cache-Control') == IMMUTABLE

    @pytest.mark.parametrize('path', [
        '/ghost/assets/manifest.json',
        '/ghost/assets/model.urdf',
        '/index.html',
        '/app.js',
        '/fonts/archivo-var.woff2',
    ])
    def test_everything_else_is_never_cached(self, serving, path):
        """
        The manifest and the URDF are the cache-busting root.

        They name the hashed files, so a regenerated asset set is picked up
        on the next load with no window in which a stale mesh is drawn.
        """
        assert serving.request('GET', path).header('Cache-Control') == 'no-store'

    def test_api_responses_are_never_cached(self, serving):
        """The lane is about files on disk; nothing dynamic enters it."""
        for path in ('/api/scene', '/api/capabilities'):
            response = serving.request('GET', path)
            assert response.header('Cache-Control') == 'no-store', path

    def test_the_lane_is_a_prefix_rule_that_cannot_leak_upward(
            self, serving, static_root):
        """
        A file beside the meshes directory is not in the lane.

        The prefix ends in a slash for exactly this reason: without it a
        future `ghost/assets/meshes_backup/` would be cached forever.
        """
        (static_root / 'ghost' / 'assets' / 'stray.bin').write_bytes(b'\x00')
        response = serving.request('GET', '/ghost/assets/stray.bin')
        assert response.status == 200
        assert response.header('Cache-Control') == 'no-store'


class TestTraversalStillRefused:
    """The new lane must not have opened a way out of the static root."""

    @pytest.mark.parametrize('path', [
        '/ghost/assets/../../package.xml',
        '/ghost/assets/meshes/../../../../package.xml',
        '/ghost//assets/manifest.json',
    ])
    def test_a_path_that_leaves_the_root_is_not_found(self, serving, path):
        """Containment is lexical, and the asset tree adds no new shape to it."""
        assert serving.request('GET', path).status == 404

    def test_the_installed_layout_is_what_is_actually_served(
            self, serving, static_root):
        """A file the build installs is a file the page can fetch."""
        assert os.path.isfile(str(static_root / 'ghost' / 'assets' / 'model.urdf'))
        assert serving.request('GET', '/ghost/assets/model.urdf').status == 200


class TestTheRealInstalledTree:
    """
    The same lane, against the tree colcon actually produced.

    Every case above builds its own tree of ordinary files. The tree a
    developer runs is not that: under ``--symlink-install`` the generated
    assets are installed as SYMLINKS into the build directory, whose targets
    lie outside the static root entirely. So a containment check that ever
    resolved a path before comparing it would 404 every mesh on the build
    everybody actually uses, while every synthetic case above stayed green.

    That is the only reason this class exists, and it is why it insists on
    the real tree rather than simulating one: a simulated symlink proves
    nothing about what the install step really wrote.
    """

    @pytest.fixture()
    def installed_root(self):
        """Return the installed static tree, or skip when it is not built."""
        try:
            from ament_index_python.packages import get_package_share_directory
            root = os.path.join(
                get_package_share_directory('franka_web'), 'static')
        except Exception:                 # noqa: BLE001 - not built is normal
            pytest.skip('franka_web is not installed in this workspace')
        if not os.path.isfile(os.path.join(root, 'ghost', 'assets',
                                           'manifest.json')):
            pytest.skip('the scene assets have not been generated here')
        return root

    @pytest.fixture()
    def installed(self, tmp_path, installed_root):
        """Serve the installed tree over the real HTTP surface."""
        server = GhostServer(tmp_path, build_ghost(solver=StubSolver()),
                             static_root=installed_root)
        yield server
        server.close()

    def test_a_generated_mesh_is_served_with_the_immutable_lane(
            self, installed, installed_root):
        """The content-addressed file, over the wire, from the real install."""
        meshes = os.path.join(installed_root, 'ghost', 'assets', 'meshes')
        binaries = sorted(name for name in os.listdir(meshes)
                          if name.endswith('.bin'))
        assert binaries, 'the generated mesh set is empty'
        response = installed.request(
            'GET', '/ghost/assets/meshes/' + binaries[0])
        assert response.status == 200
        assert response.header('Content-Type') == 'application/octet-stream'
        assert response.header('Cache-Control') == IMMUTABLE

    @pytest.mark.parametrize('path, content_type', [
        ('/ghost/assets/manifest.json', 'application/json; charset=utf-8'),
        ('/ghost/assets/model.urdf', 'application/xml'),
    ])
    def test_the_cache_busting_root_is_never_cached(
            self, installed, path, content_type):
        """The two files that name the meshes must always be re-fetched."""
        response = installed.request('GET', path)
        assert response.status == 200
        assert response.header('Content-Type') == content_type
        assert response.header('Cache-Control') == 'no-store'
