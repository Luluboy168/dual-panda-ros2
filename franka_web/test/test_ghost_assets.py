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
The generated scene assets: determinism, provenance, and the right lab.

The whole argument for generating these files rather than committing them is
that a committed tree goes stale behind a description change without anyone
noticing. This file is what makes that argument true: it regenerates from
the installed description and checks the result against that description,
never against a recorded copy of itself.

The base-separation case is the one that matters most. The two arms stand a
metre apart in the real lab; a scene drawn half a metre out would look
entirely plausible and be entirely wrong.
"""

import hashlib
import json
import os
import re
import shutil
import xml.etree.ElementTree as ElementTree

import pytest

xacro = pytest.importorskip('xacro', reason='xacro is not installed')
ament = pytest.importorskip('ament_index_python.packages',
                            reason='the ament index is not available')

from franka_web.ghost_assets.asset_prep import (   # noqa: E402, I100
    MAX_ASSET_BYTES, prepare_assets)
from franka_web.ghost_assets.urdf_export import (  # noqa: E402
    collect_mesh_references, XACRO_ARGS)

MESH_NAME = re.compile(r'^link[0-7]\.[0-9a-f]{12}\.ghostmesh\.(json|bin)$')


def description_share():
    """Return the installed description's share directory, or skip."""
    try:
        return ament.get_package_share_directory('franka_description')
    except Exception:                     # noqa: BLE001 - a missing package
        pytest.skip('franka_description is not installed')


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    """Generate the asset tree once for the whole module."""
    description_share()
    if shutil.which('xacro') is None:
        pytest.skip('the xacro executable is not on PATH')
    root = tmp_path_factory.mktemp('ghost_assets')
    result = prepare_assets(root)
    return root, result


def read_manifest(root):
    """Return the generated manifest."""
    with open(os.path.join(str(root), 'manifest.json'), encoding='utf-8') as handle:
        return json.load(handle)


def file_digest(path):
    """Return one file's SHA-256."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


class TestGeneratedTree:
    """What the build produces, and what it says about itself."""

    def test_the_tree_holds_a_urdf_a_manifest_and_sixteen_mesh_files(self, generated):
        """Eight meshes, each as metadata plus binary. Both arms share the set."""
        root, result = generated
        assert os.path.isfile(os.path.join(str(root), 'model.urdf'))
        assert os.path.isfile(os.path.join(str(root), 'manifest.json'))
        meshes = sorted(os.listdir(os.path.join(str(root), 'meshes')))
        assert len(meshes) == 16
        assert result.mesh_count == 8
        assert all(MESH_NAME.match(name) for name in meshes), meshes

    def test_each_output_name_carries_a_digest_of_its_own_content(self, generated):
        """This is what makes caching them forever safe."""
        root, _result = generated
        meshes = os.path.join(str(root), 'meshes')
        for name in sorted(os.listdir(meshes)):
            if not name.endswith('.bin'):
                continue
            stem, digest12, _rest = name.split('.', 2)
            assert file_digest(os.path.join(meshes, name)).startswith(digest12)
            assert stem.startswith('link')

    def test_the_metadata_names_its_own_binary(self, generated):
        """The renderer resolves this relative to the metadata's own URL."""
        root, _result = generated
        meshes = os.path.join(str(root), 'meshes')
        for name in sorted(os.listdir(meshes)):
            if not name.endswith('.json'):
                continue
            with open(os.path.join(meshes, name), encoding='utf-8') as handle:
                metadata = json.load(handle)
            assert metadata['schema'] == 'franka.ghost.mesh/1'
            assert os.path.isfile(os.path.join(meshes, metadata['bin']))
            assert metadata['bin'] == name.replace('.json', '.bin')

    def test_the_manifest_records_the_arguments_it_expanded_with(self, generated):
        """
        A digest without its arguments cannot be reproduced by anyone.

        Another consumer hashing a differently-parameterised expansion of the
        same description would get a different answer and have no way to see
        why.
        """
        root, _result = generated
        manifest = read_manifest(root)
        assert manifest['schema'] == 'franka.ghost.manifest/1'
        assert manifest['generated_from']['args'] == XACRO_ARGS
        assert manifest['generated_from']['xacro'].startswith('franka_description/')

    def test_every_input_digest_matches_a_fresh_reading(self, generated):
        """The manifest's provenance is checked against the real files."""
        root, _result = generated
        manifest = read_manifest(root)
        share = description_share()
        for uri, digest in manifest['generated_from']['mesh_sha256'].items():
            relative = uri.split('package://franka_description/', 1)[1]
            assert file_digest(os.path.join(share, relative)) == digest

    def test_the_recorded_urdf_digest_is_the_urdf_that_was_written(self, generated):
        """The digest the scene reports is the file the scene loads."""
        root, _result = generated
        manifest = read_manifest(root)
        assert (file_digest(os.path.join(str(root), 'model.urdf'))
                == manifest['generated_from']['urdf_sha256'])

    def test_every_mesh_the_urdf_references_was_converted(self, generated):
        """Nothing the renderer will ask for is missing from the manifest."""
        root, _result = generated
        with open(os.path.join(str(root), 'model.urdf'), encoding='utf-8') as handle:
            urdf = handle.read()
        manifest = read_manifest(root)
        assert set(collect_mesh_references(urdf)) == set(manifest['meshes'])

    def test_the_tree_stays_under_the_ceiling(self, generated):
        """A future end-effector mesh set is a conversation, not a surprise."""
        _root, result = generated
        assert result.asset_bytes < MAX_ASSET_BYTES

    def test_a_second_run_produces_the_same_bytes_and_does_no_work(self, generated):
        """
        Determinism, and the fast path that makes generation affordable.

        Two runs over unchanged inputs must agree byte for byte -- otherwise
        the content-addressed names would churn on every build and the
        immutable cache lane would be a lie.
        """
        root, first = generated
        before = {name: file_digest(os.path.join(str(root), 'meshes', name))
                  for name in sorted(os.listdir(os.path.join(str(root), 'meshes')))}
        second = prepare_assets(root)
        after = {name: file_digest(os.path.join(str(root), 'meshes', name))
                 for name in sorted(os.listdir(os.path.join(str(root), 'meshes')))}
        assert before == after
        assert second.regenerated is False
        assert second.asset_bytes == first.asset_bytes

    def test_a_changed_input_is_not_skipped(self, generated, tmp_path):
        """The fast path is keyed on the inputs, so it cannot mask a change."""
        root, _result = generated
        manifest_path = os.path.join(str(root), 'manifest.json')
        manifest = read_manifest(root)
        manifest['generated_from']['urdf_sha256'] = '0' * 64
        with open(manifest_path, 'w', encoding='utf-8') as handle:
            json.dump(manifest, handle)
        assert prepare_assets(root).regenerated is True


class TestTheRightLab:
    """The generated description must describe the bench that exists."""

    def base_origins(self, root):
        """Return the two arm bases' origins, parsed from the URDF."""
        tree = ElementTree.parse(os.path.join(str(root), 'model.urdf'))
        origins = {}
        for joint in tree.getroot().iter('joint'):
            name = joint.attrib.get('name', '')
            if not name.endswith('_joint_base_link'):
                continue
            origin = joint.find('origin')
            origins[name] = [float(value)
                             for value in origin.attrib['xyz'].split()]
        return origins

    def test_the_two_bases_stand_one_metre_apart(self, generated):
        """
        Parsed, not grepped.

        The description passes the literal string "0 +0.50 0" through
        unevaluated, so a grep for "0 0.5 0" finds nothing on a correctly
        fixed tree and everything on a broken one -- backwards. Parsing the
        two origins says what is actually true, and survives a reformatting
        of the literal.
        """
        root, _result = generated
        origins = self.base_origins(root)
        assert set(origins) == {'panda1_joint_base_link', 'panda2_joint_base_link'}
        first = origins['panda1_joint_base_link']
        second = origins['panda2_joint_base_link']
        assert abs(first[1] - second[1]) == pytest.approx(1.00, abs=1e-9)
        assert first[0] == second[0] == 0.0
        assert first[2] == second[2] == 0.0

    def test_neither_base_carries_the_inherited_half_separation(self, generated):
        """
        The old wrong number, checked where it would actually appear.

        0.26 still occurs elsewhere in the description -- link5 carries a
        collision cylinder at z = -0.26 -- so a whole-file grep for it fails
        on a correct tree. The claim that matters is about the two base
        joints, and that is what this reads.
        """
        root, _result = generated
        for origin in self.base_origins(root).values():
            assert all(abs(abs(value) - 0.26) > 1e-9 for value in origin)

    def test_both_arms_are_in_the_description(self, generated):
        """One mesh set, two arms, one source of truth for the separation."""
        root, _result = generated
        tree = ElementTree.parse(os.path.join(str(root), 'model.urdf'))
        links = {link.attrib['name'] for link in tree.getroot().iter('link')}
        for arm_id in ('panda1', 'panda2'):
            for index in range(9):
                assert '{}_link{}'.format(arm_id, index) in links
