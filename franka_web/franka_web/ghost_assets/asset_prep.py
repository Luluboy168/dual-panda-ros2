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
Orchestrate deterministic generation of the scene's assets.

Run once by the build: it expands the dual-Panda description, converts the
eight COLLADA meshes into the browser format, and writes a manifest naming
every input digest. A second run over unchanged inputs re-hashes and returns
without work.

Nothing here is committed to git. The measured cost of a cold run is about a
second, and a generated tree cannot go stale behind a description change --
which is not theoretical: the base-separation fix moved both arms, and a
committed model.urdf would have kept drawing the old lab with a green test
suite.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .mesh_convert import convert_mesh
from .urdf_export import (
    collect_mesh_references,
    description_share_directory,
    expand_dual_urdf,
    sha256_bytes,
    XACRO_ARGS,
    XACRO_RELATIVE_PATH,
)


MANIFEST_SCHEMA = 'franka.ghost.manifest/1'
#: The generated tree measures 8.73 MiB today, so this ceiling leaves 3.27
#: MiB of headroom. A future end-effector mesh set that blows it is a
#: deliberate conversation, not a silent regression.
MAX_ASSET_BYTES = 12 * 1024 * 1024


@dataclass(frozen=True)
class PrepResult:
    """Summary of an asset preparation run."""

    regenerated: bool
    asset_bytes: int
    mesh_count: int


def default_package_root() -> Path:
    """Return the source package root, including under a symlink install."""
    return Path(__file__).resolve().parents[1]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _resolve_package_uri(uri: str, description_share: Path) -> Path:
    prefix = 'package://'
    if not uri.startswith(prefix):
        raise ValueError(f'not a package URI: {uri}')
    package_and_path = uri[len(prefix):].split('/', 1)
    if len(package_and_path) != 2 or not all(package_and_path):
        raise ValueError(f'invalid package URI: {uri}')
    package, relative = package_and_path
    if package == 'franka_description':
        return description_share / relative
    return Path(get_package_share_directory(package)) / relative


def _inputs(
    mesh_references: tuple[str, ...],
    description_share: Path,
) -> tuple[dict[str, Path], dict[str, str]]:
    paths = {
        uri: _resolve_package_uri(uri, description_share)
        for uri in mesh_references
    }
    for uri, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f'mesh for {uri} not found: {path}')
    hashes = {uri: _file_sha256(path) for uri, path in paths.items()}
    return paths, hashes


def _manifest_inputs(urdf_hash: str, mesh_hashes: dict[str, str]) -> dict:
    """
    Return the manifest fields that depend only on the INPUTS.

    The mesh mapping is deliberately absent: an output name now carries a
    digest of its own content, so it cannot be known before the conversion
    runs. The input digests fully determine the outputs, which is what makes
    the skip check below sound without it.
    """
    return {
        'schema': MANIFEST_SCHEMA,
        'generated_from': {
            'xacro': f'franka_description/{XACRO_RELATIVE_PATH.as_posix()}',
            'args': XACRO_ARGS,
            'urdf_sha256': urdf_hash,
            'mesh_sha256': mesh_hashes,
        },
        'urdf': 'model.urdf',
    }


def _output_hashes(
    asset_root: Path,
    manifest: dict,
    expected_paths: set[str],
) -> dict[str, str] | None:
    output_hashes = manifest.get('asset_sha256')
    if not isinstance(output_hashes, dict) or set(output_hashes) != expected_paths:
        return None
    for relative, expected in output_hashes.items():
        path = asset_root / relative
        if not path.is_file() or _file_sha256(path) != expected:
            return None
    return output_hashes


def _can_skip(asset_root: Path, expected: dict) -> bool:
    manifest_path = asset_root / 'manifest.json'
    if not manifest_path.is_file():
        return False
    try:
        existing = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (json.JSONDecodeError, OSError):
        return False
    for key in ('schema', 'generated_from', 'urdf'):
        if existing.get(key) != expected.get(key):
            return False
    # The mesh mapping is compared by its KEYS -- the source URIs -- because
    # its values are content-addressed names that only a conversion can
    # produce. The values are then verified byte for byte below.
    if not isinstance(existing.get('meshes'), dict):
        return False
    if set(existing['meshes']) != set(expected['generated_from']['mesh_sha256']):
        return False
    expected_paths = {'model.urdf'}
    for metadata_path in existing['meshes'].values():
        expected_paths.add(metadata_path)
        expected_paths.add(metadata_path.removesuffix('.json') + '.bin')
    return _output_hashes(asset_root, existing, expected_paths) is not None


def _asset_size(asset_root: Path) -> int:
    return sum(path.stat().st_size for path in asset_root.rglob('*') if path.is_file())


def prepare_assets(
    asset_root: Path,
    description_share: Path | None = None,
) -> PrepResult:
    """
    Generate model.urdf, the mesh set and the manifest into ``asset_root``.

    The renderer is NOT copied here. It is a committed static file, because a
    build must not require a distribution package to be installed before the
    console can be built.
    """
    asset_root = Path(asset_root).resolve()
    share = Path(description_share or description_share_directory()).resolve()

    urdf_text = expand_dual_urdf(share)
    urdf_bytes = urdf_text.encode('utf-8')
    urdf_hash = sha256_bytes(urdf_bytes)
    mesh_references = collect_mesh_references(urdf_text)
    mesh_paths, mesh_hashes = _inputs(mesh_references, share)
    expected = _manifest_inputs(urdf_hash, mesh_hashes)

    if _can_skip(asset_root, expected):
        size = _asset_size(asset_root)
        if size > MAX_ASSET_BYTES:
            raise ValueError(
                f'generated asset payload is {size} bytes; '
                f'limit is {MAX_ASSET_BYTES}'
            )
        return PrepResult(False, size, len(mesh_hashes))

    mesh_root = asset_root / 'meshes'
    if mesh_root.is_dir():
        # Content-addressed names change with the content, so a stale mesh
        # from a previous description would otherwise sit in the tree
        # forever, counted by the size ceiling and served to nobody.
        for stale in sorted(mesh_root.iterdir()):
            if stale.is_file():
                stale.unlink()
    mesh_root.mkdir(parents=True, exist_ok=True)
    (asset_root / 'model.urdf').write_bytes(urdf_bytes)
    generated_files = ['model.urdf']
    mesh_outputs = {}
    for uri, source_path in mesh_paths.items():
        source_label = uri.removeprefix('package://')
        converted = convert_mesh(source_path, mesh_root, source_label)
        mesh_outputs[uri] = f'meshes/{converted.metadata_path.name}'
        generated_files.extend(
            [
                f'meshes/{converted.metadata_path.name}',
                f'meshes/{converted.binary_path.name}',
            ]
        )

    manifest = dict(expected)
    manifest['meshes'] = mesh_outputs
    manifest['asset_sha256'] = {
        relative: _file_sha256(asset_root / relative)
        for relative in sorted(generated_files)
    }
    (asset_root / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, separators=(',', ': ')) + '\n',
        encoding='utf-8',
    )
    size = _asset_size(asset_root)
    if size > MAX_ASSET_BYTES:
        raise ValueError(
            f'generated asset payload is {size} bytes; limit is {MAX_ASSET_BYTES}'
        )
    return PrepResult(True, size, len(mesh_outputs))
