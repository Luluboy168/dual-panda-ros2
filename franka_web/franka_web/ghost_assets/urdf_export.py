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

"""Expand the dual-Panda xacro into a browser-ready URDF."""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


XACRO_RELATIVE_PATH = Path('robots/real/dual_panda_arm.urdf.xacro')
XACRO_ARGS = {
    'use_fake_hardware': 'true',
    'arm_id_1': 'panda1',
    'arm_id_2': 'panda2',
    'robot_ip_1': 'dont-care',
    'robot_ip_2': 'dont-care',
}


def sha256_bytes(data: bytes) -> str:
    """Return the lowercase SHA-256 digest of *data*."""
    return hashlib.sha256(data).hexdigest()


def description_share_directory() -> Path:
    """Resolve the installed ``franka_description`` share directory."""
    return Path(get_package_share_directory('franka_description'))


def expand_dual_urdf(
    description_share: Path | None = None,
    xacro_executable: str | Path | None = None,
) -> str:
    """Expand the dual-Panda xacro with fake-hardware-only arguments."""
    share = Path(description_share or description_share_directory()).resolve()
    xacro_path = share / XACRO_RELATIVE_PATH
    if not xacro_path.is_file():
        raise FileNotFoundError(f'dual-Panda xacro not found: {xacro_path}')

    executable = str(xacro_executable or shutil.which('xacro') or 'xacro')
    # Run from the share prefix with a stable package-relative input path. Besides
    # making includes work exactly as they do for an installed package, this
    # prevents xacro's generated-file banner from embedding a worktree or
    # install-prefix path in model.urdf.
    stable_input = Path('franka_description') / XACRO_RELATIVE_PATH
    command = [executable, stable_input.as_posix()]
    command.extend(f'{name}:={value}' for name, value in XACRO_ARGS.items())
    try:
        result = subprocess.run(
            command,
            check=True,
            capture_output=True,
            cwd=share.parent,
            encoding='utf-8',
        )
    except subprocess.CalledProcessError as error:
        detail = error.stderr.strip() or error.stdout.strip() or 'no diagnostic output'
        raise RuntimeError(f'xacro expansion failed: {detail}') from error
    return result.stdout


def collect_mesh_references(urdf_text: str) -> tuple[str, ...]:
    """Collect sorted, unique ``package://`` mesh filenames from a URDF."""
    root = ET.fromstring(urdf_text)
    references = {
        mesh.attrib['filename']
        for mesh in root.iter('mesh')
        if mesh.attrib.get('filename', '').startswith('package://')
    }
    return tuple(sorted(references))


def export_urdf(
    output_path: Path,
    description_share: Path | None = None,
    xacro_executable: str | Path | None = None,
) -> tuple[str, tuple[str, ...]]:
    """Expand and write ``model.urdf``, returning its digest and mesh URIs."""
    urdf_text = expand_dual_urdf(description_share, xacro_executable)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(urdf_text, encoding='utf-8')
    return sha256_bytes(urdf_text.encode('utf-8')), collect_mesh_references(urdf_text)
