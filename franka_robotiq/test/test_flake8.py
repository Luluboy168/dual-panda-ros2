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
Package-wide ament_flake8 gate, run in a subprocess.

flake8 checks files through a ``multiprocessing`` pool, which FORKS. The rest
of this package's suite builds real ``rclpy`` nodes, and those leave executor
threads behind: forking a threaded process is the classic way to inherit a
held lock and deadlock in the child. In one pytest process -- which is exactly
what ``colcon test`` runs -- that turns this gate into a hang, or into the
segmentation fault the same fork produces when it does not hang. Running the
same tool with the same arguments in a fresh process removes the hazard
without weakening a single assertion.
"""

from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Every Python file passes flake8 with the ament configuration."""
    package_root = str(Path(__file__).resolve().parents[1])
    completed = subprocess.run(
        [sys.executable, '-m', 'ament_flake8.main',
         '--linelength', '99', package_root],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        timeout=300)
    output = completed.stdout.decode('utf-8', 'replace')
    assert completed.returncode == 0, 'found flake8 errors:\n{}'.format(output)
