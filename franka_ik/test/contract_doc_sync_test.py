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

"""Keep the integration contract's rosidl blocks byte-identical to their sources."""

import os
from pathlib import Path
import re


IK_SOURCE_DIR = Path(os.environ['FRANKA_IK_SOURCE_DIR'])
INTERFACES_SOURCE_DIR = Path(os.environ['FRANKA_IK_INTERFACES_SOURCE_DIR'])
EXPECTED_PATHS = {
    'msg/IkRequest.msg',
    'msg/IkSolution.msg',
    'msg/IkResult.msg',
    'msg/ChainInfo.msg',
    'srv/SolveIk.srv',
    'srv/GetChainInfo.srv',
}


def test_canonical_rosidl_blocks_are_byte_identical():
    """Reject stale, missing, or extra public-interface blocks in CONTRACT.md."""
    contract = (IK_SOURCE_DIR / 'doc' / 'CONTRACT.md').read_text(encoding='utf-8')
    pattern = re.compile(
        r'^### franka_ik_interfaces/((?:msg|srv)/[^\n]+)\n\n```text\n(.*?)```$',
        re.MULTILINE | re.DOTALL,
    )
    documented = dict(pattern.findall(contract))
    assert set(documented) == EXPECTED_PATHS
    for relative_path in sorted(EXPECTED_PATHS):
        source = (INTERFACES_SOURCE_DIR / relative_path).read_text(encoding='utf-8')
        assert documented[relative_path] == source
