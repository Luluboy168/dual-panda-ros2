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
Shared pytest wiring: make test/support importable as ``support``.

While the configuration subsystem is being built in a parallel change, this
also installs ``support.part1_stub`` under the real module names when — and
only when — ``franka_web.config`` / ``franka_web.defaults`` cannot be
imported. The call is a no-op once the real modules exist; delete it, and
``support/part1_stub/``, when they land.
"""

import os
import sys

sys.dont_write_bytecode = True

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from support import part1_stub  # noqa: E402 - the path above must be set first

USING_CONFIG_DOUBLE = part1_stub.install()

if USING_CONFIG_DOUBLE:
    # Tests that start a real subprocess importing franka_web -- the launch
    # guardian, the end-to-end server -- get no conftest, so the double
    # travels to them on PYTHONPATH via a sitecustomize. This whole block
    # goes away with support/part1_stub.
    _SHIM_DIR = os.path.join(_TEST_DIR, 'support', 'subprocess_shim')
    _INHERITED = os.environ.get('PYTHONPATH', '')
    if _SHIM_DIR not in _INHERITED.split(os.pathsep):
        os.environ['PYTHONPATH'] = (
            _SHIM_DIR + (os.pathsep + _INHERITED if _INHERITED else ''))
