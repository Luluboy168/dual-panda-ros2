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
Carry the configuration double into every child interpreter of the suite.

Several tests start a real subprocess that imports ``franka_web`` -- the
launch guardian, the end-to-end server -- and those interpreters get no
``conftest``. ``test/conftest.py`` puts THIS directory on ``PYTHONPATH``, so
``site`` imports this module in every child and the same doubles are
installed there.

DELETE THIS DIRECTORY, with the rest of ``support/part1_stub``, once the real
configuration subsystem is merged. Every step here is best-effort: an
interpreter that has nothing to do with franka_web must not fail to start
because of it.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_TEST_DIR = os.path.dirname(os.path.dirname(_HERE))

try:
    if _TEST_DIR not in sys.path:
        sys.path.insert(0, _TEST_DIR)
    from support import part1_stub
    part1_stub.install()
except Exception:  # noqa: BLE001 - never break an unrelated interpreter
    pass
