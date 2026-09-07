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

"""Shared pytest wiring: make test/support importable as ``support``."""

import os
import sys

sys.dont_write_bytecode = True

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)


def pytest_addoption(parser):
    """
    Register the one opt-in switch the browser suite has.

    A flag and not an environment variable, deliberately: this package has no
    environment contract at all -- a scan in test_review_regressions.py
    enforces that with no allowance -- and a test switch is a thing you pass
    to the runner, not a thing the package reads about itself.
    """
    parser.addoption(
        '--real-console', action='store_true', default=False,
        help='also run the case that drives a real franka_web console: it '
             'starts a server, a Simulate session and an IK service, holds '
             'port 8770, and takes a minute or two')
