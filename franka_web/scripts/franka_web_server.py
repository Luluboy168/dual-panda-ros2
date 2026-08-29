#!/usr/bin/env python3

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

"""Thin installed launcher for the franka_web server (franka_status.py style)."""

import sys


def main(argv=None):
    """Load the installed server entry point without exposing import tracebacks."""
    try:
        from franka_web.server import main as server_main
    except (Exception, KeyboardInterrupt):
        print('franka_web_server: initialization failed; is the workspace sourced?',
              file=sys.stderr)
        return 2
    return server_main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
