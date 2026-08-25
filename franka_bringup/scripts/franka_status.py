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

import json
import sys


def main(argv=None):
    """Load the installed status contract without exposing import tracebacks to operators."""
    try:
        from franka_bringup.status import main as status_main
    except (Exception, KeyboardInterrupt):
        print(json.dumps(
            {'error': 'franka_status initialization failed', 'ok': False},
            sort_keys=True, separators=(',', ':')), file=sys.stderr)
        return 2
    return status_main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
