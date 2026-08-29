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

# [THROWAWAY] Session C standalone server entry point; delete at merge.
"""Run the loopback-only ghost development server."""

import argparse
from pathlib import Path
from typing import Optional, Sequence

from franka_ghost.dev_server import create_server
from franka_ghost.joint_source import create_joint_source


def _positive_rate(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('rate must be greater than zero')
    return parsed


def build_parser() -> argparse.ArgumentParser:
    """Build the intentionally host-option-free command-line interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--web-root', type=Path, required=True)
    parser.add_argument('--source', choices=('ros', 'demo'), default='demo')
    parser.add_argument('--topic', default='/franka/joint_states')
    parser.add_argument('--rate', type=_positive_rate, default=20.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run until interrupted and close both HTTP and joint-source threads."""
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        source = create_joint_source(args.source, topic=args.topic)
        server = create_server(
            port=args.port,
            web_root=args.web_root,
            source=source,
            rate_hz=args.rate,
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))

    host, port = server.server_address
    print(f'franka ghost dev server: http://{host}:{port} ({args.source})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
