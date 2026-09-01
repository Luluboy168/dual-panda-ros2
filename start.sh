#!/usr/bin/env bash
# Start the franka_web operator console from this workspace.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$here/install/setup.bash" ]; then
  echo "start.sh: this workspace is not built yet. Build it with:" >&2
  echo "  cd \"$here\" && colcon build --symlink-install --cmake-args -DFranka_DIR=/path/to/libfranka/build" >&2
  echo "Already built somewhere else? Source that workspace's install/setup.bash," >&2
  echo "then run: ros2 run franka_web franka_web_server" >&2
  exit 1
fi
set +u                                  # the colcon setup scripts read unset variables
# shellcheck source=/dev/null
. "$here/install/setup.bash"
set -u
exec ros2 run franka_web franka_web_server "$@"
