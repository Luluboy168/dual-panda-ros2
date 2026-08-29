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
Entry point and process wiring for the franka_web server.

One process, four roles (plan §3.1): the MAIN thread runs the
``SessionSupervisor`` loop and is the only thread that ever spawns child
processes (``PR_SET_PDEATHSIG`` fires when the spawning thread dies, so
"parent thread died" must equal "process died"); a ``ros`` daemon thread
spins the single rclpy node; ``http`` daemon threads serve requests; a
``pump`` daemon thread publishes the 5 Hz state frames and 10 s pings.

There are deliberately no command-line options besides ``--help`` and
``--check-config``: every setting is environment-sourced (see
:mod:`franka_web.config`), so a robot address can never appear in a process
list or a shell history file.
"""

import argparse
import os
import signal
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from franka_web import config
from franka_web.config import ConfigError, SERVER_NAME, SERVER_VERSION, Settings

_ENVIRONMENT_HELP = """\
environment (the only configuration surface; there are no value-carrying flags):
  FRANKA_WEB_BIND            listen address, 127.0.0.1 (default) or ::1 only
  FRANKA_WEB_PORT            listen port, 1024..65535 (default 8781)
  FRANKA_WEB_STATE_DIR       private server state directory (created 0700)
  FRANKA_WEB_RECORDING_ROOT  existing private directory for session bags
  FRANKA_WEB_FRANKA_DIR      libfranka build directory for the RT preflight
  FRANKA_WEB_ROBOT_IP_1/_2   robot addresses for dual watch/motion sessions
  FRANKA_WEB_ROBOT_IP        robot address for single-arm watch/motion sessions
  ROS_DOMAIN_ID              required; propagated to every child, never invented

Robot addresses are read from this environment only. They are never accepted
from the browser, never logged, and never included in any response.
"""


def build_parser():
    """Build the argument parser (shared by ``--help`` output and tests)."""
    parser = argparse.ArgumentParser(
        prog='franka_web_server',
        description='Localhost-only operator web interface for the multipanda bringup '
                    '({} {}).'.format(SERVER_NAME, SERVER_VERSION),
        epilog=_ENVIRONMENT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--check-config',
        action='store_true',
        help='validate the FRANKA_WEB_* environment and exit (0 valid, 2 invalid)',
    )
    return parser


def main(argv=None):
    """Run the franka_web server entry point."""
    args = build_parser().parse_args(argv)
    try:
        settings = Settings.from_env()
    except ConfigError as error:
        print('franka_web_server: configuration invalid: {}'.format(error), file=sys.stderr)
        return 2
    if args.check_config:
        print('franka_web_server: configuration valid')
        return 0
    return serve(settings)


def serve(settings):
    """Wire everything and run the supervisor loop on this (main) thread."""
    # Imports here keep `--help`/`--check-config` usable without a spinning
    # ROS context (and fast); serve() is the only caller that needs them.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from franka_web.gains import GainsStore
    from franka_web.http_api import App, build_server
    from franka_web.launcher import LauncherError, PidfileLock
    from franka_web.lock import OperatorLock
    from franka_web.ros_bridge import FrankaWebBridge
    from franka_web.session import SessionSupervisor
    from franka_web.sse import Broker

    os.makedirs(settings.state_dir, mode=0o700, exist_ok=True)
    pidfile = PidfileLock(os.path.join(settings.state_dir, 'franka_web.pid'))
    try:
        pidfile.acquire()
    except LauncherError as error:
        print('franka_web_server: {}'.format(error), file=sys.stderr)
        return 1

    rclpy.init()
    bridge = FrankaWebBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, name='ros', daemon=True)

    lock = OperatorLock()
    broker = Broker()
    gains_store = GainsStore(settings.state_dir)
    supervisor = SessionSupervisor(settings, bridge, lock, broker,
                                   gains_store=gains_store)
    bridge.set_jog_callback(supervisor.jog_stream_tick)
    static_root = os.path.join(get_package_share_directory('franka_web'), 'static')
    app = App(settings=settings, supervisor=supervisor, lock=lock,
              broker=broker, static_root=static_root, gains_store=gains_store)
    httpd = build_server(app)
    http_thread = threading.Thread(target=httpd.serve_forever, name='http', daemon=True)

    shutdown_event = threading.Event()

    def _on_signal(signum, _frame):
        shutdown_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    pump_thread = threading.Thread(
        target=_frame_pump, name='pump', daemon=True,
        args=(supervisor, lock, broker, shutdown_event))

    def _close_http_on_shutdown():
        # The moment shutdown is signalled, stop accepting HTTP and unwind
        # the SSE streams -- BEFORE the supervisor's (potentially long)
        # teardown, so no operator command can be enqueued against a dying
        # server and no stream thread lingers.
        shutdown_event.wait()
        httpd.shutdown()
        broker.close_all()

    http_closer = threading.Thread(
        target=_close_http_on_shutdown, name='http-closer', daemon=True)

    ros_thread.start()
    http_thread.start()
    pump_thread.start()
    http_closer.start()
    print('franka_web_server: {} {} serving on http://{}:{} (domain {}). {}'.format(
        SERVER_NAME, SERVER_VERSION, settings.bind, settings.port,
        settings.ros_domain_id, config.STOP_ADVISORY))

    try:
        supervisor.run_forever(shutdown_event)
    finally:
        shutdown_event.set()
        httpd.shutdown()
        executor.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown()
        pidfile.release()
    return 0


def _frame_pump(supervisor, lock, broker, shutdown_event):
    """Publish 5 Hz state frames and 10 s pings; watch the operator lock."""
    from franka_web.session import rfc3339
    next_ping = time.monotonic()
    interval = 1.0 / config.STATE_FRAME_HZ
    was_locked = lock.state()['locked']
    while not shutdown_event.is_set():
        broker.publish('state', supervisor.frame())
        locked = lock.state()['locked']
        if was_locked and not locked:
            # Edge-triggered: exactly one release notification per expiry,
            # never a 5 Hz stream of them (review finding S2).
            supervisor.operator_released()
        was_locked = locked
        now = time.monotonic()
        if now >= next_ping:
            broker.publish('ping', {'schema_version': config.SCHEMA_VERSION,
                                    't': rfc3339()})
            next_ping = now + config.SSE_PING_INTERVAL_S
        shutdown_event.wait(interval)


if __name__ == '__main__':
    raise SystemExit(main())
