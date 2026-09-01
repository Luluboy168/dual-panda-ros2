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

One process, four roles: the MAIN thread runs the ``SessionSupervisor`` loop
and is the only thread that ever spawns child processes
(``PR_SET_PDEATHSIG`` fires when the spawning thread dies, so "parent thread
died" must equal "process died"); a ``ros`` daemon thread spins the single
rclpy node; ``http`` daemon threads serve requests; a ``pump`` daemon thread
publishes the 5 Hz state frames, the coalesced log batches and the 10 s pings.

There are two command-line flags and no others. ``--config PATH`` overrides
where the optional configuration file is read from, and ``--check-config``
validates it and exits. Everything else lives in that one file
(:mod:`franka_web.config`); there are no environment variables.
"""

import argparse
import os
import signal
import socket
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from franka_web import config, defaults

_CONFIG_HELP = """\
configuration:
  One optional YAML file, read from $XDG_CONFIG_HOME/franka_web/config.yaml
  (or ~/.config/franka_web/config.yaml). If it is missing, the server runs on
  pure defaults and says so in its banner. If it is present it is validated at
  startup: a valid file produces no extra output, an invalid one produces one
  helpful line naming the key and exits 2.

  Run `franka_web_server --check-config` to validate the file without starting
  the server. An installed, fully commented example ships with the package.

  Angles in the file are in DEGREES. The impedance controller's watchdog
  timing is fixed by its reviewed timing policy and is not settable from the
  file; it is reported read-only by GET /api/config.
"""

#: The newest N log events reach the SSE stream per pump tick. Anything older
#: in the same tick stays in the ring; the frame's `logs.last_seq` plus
#: GET /api/logs?since= backfills it.
_LOG_EVENTS_PER_TICK = 16

#: The production SSE queue depth. A `ros2 launch` emits hundreds of lines in
#: its first seconds -- exactly the window in which the startup checklist, the
#: hint and the drawer matter most -- so at the constructor's bare default of
#: 4 every `state` frame in that window would be evicted by `log` events.
#: Both halves are required: this depth AND the per-tick drain cap above.
_PRODUCTION_QUEUE_DEPTH = 64


def build_parser():
    """Build the argument parser (shared by ``--help`` output and tests)."""
    parser = argparse.ArgumentParser(
        prog='franka_web_server',
        description='Operator web console for the multipanda bringup '
                    '({} {}).'.format(defaults.SERVER_NAME,
                                      defaults.SERVER_VERSION),
        epilog=_CONFIG_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--config', default=None, metavar='PATH',
        help='configuration file (default: the XDG path shown below)')
    parser.add_argument(
        '--check-config', action='store_true',
        help='validate the configuration file and exit (0 valid, 2 invalid)')
    return parser


def main(argv=None):
    """Run the franka_web server entry point."""
    args = build_parser().parse_args(argv)
    try:
        settings = config.load(args.config, make_dirs=not args.check_config)
    except config.ConfigError as error:
        # Verbatim, with no prefix: the message already starts with the file
        # path, names the key, says what was found and says what is allowed.
        print(str(error), file=sys.stderr)
        return 2
    except Exception:  # noqa: BLE001 - an operator never sees a traceback
        print('franka_web_server: the configuration could not be read',
              file=sys.stderr)
        return 2
    if args.check_config:
        print('franka_web_server: configuration valid')
        return 0
    return serve(settings)


def _lan_address():
    """Return the host's first non-loopback IPv4, or None."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # RFC 5737 TEST-NET-1 on a UDP socket: no packet is ever sent, the
        # connect() only makes the kernel choose a source address.
        probe.connect(('192.0.2.1', 9))
        address = probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()
    return None if address.startswith('127.') else address


def banner_lines(settings):
    """Return the startup banner, one string per line."""
    lines = ['{} {} — open http://localhost:{}'.format(
        defaults.SERVER_NAME, defaults.SERVER_VERSION, settings.port)]
    if settings.bind in ('0.0.0.0', '::'):
        address = _lan_address()
        if address is not None:
            lines.append('  or from the lab network: http://{}:{}'.format(
                address, settings.port))
    lines.append('  config: {}'.format(
        settings.config_path if settings.config_present
        else 'defaults (no file at {})'.format(settings.config_path)))
    lines.append('  robots: panda1 {} · panda2 {} · domain {}'.format(
        settings.robot_ip('panda1'), settings.robot_ip('panda2'),
        settings.ros_domain_id))
    lines.append('  recordings: {}'.format(settings.recording_root))
    lines.append(defaults.STOP_ADVISORY)
    return lines


def _warn_on_changed_torque_ceilings(settings, log_bus):
    """
    Emit ONE warn line per arm whose torque ceilings left the proven set.

    Torque ceilings stay editable and are bounded by the Panda hardware
    ceiling, so this does not refuse and does not nag per session: it says
    once, at startup, that the safety bound is not the live-proven one.
    """
    for arm_id in sorted(defaults.DEFAULT_PROFILES):
        configured = tuple(settings.profile(arm_id).max_effort_nm)
        proven = tuple(defaults.DEFAULT_PROFILES[arm_id]['max_effort_nm'])
        if configured != proven:
            log_bus.emit(
                'warn',
                '{}: torque ceilings differ from the proven set: configured '
                '{} vs proven {}'.format(arm_id, configured, proven))


def serve(settings):
    """Wire everything and run the supervisor loop on this (main) thread."""
    # Imports here keep `--help`/`--check-config` usable without a spinning
    # ROS context (and fast); serve() is the only caller that needs them.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from franka_web.gains import ProfileStore
    from franka_web.http_api import App, build_server
    from franka_web.launcher import LauncherError, PidfileLock
    from franka_web.lock import OperatorLock
    from franka_web.logbus import LogBus
    from franka_web.ros_bridge import FrankaWebBridge
    from franka_web.session import SessionSupervisor
    from franka_web.sse import Broker

    # The bus comes up first, before the ROS bridge, so early lines are
    # captured and the drawer is not blind during startup.
    log_bus = LogBus()
    log_bus.emit('info', 'franka_web {} starting'.format(defaults.SERVER_VERSION))

    # Both directories are created symmetrically at 0700 when missing, and
    # NEITHER is a boot gate. config.load(make_dirs=True) already did this;
    # repeating it here is a harmless idempotent belt on both straps. The
    # reviewed recorder keeps its own owner-only check and remains the
    # authority on the recording root -- when it refuses, the server surfaces
    # the recorder's own sentence plus a chmod hint and nothing else.
    os.makedirs(settings.state_dir, mode=0o700, exist_ok=True)
    os.makedirs(settings.recording_root, mode=0o700, exist_ok=True)

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
    broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
    profile_store = ProfileStore(settings.state_dir)
    supervisor = SessionSupervisor(settings, bridge, lock, broker,
                                   profile_store=profile_store,
                                   log_bus=log_bus)
    bridge.set_jog_callback(supervisor.jog_stream_tick)
    static_root = os.path.join(get_package_share_directory('franka_web'), 'static')
    app = App(settings=settings, supervisor=supervisor, lock=lock,
              broker=broker, static_root=static_root,
              profile_store=profile_store, log_bus=log_bus)
    httpd = build_server(app)
    http_thread = threading.Thread(target=httpd.serve_forever, name='http', daemon=True)

    shutdown_event = threading.Event()

    def _on_signal(signum, _frame):
        shutdown_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    pump_thread = threading.Thread(
        target=_frame_pump, name='pump', daemon=True,
        args=(supervisor, lock, broker, shutdown_event, log_bus))

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
    for line in banner_lines(settings):
        print(line)
        log_bus.emit('info', line.strip())
    _warn_on_changed_torque_ceilings(settings, log_bus)

    try:
        supervisor.run_forever(shutdown_event)
    finally:
        shutdown_event.set()
        httpd.shutdown()
        executor.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown()
        pidfile.release()
    return 0


def _frame_pump(supervisor, lock, broker, shutdown_event, log_bus):
    """Publish 5 Hz state frames, coalesced log batches and 10 s pings."""
    from franka_web.session import rfc3339
    next_ping = time.monotonic()
    interval = 1.0 / defaults.STATE_FRAME_HZ
    while not shutdown_event.is_set():
        # Log events go out BEFORE the state frame, so the frame's
        # `logs.last_seq` is never ahead of the last published `log` event.
        # `debug` lines are captured and served by GET /api/logs, but never
        # streamed -- the drawer would be unreadable.
        pending = [line for line in log_bus.drain_pending()
                   if line.level != 'debug']
        for line in pending[-_LOG_EVENTS_PER_TICK:]:
            broker.publish('log', line.event())
        broker.publish('state', supervisor.frame())
        now = time.monotonic()
        if now >= next_ping:
            broker.publish('ping', {'schema_version': defaults.SCHEMA_VERSION,
                                    't': rfc3339()})
            next_ping = now + defaults.SSE_PING_INTERVAL_S
        shutdown_event.wait(interval)


if __name__ == '__main__':
    raise SystemExit(main())
