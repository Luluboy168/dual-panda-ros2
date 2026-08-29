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
Frozen constants and environment-sourced settings for the franka_web server.

Every tunable the server uses lives here, in one place: network binding, the
jog contract numbers, stream cadences, timeouts, TTLs and size caps. The
numeric values are not free choices -- each one is pinned to a reviewed fact
of the stack it talks to (the impedance controller's watchdog and header-age
windows, the validator's config size cap, the recorder's duration bound, the
Fast-DDS domain-id ceiling) and the pin is stated next to the value.

``Settings.from_env`` reads and validates the ``FRANKA_WEB_*`` environment.
Robot addresses are deliberately env-only: they never appear as a default in
any tracked file, never in ``repr(Settings)``, and never in an error message
raised from this module (`AGENTS.md` and the session rules both forbid a
tracked or logged robot address).

The recording-root validation mirrors, check for check, what
``franka_bringup.recorder._open_directory_no_symlinks`` enforces on the same
path (normalized absolute path, every component opened with ``O_NOFOLLOW``,
final directory owned by this user with no group/other permission bits).
The recorder would refuse anyway at session start; refusing at server boot
with a readable message is friendlier to the operator.
"""

from dataclasses import dataclass, field, fields
import os
import re
import stat

# --- identity ---------------------------------------------------------------

SERVER_NAME = 'franka_web'
SERVER_VERSION = '0.1.0'
SCHEMA_VERSION = 1

# --- network binding (see plan section 5.7: localhost only, no exceptions) --

ALLOWED_BIND = ('127.0.0.1', '::1')
DEFAULT_BIND = '127.0.0.1'
DEFAULT_PORT = 8781
PORT_MINIMUM = 1024   # below this is privileged; the server never runs as root
PORT_MAXIMUM = 65535

# --- the jog / motion contract (pinned to the impedance controller) ---------

JOINT_COUNT = 7
JOG_STEP_RAD = 0.034906585        # 2 degrees, the one fixed step of the UI
JOG_STREAM_HZ = 20.0              # 2x the 10 Hz floor of the 0.1 s watchdog
WATCHDOG_TIMEOUT_S = 0.1          # controller watchdog_timeout (reviewed)
MAX_HEADER_AGE_S = 1.0            # controller max_header_age (reviewed)

# --- state fan-out ----------------------------------------------------------

STATE_FRAME_HZ = 5.0              # SSE `state` event cadence
SSE_PING_INTERVAL_S = 10.0        # SSE `ping` event cadence
SSE_QUEUE_DEPTH = 4               # per-subscriber bounded queue, drop-oldest

# --- operator lock ----------------------------------------------------------

OPERATOR_LOCK_TTL_S = 15.0
OPERATOR_HEARTBEAT_INTERVAL_S = 5.0

# --- supervisor timing ------------------------------------------------------

SUPERVISOR_TICK_S = 0.1
PREFLIGHT_TIMEOUT_S = 30.0
STARTING_TIMEOUT_S = 60.0
SERVICE_CALL_TIMEOUT_S = 5.0

# --- freshness and fault thresholds -----------------------------------------

ENABLE_JOINT_STATE_MAX_AGE_S = 0.2   # enable refuses on older samples
JOINT_STATE_STALE_FAULT_S = 1.0      # fault rule F6
POSE_CACHE_TTL_S = 120.0             # fence-vs-pose precondition cache
CCSR_FAULT_THRESHOLD = 0.95          # fault rule F4 (PHASE10 stop procedure)
CCSR_FAULT_SUSTAIN_S = 5.0

# --- recording --------------------------------------------------------------

RECORDING_SEGMENT_DURATION_S = 3600  # franka_record's hard --duration cap

# --- child stop escalation (same ladder as franka_bringup's recorder) -------

STOP_SIGINT_WAIT_S = 10.0
STOP_SIGTERM_WAIT_S = 5.0
STOP_SIGKILL_WAIT_S = 5.0

# The recorder child gets a longer first stage: after SIGINT, franka_record
# legitimately runs its own bounded ladder against `ros2 bag record` (up to
# ~25 s worst case). Escalating past SIGTERM kills it mid-seal and produces
# the unsealed, reindex-required bag it exists to prevent, so SIGINT gets
# 30 s, SIGTERM (still handled, still seals) 10 s, SIGKILL is last resort.

RECORDER_STOP_SIGINT_WAIT_S = 30.0
RECORDER_STOP_SIGTERM_WAIT_S = 10.0
RECORDER_STOP_SIGKILL_WAIT_S = 5.0

# --- gains upload -----------------------------------------------------------

MAX_GAINS_BYTES = 65536              # validator's MAXIMUM_CONFIG_BYTES

# --- ROS domain -------------------------------------------------------------

ROS_DOMAIN_ID_MAXIMUM = 232          # Fast-DDS port-arithmetic hard ceiling

# --- fixed operator-facing strings ------------------------------------------

STOP_ADVISORY = 'The physical stop buttons are the only real stop.'

_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW

# ASCII decimal digits only: int() alone would also accept '8_0', '+80' and
# non-ASCII digits, all of which rcl's strtoul parses differently (or rejects),
# silently landing children on the wrong DDS domain.
_DECIMAL_RE = re.compile('[0-9]+')

# A plausible robot address: hostname/IPv4 shape, no whitespace, no leading
# dash, nothing that could smuggle a second token into a launch argument.
_ADDRESS_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,253}')


class ConfigError(ValueError):
    """A FRANKA_WEB_* environment value is missing, malformed, or unsafe."""


def _read(environ, name):
    """Return the trimmed value of ``name`` or ``None``; blank means unset."""
    value = environ.get(name)
    if value is None:
        return None
    value = value.strip()
    return value or None


def _parse_port(text):
    """Parse and range-check the listen port (ASCII decimal digits only)."""
    if not _DECIMAL_RE.fullmatch(text):
        raise ConfigError('FRANKA_WEB_PORT must be a plain decimal integer, got {!r}'.format(text))
    port = int(text, 10)
    if not PORT_MINIMUM <= port <= PORT_MAXIMUM:
        raise ConfigError(
            'FRANKA_WEB_PORT must be within {}..{}, got {}'.format(
                PORT_MINIMUM, PORT_MAXIMUM, port))
    return port


def _parse_domain_id(text):
    """Parse and range-check ROS_DOMAIN_ID against the Fast-DDS ceiling."""
    if text is None:
        raise ConfigError(
            'ROS_DOMAIN_ID must be set explicitly; franka_web never invents a domain')
    if not _DECIMAL_RE.fullmatch(text):
        raise ConfigError(
            'ROS_DOMAIN_ID must be a plain decimal integer, got {!r}'.format(text))
    domain_id = int(text, 10)
    if not 0 <= domain_id <= ROS_DOMAIN_ID_MAXIMUM:
        raise ConfigError(
            'ROS_DOMAIN_ID must be within 0..{}, got {}'.format(ROS_DOMAIN_ID_MAXIMUM, domain_id))
    return domain_id


def _validate_robot_address(name, value):
    """
    Shape-check a robot address without ever placing it in a message.

    The value becomes a ``ros2 launch`` argument, so anything with whitespace,
    a leading dash, or characters outside hostname/IPv4 shape is refused --
    and the refusal deliberately never echoes the value itself.
    """
    if not _ADDRESS_RE.fullmatch(value):
        raise ConfigError(
            '{} is not a plausible robot address; refusing to pass it to a launch '
            '(the value is never echoed)'.format(name))
    return value


def _require_normalized_absolute(name, path):
    """
    Reject relative or non-normalized directory paths outright.

    Trailing slashes are stripped rather than refused (the recorder's
    ``Path``-based check tolerates them too); an embedded NUL is refused
    here so no later ``os`` call can raise ``ValueError`` on it.
    """
    if '\x00' in path:
        raise ConfigError('{} contains an invalid character'.format(name))
    stripped = path.rstrip(os.sep) or os.sep
    if not os.path.isabs(stripped) or os.path.normpath(stripped) != stripped:
        raise ConfigError('{} must be a normalized absolute path'.format(name))
    return stripped


def _walk_directory_no_symlinks(name, path):
    """
    Open ``path`` component by component with ``O_NOFOLLOW`` and return the fd.

    This is the recorder's `_open_directory_no_symlinks` walk: it cannot be
    raced through a symlink swap because no component is ever resolved through
    a symlink at all.
    """
    descriptor = os.open('/', _DIRECTORY_FLAGS)
    walked = '/'
    try:
        for component in path.split(os.sep):
            if not component:
                continue
            walked = os.path.join(walked, component)
            try:
                next_descriptor = os.open(component, _DIRECTORY_FLAGS, dir_fd=descriptor)
            except FileNotFoundError:
                raise ConfigError('{} does not exist'.format(name)) from None
            except PermissionError:
                raise ConfigError(
                    '{} is not accessible (permission denied on a path '
                    'component)'.format(name)) from None
            except OSError:
                # A symlink opened with O_NOFOLLOW|O_DIRECTORY surfaces as
                # ENOTDIR on Linux, the same errno as a plain file; lstat the
                # walked prefix only to pick the right refusal message.
                if os.path.islink(walked):
                    raise ConfigError(
                        '{} must contain no symlink component'.format(name)) from None
                raise ConfigError('{} must be a directory'.format(name)) from None
            os.close(descriptor)
            descriptor = next_descriptor
        return descriptor
    except ConfigError:
        os.close(descriptor)
        raise
    except Exception:
        # Nothing else is expected here; never leak the fd or a traceback.
        os.close(descriptor)
        raise ConfigError('{} is not a usable path'.format(name)) from None


def _validate_owned_directory(name, path, geteuid, require_private, deny_shared_write=False):
    """
    Require an existing directory, reached without symlinks, owned by us.

    With ``require_private`` the directory must carry mode 0700 exactly: no
    group/other bits (the recorder's rule) AND all three owner bits -- a
    0500 root would pass the recorder's boot-visible checks and then fail
    every session at ``mkdir`` time, which is exactly the late failure this
    boot gate exists to prevent. With ``deny_shared_write`` only group/other
    *write* bits are refused (for parents of directories we will create).
    Intermediate path components are never mode-checked, only opened with
    ``O_NOFOLLOW`` (exactly the recorder's walk).
    """
    path = _require_normalized_absolute(name, path)
    descriptor = _walk_directory_no_symlinks(name, path)
    try:
        try:
            details = os.fstat(descriptor)
        except OSError:
            raise ConfigError('{} could not be inspected'.format(name)) from None
        if not stat.S_ISDIR(details.st_mode):
            raise ConfigError('{} must be a directory'.format(name))
        if details.st_uid != geteuid():
            raise ConfigError('{} must be owned by the user running the server'.format(name))
        mode = stat.S_IMODE(details.st_mode)
        if require_private:
            if mode & 0o077:
                raise ConfigError(
                    '{} must have no group or other permission bits '
                    '(expected mode 0700)'.format(name))
            if mode & 0o700 != 0o700:
                raise ConfigError(
                    '{} must be owner-readable, writable and searchable '
                    '(expected mode 0700)'.format(name))
        if deny_shared_write and mode & 0o022:
            raise ConfigError('{} must not be writable by group or others'.format(name))
    finally:
        os.close(descriptor)
    return path


def _validate_state_dir(path, geteuid):
    """
    Validate the state directory path; it may not exist yet.

    If it exists it must be a private (mode 0700) directory owned by us. If
    not, its parent must be a symlink-free directory owned by us and not
    writable by group or others (a shared-writable parent would let another
    local user pre-create or replace the state dir that holds the operator
    lock token), and the server creates the state directory itself (mode
    0700) at boot.
    """
    path = _require_normalized_absolute('FRANKA_WEB_STATE_DIR', path)
    if os.path.lexists(path):
        return _validate_owned_directory(
            'FRANKA_WEB_STATE_DIR', path, geteuid, require_private=True)
    parent = os.path.dirname(path)
    _validate_owned_directory(
        'FRANKA_WEB_STATE_DIR parent', parent, geteuid,
        require_private=False, deny_shared_write=True)
    return path


def _validate_franka_dir(path, geteuid):
    """
    Validate the libfranka build directory used by the RT preflight.

    Same walk as every other directory here: symlink-free, owned by us, not
    writable by group or others. No 0700 requirement -- a build tree is
    normally 0755.
    """
    return _validate_owned_directory(
        'FRANKA_WEB_FRANKA_DIR', path, geteuid,
        require_private=False, deny_shared_write=True)


@dataclass(frozen=True)
class Settings:
    """
    Validated server settings, sourced from the environment only.

    The three robot-address fields carry ``repr=False``: a ``Settings`` value
    can be logged safely and no address ever reaches a log line, an API
    response, or an exception message.

    ``repr=False`` protects only ``repr``/``str``. ``dataclasses.asdict``,
    ``astuple`` and ``vars`` still expose the addresses -- NEVER serialize a
    ``Settings`` wholesale; anything leaving the process must be built
    field-by-field, skipping :func:`_redacted_field_names`.
    """

    bind: str
    port: int
    state_dir: str
    recording_root: str
    ros_domain_id: int
    franka_dir: str = None
    robot_ip_1: str = field(default=None, repr=False)
    robot_ip_2: str = field(default=None, repr=False)
    robot_ip_single: str = field(default=None, repr=False)

    @classmethod
    def from_env(cls, environ=None, geteuid=os.geteuid):
        """
        Read and validate every FRANKA_WEB_* setting from ``environ``.

        ``environ`` defaults to ``os.environ``; tests pass a plain dict.
        ``geteuid`` is injectable so ownership refusal is testable without
        root. Raises :class:`ConfigError` on the first invalid value.
        """
        if environ is None:
            environ = os.environ

        bind = _read(environ, 'FRANKA_WEB_BIND') or DEFAULT_BIND
        if bind not in ALLOWED_BIND:
            raise ConfigError(
                'FRANKA_WEB_BIND must be one of {}; refusing to bind a routable address'.format(
                    ', '.join(ALLOWED_BIND)))

        port_text = _read(environ, 'FRANKA_WEB_PORT')
        port = DEFAULT_PORT if port_text is None else _parse_port(port_text)

        state_dir = _read(environ, 'FRANKA_WEB_STATE_DIR')
        if state_dir is None:
            raise ConfigError('FRANKA_WEB_STATE_DIR must be set')
        state_dir = _validate_state_dir(state_dir, geteuid)

        recording_root = _read(environ, 'FRANKA_WEB_RECORDING_ROOT')
        if recording_root is None:
            raise ConfigError('FRANKA_WEB_RECORDING_ROOT must be set')
        recording_root = _validate_owned_directory(
            'FRANKA_WEB_RECORDING_ROOT', recording_root, geteuid, require_private=True)

        franka_dir = _read(environ, 'FRANKA_WEB_FRANKA_DIR')
        if franka_dir is not None:
            franka_dir = _validate_franka_dir(franka_dir, geteuid)

        addresses = {}
        for field_name, env_name in (
                ('robot_ip_1', 'FRANKA_WEB_ROBOT_IP_1'),
                ('robot_ip_2', 'FRANKA_WEB_ROBOT_IP_2'),
                ('robot_ip_single', 'FRANKA_WEB_ROBOT_IP')):
            value = _read(environ, env_name)
            addresses[field_name] = (
                None if value is None else _validate_robot_address(env_name, value))

        return cls(
            bind=bind,
            port=port,
            state_dir=state_dir,
            recording_root=recording_root,
            ros_domain_id=_parse_domain_id(_read(environ, 'ROS_DOMAIN_ID')),
            franka_dir=franka_dir,
            **addresses,
        )


def _redacted_field_names():
    """Return the Settings field names whose values must never be shown."""
    return tuple(f.name for f in fields(Settings) if not f.repr)
