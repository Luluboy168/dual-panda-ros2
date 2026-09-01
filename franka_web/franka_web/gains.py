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
Render the session's controller profile from config, and content-address it.

There is no upload surface. A Motion session's impedance parameters come from
the operator's ``config.yaml`` (or the baked-in defaults), are RENDERED here
into the ``/dual_arm_joint_impedance_controller`` document the reviewed
launch expects, VALIDATED by the reviewed validator in-process, and written
to ``<state_dir>/profiles/<sha256>.yaml`` -- the exact path that
``controller_param_file:=`` receives.

The validator is IMPORTED, never shelled out to
------------------------------------------------
``franka_validate_controller_config``'s ``main()`` takes only
``--controller-name``. It has no ``--arm-id``, so it routes every file
through the two-arm ``validate_controller_config_text`` and CANNOT validate
the one-arm (``arm_count: 1``, ``arm_1`` only) layout at all. A single-arm
session's profile therefore has to go through
``validate_single_controller_config_text`` in process, and that is what
:meth:`ProfileStore.materialize` does.

Content addressing, and why the store is idempotent
---------------------------------------------------
The file name is the SHA-256 the validator computes over the rendered text,
so the same profile always lands on the same path and re-rendering it returns
the record that already exists. Creation goes through
``O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC`` relative to a directory file
descriptor opened with ``O_NOFOLLOW`` at every component: no symlink is ever
followed, and an existing name is never truncated -- it is read back and
re-hashed instead.

What raises what
----------------
:class:`ProfileStoreError` (code ``profile_invalid``) means the RENDERED
document was refused by the reviewed validator: a server bug, or a
hand-edited configuration that slipped the loader. Its detail is the
validator's own sentence, verbatim. Anything wrong with the STORE ITSELF (the
profiles directory replaced by a symlink, a foreign file squatting a
content-addressed name, a permission mode nobody sane set) raises ``OSError``
instead and is deliberately NOT dressed up as a 4xx: the profile was fine,
the filesystem is not.
"""

from dataclasses import dataclass
import errno
import hashlib
import os
import stat
from types import MappingProxyType

from franka_bringup.controller_config_validator import (
    ControllerConfigError,
    load_strict_yaml,
    validate_controller_config_text,
    validate_single_controller_config_text,
)
from franka_web import defaults

# The three legal arm selections, mapped to the arm IDs they name in slot
# order. `both` is the two-arm layout; `panda1`/`panda2` are the one-arm
# layout, where the single `arm_1` block carries the CHOSEN arm's ID.
ARM_SELECTIONS = MappingProxyType({
    'panda1': ('panda1',),
    'panda2': ('panda2',),
    'both': ('panda1', 'panda2'),
})

# Dual configs address their arms by slot; single configs have only `arm_1`.
_DUAL_SLOTS = ('arm_1', 'arm_2')
_SINGLE_SLOTS = ('arm_1',)

# The per-arm fence the console draws and the jog model clamps against. A key
# the validated config does not carry is reported as `None` rather than
# omitted, so every fence entry has the same shape.
FENCE_KEYS = (
    'position_lower',
    'position_upper',
    'max_target_velocity',
    'k_gains',
    'd_gains',
    'max_effort',
)

#: Under the state dir, created 0700 on first use.
PROFILE_DIR_NAME = 'profiles'

#: The reviewed validator's own MAXIMUM_CONFIG_BYTES.
MAXIMUM_PROFILE_BYTES = 65536

_PROFILE_SUFFIX = '.yaml'
_FILE_MODE = 0o600
_DIRECTORY_MODE = 0o700
_FILE_FLAGS = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC
_READ_FLAGS = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW


class ProfileStoreError(Exception):
    """A rendered profile the reviewed validator refused."""

    def __init__(self, detail):
        """Store the closed-set code and the validator's own sentence."""
        super().__init__(detail)
        self.code = 'profile_invalid'
        self.detail = detail


@dataclass(frozen=True)
class StoredProfile:
    """
    One validated controller profile, on disk under its own content hash.

    ``path`` is absolute, normalized and was reached without following a
    single symlink -- it is exactly what ``controller_param_file:=``
    receives. ``fence`` maps arm ID to the six per-arm limit vectors, as a
    read-only mapping of read-only mappings of tuples: the jog model clamps
    against exactly this data, so a consumer must not be able to rewrite it.
    """

    config_sha256: str
    controller_name: str
    controller_type: str
    path: str
    arms: tuple
    command_interfaces: tuple
    state_interfaces: tuple
    fence: dict


def _number(value):
    """Render one float so the file round-trips the exact double."""
    return repr(float(value))


def _vector(values):
    """Render a 7-vector as a YAML flow sequence of exact doubles."""
    return '[' + ', '.join(_number(value) for value in values) + ']'


def _arm_block(slot, profile):
    """Render one ``arm_N:`` block from a MotionProfile."""
    joint_names = ', '.join(
        '{}_joint{}'.format(profile.arm_id, index)
        for index in range(1, defaults.JOINT_COUNT + 1))
    return (
        '    {slot}:\n'
        '      arm_id: {arm_id}\n'
        '      joint_names: [{joint_names}]\n'
        '      k_gains: {k_gains}\n'
        '      d_gains: {d_gains}\n'
        '      max_effort: {max_effort}\n'
        '      position_lower: {position_lower}\n'
        '      position_upper: {position_upper}\n'
        '      max_target_velocity: {max_target_velocity}\n'
    ).format(
        slot=slot,
        arm_id=profile.arm_id,
        joint_names=joint_names,
        k_gains=_vector(profile.k_gains),
        d_gains=_vector(profile.d_gains),
        max_effort=_vector(profile.max_effort_nm),
        position_lower=_vector(profile.position_lower_rad),
        position_upper=_vector(profile.position_upper_rad),
        max_target_velocity=_vector(profile.max_target_velocity_rad_s),
    )


def render_profile_yaml(arm_ids, profiles):
    """
    Render the impedance-controller document for exactly these arms.

    ``arm_ids`` is the session's arm tuple in slot order and ``profiles``
    maps each of them to a ``config.MotionProfile``. The dual layout carries
    ``arm_1``/``arm_2`` and no ``arm_count``; the single layout carries
    ``arm_count: 1`` and one ``arm_1`` block whose ``arm_id`` is the SELECTED
    arm. Numbers are emitted with ``repr(float(...))`` so the file round-trips
    the exact doubles the fence and the jog clamp compare against -- a printed
    and reparsed bound would be a different number from the one the model
    clamps to and the controller's own ``accept`` checks.
    """
    arm_ids = tuple(arm_ids)
    if len(arm_ids) not in (1, 2):
        raise ValueError('a controller profile covers one or two arms')
    slots = _DUAL_SLOTS if len(arm_ids) == 2 else _SINGLE_SLOTS
    timing = profiles[arm_ids[0]]
    lines = [
        '/{}:'.format(defaults.MOTION_CONTROLLER),
        '  ros__parameters:',
    ]
    if len(arm_ids) == 1:
        lines.append('    arm_count: 1')
    lines.extend([
        '    watchdog_timeout: {}'.format(_number(timing.watchdog_timeout_s)),
        '    max_header_age: {}'.format(_number(timing.max_header_age_s)),
        '    future_tolerance: {}'.format(_number(timing.future_tolerance_s)),
    ])
    text = '\n'.join(lines) + '\n'
    for slot, arm_id in zip(slots, arm_ids):
        text += _arm_block(slot, profiles[arm_id])
    return text


def _require_normalized_absolute(path):
    """Return ``path`` without trailing separators, or raise ``ValueError``."""
    if not path:
        raise ValueError('the state directory path must not be empty')
    if '\x00' in path:
        raise ValueError('the state directory path contains an invalid character')
    stripped = path.rstrip(os.sep) or os.sep
    if not os.path.isabs(stripped) or os.path.normpath(stripped) != stripped:
        raise ValueError('the state directory must be a normalized absolute path')
    return stripped


def _open_directory_no_symlinks(path):
    """
    Open ``path`` component by component with ``O_NOFOLLOW`` and return the fd.

    This is ``franka_bringup.recorder._open_directory_no_symlinks`` again:
    because no component is ever resolved THROUGH a symlink, the walk cannot
    be raced by swapping one in.
    """
    descriptor = os.open(os.sep, _DIRECTORY_FLAGS)
    try:
        for component in path.split(os.sep):
            if not component:
                continue
            next_descriptor = os.open(component, _DIRECTORY_FLAGS, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = next_descriptor
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _fence_for_arm(arm):
    """Project one validated arm block onto the six fence keys."""
    return MappingProxyType({
        key: (None if arm.get(key) is None else tuple(float(value) for value in arm[key]))
        for key in FENCE_KEYS
    })


def _fence(text, controller_name, arm_ids):
    """
    Re-parse an already-validated config and project it onto the fence.

    A second strict parse rather than a threaded-through value, because
    ``ValidatedControllerConfig`` deliberately carries interfaces, not limits.
    The text passed the validator moments ago, so this parse cannot fail.
    """
    parameters = load_strict_yaml(text)['/' + controller_name]['ros__parameters']
    slots = _DUAL_SLOTS if len(arm_ids) == 2 else _SINGLE_SLOTS
    return MappingProxyType({
        arm_id: _fence_for_arm(parameters[slot])
        for slot, arm_id in zip(slots, arm_ids)
    })


class ProfileStore:
    """
    Render, validate and content-address the session's controller profile.

    One instance per server. The instance is not internally locked: the HTTP
    layer serializes session starts behind the single operator lock, and the
    on-disk half is race-free on its own -- ``O_EXCL`` picks the winner, and
    both writers would be writing identical bytes to a name derived from
    those very bytes.
    """

    def __init__(self, state_dir, utcnow=None):
        """Bind the store to ``state_dir``; nothing is created until first use."""
        self._state_dir = _require_normalized_absolute(str(state_dir))
        self._profile_dir = os.path.join(self._state_dir, PROFILE_DIR_NAME)
        self._utcnow = utcnow
        self._records = {}

    @property
    def profile_dir(self):
        """Return the absolute directory the store writes its files into."""
        return self._profile_dir

    def materialize(self, arms, profiles):
        """
        Return the :class:`StoredProfile` a Motion session hands to the launch.

        ``arms`` is the session's arm selection (``panda1``/``panda2``/
        ``both``) and ``profiles`` maps every arm it names to a
        ``config.MotionProfile``. Raises :class:`ProfileStoreError` carrying
        the reviewed validator's own sentence when the rendered document is
        refused.
        """
        if arms not in ARM_SELECTIONS:
            raise ProfileStoreError(
                'arms must be one of {}'.format(', '.join(sorted(ARM_SELECTIONS))))
        arm_ids = ARM_SELECTIONS[arms]
        text = render_profile_yaml(arm_ids, profiles)
        encoded = text.encode('utf-8')
        if len(encoded) > MAXIMUM_PROFILE_BYTES:
            raise ProfileStoreError(
                'the rendered controller profile is {} bytes; the limit is '
                '{}'.format(len(encoded), MAXIMUM_PROFILE_BYTES))
        try:
            if arms == 'both':
                validated = validate_controller_config_text(
                    text, defaults.MOTION_CONTROLLER)
            else:
                validated = validate_single_controller_config_text(
                    text, defaults.MOTION_CONTROLLER, arms)
        except ControllerConfigError as error:
            # Verbatim, on purpose: the validator's sentences are
            # deterministic and name the exact key at fault.
            raise ProfileStoreError(str(error)) from error

        # Written unconditionally, even for a repeat: `_write` proves the
        # bytes on disk are still the bytes this hash names, and re-creates
        # the file if something removed it since.
        path = self._write(validated.config_sha256, encoded)
        existing = self._records.get(validated.config_sha256)
        if existing is not None:
            return existing
        record = StoredProfile(
            config_sha256=validated.config_sha256,
            controller_name=validated.controller_name,
            controller_type=validated.controller_type,
            path=path,
            arms=tuple(arm_ids),
            command_interfaces=tuple(validated.command_interfaces),
            state_interfaces=tuple(validated.state_interfaces),
            fence=_fence(text, validated.controller_name, arm_ids),
        )
        self._records[record.config_sha256] = record
        return record

    def verify(self, record):
        """
        Re-prove the on-disk file still hashes to its own name.

        Called at the last web-owned boundary before the launch spawn: start
        acceptance and that spawn are separated by the RT preflight and
        recorder startup, and a same-user edit in that interval must not make
        the console's fence describe A while the launch seals B. Raises
        ``OSError``; store corruption is an operational failure, never an
        operator-correctable refusal.
        """
        directory = self._open_profile_dir()
        try:
            self._verify_existing(
                directory, record.config_sha256 + _PROFILE_SUFFIX,
                record.config_sha256, record.path)
        finally:
            os.close(directory)
        return record

    def _write(self, sha256, data):
        """
        Create ``<profile_dir>/<sha256>.yaml`` holding ``data``; return its path.

        ``O_EXCL`` means an existing name is never truncated: its content is
        read back through ``O_NOFOLLOW`` and re-hashed instead, so a repeat of
        identical bytes succeeds and a squatted, symlinked or corrupted name
        raises ``OSError`` rather than being trusted or overwritten.
        """
        name = sha256 + _PROFILE_SUFFIX
        path = os.path.join(self._profile_dir, name)
        directory = self._open_profile_dir()
        try:
            try:
                descriptor = os.open(name, _FILE_FLAGS, _FILE_MODE, dir_fd=directory)
            except FileExistsError:
                self._verify_existing(directory, name, sha256, path)
                return path
            try:
                # The mode argument to open() is masked by the process umask.
                # 0600 on a file the launch has to read back is a guarantee,
                # not a preference, so it is set again on the descriptor.
                os.fchmod(descriptor, _FILE_MODE)
                offset = 0
                while offset < len(data):
                    offset += os.write(descriptor, data[offset:])
            except BaseException:
                os.close(descriptor)
                # A short write would leave a truncated file under a name
                # that promises its own content hash. Never leave that.
                try:
                    os.unlink(name, dir_fd=directory)
                except OSError:
                    pass
                raise
            os.close(descriptor)
        finally:
            os.close(directory)
        return path

    def _verify_existing(self, directory, name, sha256, path):
        """Raise unless ``name`` already holds exactly the bytes ``sha256`` names."""
        descriptor = os.open(name, _READ_FLAGS, dir_fd=directory)
        try:
            details = os.fstat(descriptor)
            if not stat.S_ISREG(details.st_mode) or details.st_size > MAXIMUM_PROFILE_BYTES:
                raise OSError(
                    errno.EEXIST,
                    'stored profile object is not a bounded regular file', path)
            # Read to EOF rather than trusting one os.read to return
            # everything; a short read would fail the hash check and libel an
            # honest file.
            data = b''
            while len(data) <= MAXIMUM_PROFILE_BYTES:
                chunk = os.read(descriptor, MAXIMUM_PROFILE_BYTES + 1 - len(data))
                if not chunk:
                    break
                data += chunk
        finally:
            os.close(descriptor)
        if hashlib.sha256(data).hexdigest() != sha256:
            raise OSError(errno.EEXIST, 'stored profile object does not match its name', path)

    def _open_profile_dir(self):
        """
        Return an fd for the profiles directory, creating it 0700 on first use.

        Every component of the state directory is opened with ``O_NOFOLLOW``,
        and so is the profiles directory itself -- a symlink anywhere along
        the way is an ``OSError``, not a redirect. A directory that ALREADY
        existed is verified and never repaired: a mode other than 0700 means
        something that is not this server has been here.
        """
        parent = _open_directory_no_symlinks(self._state_dir)
        try:
            try:
                os.mkdir(PROFILE_DIR_NAME, _DIRECTORY_MODE, dir_fd=parent)
            except FileExistsError:
                pass
            else:
                os.chmod(PROFILE_DIR_NAME, _DIRECTORY_MODE, dir_fd=parent)
            descriptor = os.open(PROFILE_DIR_NAME, _DIRECTORY_FLAGS, dir_fd=parent)
        finally:
            os.close(parent)
        try:
            details = os.fstat(descriptor)
            if details.st_uid != os.geteuid():
                raise OSError(
                    errno.EPERM, 'the profiles directory is not owned by this user',
                    self._profile_dir)
            mode = stat.S_IMODE(details.st_mode)
            if mode & 0o077:
                raise OSError(
                    errno.EPERM,
                    'the profiles directory has group or other permission bits',
                    self._profile_dir)
            if mode & _DIRECTORY_MODE != _DIRECTORY_MODE:
                raise OSError(
                    errno.EPERM,
                    'the profiles directory is not owner-readable, writable and '
                    'searchable',
                    self._profile_dir)
        except BaseException:
            os.close(descriptor)
            raise
        return descriptor
