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
Upload, validate and content-address the operator's controller YAML.

This is the whole of plan section 5.3. A ``POST /api/gains`` body arrives as raw
bytes; what leaves is a :class:`StoredGains` record naming an absolute,
normalized, symlink-free file that ``POST /api/session/start`` hands to
``controller_param_file:=``.

The web validation is a PREVIEW, never the authority
----------------------------------------------------
``franka_bringup.operator_launch._read_regular_validated_config`` re-reads and
re-validates the very same path at launch time and seals the bytes into a
``memfd``. This module exists so a bad config fails in the operator's browser in
milliseconds instead of thirty seconds later inside a launch, and so the failure
carries the validator's own deterministic, user-correctable sentence rather than
a paraphrase of it (plan section 6.5: ``detail`` is the validator's message,
verbatim).

The validator is IMPORTED, never shelled out to (plan section 0.7, hazard 3)
---------------------------------------------------------------------------
``franka_validate_controller_config``'s ``main()`` takes only
``--controller-name``. It has no ``--arm-id``, so it routes every file through
the two-arm ``validate_controller_config_text`` and CANNOT validate the one-arm
(``arm_count: 1``, ``arm_1`` only) layout at all -- a one-arm config handed to
the CLI is rejected for missing ``arm_count`` and carrying no ``arm_2``. A
single session's gains therefore have to go through
``validate_single_controller_config_text`` in-process, and that is what
:meth:`GainsStore.upload` does.

Content addressing, and why the store is idempotent
---------------------------------------------------
The file name is the SHA-256 of the uploaded text, so the same bytes always land
on the same path and re-uploading them returns the record that already exists.
Creation goes through ``O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC`` relative
to a directory file descriptor opened with ``O_NOFOLLOW`` at every component: no
symlink is ever followed, and an existing name is never truncated.

What raises what
----------------
:class:`GainsError` is for failures the OPERATOR can fix -- their bytes were too
big, not UTF-8, not a reviewed controller, not a legal arm selection, or not a
valid config. Anything wrong with the STORE ITSELF (the gains directory replaced
by a symlink, a foreign file squatting a content-addressed name, a permission
mode nobody sane set) raises ``OSError`` instead and is deliberately NOT dressed
up as a 400: the operator's YAML was fine, the filesystem is not, and the honest
answer to that is a 500 -- so the failure is loud and nobody edits their YAML in
response to an attack on the state directory.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
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
from franka_web import config

# The three legal values of the `arms` query parameter (plan section 6.5),
# mapped to the arm IDs they name, in slot order. `both` is the two-arm layout;
# `panda1`/`panda2` are the one-arm layout, where the config's single `arm_1`
# block carries the CHOSEN arm's ID rather than always panda1 (section 0.6).
ARM_SELECTIONS = MappingProxyType({
    'panda1': ('panda1',),
    'panda2': ('panda2',),
    'both': ('panda1', 'panda2'),
})

# Dual configs address their arms by slot; single configs have only `arm_1`.
_DUAL_SLOTS = ('arm_1', 'arm_2')
_SINGLE_SLOTS = ('arm_1',)

# The per-arm fence the Motion card draws and the jog model clamps against
# (plan section 6.5). A key the validated config does not carry is reported as
# `None` rather than omitted, so every fence entry has the same shape whichever
# controller produced it: `dual_arm_joint_hold_controller` configures no
# position or target-velocity limits at all, so its first three come back null.
FENCE_KEYS = (
    'position_lower',
    'position_upper',
    'max_target_velocity',
    'k_gains',
    'd_gains',
    'max_effort',
)

_GAINS_SUFFIX = '.yaml'
_FILE_MODE = 0o600
_DIRECTORY_MODE = 0o700
_FILE_FLAGS = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC
_READ_FLAGS = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW


class GainsError(Exception):
    """A refused gains upload or lookup, carrying a stable section 6.14 code."""

    def __init__(self, code, detail):
        """Store the closed-set ``code`` and the operator-facing ``detail``."""
        super().__init__(detail)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class StoredGains:
    """
    One validated controller config, on disk under its own content hash.

    ``path`` is absolute, normalized and was reached without following a single
    symlink -- it is exactly what ``controller_param_file:=`` receives.

    ``fence`` maps arm ID to the six per-arm limit vectors, with ``None`` for
    the ones the controller does not configure. It is a read-only mapping of
    read-only mappings of tuples: a frozen record whose interior a consumer
    could rewrite would not be frozen at all, and the jog model clamps against
    exactly this data. :meth:`response` renders the wire copy as plain lists.
    """

    config_sha256: str
    controller_name: str
    controller_type: str
    path: str
    uploaded_at: str
    arms: tuple
    command_interfaces: tuple
    state_interfaces: tuple
    fence: dict

    def summary(self):
        """Return the section 6.6 list entry for ``GET /api/gains``."""
        return {
            'config_sha256': self.config_sha256,
            'controller_name': self.controller_name,
            'arms': list(self.arms),
            'uploaded_at': self.uploaded_at,
            'path': self.path,
        }

    def response(self):
        """Return the section 6.5 ``POST /api/gains`` body, minus ``ok``."""
        body = self.summary()
        body.update({
            'controller_type': self.controller_type,
            'command_interfaces': list(self.command_interfaces),
            'state_interfaces': list(self.state_interfaces),
            'fence': {
                arm_id: {
                    key: (None if values is None else list(values))
                    for key, values in limits.items()
                }
                for arm_id, limits in self.fence.items()
            },
        })
        return body


def _rfc3339(moment):
    """Format an aware UTC datetime as RFC 3339 with microseconds and a Z."""
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'


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

    This is ``franka_bringup.recorder._open_directory_no_symlinks`` (and
    ``config._walk_directory_no_symlinks``) again: because no component is ever
    resolved THROUGH a symlink, the walk cannot be raced by swapping one in.
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
    """Project one validated arm block onto the six section 6.5 fence keys."""
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


class GainsStore:
    """
    The store behind ``POST /api/gains`` and ``GET /api/gains``.

    One instance per server. Every record it has accepted is held in memory in
    upload order (:meth:`entries` reports them newest first) and on disk at
    ``<state_dir>/<config.GAINS_DIR_NAME>/<config_sha256>.yaml``.

    The instance is not internally locked: the HTTP layer serializes mutating
    requests behind the single operator lock, and the on-disk half is race-free
    on its own -- ``O_EXCL`` picks the winner, and both writers would be writing
    identical bytes to a name derived from those very bytes.
    """

    def __init__(self, state_dir, utcnow=None):
        """
        Bind the store to ``state_dir``; nothing is created until first upload.

        ``utcnow`` is an injectable callable returning an aware UTC datetime, so
        ``uploaded_at`` is deterministic under test.
        """
        self._state_dir = _require_normalized_absolute(str(state_dir))
        self._gains_dir = os.path.join(self._state_dir, config.GAINS_DIR_NAME)
        self._utcnow = utcnow or (lambda: datetime.now(timezone.utc))
        self._records = {}

    @property
    def gains_dir(self):
        """Return the absolute directory the store writes its files into."""
        return self._gains_dir

    def upload(self, raw, controller_name, arms):
        """
        Validate ``raw`` for ``controller_name``/``arms``, store it, describe it.

        The order of the four cheap refusals is fixed: size first (so an
        oversized body is never decoded, hashed or parsed), then UTF-8, then the
        web controller allowlist, then the arm selection. Only then does the text
        reach the validator, whose message becomes ``detail`` verbatim.
        """
        if not isinstance(raw, (bytes, bytearray, memoryview)):
            raise TypeError(
                'a gains body is raw bytes, not {}'.format(type(raw).__name__))
        if len(raw) > config.MAX_GAINS_BYTES:
            raise GainsError(
                'gains_too_large',
                'configuration is {} bytes; the limit is {}'.format(
                    len(raw), config.MAX_GAINS_BYTES))
        data = bytes(raw)
        try:
            text = data.decode('utf-8')
        except UnicodeDecodeError:
            raise GainsError('gains_invalid', 'configuration is not valid UTF-8') from None
        if controller_name not in config.WEB_CONTROLLERS:
            raise GainsError(
                'controller_not_reviewed',
                '{!r} is not one of the controllers this interface offers'.format(
                    controller_name))
        if arms not in ARM_SELECTIONS:
            raise GainsError(
                'invalid_arms',
                'arms must be one of {}'.format(', '.join(sorted(ARM_SELECTIONS))))

        arm_ids = ARM_SELECTIONS[arms]
        try:
            if arms == 'both':
                validated = validate_controller_config_text(text, controller_name)
            else:
                validated = validate_single_controller_config_text(text, controller_name, arms)
        except ControllerConfigError as error:
            # Verbatim, on purpose (plan section 6.5): the validator's sentences
            # are deterministic and name the exact key the operator must fix.
            raise GainsError('gains_invalid', str(error)) from error

        # Written unconditionally, even for a re-upload: `_write` proves the
        # bytes on disk are still the bytes this hash names, and re-creates the
        # file if something removed it since.
        path = self._write(validated.config_sha256, text.encode('utf-8'))
        existing = self._records.get(validated.config_sha256)
        if existing is not None:
            return existing

        record = StoredGains(
            config_sha256=validated.config_sha256,
            controller_name=validated.controller_name,
            controller_type=validated.controller_type,
            path=path,
            uploaded_at=_rfc3339(self._utcnow()),
            arms=tuple(arm_ids),
            command_interfaces=tuple(validated.command_interfaces),
            state_interfaces=tuple(validated.state_interfaces),
            fence=_fence(text, validated.controller_name, arm_ids),
        )
        self._records[record.config_sha256] = record
        return record

    def get(self, sha256):
        """Return the record for ``sha256``, or ``None`` if it is unknown."""
        if not isinstance(sha256, str):
            return None
        return self._records.get(sha256)

    def entries(self):
        """Return every record's section 6.6 summary, newest upload first."""
        return [record.summary() for record in reversed(list(self._records.values()))]

    def match(self, sha256, controller_name, arms):
        """
        Return the record ``sha256`` names, or refuse the session start.

        The three refusals stay distinct on the wire (plan section 6.7) because
        they are distinct operator mistakes: an unknown hash is a stale page, a
        controller mismatch is the wrong file, and an arm mismatch is a two-arm
        config aimed at a one-arm session (or the reverse) -- which would be
        caught again by ``operator_launch``, but only after the robots came up.
        """
        if arms not in ARM_SELECTIONS:
            raise GainsError(
                'invalid_arms',
                'arms must be one of {}'.format(', '.join(sorted(ARM_SELECTIONS))))
        record = self.get(sha256)
        if record is None:
            raise GainsError('gains_unknown', 'no uploaded configuration has that sha256')
        if record.controller_name != controller_name:
            raise GainsError(
                'gains_controller_mismatch',
                'that configuration was validated for {}, not {}'.format(
                    record.controller_name, controller_name))
        if record.arms != ARM_SELECTIONS[arms]:
            raise GainsError(
                'gains_arms_mismatch',
                'that configuration was validated for {}, not {}'.format(
                    '+'.join(record.arms), '+'.join(ARM_SELECTIONS[arms])))
        # The in-memory record describes the bytes accepted at upload time;
        # it is not evidence that the owner-writable content-addressed object
        # still contains those bytes.  Re-open through the same symlink-free
        # directory walk used by upload and prove the filename's hash
        # immediately before the record can authorize Watch or Motion.  Store
        # corruption is an operational failure (OSError/HTTP 500), never an
        # operator-correctable mismatch dressed up as a 4xx.
        directory = self._open_gains_dir()
        try:
            self._verify_existing(
                directory, sha256 + _GAINS_SUFFIX, sha256, record.path)
        finally:
            os.close(directory)
        return record

    def _write(self, sha256, data):
        """
        Create ``<gains_dir>/<sha256>.yaml`` holding ``data``; return its path.

        ``O_EXCL`` means an existing name is never truncated: its content is read
        back through ``O_NOFOLLOW`` and re-hashed instead, so a re-upload of
        identical bytes succeeds and a squatted, symlinked or corrupted name
        raises ``OSError`` rather than being trusted or overwritten.
        """
        name = sha256 + _GAINS_SUFFIX
        path = os.path.join(self._gains_dir, name)
        directory = self._open_gains_dir()
        try:
            try:
                descriptor = os.open(name, _FILE_FLAGS, _FILE_MODE, dir_fd=directory)
            except FileExistsError:
                self._verify_existing(directory, name, sha256, path)
                return path
            try:
                # The mode argument to open() is masked by the process umask.
                # 0600 on a file the launch has to read back is a guarantee, not
                # a preference, so it is set again on the descriptor we hold.
                os.fchmod(descriptor, _FILE_MODE)
                offset = 0
                while offset < len(data):
                    offset += os.write(descriptor, data[offset:])
            except BaseException:
                os.close(descriptor)
                # A short write would leave a truncated file under a name that
                # promises its own content hash. Never leave that behind.
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
            if not stat.S_ISREG(details.st_mode) or details.st_size > config.MAX_GAINS_BYTES:
                raise OSError(
                    errno.EEXIST, 'stored gains object is not a bounded regular file', path)
            # Read to EOF rather than trusting one os.read to return everything;
            # a short read would fail the hash check and libel an honest file.
            data = b''
            while len(data) <= config.MAX_GAINS_BYTES:
                chunk = os.read(descriptor, config.MAX_GAINS_BYTES + 1 - len(data))
                if not chunk:
                    break
                data += chunk
        finally:
            os.close(descriptor)
        if hashlib.sha256(data).hexdigest() != sha256:
            raise OSError(errno.EEXIST, 'stored gains object does not match its name', path)

    def _open_gains_dir(self):
        """
        Return an fd for the gains directory, creating it 0700 on first use.

        Every component of the state directory is opened with ``O_NOFOLLOW``, and
        so is the gains directory itself -- a symlink anywhere along the way is
        an ``OSError``, not a redirect. A directory that ALREADY existed is
        verified and never repaired: a mode other than 0700 means something that
        is not this server has been here, and the store refuses outright.

        The mode passed to ``mkdir`` is masked by the process umask, so a
        directory this call just created is chmod-ed to 0700 explicitly. That
        ``chmod`` resolves a name rather than a descriptor, but only ever one
        this call created by ``mkdir`` one syscall earlier inside a 0700
        state directory -- swapping a symlink in there needs write access this
        user alone has.
        """
        parent = _open_directory_no_symlinks(self._state_dir)
        try:
            try:
                os.mkdir(config.GAINS_DIR_NAME, _DIRECTORY_MODE, dir_fd=parent)
            except FileExistsError:
                pass
            else:
                os.chmod(config.GAINS_DIR_NAME, _DIRECTORY_MODE, dir_fd=parent)
            descriptor = os.open(config.GAINS_DIR_NAME, _DIRECTORY_FLAGS, dir_fd=parent)
        finally:
            os.close(parent)
        try:
            details = os.fstat(descriptor)
            if details.st_uid != os.geteuid():
                raise OSError(
                    errno.EPERM, 'the gains directory is not owned by this user', self._gains_dir)
            mode = stat.S_IMODE(details.st_mode)
            if mode & 0o077:
                raise OSError(
                    errno.EPERM, 'the gains directory has group or other permission bits',
                    self._gains_dir)
            if mode & _DIRECTORY_MODE != _DIRECTORY_MODE:
                raise OSError(
                    errno.EPERM,
                    'the gains directory is not owner-readable, writable and searchable',
                    self._gains_dir)
        except BaseException:
            os.close(descriptor)
            raise
        return descriptor
