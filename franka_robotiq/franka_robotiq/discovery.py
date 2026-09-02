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
Serial binding: which adapter drives which arm, and how that is proved.

Two identical USB-RS485 adapters in one cell are assigned device nodes in
probe order, and that order changes across reboots and replugs. With two arms
that is not a nuisance, it is a wrong-arm command. So:

    A gripper is identified by its adapter's stable device name, and by
    nothing else. This module opens exactly the configured name. It never
    scans, never picks "the only one present", and never falls back to
    /dev/ttyUSB* enumeration order.

There is no fallback path in this file for a reason: a fallback is the failure
mode. If the configured name is absent, resolution fails with a message that
lists what *is* present, so the operator's next action is a copy-paste, and
the adapter that was found is never opened.

The module also owns every sentence about binding, so the node and the web
configuration loader print the same words rather than each paraphrasing the
rule.

Imports: ``os`` and ``errno`` from the standard library. Nothing else.
"""

import errno
import os

DEFAULT_BY_ID_ROOT = '/dev/serial/by-id'
DEFAULT_BY_PATH_ROOT = '/dev/serial/by-path'

#: How far up the sysfs device tree to look for the USB device node that
#: carries the adapter's serial. A fixed '../../serial' is right for one FTDI
#: topology and wrong for the next hub arrangement.
_SYSFS_WALK_LIMIT = 6

_DOC_POINTER = 'See franka_robotiq/doc/SERIAL_BINDING.md.'

#: The same pointer where the contract's printed sentence continues rather than
#: restarts -- section 4.3 rule 2 reads '...its own adapter; see
#: franka_robotiq/doc/SERIAL_BINDING.md.', with a lowercase s after the
#: semicolon. doc/SERIAL_BINDING.md quotes that sentence word for word, so the
#: capital form would put a mid-sentence capital in an operator's log line and
#: break the quote at the same time.
_DOC_POINTER_CONTINUED = _DOC_POINTER[0].lower() + _DOC_POINTER[1:]


class BindingError(Exception):
    """
    A serial binding that cannot be honoured. The message is the teaching text.

    The base class is ``Exception`` and not ``ValueError`` on purpose: a
    ``ValueError`` would be swallowed by a generic input-validation handler in
    the configuration loader, and a binding refusal must reach the operator.
    """


def _check_basename(key, name):
    """Reject anything that is not a bare directory entry name."""
    if not isinstance(name, str) or not name:
        raise BindingError(
            '{} is empty. Give it the name of an entry under '
            '{}. {}'.format(key, DEFAULT_BY_ID_ROOT, _DOC_POINTER))
    if name.startswith('/'):
        raise BindingError(
            '{} is {}, an absolute path. Give just the entry name, not a '
            'path. {}'.format(key, name, _DOC_POINTER))
    if '/' in name:
        raise BindingError(
            '{} is {}, which contains a path separator. Give just the entry '
            'name. {}'.format(key, name, _DOC_POINTER))
    if '*' in name or '?' in name:
        raise BindingError(
            '{} is {}, which looks like a pattern. This driver opens exactly '
            'one named adapter and never matches patterns, because matching '
            "is how one arm ends up driving the other arm's gripper. "
            '{}'.format(key, name, _DOC_POINTER))
    if name in ('.', '..'):
        raise BindingError(
            '{} is {!r}, which names a directory, not an adapter. '
            '{}'.format(key, name, _DOC_POINTER))
    return name


def _resolve_in(key, name, root):
    """Resolve one basename under ``root`` to a device path."""
    _check_basename(key, name)
    path = os.path.join(root, name)
    if not os.path.islink(path):
        # No arm id is available here, so this message names the path and
        # lists what is present without inventing an arm. The arm-prefixed
        # form an operator reads is missing_adapter_message(), composed by
        # the caller, which knows which arm it is starting.
        raise BindingError('\n'.join(
            ['no adapter at {}'.format(path)]
            + _adapters_present_lines(root) + [_DOC_POINTER]))
    target = os.path.realpath(path)
    if not target.startswith('/dev/'):
        raise BindingError(
            '{} points at {}, which is not a device under /dev. Refusing to '
            'open it. {}'.format(path, target, _DOC_POINTER))
    return target


def resolve(serial_id, *, root=DEFAULT_BY_ID_ROOT):
    """
    Resolve one by-id entry name to the device path it points at.

    Opens exactly the configured name. It never scans a directory for
    candidates, never picks the only adapter present, and never falls back to
    a probe-order device node under /dev.
    """
    return _resolve_in('serial_id', serial_id, root)


def resolve_by_path(usb_path, *, root=DEFAULT_BY_PATH_ROOT):
    """
    Resolve one by-path entry name to the device path it points at.

    The by-path fallback binds to the physical USB port rather than to the
    adapter, for adapters whose USB descriptor carries no unique serial. It
    survives replacing the adapter and does not survive re-cabling; by-id is
    the other way round. The configuration says which; this module never
    silently chooses.
    """
    return _resolve_in('usb_path', usb_path, root)


def list_adapters(root=DEFAULT_BY_ID_ROOT):
    """Return sorted entry names present under ``root``; empty when absent."""
    try:
        return sorted(os.listdir(root))
    except OSError as exc:
        if exc.errno in (errno.ENOENT, errno.ENOTDIR, errno.EACCES):
            return []
        raise


def _adapters_present_lines(root):
    """Build the "what is there instead" half of an absent-adapter message."""
    present = list_adapters(root)
    if present:
        return ['Adapters present now:'] + ['  ' + item for item in present]
    return ['No USB serial adapters are present at all.']


def missing_adapter_message(arm, name, root=DEFAULT_BY_ID_ROOT,
                            key='serial_id'):
    """
    Compose the refusal for a configured adapter that is not there.

    It lists what IS present so the next action is a copy-paste, and it never
    offers to open the adapter it found.
    """
    lines = ['{}: no gripper at {}'.format(arm, os.path.join(root, name))]
    lines.extend(_adapters_present_lines(root))
    lines.append('If the gripper was replaced, put the new name in '
                 'grippers.{}.{}.'.format(arm, key))
    lines.append(_DOC_POINTER)
    return '\n'.join(lines)


def check_binding(arm, serial_id, usb_path):
    """
    Exactly one of serial_id / usb_path when the gripper is enabled.

    Raises ``BindingError`` when neither is set, and when both are.
    """
    has_serial = bool(serial_id)
    has_path = bool(usb_path)
    if not has_serial and not has_path:
        raise BindingError(
            'grippers.{arm} is enabled but neither grippers.{arm}.serial_id '
            'nor grippers.{arm}.usb_path is set. One of them must name the '
            "adapter that drives this arm's gripper. {doc}".format(
                arm=arm, doc=_DOC_POINTER))
    if has_serial and has_path:
        raise BindingError(
            'grippers.{arm} sets both serial_id and usb_path. They are two '
            'ways of naming the same adapter and only one may be used, so '
            'that there is one answer to which device this arm opens. '
            '{doc}'.format(arm=arm, doc=_DOC_POINTER))


def _configured_name(binding):
    """Return the name half of one ``(serial_id, usb_path)`` pair, or ''."""
    serial_id, usb_path = binding
    return serial_id or usb_path or ''


def check_cross_arm(bindings, *, resolver=None):
    """
    Two arms must not name -- or resolve to -- the same adapter.

    ``bindings`` is a mapping ``{arm_id: (serial_id, usb_path)}`` with ``''``
    for the unset one of each pair. ``resolver``, when given, is called as
    ``resolver(name) -> realpath`` and may raise; a raise means "absent",
    which is not a collision. Raises ``BindingError`` on a collision.
    """
    if not hasattr(bindings, 'items'):
        raise BindingError(
            'check_cross_arm takes a mapping of arm id to (serial_id, '
            'usb_path); got {}'.format(type(bindings).__name__))
    named = {}
    for arm, binding in sorted(bindings.items()):
        name = _configured_name(binding)
        if not name:
            continue
        if name in named:
            raise BindingError(
                'grippers.{first}.serial_id and grippers.{second}.serial_id '
                'are the same adapter ({name}). One adapter cannot drive two '
                'grippers. Give each arm the serial of its own adapter; '
                '{doc}'.format(first=named[name], second=arm, name=name,
                               doc=_DOC_POINTER_CONTINUED))
        named[name] = arm
    if resolver is None:
        return
    resolved = {}
    for name, arm in sorted(named.items(), key=lambda item: item[1]):
        try:
            target = resolver(name)
        except Exception:
            # An adapter that is not plugged in is absent, not a collision.
            continue
        if not target:
            continue
        if target in resolved:
            first_arm, first_name = resolved[target]
            raise BindingError(
                'grippers.{first}.serial_id ({first_name}) and '
                'grippers.{second}.serial_id ({second_name}) are the same '
                'adapter (both resolve to {target}). One adapter cannot drive '
                'two grippers. Give each arm the serial of its own adapter; '
                '{doc}'.format(first=first_arm, first_name=first_name,
                               second=arm, second_name=name, target=target,
                               doc=_DOC_POINTER_CONTINUED))
        resolved[target] = (arm, name)


def adapter_serial_from_sysfs(device_path, *, sysfs_root='/sys'):
    """
    Best-effort USB serial for a /dev/ttyUSB* node; None when unreadable.

    Walks up from /sys/class/tty/<name>/device and returns the contents of the
    first directory that holds BOTH ``idVendor`` and ``serial``. A fixed
    ``../../serial`` is correct for one FTDI topology and wrong for the next
    hub arrangement, so the walk looks for the pair rather than counting
    levels.

    Returns ``None`` -- never a guess and never an exception -- when the tree
    is not there at all, which is the ordinary case for a by-path binding, for
    an adapter whose descriptor carries no serial, and for a pty.
    """
    if not device_path:
        return None
    name = os.path.basename(device_path)
    node = os.path.join(sysfs_root, 'class', 'tty', name, 'device')
    try:
        current = os.path.realpath(node)
    except OSError:
        return None
    if not os.path.isdir(current):
        return None
    for _ in range(_SYSFS_WALK_LIMIT):
        vendor = os.path.join(current, 'idVendor')
        serial = os.path.join(current, 'serial')
        if os.path.exists(vendor) and os.path.exists(serial):
            try:
                with open(serial, 'r') as handle:
                    value = handle.read().strip()
            except OSError:
                return None
            return value or None
        parent = os.path.dirname(current)
        if parent == current:
            return None
        current = parent
    return None


def identity_matches(by_id_name, reported_serial):
    """
    Return True when the adapter's serial appears in the by-id name.

    Substring containment, not a parsed token. udev builds by-id names as
    vendor, model and serial joined by underscores, and a model string that
    itself contains an underscore makes "the token after the last underscore"
    the wrong answer. A false negative here would refuse a correctly bound
    gripper -- a failure in the dangerous direction, because the operator's
    fix would be to delete the check -- so the test is deliberately
    permissive and the parse below is used only to word the refusal.
    """
    if not by_id_name or not reported_serial:
        return False
    return reported_serial in by_id_name


def serial_hint_from_by_id(by_id_name):
    """
    Best guess at the serial inside a by-id name, for messages only.

    Never used to decide anything: :func:`identity_matches` does that by
    containment.
    """
    if not by_id_name:
        return ''
    stem = by_id_name.split('-if')[0]
    parts = stem.split('_')
    return parts[-1] if parts else ''


def identity_mismatch_message(arm, by_id_name, reported_serial, other_arm):
    """
    Compose the refusal for an adapter whose serial is not the bound one.

    ``other_arm`` is the id of the arm the reported serial is believed to
    belong to. Pass ``''`` when that is not known; the sentence is then worded
    without naming another arm, and it still refuses and still ends "Nothing
    was commanded."
    """
    expected = serial_hint_from_by_id(by_id_name) or by_id_name
    if other_arm:
        refusal = ("Refusing to drive {arm}'s gripper through {other}'s "
                   'adapter.'.format(arm=arm, other=other_arm))
    else:
        refusal = ("Refusing to drive {arm}'s gripper through another arm's "
                   'adapter.'.format(arm=arm))
    return ('{arm}: the adapter at that path reports serial {reported}, but '
            '{arm} is bound to {expected}. {refusal} Nothing was '
            'commanded.'.format(arm=arm, reported=reported_serial,
                                expected=expected, refusal=refusal))


def identity_unavailable_message(arm):
    """Say in one line that the check could not run, and why that is not a pass."""
    return ('{arm}: the adapter does not report a USB serial, so the identity '
            'of this gripper could not be re-verified. The binding rests on '
            'the physical port alone. {doc}'.format(arm=arm, doc=_DOC_POINTER))
