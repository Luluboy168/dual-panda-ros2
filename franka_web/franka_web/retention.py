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
The size cap on stored recordings: one plan, four rules, plain-words lines.

The recorder writes roughly 4 MB every second, so an hour of Watch is about
14 GB and a busy week fills a disk. ``recordings.max_total_gb`` is the one
number that bounds it: when the sealed recordings under the recordings root
add up to more than the cap, the OLDEST sessions are removed until the total
is at or under it. The pass runs twice -- at server startup, before the page
is served, and after each session's bag is sealed -- so the operator sees the
result rather than discovering a full disk.

THE FOUR RULES, all of them load-bearing
----------------------------------------
1. **Never the session recording right now.** The active directory (and every
   other segment of its rollover chain) is counted toward the total and is
   never a removal candidate, however far over the cap it puts the total.
2. **Never an unsealed directory.** A session directory with no
   ``metadata.yaml`` did not finish: either a recording is in flight or a
   session crashed, and a crashed session's bag is EVIDENCE. Unsealed
   directories are counted toward the total, named in the summary line, and
   never removed.
3. **Only this server's own directories, only inside the root.** A name must
   match :data:`SESSION_NAME_PATTERN` exactly -- ``web-YYYYmmdd-HHMMSS`` with
   an optional ``-NNN`` chain suffix -- and the entry must be a real directory
   in the root, not a symlink to one. Everything else in the root (a file, a
   foreign directory, a symlink pointing anywhere at all) is invisible to this
   module: never counted, never followed, never removed.
4. **A directory that will not go is reported, not fatal.** One removal that
   fails is one warn line; the pass carries on with the next candidate.

ORDER IS BY NAME, NOT BY MTIME
------------------------------
Sessions are named for the moment they started, and that name is the only
honest age. An mtime is not: copying a tree, a backup tool, a ``touch``, or
even reading with a mount that updates times can reorder the whole root and
silently make the retention pass delete the newest recordings first. The plan
sorts by the directory's own timestamp NAME, and the fixed-width zero-padded
format makes plain lexicographic order chronological order.

UNITS
-----
One GB is 1 000 000 000 bytes (:data:`franka_web.defaults.BYTES_PER_GB`) --
what disk vendors, ``du --si`` and these log lines all mean by "GB". Sizes are
the apparent sizes of the regular files under a session directory, which is
what ``du -sb`` reports; a bag is dense, so this is within a rounding error of
the space it actually occupies.

LAYERING
--------
:func:`plan` is pure: it reads the filesystem but changes nothing, and returns
what WOULD be removed, what is kept, and the sizes. :func:`run` applies a plan
and emits the lines. Everything here is unit-testable against a temporary
directory with no server, no ROS and no recorder.
"""

from dataclasses import dataclass
import os
import re
import shutil

from franka_web import defaults

#: The server's own session-directory name: ``web-YYYYmmdd-HHMMSS`` with the
#: optional ``-002``.. rollover suffix. Assembled from the same two shapes
#: ``franka_web.recording`` builds names with; ``test_retention`` pins the two
#: against each other so they cannot drift.
SESSION_NAME_PATTERN = re.compile(r'(web-[0-9]{8}-[0-9]{6})(?:-[0-9]{3,})?')

#: The file rosbag2 writes last. Its presence is the definition of "sealed".
METADATA_NAME = 'metadata.yaml'

#: Where the recorder puts the bag inside a session directory. The metadata
#: file is looked for there first and directly under the session directory
#: second, so this module does not depend on one layout staying forever.
BAG_SUBDIRECTORY = 'bag'


@dataclass(frozen=True)
class SessionDirectory:
    """One recording directory in the root: its name, size and seal state."""

    name: str
    path: str
    size_bytes: int
    sealed: bool


@dataclass(frozen=True)
class RetentionPlan:
    """What one retention pass would do, computed without changing anything."""

    root: str
    cap_gb: float
    cap_bytes: int
    entries: tuple
    total_bytes: int
    remove: tuple
    keep: tuple
    unsealed: tuple
    protected: tuple
    remaining_bytes: int
    over_cap: bool

    @property
    def unlimited(self):
        """Return True when no cap is in force and the pass removes nothing."""
        return self.cap_bytes is None


@dataclass(frozen=True)
class RetentionResult:
    """What one retention pass actually did."""

    plan: RetentionPlan
    removed: tuple
    failed: tuple
    remaining_bytes: int
    lines: tuple


# --- units and wording -------------------------------------------------------

#: The smallest figure these lines print as a number. Below it ``{:.3g}``
#: turns into scientific notation -- a session of a few tens of kilobytes
#: logged as ``2e-05 GB`` -- and this drawer is read by a lab owner, not by a
#: log parser. A hundredth of a gigabyte is also the point below which the
#: exact figure tells nobody anything.
SMALLEST_PRINTED_GB = 0.01

#: What the lines say instead of a number below that floor.
BELOW_FLOOR_TEXT = 'less than 0.01'


def format_gb(value_gb):
    """Render a GB figure the way the log lines write it: 50, 48.2, 0.05."""
    value = float(value_gb)
    if value >= 1.0:
        text = '{:.1f}'.format(value)
    elif value >= SMALLEST_PRINTED_GB:
        text = '{:.3g}'.format(value)
    elif value > 0.0:
        # Never 0: a directory that exists is not nothing, and telling the
        # owner a removed session was "0 GB" invites them to doubt the line.
        return BELOW_FLOOR_TEXT
    else:
        return '0'
    return text[:-2] if text.endswith('.0') else text


def format_bytes(size_bytes):
    """Render a byte count as its GB figure."""
    return format_gb(float(size_bytes) / defaults.BYTES_PER_GB)


def removal_line(entry, remaining_bytes, cap_gb):
    """Return the one plain-words line a single removal writes."""
    return ('retention: removed {} ({} GB); recordings now {} of {} GB'.format(
        entry.name, format_bytes(entry.size_bytes),
        format_bytes(remaining_bytes), format_gb(cap_gb)))


def failure_line(entry, error):
    """Return the one line a removal that could not be done writes."""
    reason = getattr(error, 'strerror', None) or str(error)
    return ('retention: could not remove {}: {}; keeping it'.format(
        entry.name, reason))


def summary_line(plan, removed=(), failed=()):
    """Return the single line that ends every pass."""
    kept = len(plan.entries) - len(removed)
    remaining = plan.total_bytes - sum(entry.size_bytes for entry in removed)
    if plan.unlimited:
        return ('retention: no size cap is set '
                '(recordings.max_total_gb: {}); {} sessions hold {} GB'.format(
                    defaults.RECORDING_RETENTION_UNLIMITED, kept,
                    format_bytes(remaining)))
    clauses = ['retention: {} sessions hold {} of {} GB'.format(
        kept, format_bytes(remaining), format_gb(plan.cap_gb))]
    clauses.append('removed {}'.format(len(removed)) if removed
                   else 'nothing to remove')
    if plan.unsealed:
        clauses.append('{} unsealed {} kept: {}'.format(
            len(plan.unsealed),
            'session' if len(plan.unsealed) == 1 else 'sessions',
            ', '.join(entry.name for entry in plan.unsealed)))
    if failed:
        clauses.append('{} could not be removed'.format(len(failed)))
    if remaining > plan.cap_bytes:
        clauses.append('still above the cap, and nothing else may be removed')
    return '; '.join(clauses)


# --- reading the root --------------------------------------------------------

def is_session_name(name):
    """Return True when ``name`` is one this server's recorder would write."""
    return bool(SESSION_NAME_PATTERN.fullmatch(name))


def chain_names(active_name):
    """
    Return a predicate matching every directory the active chain owns.

    A session longer than an hour is a CHAIN -- ``web-X``, ``web-X-002``,
    ``web-X-003`` -- and the recorder is writing only the last of them, but
    the earlier segments are that same live session's data. All of them are
    protected while it runs.
    """
    matched = (SESSION_NAME_PATTERN.fullmatch(active_name)
               if active_name else None)
    if matched is None:
        return frozenset()
    # Group 1 is the timestamp half, so the suffix is stripped only when the
    # name really carries one. A plain `web-20260903-105230` keeps its own
    # `-105230`: reading that as a chain suffix would protect -- and so
    # exempt from the cap for ever -- every session recorded that same day.
    return frozenset((active_name, matched.group(1)))


def _protects(protected_bases, name):
    """Return True when ``name`` belongs to a protected recording chain."""
    return any(name == base or name.startswith(base + '-')
               for base in protected_bases)


def directory_size(path):
    """
    Return the total apparent size of the regular files under ``path``.

    Symlinks are never followed, in the walk or in the measurement: a link
    contributes its own (tiny) size and nothing of its target, so a link into
    a 200 GB archive can neither inflate the total nor lead the walk out of
    the recordings root.
    """
    total = 0
    for directory, subdirectories, names in os.walk(path, followlinks=False):
        subdirectories[:] = [name for name in subdirectories
                             if not os.path.islink(os.path.join(directory, name))]
        for name in names:
            try:
                status = os.lstat(os.path.join(directory, name))
            except OSError:
                continue
            total += status.st_size
    return total


def is_sealed(path):
    """Return True when the session directory holds rosbag2's metadata file."""
    candidates = (os.path.join(path, BAG_SUBDIRECTORY, METADATA_NAME),
                  os.path.join(path, METADATA_NAME))
    return any(os.path.isfile(candidate) for candidate in candidates)


def scan(root):
    """
    Return every session directory in ``root``, oldest first by NAME.

    Rule 3 lives here: an entry is a session directory only when its name
    matches the recorder's own pattern exactly AND it is a real directory
    rather than a symlink to one. A missing or unreadable root is an empty
    list, not an exception.
    """
    try:
        names = sorted(os.listdir(root))
    except OSError:
        return []
    found = []
    for name in names:
        if not is_session_name(name):
            continue
        path = os.path.join(root, name)
        if os.path.islink(path) or not os.path.isdir(path):
            continue
        found.append(SessionDirectory(name=name, path=path,
                                      size_bytes=directory_size(path),
                                      sealed=is_sealed(path)))
    return found


# --- the plan ----------------------------------------------------------------

def plan(root, max_total_gb, active_name=None, entries=None):
    """
    Return the :class:`RetentionPlan` for ``root`` at ``max_total_gb``.

    ``max_total_gb`` is a positive number of GB, or ``None`` for the
    documented ``unlimited`` spelling, which plans no removal at all.
    ``active_name`` is the directory the recorder is writing right now, or
    ``None``. ``entries`` lets a caller supply an already-scanned list; it is
    for tests and for a caller that wants the sizes measured once.

    Nothing on disk is changed. The returned plan says what WOULD go, in the
    order it would go.
    """
    if entries is None:
        entries = scan(root)
    entries = tuple(sorted(entries, key=lambda entry: entry.name))
    total = sum(entry.size_bytes for entry in entries)
    unsealed = tuple(entry for entry in entries if not entry.sealed)
    protected_bases = chain_names(active_name)
    protected = tuple(entry for entry in entries
                      if _protects(protected_bases, entry.name))
    cap_bytes = (None if max_total_gb is None
                 else int(float(max_total_gb) * defaults.BYTES_PER_GB))

    remove = []
    remaining = total
    if cap_bytes is not None:
        for entry in entries:
            if remaining <= cap_bytes:
                break
            if not entry.sealed:
                continue
            if _protects(protected_bases, entry.name):
                continue
            remove.append(entry)
            remaining -= entry.size_bytes
    removed_names = {entry.name for entry in remove}
    return RetentionPlan(
        root=root,
        cap_gb=(None if max_total_gb is None else float(max_total_gb)),
        cap_bytes=cap_bytes,
        entries=entries,
        total_bytes=total,
        remove=tuple(remove),
        keep=tuple(entry for entry in entries if entry.name not in removed_names),
        unsealed=unsealed,
        protected=protected,
        remaining_bytes=remaining,
        over_cap=cap_bytes is not None and remaining > cap_bytes,
    )


# --- applying it -------------------------------------------------------------

def _removable(root, entry):
    """Re-check rule 3 at the moment of removal, against the live filesystem."""
    if not is_session_name(entry.name):
        return False
    path = os.path.join(root, entry.name)
    if path != entry.path:
        return False
    return not os.path.islink(path) and os.path.isdir(path)


def run(root, max_total_gb, active_name=None, *, emit=None, remove=None):
    """
    Plan one pass, apply it, and emit one line per removal plus a summary.

    ``emit`` is ``(level, message)`` -- normally ``LogBus.emit``; ``remove``
    is the deletion callable, ``shutil.rmtree`` by default, injectable so a
    test can prove the plan without deleting anything. Nothing here raises:
    a directory that will not go is one warn line and the pass continues.
    """
    remover = remove or shutil.rmtree
    lines = []

    def _say(level, message):
        lines.append((level, message))
        if emit is not None:
            emit(level, message)

    computed = plan(root, max_total_gb, active_name)
    removed = []
    failed = []
    remaining = computed.total_bytes
    for entry in computed.remove:
        if not _removable(root, entry):
            failed.append(entry)
            _say('warn', failure_line(entry, 'it is no longer a session '
                                             'directory in the recordings root'))
            continue
        try:
            remover(entry.path)
        except OSError as error:
            failed.append(entry)
            _say('warn', failure_line(entry, error))
            continue
        removed.append(entry)
        remaining -= entry.size_bytes
        _say('info', removal_line(entry, remaining, computed.cap_gb))
    _say('info', summary_line(computed, tuple(removed), tuple(failed)))
    return RetentionResult(plan=computed, removed=tuple(removed),
                           failed=tuple(failed), remaining_bytes=remaining,
                           lines=tuple(lines))
