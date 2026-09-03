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
The recordings size cap: the four rules, the order, and the lines it writes.

Every case here builds a real temporary recordings root out of SPARSE files,
so a "13 GB" session costs no disk and no time: ``truncate`` sets the apparent
size the retention pass measures, which is the same number ``du -sb`` reports.

The four rules each get their own class, because each one is the difference
between a retention pass and a data-loss bug:

* the ACTIVE session is never removed (:class:`TestTheActiveSessionIsNeverRemoved`);
* an UNSEALED directory is never removed (:class:`TestUnsealedDirectoriesSurvive`);
* only this server's own directories, inside the root, are ever touched
  (:class:`TestNothingElseInTheRootIsTouched`);
* the ORDER is the timestamp in the name, never an mtime
  (:class:`TestOldestFirstByName`).
"""

import os
import re
import subprocess
import sys

from franka_web import defaults, retention
from franka_web.recording import segment_name, session_name
import pytest

GB = defaults.BYTES_PER_GB


def make_session(root, name, size_bytes, *, sealed=True, mtime=None):
    """
    Create one fake session directory of ``size_bytes`` under ``root``.

    The bag file is sparse: its apparent size is what the pass measures, so a
    multi-gigabyte session costs nothing to build. ``sealed`` writes the
    ``metadata.yaml`` rosbag2 writes last; ``mtime`` back-dates the directory
    so a pass that sorted by modification time would order differently.
    """
    bag = os.path.join(str(root), name, 'bag')
    os.makedirs(bag, exist_ok=True)
    overhead = 0
    if sealed:
        metadata = os.path.join(bag, 'metadata.yaml')
        with open(metadata, 'w', encoding='utf-8') as handle:
            handle.write('rosbag2_bagfile_information:\n  files: [bag_0.mcap]\n')
        overhead = os.path.getsize(metadata)
    with open(os.path.join(bag, 'bag_0.mcap'), 'wb') as handle:
        handle.truncate(max(0, int(size_bytes) - overhead))
    path = os.path.join(str(root), name)
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def names(entries):
    """Return the directory names of a sequence of plan entries."""
    return [entry.name for entry in entries]


def listing(root):
    """Return everything that still exists directly under ``root``."""
    return sorted(os.listdir(str(root)))


def messages(result):
    """Return just the message halves of a pass's emitted lines."""
    return [message for _level, message in result.lines]


@pytest.fixture()
def root(tmp_path):
    """Return an empty recordings root."""
    path = tmp_path / 'recordings'
    path.mkdir(mode=0o700)
    return path


class TestTheNamePattern:
    """The pattern is the recorder's own, and it is exact."""

    def test_it_matches_the_names_the_recorder_actually_writes(self):
        """
        The two modules must agree, so this derives one from the other.

        A pattern that drifted from ``recording.session_name`` would make the
        pass either blind to real sessions (the disk fills anyway) or willing
        to remove directories this server never wrote.
        """
        base = session_name()
        assert retention.is_session_name(base)
        for sequence in (2, 3, 999):
            assert retention.is_session_name(segment_name(base, sequence))

    @pytest.mark.parametrize('name', [
        'web-20260901-131757', 'web-20260901-131757-002',
        'web-19700101-000000', 'web-20260902-053837-003'])
    def test_accepted(self, name):
        """Every shape the recorder can produce is accepted."""
        assert retention.is_session_name(name)

    @pytest.mark.parametrize('name', [
        '', 'web', 'web-2026091-131757', 'web-20260901-13175',
        'web-20260901-131757-2', 'web-20260901-131757.bak', 'notes',
        'Web-20260901-131757', 'web-20260901-131757 ', '../web-20260901-131757',
        'web-20260901-131757/bag', 'lab-notes-20260901-131757'])
    def test_refused(self, name):
        """Anything else is invisible to the pass."""
        assert not retention.is_session_name(name)


class TestOldestFirstByName:
    """PROOF 1: the oldest sessions go, and 'oldest' means the name."""

    def build(self, root):
        """
        Five sealed sessions of 10 GB each, with mtimes DELIBERATELY inverted.

        The newest name carries the oldest mtime and the oldest name the
        newest, so a pass that sorted by modification time would remove
        exactly the wrong five.
        """
        order = ['web-20260101-000001', 'web-20260102-000002',
                 'web-20260103-000003', 'web-20260104-000004',
                 'web-20260105-000005']
        for index, name in enumerate(order):
            make_session(root, name, 10 * GB,
                         mtime=1900000000 - index * 86400)
        return order

    def test_the_plan_removes_the_oldest_until_it_is_under_the_cap(self, root):
        """25 GB of cap over 50 GB of recordings: the three oldest go."""
        order = self.build(root)
        computed = retention.plan(str(root), 25.0)
        assert computed.total_bytes == 50 * GB
        assert names(computed.remove) == order[:3]
        assert names(computed.keep) == order[3:]
        assert computed.remaining_bytes == 20 * GB
        assert computed.over_cap is False

    def test_the_order_is_the_name_even_when_the_mtimes_disagree(self, root):
        """
        This is the mutation guard: sort by mtime and this case fails.

        The mtimes are the exact reverse of the names, so an mtime sort
        removes the three NEWEST sessions -- the ones an operator was most
        likely still working with.
        """
        order = self.build(root)
        computed = retention.plan(str(root), 25.0)
        by_mtime = sorted(order, key=lambda name: os.stat(
            os.path.join(str(root), name)).st_mtime)
        assert by_mtime == list(reversed(order)), 'the fixture stopped shuffling'
        assert names(computed.remove) == order[:3]
        assert names(computed.remove) != by_mtime[:3]

    def test_the_pass_actually_removes_them_and_reports_each_one(self, root):
        """The applied pass leaves the two newest and writes four lines."""
        order = self.build(root)
        result = retention.run(str(root), 25.0)
        assert listing(root) == order[3:]
        assert names(result.removed) == order[:3]
        assert result.remaining_bytes == 20 * GB
        assert messages(result)[:3] == [
            'retention: removed web-20260101-000001 (10 GB); '
            'recordings now 40 of 25 GB',
            'retention: removed web-20260102-000002 (10 GB); '
            'recordings now 30 of 25 GB',
            'retention: removed web-20260103-000003 (10 GB); '
            'recordings now 20 of 25 GB',
        ]
        assert messages(result)[-1] == (
            'retention: 2 sessions hold 20 of 25 GB; removed 3')

    def test_a_root_already_under_the_cap_loses_nothing(self, root):
        """The common case writes one summary line and removes nothing."""
        order = self.build(root)
        result = retention.run(str(root), 500.0)
        assert listing(root) == order
        assert result.removed == ()
        assert messages(result) == [
            'retention: 5 sessions hold 50 of 500 GB; nothing to remove']

    def test_plan_alone_never_deletes_anything(self, root):
        """The planning half is pure: it reads the root and changes nothing."""
        order = self.build(root)
        computed = retention.plan(str(root), 1.0)
        assert names(computed.remove) == order
        assert listing(root) == order

    def test_unlimited_removes_nothing_and_says_so(self, root):
        """The documented off switch is a no-op pass with an honest line."""
        order = self.build(root)
        result = retention.run(str(root), None)
        assert listing(root) == order
        assert result.plan.unlimited is True
        assert messages(result) == [
            'retention: no size cap is set (recordings.max_total_gb: '
            'unlimited); 5 sessions hold 50 GB']

    def test_a_chain_segment_is_an_ordinary_candidate_in_name_order(self, root):
        """``-002`` sorts after its base, which is also its real age."""
        make_session(root, 'web-20260101-000001', 10 * GB)
        make_session(root, 'web-20260101-000001-002', 10 * GB)
        make_session(root, 'web-20260101-000002', 10 * GB)
        computed = retention.plan(str(root), 15.0)
        assert names(computed.remove) == ['web-20260101-000001',
                                          'web-20260101-000001-002']

    def test_an_empty_or_missing_root_is_a_single_summary_line(self, root):
        """Nothing to do is not an error, at startup or anywhere else."""
        result = retention.run(str(root / 'not-created-yet'), 50.0)
        assert result.removed == ()
        assert messages(result) == [
            'retention: 0 sessions hold 0 of 50 GB; nothing to remove']


class TestTheActiveSessionIsNeverRemoved:
    """PROOF 2: the recording in flight is protected, cap or no cap."""

    def test_it_survives_even_when_it_alone_exceeds_the_cap(self, root):
        """
        60 GB recording now, a 50 GB cap, and nothing else to give.

        The pass empties everything it legitimately can, reports that it is
        still above the cap, and does not touch the live bag. Dropping the
        active-session guard fails here.
        """
        make_session(root, 'web-20260101-000001', 5 * GB)
        make_session(root, 'web-20260601-120000', 60 * GB, sealed=False)
        result = retention.run(str(root), 50.0,
                               active_name='web-20260601-120000')
        assert listing(root) == ['web-20260601-120000']
        assert names(result.removed) == ['web-20260101-000001']
        assert result.remaining_bytes == 60 * GB
        assert messages(result)[-1] == (
            'retention: 1 sessions hold 60 of 50 GB; removed 1; 1 unsealed '
            'session kept: web-20260601-120000; still above the cap, and '
            'nothing else may be removed')

    def test_a_sealed_active_directory_is_still_protected(self, root):
        """
        Being sealed does not unprotect it; it is still being written.

        A rolled-over segment seals its predecessor while the chain runs, so
        "sealed" and "not in use" are different questions and only the
        active-name guard answers the second one.
        """
        make_session(root, 'web-20260101-000001', 60 * GB)
        computed = retention.plan(str(root), 1.0,
                                  active_name='web-20260101-000001')
        assert computed.remove == ()
        assert names(computed.protected) == ['web-20260101-000001']
        assert computed.over_cap is True

    def test_every_segment_of_the_live_chain_is_protected(self, root):
        """
        A session past its first hour owns several directories, not one.

        The recorder is writing ``-003``; ``-002`` and the base are the same
        live session's earlier hours, and removing them mid-session would
        tear a hole in the recording an operator is still making.
        """
        make_session(root, 'web-20260101-000001', 10 * GB)
        make_session(root, 'web-20260601-120000', 10 * GB)
        make_session(root, 'web-20260601-120000-002', 10 * GB)
        make_session(root, 'web-20260601-120000-003', 10 * GB, sealed=False)
        result = retention.run(str(root), 5.0,
                               active_name='web-20260601-120000-003')
        assert listing(root) == ['web-20260601-120000',
                                 'web-20260601-120000-002',
                                 'web-20260601-120000-003']
        assert names(result.removed) == ['web-20260101-000001']

    def test_the_timestamp_is_not_mistaken_for_a_chain_suffix(self, root):
        """
        A live ``web-20260903-105230`` must not protect that whole DAY.

        The chain suffix and the ``HHMMSS`` half are both runs of digits
        after a dash. Stripping the wrong one turns every session recorded on
        the same day into a protected directory, and the cap silently stops
        working on exactly the busiest day the lab has.
        """
        for name in ('web-20260903-044925', 'web-20260903-104949',
                     'web-20260903-105230'):
            make_session(root, name, 20 * GB)
        computed = retention.plan(str(root), 50.0,
                                  active_name='web-20260903-105230')
        assert names(computed.protected) == ['web-20260903-105230']
        assert names(computed.remove) == ['web-20260903-044925']
        assert retention.chain_names('web-20260903-105230') == frozenset(
            ('web-20260903-105230',))
        assert retention.chain_names('web-20260903-105230-002') == frozenset(
            ('web-20260903-105230-002', 'web-20260903-105230'))

    def test_no_active_name_protects_nothing(self, root):
        """The guard is a name, not a mood: without one every seal is fair."""
        make_session(root, 'web-20260101-000001', 60 * GB)
        computed = retention.plan(str(root), 1.0)
        assert names(computed.remove) == ['web-20260101-000001']
        assert computed.protected == ()


class TestUnsealedDirectoriesSurvive:
    """PROOF 3: a crashed session's bag is evidence, and it is counted."""

    def test_it_is_never_removed_is_counted_and_is_named_in_the_summary(
            self, root):
        """
        The unsealed 30 GB is the reason the pass cannot reach the cap.

        Counting it and saying so is the whole point: a pass that quietly
        ignored it would delete good recordings for ever and never explain
        why the total refused to come down. Dropping the unsealed guard
        fails here.
        """
        make_session(root, 'web-20260101-000001', 30 * GB, sealed=False)
        make_session(root, 'web-20260102-000002', 10 * GB)
        make_session(root, 'web-20260103-000003', 10 * GB)
        result = retention.run(str(root), 45.0)
        assert listing(root) == ['web-20260101-000001', 'web-20260103-000003']
        assert names(result.removed) == ['web-20260102-000002']
        assert names(result.plan.unsealed) == ['web-20260101-000001']
        # Counted: 50, not the 20 the two sealed sessions hold. Ignore the
        # unsealed 30 and this root is already under a 45 GB cap and nothing
        # would have been removed at all.
        assert result.plan.total_bytes == 50 * GB
        assert result.remaining_bytes == 40 * GB
        assert messages(result)[-1] == (
            'retention: 2 sessions hold 40 of 45 GB; removed 1; 1 unsealed '
            'session kept: web-20260101-000001')

    def test_several_unsealed_directories_are_all_named(self, root):
        """The summary names them, so the operator can go and look."""
        make_session(root, 'web-20260101-000001', 1 * GB, sealed=False)
        make_session(root, 'web-20260102-000002', 1 * GB, sealed=False)
        result = retention.run(str(root), 0.5)
        assert listing(root) == ['web-20260101-000001', 'web-20260102-000002']
        assert ('2 unsealed sessions kept: web-20260101-000001, '
                'web-20260102-000002') in messages(result)[-1]

    def test_a_metadata_file_beside_the_bag_directory_also_seals(self, root):
        """A layout change in the recorder must not un-seal every session."""
        path = make_session(root, 'web-20260101-000001', 1 * GB, sealed=False)
        with open(os.path.join(path, 'metadata.yaml'), 'w',
                  encoding='utf-8') as handle:
            handle.write('rosbag2_bagfile_information: {}\n')
        assert retention.is_sealed(path)
        computed = retention.plan(str(root), 0.5)
        assert names(computed.remove) == ['web-20260101-000001']


class TestNothingElseInTheRootIsTouched:
    """PROOF 4: the pattern, the root, and never a symlink."""

    def test_a_foreign_directory_and_a_loose_file_are_left_alone(self, root):
        """
        Only names this server writes are candidates, or visible at all.

        Removing the pattern check fails here -- and the failure it prevents
        is a lab owner who pointed the recordings key at a directory that
        already had things in it.
        """
        make_session(root, 'web-20260101-000001', 1 * GB)
        os.makedirs(os.path.join(str(root), 'calibration-2026'))
        with open(os.path.join(str(root), 'calibration-2026', 'poses.yaml'),
                  'wb') as handle:
            handle.truncate(40 * GB)
        with open(os.path.join(str(root), 'NOTES.md'), 'w',
                  encoding='utf-8') as handle:
            handle.write('do not delete\n')
        result = retention.run(str(root), 5.0)
        assert listing(root) == ['NOTES.md', 'calibration-2026',
                                 'web-20260101-000001']
        assert result.removed == ()
        # The foreign 40 GB is not counted either: this pass is responsible
        # for what it wrote and for nothing else. Counting it would put this
        # root 36 GB over the cap and delete the one real session.
        assert result.plan.total_bytes == 1 * GB
        assert messages(result) == [
            'retention: 1 sessions hold 1 of 5 GB; nothing to remove']

    def test_a_symlink_wearing_a_session_name_is_not_a_session(
            self, tmp_path, root):
        """
        A link out of the root is not followed, not counted, not removed.

        This is the case that turns a retention pass into an arbitrary
        deletion primitive: a link named like a session, pointing at a home
        directory, would otherwise be a candidate.
        """
        outside = tmp_path / 'somebody-elses-data'
        outside.mkdir()
        with open(str(outside / 'thesis.tex'), 'wb') as handle:
            handle.truncate(80 * GB)
        os.symlink(str(outside),
                   os.path.join(str(root), 'web-20260101-000001'))
        make_session(root, 'web-20260102-000002', 10 * GB)
        result = retention.run(str(root), 1.0)
        assert os.path.isdir(str(outside))
        assert os.path.isfile(str(outside / 'thesis.tex'))
        assert 'web-20260101-000001' in listing(root)
        assert names(result.plan.entries) == ['web-20260102-000002']
        assert result.plan.total_bytes == 10 * GB

    def test_a_symlink_inside_a_session_is_measured_but_not_followed(
            self, tmp_path, root):
        """A link inside a bag cannot inflate the total by its target."""
        outside = tmp_path / 'archive'
        outside.mkdir()
        with open(str(outside / 'huge.mcap'), 'wb') as handle:
            handle.truncate(90 * GB)
        path = make_session(root, 'web-20260101-000001', 1 * GB)
        os.symlink(str(outside / 'huge.mcap'),
                   os.path.join(path, 'bag', 'linked.mcap'))
        os.symlink(str(outside), os.path.join(path, 'linked_dir'))
        size = retention.directory_size(path)
        assert size < 2 * GB, size

    def test_a_directory_that_cannot_be_removed_is_reported_not_fatal(
            self, root):
        """One stubborn directory is a warn line; the pass keeps going."""
        make_session(root, 'web-20260101-000001', 10 * GB)
        make_session(root, 'web-20260102-000002', 10 * GB)
        make_session(root, 'web-20260103-000003', 10 * GB)
        refused = []

        def remove(path):
            """Refuse the first removal the way a read-only mount would."""
            if os.path.basename(path) == 'web-20260101-000001':
                refused.append(path)
                raise PermissionError(13, 'Permission denied')
            os.rename(path, path + '.gone')

        result = retention.run(str(root), 5.0, remove=remove)
        assert refused
        assert names(result.failed) == ['web-20260101-000001']
        assert names(result.removed) == ['web-20260102-000002',
                                         'web-20260103-000003']
        lines = messages(result)
        assert lines[0] == ('retention: could not remove web-20260101-000001: '
                            'Permission denied; keeping it')
        assert result.lines[0][0] == 'warn'
        assert '1 could not be removed' in lines[-1]

    def test_every_removal_is_a_path_inside_the_root(self, root):
        """The applied path is always root/name, never anything computed."""
        make_session(root, 'web-20260101-000001', 10 * GB)
        make_session(root, 'web-20260102-000002', 10 * GB)
        seen = []
        retention.run(str(root), 5.0, remove=seen.append)
        for path in seen:
            assert os.path.dirname(path) == str(root)
            assert retention.is_session_name(os.path.basename(path))


class TestTheWording:
    """The lines an operator reads, pinned where they are formatted."""

    @pytest.mark.parametrize('value_gb,text', [
        (50.0, '50'), (48.23, '48.2'), (13.137, '13.1'), (1.0, '1'),
        (0.5, '0.5'), (0.004, '0.004'), (0.0, '0'), (100.0, '100')])
    def test_a_gb_figure_reads_the_way_a_person_would_write_it(
            self, value_gb, text):
        """No thousandths on a fifty-gigabyte number, no zeroes on a small one."""
        assert retention.format_gb(value_gb) == text

    def test_the_removal_line_is_the_one_the_specification_shows(self):
        """One line, one session, the new total, the cap."""
        entry = retention.SessionDirectory(
            name='web-20260901-143853', path='/x/web-20260901-143853',
            size_bytes=13137282765, sealed=True)
        assert retention.removal_line(entry, 48234000000, 50.0) == (
            'retention: removed web-20260901-143853 (13.1 GB); '
            'recordings now 48.2 of 50 GB')

    def test_every_line_is_plain_words_with_no_jargon_or_paths(self, root):
        """The drawer is read by a lab owner, not by a log parser."""
        make_session(root, 'web-20260101-000001', 10 * GB)
        make_session(root, 'web-20260102-000002', 10 * GB, sealed=False)
        result = retention.run(str(root), 5.0)
        for line in messages(result):
            assert line.startswith('retention: ')
            assert str(root) not in line
            assert not re.search(r'[{}<>]|Traceback', line)


class TestThePureFunctionNeedsNothing:
    """The module is importable and usable with no server and no ROS."""

    def test_it_imports_and_plans_in_a_bare_interpreter(self, root, tmp_path):
        """A pure module with a filesystem argument, and nothing else."""
        make_session(root, 'web-20260101-000001', 10 * GB)
        script = (
            'from franka_web import retention;'
            'p = retention.plan({!r}, 1.0);'
            'print([e.name for e in p.remove])'.format(str(root)))
        completed = subprocess.run(
            [sys.executable, '-c', script], capture_output=True, text=True,
            check=False)
        assert completed.returncode == 0, completed.stderr
        assert "['web-20260101-000001']" in completed.stdout
        assert os.path.isdir(os.path.join(str(root), 'web-20260101-000001'))
