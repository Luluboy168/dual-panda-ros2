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
Two things a reader of this package must be able to rely on.

A test named in the prose must exist, because the prose's whole authority is
that its claims are checked somewhere a reader can open; a citation to a
deleted file is a claim with nothing behind it, and it reads exactly like a
claim with something behind it.  And no shipped file may name the private
planning tree, which is not part of this repository: a reader who follows such
a name finds nothing, and the name discloses a path that was never shipped.

Both are cheap greps.  Both were written because the review found one instance
of each.
"""

from pathlib import Path
import re

from conftest import SOURCE_DIR

import pytest


TEST_DIR = Path(__file__).resolve().parent
#: Every prose file that ships with the package.
DOCUMENTS = ('README.md', 'doc/CONTRACT.md')
#: A python test module named in prose.  Anchored on the ``test_`` prefix
#: because that is how every module under ``test/`` is named and how a reader
#: recognises one in a sentence.
CITATION = re.compile(r'\btest_[a-z0-9_]*\.py\b')
#: The private planning tree, its plan documents and its gate marker.  None of
#: these may appear in anything this package ships.  ``_PLAN`` is matched on a
#: word boundary so that an ordinary identifier such as ``BASE_PLANE_TOLERANCE``
#: is not a false positive.
NOTES_MARKERS = (r'multipanda_ros2_jazzy_notes', r'_PLAN\b', r'STOPGATE')
#: Directories that hold build output or caches rather than shipped files.
SKIPPED_DIRECTORIES = ('__pycache__', '.pytest_cache', 'build', 'install', 'log')
#: The file extensions a marker could hide in.  The generated artefacts are
#: included: they are shipped text, and a comment in one would ship too.
SCANNED_SUFFIXES = ('.py', '.md', '.yaml', '.yml', '.xml', '.txt', '.cfg',
                    '.json', '.svg', '.cmake', '.srdf', '.urdf', '.xacro')


def _shipped_files():
    for path in sorted(SOURCE_DIR.rglob('*')):
        if not path.is_file() or path.suffix not in SCANNED_SUFFIXES:
            continue
        if any(part in SKIPPED_DIRECTORIES for part in path.parts):
            continue
        yield path


def _markers_in(text):
    return sorted(marker for marker in NOTES_MARKERS
                  if re.search(marker, text) is not None)


@pytest.mark.parametrize('document', DOCUMENTS)
def test_every_test_module_the_prose_names_exists(document):
    """
    A citation is a promise that the reader can go and look.

    Section 5b cited ``test_mesh_is_inert.py`` for six commits after the file
    was deleted, and cited it for the OPPOSITE of what the package now does.
    Nothing in the suite noticed, because nothing in the suite read the prose.
    """
    text = (SOURCE_DIR / document).read_text(encoding='utf-8')
    cited = sorted(set(CITATION.findall(text)))
    assert cited, document
    missing = [name for name in cited if not (TEST_DIR / name).is_file()]
    assert missing == [], '{} cites {}'.format(document, missing)


def test_no_shipped_file_names_the_private_planning_tree():
    """
    The notes tree is not part of this repository and may not be named in it.

    This module is the one exception, because it has to spell the markers out
    to look for them; the positive control below is what keeps that exception
    from being a hole.
    """
    offending = {}
    for path in _shipped_files():
        if path.resolve() == Path(__file__).resolve():
            continue
        found = _markers_in(path.read_text(encoding='utf-8', errors='replace'))
        if found:
            offending[str(path.relative_to(SOURCE_DIR))] = found
    assert offending == {}


def test_the_marker_scan_would_catch_the_instance_it_was_written_for():
    """The positive control: the scanner is not vacuous."""
    assert _markers_in('#: Matches COVERAGE_FIX' + "_PLAN's") == [r'_PLAN\b']
    assert _markers_in('see /home/x/multipanda_ros2' + '_jazzy_notes/plans/a.md')
    assert _markers_in('STOP' + 'GATE 3 is open')
    # ...and it does not fire on ordinary package identifiers.
    assert _markers_in('BASE_PLANE_TOLERANCE = 1e-9') == []
    assert _markers_in('the plan is written down') == []
