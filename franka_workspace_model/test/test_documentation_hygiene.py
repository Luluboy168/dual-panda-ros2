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
A test named in the prose must exist.

The prose's whole authority is that its claims are checked somewhere a reader
can open.  A citation to a deleted file is a claim with nothing behind it, and
it reads exactly like a claim with something behind it.

It is a cheap grep.  It was written because the review found one instance.
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
