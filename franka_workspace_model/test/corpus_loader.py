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
Strict loader for the validation corpus.

The corpus is the oracle: every expectation in it was derived by hand or by an
independent forward-kinematics chain, never by running the checker under test.
The loader is as strict as the cell model's, and it refuses an entry with no
rationale - an entry that merely asserts is an entry nobody can re-check.
"""

from pathlib import Path

from franka_workspace_model.strictyaml import (exact_keys, load_strict_yaml,
                                               read_bounded_regular_text,
                                               WorkspaceModelError)


CATEGORIES = ('known_clear', 'self', 'cross_arm', 'containment', 'environment',
              'keep_out', 'joint_limit')
CONTACT_KINDS = ('joint_limit', 'self', 'cross_arm', 'containment', 'environment',
                 'keep_out')
ENTRY_KEYS = ('id', 'q', 'expect', 'rationale')
EXPECT_REQUIRED = ('ok', 'kinds', 'forbidden_kinds')
EXPECT_OPTIONAL = ('witness', 'min_clearance_max')
BOOLEAN_PATHS = ('entries[].expect.ok',)


class CorpusEntry:
    """One hand-derived expectation."""

    def __init__(self, category, entry, path):
        context = '{}:{}'.format(path.name, entry.get('id', '<unnamed>'))
        exact_keys(entry, ENTRY_KEYS, context)
        self.category = category
        self.id = entry['id']
        if not isinstance(self.id, str) or not self.id:
            raise WorkspaceModelError('{}: id must be a non-empty string'.format(context))
        self.q = {}
        if not isinstance(entry['q'], dict) or not entry['q']:
            raise WorkspaceModelError('{}: q must map arm_id to joint values'.format(
                context))
        for arm_id, values in entry['q'].items():
            if not isinstance(values, list) or len(values) != 7:
                raise WorkspaceModelError(
                    '{}: arm {} needs exactly seven joint values'.format(context, arm_id))
            self.q[arm_id] = [float(value) for value in values]
        expect = entry['expect']
        if not isinstance(expect, dict):
            raise WorkspaceModelError('{}: expect must be a mapping'.format(context))
        unknown = set(expect) - set(EXPECT_REQUIRED) - set(EXPECT_OPTIONAL)
        missing = set(EXPECT_REQUIRED) - set(expect)
        if unknown or missing:
            raise WorkspaceModelError(
                '{}: expect keys differ; missing={} unknown={}'.format(
                    context, sorted(missing), sorted(unknown)))
        self.ok = expect['ok']
        if not isinstance(self.ok, bool):
            raise WorkspaceModelError('{}: expect.ok must be a boolean'.format(context))
        self.kinds = tuple(expect['kinds'])
        self.forbidden_kinds = tuple(expect['forbidden_kinds'])
        for kind in self.kinds + self.forbidden_kinds:
            if kind not in CONTACT_KINDS:
                raise WorkspaceModelError(
                    '{}: unknown contact kind {!r}'.format(context, kind))
        if set(self.kinds) & set(self.forbidden_kinds):
            raise WorkspaceModelError(
                '{}: a kind cannot be both required and forbidden'.format(context))
        self.witness = None
        if 'witness' in expect:
            witness = exact_keys(expect['witness'], ('a', 'b'),
                                 '{}: witness'.format(context))
            self.witness = (witness['a'], witness['b'])
        self.min_clearance_max = None
        if 'min_clearance_max' in expect:
            self.min_clearance_max = float(expect['min_clearance_max'])
        self.rationale = entry['rationale']
        if not isinstance(self.rationale, str) or len(self.rationale.strip()) < 40:
            raise WorkspaceModelError(
                '{}: rationale is mandatory and must explain why the answer is known '
                'without running the checker'.format(context))

    def __repr__(self):
        return 'CorpusEntry({})'.format(self.id)


def load_corpus(directory: Path):
    """Load every category file, refusing a mis-filed or duplicated entry."""
    entries = []
    seen = set()
    for category in CATEGORIES:
        path = Path(directory) / '{}.yaml'.format(category)
        text = read_bounded_regular_text(path, 'the {} corpus'.format(category))
        document = load_strict_yaml(text, BOOLEAN_PATHS)
        exact_keys(document, ('schema_version', 'category', 'entries'), path.name)
        if document['schema_version'] != 1:
            raise WorkspaceModelError('{}: schema_version must be 1'.format(path.name))
        if document['category'] != category:
            raise WorkspaceModelError(
                '{}: category {!r} does not match the filename stem'.format(
                    path.name, document['category']))
        if not isinstance(document['entries'], list) or not document['entries']:
            raise WorkspaceModelError('{}: entries must be non-empty'.format(path.name))
        for entry in document['entries']:
            parsed = CorpusEntry(category, entry, path)
            if parsed.id in seen:
                raise WorkspaceModelError(
                    'duplicate corpus entry id {!r}; ids are unique across the whole '
                    'corpus'.format(parsed.id))
            seen.add(parsed.id)
            entries.append(parsed)
    return tuple(entries)
