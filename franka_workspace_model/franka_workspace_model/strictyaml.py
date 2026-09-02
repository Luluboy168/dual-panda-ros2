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
Strict YAML loading for every file this package reads.

Modelled on the reviewed controller-config validator in ``franka_bringup``:
bounded regular-file reads, one document, no anchors, no aliases, no merge keys,
no explicit tags, no duplicate keys, no non-string keys, no nulls, no non-finite
numbers, a nesting bound and a scalar bound.

One deliberate widening: the cell model declares boolean-valued keys, which the
precedent rejects outright.  Booleans are therefore accepted, but only at the
exact key paths the caller declares, and never as a mapping key.
"""

import math
import os
from pathlib import Path
import stat
from typing import Any, Mapping, Sequence

import yaml
from yaml.events import AliasEvent
from yaml.events import DocumentStartEvent
from yaml.events import MappingEndEvent
from yaml.events import MappingStartEvent
from yaml.events import ScalarEvent
from yaml.events import SequenceEndEvent
from yaml.events import SequenceStartEvent


MAXIMUM_MODEL_BYTES = 65536
MAXIMUM_YAML_DEPTH = 8
MAXIMUM_YAML_SCALARS = 2048


class WorkspaceModelError(ValueError):
    """
    A deterministic, user-correctable workspace-model failure.

    The package raises no other exception type across its public surface, and
    lets no library exception escape unwrapped.  It is a load and validation
    error type: a detected collision is a normal return with ``ok`` false.
    """


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """A SafeLoader that refuses duplicate keys, non-string keys and merge keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise WorkspaceModelError('every YAML mapping key must be a string')
        if key == '<<':
            raise WorkspaceModelError('YAML merge keys are forbidden')
        if key in mapping:
            raise WorkspaceModelError('duplicate YAML key: {}'.format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _scan_yaml_events(text: str) -> None:
    document_count = 0
    depth = 0
    scalar_count = 0
    try:
        for event in yaml.parse(text):
            if isinstance(event, DocumentStartEvent):
                document_count += 1
                if document_count > 1:
                    raise WorkspaceModelError('multiple YAML documents are forbidden')
            if isinstance(event, AliasEvent):
                raise WorkspaceModelError('YAML aliases are forbidden')
            if getattr(event, 'anchor', None) is not None:
                raise WorkspaceModelError('YAML anchors are forbidden')
            if getattr(event, 'tag', None) is not None:
                raise WorkspaceModelError('explicit YAML tags are forbidden')
            if isinstance(event, ScalarEvent):
                scalar_count += 1
                if scalar_count > MAXIMUM_YAML_SCALARS:
                    raise WorkspaceModelError('YAML scalar count exceeds the fixed limit')
                if event.value == '<<':
                    raise WorkspaceModelError('YAML merge keys are forbidden')
            if isinstance(event, (MappingStartEvent, SequenceStartEvent)):
                depth += 1
                if depth > MAXIMUM_YAML_DEPTH:
                    raise WorkspaceModelError('YAML nesting exceeds the fixed limit')
            elif isinstance(event, (MappingEndEvent, SequenceEndEvent)):
                depth -= 1
    except WorkspaceModelError:
        raise
    except yaml.YAMLError as error:
        raise WorkspaceModelError('malformed YAML') from error
    if document_count != 1 or depth != 0:
        raise WorkspaceModelError('the file must contain exactly one YAML document')


def _reject_unsafe_scalars(value: Any, path: str, boolean_paths) -> None:
    if value is None:
        raise WorkspaceModelError('null YAML values are forbidden')
    if isinstance(value, bool):
        if path not in boolean_paths:
            raise WorkspaceModelError(
                'boolean YAML values are only allowed at the declared switch keys; '
                'found one at {}'.format(path or '<root>'))
        return
    if isinstance(value, float) and not math.isfinite(value):
        raise WorkspaceModelError('non-finite YAML numbers are forbidden')
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise WorkspaceModelError('every YAML mapping key must be a string')
            _reject_unsafe_scalars(child, '{}.{}'.format(path, key) if path else key,
                                   boolean_paths)
    elif isinstance(value, list):
        for child in value:
            _reject_unsafe_scalars(child, '{}[]'.format(path), boolean_paths)
    elif not isinstance(value, (str, int, float)):
        raise WorkspaceModelError('unsupported YAML value at {}'.format(path or '<root>'))


def load_strict_yaml(text: str, boolean_paths: Sequence[str] = ()) -> Any:
    """Parse one bounded YAML document under the strict rules above."""
    encoded = text.encode('utf-8')
    if not encoded or not text.strip():
        raise WorkspaceModelError('the file is empty')
    if len(encoded) > MAXIMUM_MODEL_BYTES:
        raise WorkspaceModelError('YAML size is outside the fixed limit')
    if '\x00' in text:
        raise WorkspaceModelError('NUL bytes are forbidden')
    _scan_yaml_events(text)
    try:
        value = yaml.load(text, Loader=_UniqueKeySafeLoader)
    except WorkspaceModelError:
        raise
    except yaml.YAMLError as error:
        raise WorkspaceModelError('malformed YAML') from error
    _reject_unsafe_scalars(value, '', frozenset(boolean_paths))
    return value


def read_bounded_regular_text(path: Path, context: str) -> str:
    """Read a bounded regular file, refusing directories, devices and dangling links."""
    descriptor = -1
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode) or details.st_size > MAXIMUM_MODEL_BYTES:
            raise WorkspaceModelError('{} must be a bounded regular file'.format(context))
        data = os.read(descriptor, MAXIMUM_MODEL_BYTES + 1)
        if len(data) > MAXIMUM_MODEL_BYTES:
            raise WorkspaceModelError('{} exceeds the fixed size limit'.format(context))
        return data.decode('utf-8')
    except WorkspaceModelError:
        raise
    except (OSError, UnicodeError) as error:
        raise WorkspaceModelError('unable to read {}'.format(context)) from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def exact_keys(mapping: Any, expected: Sequence[str], context: str) -> Mapping[str, Any]:
    """Require a mapping to carry exactly the expected keys, no more and no fewer."""
    if not isinstance(mapping, dict):
        raise WorkspaceModelError('{} must be a mapping'.format(context))
    expected_set = set(expected)
    actual_set = set(mapping)
    if actual_set != expected_set:
        missing = sorted(expected_set - actual_set)
        unknown = sorted(actual_set - expected_set)
        raise WorkspaceModelError(
            '{} keys differ; missing={} unknown={}'.format(context, missing, unknown))
    return mapping
