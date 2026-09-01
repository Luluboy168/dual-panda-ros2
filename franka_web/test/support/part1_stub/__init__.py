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
Stand-in for the configuration subsystem while it is built in parallel.

This package exists ONLY so the backend and its tests can run before the
real ``franka_web.defaults`` and ``franka_web.config`` modules land. The
backend imports the real names; :func:`install` handles the two cases:

* ``franka_web.defaults`` does not exist yet, so a LAST-RESORT
  ``sys.meta_path`` finder answers it. Because the finder is APPENDED, the
  ordinary path finder is consulted first and a real module always wins.
* ``franka_web/config.py`` still exists, but carries the previous
  environment-sourced loader rather than the file-based one the backend now
  consumes. It is detected by the absence of ``load`` and shadowed in
  ``sys.modules`` -- which is safe only because this runs from ``conftest``
  before any test has imported it.

Both halves become inert the moment the real subsystem is merged.

DELETE THIS PACKAGE, and the ``install()`` call in ``test/conftest.py``,
once the real configuration subsystem is merged. :func:`is_active` reports
whether the double actually answered, so a test whose subject is a
byte-exact operator-facing configuration message can skip itself rather
than assert against this approximation.
"""

import importlib
import importlib.util
import sys

_ALIASES = {
    'franka_web.defaults': 'support.part1_stub.defaults',
    'franka_web.config': 'support.part1_stub.config',
}

_ACTIVE = False
_INSTALLED = False


def is_active():
    """Return True once the double has answered an import of a real name."""
    return _ACTIVE


class _AliasLoader:
    """Loader that hands back an already-imported module under a new name."""

    def __init__(self, module):
        """Remember the module this loader aliases."""
        self._module = module

    def create_module(self, spec):
        """Return the aliased module itself rather than a fresh one."""
        return self._module

    def exec_module(self, module):
        """Do nothing: the aliased module has already been executed."""


class _FallbackFinder:
    """Last-resort finder answering the two configuration module names."""

    def find_spec(self, fullname, path=None, target=None):
        """Return a spec aliasing the double, or None for every other name."""
        alias = _ALIASES.get(fullname)
        if alias is None:
            return None
        global _ACTIVE
        module = importlib.import_module(alias)
        _ACTIVE = True
        return importlib.util.spec_from_loader(fullname, _AliasLoader(module))


def install():
    """
    Install the doubles where they are needed; return whether any is in use.

    Both questions are forced here, so :func:`is_active` is meaningful from
    the first line of any test module.
    """
    global _ACTIVE, _INSTALLED
    if not _INSTALLED:
        sys.meta_path.append(_FallbackFinder())
        _INSTALLED = True
    import franka_web
    importlib.import_module('franka_web.defaults')
    real = importlib.import_module('franka_web.config')
    if not hasattr(real, 'load'):
        stub = importlib.import_module(_ALIASES['franka_web.config'])
        sys.modules['franka_web.config'] = stub
        franka_web.config = stub
        _ACTIVE = True
    return _ACTIVE
