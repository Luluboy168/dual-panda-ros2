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
The workspace-model wrapper: load once, cache, degrade visibly.

The collision core is pure Python; this module imports it lazily and never
lets its absence stop the console. A checker failure degrades the VERDICT and
never blocks pose authoring -- the ghost commands nothing, so there is no
motion to fail closed on. The jog fence, when it is built, is a different
consumer and inherits fail-closed in full.

Every teaching sentence the checker can produce is a literal in this file and
nowhere else, so there is exactly one author per sentence and one place to
edit when the wording changes.
"""

import os
import threading

import yaml

try:
    from franka_workspace_model import model as workspace_model
except ImportError:                    # a workspace built without the package
    workspace_model = None

    class WorkspaceModelError(Exception):
        """Stand-in so ``except`` clauses stay total without the package."""
else:
    WorkspaceModelError = workspace_model.WorkspaceModelError

#: The checker-side teaching sentences, verbatim and authored ONLY here.
#: They are rendered with textContent by the page and asserted byte for byte
#: by the scene and verdict tests; changing one is a change to what the
#: operator is taught, not a copy edit. No other file in the repository --
#: Python or JavaScript -- holds a second copy of any of them.
NOTE_PACKAGE_ABSENT = (
    'The workspace model is not installed, so poses are not '
    'collision-checked and the cell is not drawn.')
NOTE_LOAD_FAILED = (
    'The cell model could not be loaded, so poses are not collision-checked.')
NOTE_INTERLOCK_MISMATCH = (
    'The cell model was built for a different robot description, so poses '
    'are not collision-checked.')
NOTE_SCENE_INCOMPLETE = (
    'Only one arm is visible, so the two-arm check could not run.')
NOTE_PROFILE_ARM_MISMATCH = (
    'The cell model does not describe this arm on its own, so this pose was '
    'not collision-checked.')

#: The cell file's name and its location inside the model package's share
#: directory. Used only when that package is too old to say where its own
#: cell file is; a package that answers is always believed instead.
CELL_RELATIVE_PATH = os.path.join('cell', 'cell_model_v1.yaml')

#: Interlock states, exactly the three the scene payload may carry.
INTERLOCK_STATES = ('ok', 'mismatch', 'not_checked')

# No environment variable resolves the cell path, here or anywhere:
# config.py is the one configuration surface and its optional
# directories.cell_model key is the operator's override. `os` is imported for
# path joins and isfile only; this module never reads os.environ.


def _package_directory():
    """Return the installed model package's own directory, or None."""
    path = getattr(workspace_model, '__file__', None)
    return None if path is None else os.path.dirname(os.path.abspath(path))


def _share_candidates():
    """Return the cell-file paths a model package of any age would install."""
    package = _package_directory()
    if package is None:
        return ()
    candidates = [os.path.join(os.path.dirname(package), CELL_RELATIVE_PATH)]
    # A plain (non-symlink) install puts the module under
    # <prefix>/lib/python3.N/site-packages and the cell file under
    # <prefix>/share/franka_workspace_model. Walk up to find the prefix
    # rather than hard-coding a Python version.
    walk = os.path.dirname(package)
    for _ in range(4):
        walk = os.path.dirname(walk)
        if not walk or walk == os.sep:
            break
        candidates.append(os.path.join(
            walk, 'share', 'franka_workspace_model', CELL_RELATIVE_PATH))
    return tuple(candidates)


def resolve_cell_path(configured=None):
    """
    Return the cell file this console should load, or None.

    Resolution order, and there is no fourth step: the operator's
    ``directories.cell_model`` key, then the model package's own answer to
    "where did I install my cell file", then the locations such a package
    installs it at. A configured path is returned even when nothing is there,
    so the banner can name the path the operator asked for rather than
    silently looking somewhere else.
    """
    if configured:
        return configured
    default_path = None
    accessor = getattr(workspace_model, 'default_cell_model_path', None)
    if accessor is not None:
        try:
            default_path = accessor()
        except Exception:          # noqa: BLE001 - a probe never breaks boot
            default_path = None
    if default_path is not None:
        return str(default_path)
    for candidate in _share_candidates():
        if os.path.isfile(candidate):
            return candidate
    return None


def _allowed_volume(model, cell_path):
    """
    Return the six cell bounds and their id, or None.

    The model package offers an accessor for exactly this, and that is what
    is used. The YAML re-read below is the documented fallback for a
    package built before the accessor existed: it parses the same file a
    second time, AFTER ``CellModel.load`` has already validated it, so it can
    only ever read numbers the loader accepted. Two readers of one file is
    strictly worse than one, which is why it is a fallback and not the plan.
    """
    accessor = getattr(model, 'allowed_volume', None)
    if accessor is not None:
        volume = accessor()
        return {
            'id': str(volume.id),
            'frame': str(volume.frame),
            'x_min': float(volume.x_min), 'x_max': float(volume.x_max),
            'y_min': float(volume.y_min), 'y_max': float(volume.y_max),
            'z_min': float(volume.z_min), 'z_max': float(volume.z_max),
        }
    return _allowed_volume_from_yaml(cell_path)


def _allowed_volume_from_yaml(cell_path):
    """Re-read the six bounds out of the already-validated cell file."""
    if not cell_path or not os.path.isfile(cell_path):
        return None
    try:
        with open(cell_path, encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
        entry = document['allowed_volume']
        return {
            'id': str(entry['id']),
            'frame': str(entry['frame']),
            'x_min': float(entry['x_min']), 'x_max': float(entry['x_max']),
            'y_min': float(entry['y_min']), 'y_max': float(entry['y_max']),
            'z_min': float(entry['z_min']), 'z_max': float(entry['z_max']),
        }
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return None


def _model_identity(model):
    """Return the (id, revision, digest) triple every result carries."""
    model_id, revision, digest = model.model_identity()
    return {'model_id': str(model_id), 'model_revision': int(revision),
            'model_sha256': str(digest)}


class WorkspaceChecker:
    """Loads and caches one CellModel per profile; never raises outward."""

    def __init__(self, cell_path=None, log_bus=None):
        """Resolve the cell path once; load nothing until something asks."""
        self._lock = threading.Lock()      # guards the CACHE, not the check
        self._models = {}                  # profile -> CellModel
        self._errors = {}                  # profile -> str (its own sentence)
        self._cell_path = resolve_cell_path(cell_path)
        self._configured = bool(cell_path)
        self._interlock = 'not_checked'
        self._log_bus = log_bus
        self._warned = set()               # log each distinct failure once

    # -- lifecycle ----------------------------------------------------

    @property
    def cell_path(self):
        """Return the path this checker will load, or None."""
        return self._cell_path

    def model_for(self, profile):
        """Return the loaded CellModel for ``profile``, or None (cached)."""
        with self._lock:
            if profile in self._models or profile in self._errors:
                return self._models.get(profile)
        model, error = self._load(profile)
        with self._lock:
            if model is not None:
                self._models[profile] = model
            else:
                self._errors[profile] = error
        return model

    def _load(self, profile):
        """Load one profile, returning (model, None) or (None, sentence)."""
        if workspace_model is None:
            return None, NOTE_PACKAGE_ABSENT
        if self._cell_path is None:
            return None, NOTE_LOAD_FAILED + '\nNo cell file was found.'
        if not os.path.isfile(self._cell_path):
            return None, (NOTE_LOAD_FAILED
                          + '\nNo cell file was found at: '
                          + self._cell_path)
        try:
            return workspace_model.CellModel.load(
                self._cell_path, profile=profile), None
        except WorkspaceModelError as error:
            self._warn('workspace model: {}'.format(error))
            return None, NOTE_LOAD_FAILED + '\n' + str(error)
        except Exception as error:     # noqa: BLE001 - a load never kills boot
            self._warn('workspace model: {}'.format(error))
            return None, NOTE_LOAD_FAILED + '\n' + str(error)

    def set_interlock(self, state):
        """
        Record what a session-start description interlock concluded.

        The ghost performs no interlock of its own: it would compare a
        differently-parameterised expansion of the same xacro and disagree
        for no reason. This exists so that whatever DOES perform one can say
        so, and so the mismatch path is implemented rather than theoretical.
        """
        if state in INTERLOCK_STATES:
            self._interlock = state

    def _warn(self, message):
        """Put one distinct failure on the log bus, once."""
        if self._log_bus is None or message in self._warned:
            return
        self._warned.add(message)
        self._log_bus.emit('warn', message)

    # -- reporting ----------------------------------------------------

    def status(self, profile):
        """Return the cell and checker blocks of the scene payload."""
        model = self.model_for(profile)
        if model is None:
            return {
                'available': False,
                'profile': None,
                'interlock': self._interlock,
                'checker_note': self._interlock_note(),
                'cell': None,
                'cell_source': 'unavailable',
                'cell_note': self._errors.get(profile, NOTE_PACKAGE_ABSENT),
                'model': None,
            }
        cell = _allowed_volume(model, self._cell_path)
        if cell is None:
            # The model loaded but will not say where the box is: report the
            # cell as unavailable rather than draw a guess.
            return {
                'available': True,
                'profile': profile,
                'interlock': self._interlock,
                'checker_note': self._interlock_note(),
                'cell': None,
                'cell_source': 'unavailable',
                'cell_note': NOTE_LOAD_FAILED
                + '\nThe cell model does not report its measured volume.',
                'model': _model_identity(model),
            }
        return {
            'available': True,
            'profile': profile,
            'interlock': self._interlock,
            'checker_note': self._interlock_note(),
            'cell': cell,
            'cell_source': 'cell_model',
            'cell_note': None,
            'model': _model_identity(model),
        }

    def apply_note(self, profile, arm_id=None):
        """
        Return ``(sentence, code)`` when this checker can judge nothing, else None.

        The three cheap questions -- is a model loaded, does the interlock
        object, does this model describe this arm -- with none of the scene
        block around them. :meth:`status` answers the same three, but on its
        way it also resolves the measured cell volume, which for a model
        package too old to offer the accessor means re-reading and re-parsing
        the cell file. That is fine on the scene request it was written for and
        wrong on a caller that asks once per arm per state frame, which is
        what this exists for.

        The sentences and the order are :meth:`status`'s, and the two are
        asserted to agree, so this is a cheaper route to one answer rather than
        a second opinion.
        """
        model = self.model_for(profile)
        if model is None:
            return (self._errors.get(profile) or NOTE_PACKAGE_ABSENT), 'absent'
        if self._interlock == 'mismatch':
            return NOTE_INTERLOCK_MISMATCH, 'mismatch'
        # The fourth row, and it is per ARM rather than per profile: a model
        # that loaded may still not describe THIS arm, and a pose it cannot
        # judge is a pose that must be refused.
        if arm_id is not None and arm_id not in tuple(model.arm_ids()):
            return NOTE_PROFILE_ARM_MISMATCH, 'absent'
        return None

    def _interlock_note(self):
        """
        Return the sentence for a non-ok interlock, or None.

        This is the ONLY path by which the interlock sentence reaches a
        screen. ``cell_note`` cannot carry it: on a mismatch the cell still
        loads and is still drawn, so ``cell_note`` is None exactly then, and
        the scene gate asserts cell_note is non-null iff the cell source is
        unavailable. Without this field the sentence would be authored here
        and copied into the page as a second literal that nothing compares.

        It carries a sentence for ``mismatch`` and for nothing else. The
        third state, ``not_checked``, is what an ordinary console reports --
        the ghost performs no interlock of its own -- and there is no
        sentence that could honestly be shown for it: saying the cell model
        was built for a different description would be a claim nobody made.
        """
        return NOTE_INTERLOCK_MISMATCH if self._interlock == 'mismatch' else None

    def banner(self):
        """Return the one startup line describing the checker's state."""
        if workspace_model is None:
            return 'no workspace model installed'
        if self._cell_path is None:
            return 'no cell model found'
        if self.model_for('dual') is None:
            return 'cell model not loaded ({})'.format(self._cell_path)
        return 'cell model loaded ({})'.format(self._cell_path)

    # -- the check ----------------------------------------------------

    def check(self, profile, scene):
        """
        Return ``(CheckResult, None, None)`` or ``(None, sentence, code)``.

        No lock: the parsed cell structure is immutable after load and the
        core is numpy-only and ROS-free, so the call is pure and two viewers
        dragging never serialise behind each other. If that guarantee ever
        comes back qualified, one lock here costs about two milliseconds per
        solve.
        """
        model = self.model_for(profile)
        if model is None:
            return (None,
                    self._errors.get(profile, NOTE_PACKAGE_ABSENT),
                    'checker_absent')
        if self._interlock == 'mismatch':
            return None, NOTE_INTERLOCK_MISMATCH, 'interlock_mismatch'
        wanted = tuple(model.arm_ids())
        missing = [arm for arm in wanted
                   if arm not in scene or scene[arm] is None]
        if missing:
            if len(wanted) == 1:
                return None, NOTE_PROFILE_ARM_MISMATCH, 'profile_arm_mismatch'
            return None, NOTE_SCENE_INCOMPLETE, 'other_arm_pose_unknown'
        payload = {arm: tuple(scene[arm]) for arm in wanted}
        # WHY first_violation=False, which is not the cheaper option, and for
        # the reason the path check next door states at length. With True the
        # model STOPS at the first violation it finds, so `contacts` holds
        # exactly one entry however many arms are in trouble -- an answer that can say
        # "this cell is refused" and cannot say WHOSE fault it is. The console
        # draws one line per arm off `contacts`, so an early-exit answer left
        # a faulted arm with no contact naming it, which read as clear. The
        # cost is bounded: a clear configuration already evaluates everything,
        # so the worst case is unchanged and only refused cells pay.
        try:
            return model.check_configuration(payload, first_violation=False), None, None
        except WorkspaceModelError as error:
            self._warn('workspace check refused: {}'.format(error))
            return None, str(error), 'checker_error'
