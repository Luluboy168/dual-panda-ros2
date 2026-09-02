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
A scripted stand-in for the workspace checker.

It carries the model's six frozen contact fields exactly, so every contact
kind and every containment face is reachable without a real cell model -- and
so the "never clear on an exception" case can be forced, which no real cell
file can be persuaded to do on demand.
"""

from dataclasses import dataclass, field
import threading

from franka_web import workspace


@dataclass(frozen=True)
class Contact:
    """One reported reason a configuration is not allowed."""

    kind: str
    a: str
    b: str
    distance: float
    required: float
    arm_id: str


@dataclass(frozen=True)
class CheckResult:
    """The verdict on one configuration."""

    ok: bool
    min_clearance: float
    contacts: tuple = ()
    sample_index: int = None
    samples_evaluated: int = 1
    model_id: str = 'fake_cell'
    model_revision: int = 1
    model_sha256: str = '0' * 64


CELL = {'id': 'work_area', 'frame': 'cell',
        'x_min': -0.35, 'x_max': 0.9, 'y_min': -1.0, 'y_max': 1.0,
        'z_min': 0.0, 'z_max': 2.0}

MODEL = {'model_id': 'hcis_dual_panda_cell', 'model_revision': 3,
         'model_sha256': 'a' * 64}


@dataclass
class FakeChecker:
    """Answers like WorkspaceChecker, from a script rather than a cell file."""

    available: bool = True
    result: object = None            # CheckResult to return, or None
    triple: tuple = None             # (sentence, reason_code) to return
    interlock: str = 'not_checked'
    cell: dict = field(default_factory=lambda: dict(CELL))
    seen: list = field(default_factory=list)      # every scene it was asked
    profiles: list = field(default_factory=list)  # every profile it was asked
    overlaps: list = field(default_factory=list)  # concurrency evidence
    _live: int = 0
    _lock: object = field(default_factory=threading.Lock)

    def status(self, profile):
        """Return the scene payload's cell and checker blocks."""
        if not self.available:
            return {'available': False, 'profile': None,
                    'interlock': self.interlock,
                    'checker_note': self._note(),
                    'cell': None, 'cell_source': 'unavailable',
                    'cell_note': workspace.NOTE_PACKAGE_ABSENT,
                    'model': None}
        return {'available': True, 'profile': profile,
                'interlock': self.interlock,
                'checker_note': self._note(),
                'cell': dict(self.cell), 'cell_source': 'cell_model',
                'cell_note': None, 'model': dict(MODEL)}

    def _note(self):
        """Return the interlock sentence, on a mismatch and nowhere else."""
        return (workspace.NOTE_INTERLOCK_MISMATCH
                if self.interlock == 'mismatch' else None)

    def check(self, profile, scene):
        """Record the question, then answer it exactly as scripted."""
        with self._lock:
            self._live += 1
            self.overlaps.append(self._live)
        try:
            self.profiles.append(profile)
            self.seen.append({arm: tuple(value) for arm, value in scene.items()})
            if self.triple is not None:
                return (None,) + tuple(self.triple)
            if not self.available:
                return None, workspace.NOTE_PACKAGE_ABSENT, 'checker_absent'
            if self.result is not None:
                return self.result, None, None
            return CheckResult(ok=True, min_clearance=0.041), None, None
        finally:
            with self._lock:
                self._live -= 1


def collision(contact, min_clearance=-0.012):
    """Return a CheckResult carrying exactly one contact."""
    return CheckResult(ok=False, min_clearance=min_clearance,
                       contacts=(contact,), sample_index=0)


@dataclass
class FakeCellModel:
    """
    A stand-in for a loaded cell model, for the REAL checker to hold.

    The wrapper's job is to turn a model that raises into a sentence rather
    than an exception, and that path can only be exercised by a model that
    raises on demand -- which no real cell file can be persuaded to do.
    """

    arms: tuple = ('panda1', 'panda2')
    result: object = None
    raises: Exception = None
    volume: object = None
    #: When set, every call waits here for the others. A wrapper that
    #: serialised its callers would hang on it, which is the point.
    barrier: object = None
    seen: list = field(default_factory=list)

    def __getattr__(self, name):
        """Expose ``allowed_volume`` only when this model has one."""
        if name == 'allowed_volume' and self.__dict__.get('volume') is not None:
            return lambda: self.__dict__['volume']
        raise AttributeError(name)

    def arm_ids(self):
        """Return the arms this model describes."""
        return tuple(self.arms)

    def model_identity(self):
        """Return the identity triple every result carries."""
        return (MODEL['model_id'], MODEL['model_revision'],
                MODEL['model_sha256'])

    def check_configuration(self, q, *, first_violation=False):
        """Record the question and answer it, or raise as scripted."""
        self.seen.append({arm: tuple(value) for arm, value in q.items()})
        if self.barrier is not None:
            self.barrier.wait(timeout=20)
        if self.raises is not None:
            raise self.raises
        return self.result or CheckResult(ok=True, min_clearance=0.041)


def checker_holding(model, **kwargs):
    """Return a real WorkspaceChecker that has already "loaded" ``model``."""
    checker = workspace.WorkspaceChecker(**kwargs)
    for profile in ('dual', 'single'):
        checker._models[profile] = model      # noqa: SLF001 - the load seam
    return checker
