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
The pre-start host gate: running franka_rt_preflight and reading its report.

``franka_rt_preflight`` is the read-only host check a session passes through
before it leaves the ``preflight`` state (plan section 3.4): PREEMPT_RT
kernel, RT and memlock limits, a real ``SCHED_FIFO`` probe, and the build
environment. This module is the only place that knows how to invoke it and
how to read what it prints.

Two rules shape everything here.

*Blocking versus warning* (plan section 3.4). A non-PASS verdict stops a
``watch`` or ``motion`` session -- those two drive real hardware -- and is
only a warning in ``simulate``, where nothing is connected and a host without
a real-time kernel is a perfectly good place to try the interface out.
:meth:`PreflightResult.blocks_start` is that rule, and it is the single
question the supervisor asks.

*The gate never throws.* A clean FAIL, a malformed report, a missing binary
and a hung run all have to end in a verdict the supervisor can act on: an
exception escaping this module would abandon the state machine mid-transition
with children already spawned. Every failure path therefore returns
``overall='ERROR', passed=False`` with a short reason in ``error``, and
:func:`run_preflight` keeps a catch-all so that stays true even for a failure
mode nobody anticipated.

Case: the tool emits its statuses in lower case (``pass``/``warn``/``fail``,
with ``overall`` the worst of them) and upper-cases them itself when it prints
its human-readable verdict. This module does the same, so ``PASS``/``FAIL``
are what reach the API. Only ``PASS`` is a pass -- a ``WARN`` verdict is a
non-PASS verdict and therefore blocks a production session, and every non-PASS
check is carried in ``failed_checks`` so the operator sees which one it was.

The tool's exit status is deliberately ignored: it exits 2 on ``fail``, and
the report on stdout is the authority. A run that dies without printing a
report fails JSON parsing and lands on the ``ERROR`` path anyway.

Nothing in this module handles a robot address, and no message it produces
contains one (nor a captured stderr, which is not echoed at all).
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
import subprocess

from ament_index_python.packages import get_package_prefix

from franka_web import config

#: The package that installs the preflight binary.
_TOOL_PACKAGE = 'franka_bringup'
_TOOL_NAME = 'franka_rt_preflight'

#: Modes that drive real hardware; in these a non-PASS preflight blocks.
_BLOCKING_MODES = ('watch', 'motion')

_PASS = 'PASS'
_ERROR = 'ERROR'


class _ReportError(ValueError):
    """A preflight report could not be understood; the message is operator-safe."""


@dataclass(frozen=True)
class PreflightResult:
    """
    One ``franka_rt_preflight`` run, reduced to the verdict the gate needs.

    ``overall`` is the tool's verdict upper-cased (``PASS``/``WARN``/``FAIL``)
    or ``ERROR`` when the run itself could not be made or understood, in which
    case ``error`` carries a short reason and ``failed_checks`` is empty --
    there were no checks to report. ``blocking`` records the mode rule rather
    than the outcome, so a warning-only FAIL in ``simulate`` is still fully
    visible in the API frame while :meth:`blocks_start` stays ``False``.
    """

    overall: str
    passed: bool
    blocking: bool
    failed_checks: list
    ran_at: str
    error: str | None = None

    def blocks_start(self) -> bool:
        """
        Return True when this result must stop the session from starting.

        Only a FAIL verdict (or an ERROR: the run itself could not be made,
        so the host is unproven) blocks a production mode. A WARN is
        reported in the frame but does not block: the tool itself exits 0
        on warn, and a real host commonly warns (e.g. memlock not
        unlimited) while remaining runnable -- plan section 3.4 stops only
        on FAIL.
        """
        return self.blocking and self.overall not in ('PASS', 'WARN')

    def frame(self) -> dict:
        """
        Return this result as the state frame's preflight block (section 6.11).

        The checks are copied out, so a consumer serializing or annotating the
        frame cannot reach back into this frozen result.
        """
        return {
            'ran_at': self.ran_at,
            'overall': self.overall,
            'blocking': self.blocking,
            'failed_checks': [dict(check) for check in self.failed_checks],
        }


def preflight_binary() -> str:
    """Return the absolute path of the installed ``franka_rt_preflight``."""
    return os.path.join(get_package_prefix(_TOOL_PACKAGE), 'lib', _TOOL_PACKAGE, _TOOL_NAME)


def build_argv(settings) -> tuple:
    """
    Return the exact argv for one preflight run.

    ``--priority`` is left off on purpose: the tool's own default (50) is the
    reviewed value, and duplicating it here would be a second place to keep in
    step with it. ``--franka-dir`` is passed only when the operator configured
    one, so the tool falls back to its ``Franka_DIR`` default otherwise.
    """
    argv = (preflight_binary(), '--json')
    if settings.franka_dir:
        argv += ('--franka-dir', settings.franka_dir)
    return argv


def run_preflight(settings, mode, runner=subprocess.run, timeout_s=config.PREFLIGHT_TIMEOUT_S,
                  now=None):
    """
    Run the host preflight once and return its verdict; never raises.

    ``runner`` matches :func:`subprocess.run` and is the seam tests replace.
    ``now`` is an injectable clock returning an aware ``datetime`` (a plain
    ``datetime`` is accepted too); it is read before the run, so ``ran_at`` is
    when the check started. ``mode`` decides only whether the verdict blocks.
    """
    blocking = mode in _BLOCKING_MODES
    try:
        ran_at = _rfc3339(_now_utc(now))
    except Exception:
        # An injected clock must never be able to break the gate.
        ran_at = _rfc3339(datetime.now(timezone.utc))

    try:
        argv = build_argv(settings)
    except Exception:
        return _error_result(
            blocking, ran_at,
            '{} was not found (is {} installed and sourced?)'.format(_TOOL_NAME, _TOOL_PACKAGE))

    try:
        completed = runner(argv, capture_output=True, timeout=timeout_s, text=True)
    except subprocess.TimeoutExpired:
        return _error_result(
            blocking, ran_at,
            '{} did not finish within {:g} s'.format(_TOOL_NAME, timeout_s))
    except OSError as error:
        return _error_result(
            blocking, ran_at,
            '{} could not be executed ({})'.format(_TOOL_NAME, type(error).__name__))
    except Exception as error:
        return _error_result(
            blocking, ran_at,
            '{} could not be run ({})'.format(_TOOL_NAME, type(error).__name__))

    try:
        overall, failed_checks = _parse_report(getattr(completed, 'stdout', None))
    except _ReportError as error:
        return _error_result(blocking, ran_at, str(error))
    except Exception:
        return _error_result(
            blocking, ran_at, '{} produced an unusable report'.format(_TOOL_NAME))

    return PreflightResult(
        overall=overall,
        passed=overall == _PASS,
        blocking=blocking,
        failed_checks=failed_checks,
        ran_at=ran_at,
        error=None,
    )


def _error_result(blocking, ran_at, reason):
    """Build the ERROR verdict returned for any invocation-level failure."""
    return PreflightResult(
        overall=_ERROR,
        passed=False,
        blocking=blocking,
        failed_checks=[],
        ran_at=ran_at,
        error=reason,
    )


def _parse_report(stdout):
    """
    Return ``(overall, failed_checks)`` from the tool's JSON on stdout.

    Raises :class:`_ReportError` -- whose message is safe to show an operator
    -- for anything that is not a report with a string ``overall`` and a list
    of checks that each carry a string ``status``.
    """
    if not isinstance(stdout, str):
        raise _ReportError('{} produced no readable output'.format(_TOOL_NAME))
    try:
        report = json.loads(stdout)
    except ValueError:
        raise _ReportError('{} did not produce valid JSON'.format(_TOOL_NAME)) from None
    if not isinstance(report, dict):
        raise _ReportError('the {} report is not a JSON object'.format(_TOOL_NAME))

    overall = report.get('overall')
    if not isinstance(overall, str) or not overall.strip():
        raise _ReportError("the {} report has no 'overall' verdict".format(_TOOL_NAME))
    checks = report.get('checks')
    if not isinstance(checks, list):
        raise _ReportError("the {} report has no 'checks' list".format(_TOOL_NAME))

    failed_checks = []
    for check in checks:
        if not isinstance(check, dict):
            raise _ReportError('a {} check is not a JSON object'.format(_TOOL_NAME))
        status = check.get('status')
        if not isinstance(status, str) or not status.strip():
            raise _ReportError("a {} check has no 'status'".format(_TOOL_NAME))
        status = status.strip().upper()
        if status == _PASS:
            continue
        failed_checks.append({
            'status': status,
            'name': _text(check.get('name')),
            'summary': _text(check.get('summary')),
            'evidence': _text(check.get('evidence')),
        })
    return overall.strip().upper(), failed_checks


def _text(value):
    """Return ``value`` if it is a string, else '' (the field is display-only)."""
    return value if isinstance(value, str) else ''


def _now_utc(now):
    """Return an aware UTC datetime from the injected clock, or the wall clock."""
    if now is None:
        return datetime.now(timezone.utc)
    moment = now() if callable(now) else now
    if moment.tzinfo is None:
        return moment.replace(tzinfo=timezone.utc)
    return moment.astimezone(timezone.utc)


def _rfc3339(moment):
    """Format an aware UTC datetime as RFC 3339 with microseconds and a Z."""
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'
