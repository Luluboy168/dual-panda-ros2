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
Bounded, fixed-topic rosbag recording into a pinned private directory.

Shutdown contract (changed 2026-08-28, finding F-10i; amended 2026-08-29, verify findings 8-12)
----------------------------------------------------------------------------------------------
A recording ends in one of three CLEAN ways, and in all three the ``ros2 bag record`` child is
stopped through the same bounded SIGINT -> SIGTERM -> SIGKILL escalation, reaped, and its bag
sealed:

* ``--duration`` expires             -> exit 0 (unchanged);
* SIGINT or SIGTERM reaches this process -> exit 0, with ``"stopped_by"`` naming the signal;
* an internal failure                -> the pre-existing non-zero exit codes (2 request, 3
  operation), unchanged.

There is a FOURTH way, and it is not clean: see "SIGKILL" below.

EXIT-CODE CHANGE: before this fix, SIGINT reached the operator as exit 3 with
``{"error":"recording operation failed"}`` on stderr even though the bag had in fact been
sealed, and SIGTERM was worse than that -- the default disposition killed this process outright,
the ``finally:`` cleanup never ran, and the ``ros2 bag record`` child was ORPHANED. That actually
happened: three orphans survived Phase 10 and cross-captured later sessions' traffic into sealed
bags. A clean signal stop is now a SUCCESS: exit 0 and a normal result object on stdout. Callers
that treated a signal stop as a failure must be updated; callers that check ``ok``/exit 0 need no
change, and a genuinely failed recording still exits non-zero.

Ctrl-C, and why the child gets its own session
----------------------------------------------
A terminal Ctrl-C signals the whole foreground PROCESS GROUP, not just this process. While the
``ros2 bag record`` child shared this process's group it therefore received the operator's SIGINT
directly, exited with the ``ros2`` CLI's own KeyboardInterrupt status 2, and ``run_recording``
reaped that 2 and reported ``{"error":"recording request failed","ok":false}`` with exit 2 -- on a
bag that was fully sealed and with no orphan anywhere. Exit 0 held for a SIGTERM sent to this PID
but not for the gesture operators actually use (verified 2026-08-29, tools-verify finding 8).

The child is therefore started with ``start_new_session=True``: it runs in its own session and
process group, the terminal's signal reaches only this process, and the single bounded
stop-and-reap path below is what stops the recorder -- the same path a SIGTERM already took. The
parent-death guard is unaffected: CPython's child_exec calls ``setsid()`` BEFORE ``preexec_fn``,
so ``PR_SET_PDEATHSIG`` is armed after the session change and is not cleared by it (proved
end to end by ``test_sigkill_of_the_recorder_leaves_no_orphan_and_an_unsealed_bag``).

SIGKILL of this process: NO SEAL, and that is unavoidable
---------------------------------------------------------
``PR_SET_PDEATHSIG=SIGKILL`` on the child means even a SIGKILL of this process (which no handler
can intercept) cannot leave a recorder running. It covers the ORPHAN half only. It cannot seal:
the child is killed outright, so the bag directory is left holding its ``bag_0.mcap`` with **no
``metadata.yaml``**, and it must be repaired with ``ros2 bag reindex <bag-dir>`` before anything
can read it. Do not SIGKILL ``franka_record`` to stop a recording -- send SIGTERM (or SIGINT) to
its PID and let it seal.

Stopping a recording started through ``ros2 run``
-------------------------------------------------
Signal the ``franka_record`` PID, never the ``ros2 run franka_bringup franka_record`` wrapper.
PR_SET_PDEATHSIG binds the bag recorder to ``franka_record``, not to that wrapper, so killing the
wrapper leaves BOTH processes running -- the exact orphan class this module exists to prevent
(tools-verify finding 10; upstream ``ros2 run`` behaviour, not something this module can fix).

``duration_seconds`` in the result is always the REQUESTED bound
----------------------------------------------------------------
It is the ``--duration`` argument, not the elapsed time. On a signal stop the recording ends early
and ``stopped_by`` is what says so; anything computing bag coverage must take the span from the
bag itself, not from this field (tools-verify finding 12).
"""

import argparse
import ctypes
from dataclasses import dataclass
import functools
import json
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
from typing import Any


# ros2_control publishes its introspection and loop statistics through pal_statistics, which
# splits every registry into a `<base>/names` (StatisticsNames, transient-local, republished only
# when the registered key set changes) and a `<base>/values` (StatisticsValues, one sample per
# control cycle) pair, plus a `<base>/full` (Statistics) that repeats the names on every sample.
# The bare `<base>` names this allowlist used before 2026-08-28 are NOT topics and never were:
# `ros2 bag record` accepted them, waited forever for a publisher that cannot exist, and recorded
# nothing -- verified against every Phase 9 and Phase 10 sealed bag, none of which contains a
# `/controller_manager/introspection_data` or `/controller_manager/statistics` entry, and against
# a live fake_dual_state_only bringup where only the /full, /names and /values children resolve.
# names+values is recorded rather than full: the pair is self-describing when both are present and
# measured 1.10 MB/s + 426 KB/s on a 1 kHz dual bringup, against 7.79 + 3.65 MB/s for the two
# /full topics (evidence: test_logs/phase11_offline_prep_2026-08-28/
# rehearsal_introspection_bandwidth.log).
DUAL_ALLOWED_TOPICS = (
    '/controller_manager/activity',
    '/controller_manager/introspection_data/names',
    '/controller_manager/introspection_data/values',
    '/controller_manager/statistics/names',
    '/controller_manager/statistics/values',
    '/diagnostics',
    '/franka/joint_states',
    '/franka_panda1_robot_state_broadcaster/robot_state',
    '/franka_panda2_robot_state_broadcaster/robot_state',
)
# One-arm-mode bringup (launch/real/one_arm_franka.launch.py, see
# config/real/one_arm_controllers.yaml) registers the two broadcasters under fixed, unprefixed
# instance names -- the arm ID is a launch-time argument, not known when that config is written --
# unlike the dual stack's "franka_panda<N>_..." naming. Confirmed against a live
# fake_single_state_only bringup (arm_id:=panda2).
SINGLE_ALLOWED_TOPICS = (
    '/controller_manager/activity',
    '/controller_manager/introspection_data/names',
    '/controller_manager/introspection_data/values',
    '/controller_manager/statistics/names',
    '/controller_manager/statistics/values',
    '/diagnostics',
    '/franka/joint_states',
    '/franka_robot_state_broadcaster/robot_state',
)
ARM_MODE_TOPICS = {
    'dual': DUAL_ALLOWED_TOPICS,
    'single': SINGLE_ALLOWED_TOPICS,
}
# Preserved as the byte-identical default (dual) topic set for existing callers.
ALLOWED_TOPICS = DUAL_ALLOWED_TOPICS
MINIMUM_DURATION_SECONDS = 1
MAXIMUM_DURATION_SECONDS = 3600
SIGINT_FLUSH_TIMEOUT_SECONDS = 10
TERMINATE_TIMEOUT_SECONDS = 5
KILL_TIMEOUT_SECONDS = 5
# The operator stop signals that must seal the bag instead of orphaning the child (F-10i).
STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM)
_SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW
# <linux/prctl.h>: PR_SET_PDEATHSIG. Asking the kernel to signal the child when this process dies
# is the only defence that survives a SIGKILL of this process, which no handler can intercept.
_PR_SET_PDEATHSIG = 1
_PARENT_DEATH_SIGNAL = signal.SIGKILL
_ORPHANED_CHILD_EXIT_STATUS = 127


class RecorderError(RuntimeError):
    """A bounded recording request failed validation or shutdown."""


class _StopSignal(BaseException):
    """
    Internal: an operator stop signal arrived while the recorder was waiting on its child.

    Derived from BaseException, not Exception, for the same reason KeyboardInterrupt is: it is a
    control-flow signal, and no ``except Exception`` in this module or in a caller should quietly
    absorb it and leave the child running.
    """

    def __init__(self, signal_number):
        super().__init__(signal_number)
        self.signal_number = signal_number


def _stop_signal_label(signal_number):
    try:
        return signal.Signals(signal_number).name.lower()
    except ValueError:
        return 'signal_{}'.format(int(signal_number))


class _StopSignalWatch:
    """
    Turn SIGINT/SIGTERM into a bounded, sealing shutdown instead of an orphaning kill.

    The handler is installed for the whole lifetime of the child, but only RAISES while armed --
    that is, while this process is doing nothing but waiting for the child to finish. Outside
    that window (during the bounded stop-and-reap, and during descriptor cleanup) an arriving
    signal is recorded in ``signal_number`` and deferred, so that the sealing path always runs to
    completion.

    A deferred signal is NOT necessarily reported. ``run_recording`` reports ``stopped_by`` only
    for a signal that actually caused the stop: one that arrived while the watch was armed, or
    before it was armed, or during the cleanup of a child that was still running. A signal landing
    after ``--duration`` has already expired arrives too late to have caused anything, so it is
    recorded here and then discarded -- see
    ``test_a_stop_signal_arriving_during_cleanup_is_deferred_and_still_reaps_the_child``
    (corrected 2026-08-29, tools-verify finding 12).

    If the handlers cannot be installed (``signal.signal`` only works on the main thread, and
    this module is importable as a library), the watch degrades to a no-op and the recorder keeps
    exactly its previous behaviour rather than failing the recording.
    """

    def __init__(self, signal_numbers=STOP_SIGNALS):
        self._signal_numbers = tuple(signal_numbers)
        self._previous_handlers = {}
        self._raising = False
        self.installed = False
        self.signal_number = None

    def _handle(self, signal_number, _frame):
        if self.signal_number is None:
            self.signal_number = signal_number
        if self._raising:
            # Disarm before raising: a second signal must not raise out of the cleanup path.
            self._raising = False
            raise _StopSignal(signal_number)

    def __enter__(self):
        try:
            for number in self._signal_numbers:
                self._previous_handlers[number] = signal.signal(number, self._handle)
            self.installed = True
        except (OSError, RuntimeError, ValueError):
            self._restore()
        return self

    def arm(self):
        self._raising = self.installed

    def disarm(self):
        self._raising = False

    def _restore(self):
        self._raising = False
        self.installed = False
        while self._previous_handlers:
            number, previous = self._previous_handlers.popitem()
            try:
                signal.signal(number, signal.SIG_DFL if previous is None else previous)
            except (OSError, RuntimeError, ValueError):
                pass

    def __exit__(self, _type, _value, _traceback):
        self._restore()
        return False


def _load_prctl():
    """
    Resolve prctl(2) in THIS process, before any fork.

    Doing the dynamic-library work here rather than in the forked child keeps the post-fork,
    pre-exec code path down to two already-resolved calls.
    """
    try:
        prctl = ctypes.CDLL(None, use_errno=True).prctl
    except (AttributeError, OSError):
        return None
    prctl.restype = ctypes.c_int
    return prctl


_PRCTL = _load_prctl()


def _set_parent_death_signal(parent_pid, prctl=None):
    """Run in the forked child before exec: never outlive the recorder that started us."""
    if prctl is None:
        prctl = _PRCTL
    if prctl is not None:
        prctl(_PR_SET_PDEATHSIG, int(_PARENT_DEATH_SIGNAL), 0, 0, 0)
    # PR_SET_PDEATHSIG is armed at the moment of the call, so a parent that died between the
    # fork and this line would never trigger it. Re-check and exit instead of being reparented.
    if os.getppid() != parent_pid:
        os._exit(_ORPHANED_CHILD_EXIT_STATUS)


@dataclass
class PinnedRecordingSession:
    descriptor: int
    name: str

    @property
    def bag_path(self):
        return '/proc/self/fd/{}/bag'.format(self.descriptor)

    def close(self):
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1


def _open_directory_no_symlinks(path: Path) -> int:
    if not path.is_absolute() or Path(os.path.normpath(str(path))) != path:
        raise RecorderError('output root must be a normalized absolute path')
    descriptor = os.open('/', _DIRECTORY_FLAGS)
    try:
        for component in path.parts[1:]:
            next_descriptor = os.open(component, _DIRECTORY_FLAGS, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = next_descriptor
        details = os.fstat(descriptor)
        if (not stat.S_ISDIR(details.st_mode) or details.st_uid != os.geteuid() or
                stat.S_IMODE(details.st_mode) & 0o077):
            raise RecorderError(
                'output root must be an existing private directory owned by this user')
        return descriptor
    except (OSError, RecorderError) as error:
        os.close(descriptor)
        if isinstance(error, RecorderError):
            raise
        raise RecorderError('output root must contain no symlink component') from error


def _validate_name(name: str) -> None:
    if not _SAFE_NAME.fullmatch(name):
        raise RecorderError(
            'recording name must be one safe 1..64 character path component')


def _validate_duration(duration: int) -> None:
    if (isinstance(duration, bool) or not isinstance(duration, int) or
            not MINIMUM_DURATION_SECONDS <= duration <= MAXIMUM_DURATION_SECONDS):
        raise RecorderError('duration must be an integer from 1 through 3600')


def create_pinned_session(output_root: Path, name: str) -> PinnedRecordingSession:
    _validate_name(name)
    root_descriptor = _open_directory_no_symlinks(output_root)
    try:
        os.mkdir(name, mode=0o700, dir_fd=root_descriptor)
        session_descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=root_descriptor)
        os.fchmod(session_descriptor, 0o700)
        details = os.fstat(session_descriptor)
        if not stat.S_ISDIR(details.st_mode) or stat.S_IMODE(details.st_mode) != 0o700:
            os.close(session_descriptor)
            raise RecorderError('recording session directory is not private mode 0700')
        try:
            os.stat('bag', dir_fd=session_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            os.close(session_descriptor)
            raise RecorderError('recording output already exists')
        return PinnedRecordingSession(session_descriptor, name)
    except OSError as error:
        raise RecorderError('recording session must not already exist') from error
    finally:
        os.close(root_descriptor)


def _validate_arm_mode(arm_mode: str) -> tuple[str, ...]:
    try:
        return ARM_MODE_TOPICS[arm_mode]
    except (KeyError, TypeError) as error:
        raise RecorderError("arm mode must be one of: {}".format(
            ', '.join(sorted(ARM_MODE_TOPICS)))) from error


def recorder_argv(bag_path: str, arm_mode: str = 'dual') -> tuple[str, ...]:
    return (
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--output', bag_path,
        '--disable-keyboard-controls',
        '--topics', *_validate_arm_mode(arm_mode),
    )


def dry_run_plan(
        output_root: Path, name: str, duration: int, arm_mode: str = 'dual') -> dict[str, Any]:
    _validate_duration(duration)
    _validate_name(name)
    topics = _validate_arm_mode(arm_mode)
    descriptor = _open_directory_no_symlinks(output_root)
    os.close(descriptor)
    plan = {
        'argv': list(recorder_argv('/proc/self/fd/<session-fd>/bag', arm_mode)),
        'dry_run': True,
        'duration_seconds': duration,
        'name': name,
        'ok': True,
        'pass_fds': ['<session-fd>'],
        'topics': list(topics),
    }
    # arm_mode appears only for non-default modes so default dual output stays byte-identical
    # to the pre-single-arm contract (F-9d review finding).
    if arm_mode != 'dual':
        plan['arm_mode'] = arm_mode
    return plan


def _bounded_stop_and_reap(process):
    escalation = 'none'
    return_code = None
    first_cleanup_error = None
    stages = (
        ('sigint', lambda: process.send_signal(signal.SIGINT), SIGINT_FLUSH_TIMEOUT_SECONDS),
        ('terminate', process.terminate, TERMINATE_TIMEOUT_SECONDS),
        ('kill', process.kill, KILL_TIMEOUT_SECONDS),
        ('kill', process.kill, KILL_TIMEOUT_SECONDS),
    )
    for stage, send, timeout in stages:
        escalation = stage
        try:
            send()
        except BaseException as error:
            if first_cleanup_error is None:
                first_cleanup_error = error
        try:
            return_code = process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
        except BaseException as error:
            if first_cleanup_error is None:
                first_cleanup_error = error

    if return_code is None:
        failure = RecorderError('recorder did not exit after the final bounded kill and reap')
        if first_cleanup_error is not None:
            first_cleanup_error.add_note(str(failure))
            raise first_cleanup_error
        raise failure
    if first_cleanup_error is not None:
        raise first_cleanup_error
    return return_code, escalation


def run_recording(
        output_root: Path, name: str, duration: int, process_factory=subprocess.Popen,
        arm_mode: str = 'dual',
) -> dict[str, Any]:
    _validate_duration(duration)
    topics = _validate_arm_mode(arm_mode)
    session = create_pinned_session(output_root, name)
    process = None
    return_code = None
    escalation = 'none'
    original_error = None
    cleanup_error = None
    stop_signal_number = None
    try:
        # The watch spans the child's whole lifetime, including the bounded stop-and-reap in the
        # inner finally: a stop signal arriving mid-cleanup must be deferred, never allowed to
        # kill this process and orphan the child it is in the middle of stopping.
        with _StopSignalWatch() as watch:
            try:
                argv = recorder_argv(session.bag_path, arm_mode)
                try:
                    # preexec_fn runs in the forked child before exec. It carries CPython's
                    # documented constraints (unsafe if the parent is multi-threaded, rejected
                    # in subinterpreters); franka_record is a single-threaded CLI, which is the
                    # supported way to run a recording.
                    process = process_factory(
                        argv,
                        shell=False,
                        pass_fds=(session.descriptor,),
                        # Own session/process group: a terminal Ctrl-C signals the foreground
                        # GROUP, and a child that received it directly exited 2 (the ros2 CLI's
                        # KeyboardInterrupt status), which this function then reported as a
                        # failed recording on a sealed bag. See the module docstring.
                        start_new_session=True,
                        preexec_fn=functools.partial(_set_parent_death_signal, os.getpid()),
                    )
                except OSError as error:
                    raise RecorderError('unable to start ros2 bag record') from error

                try:
                    watch.arm()
                    if watch.signal_number is not None:
                        # Delivered before the watch was armed (or before the child existed):
                        # honour it now rather than blocking for the full duration.
                        stop_signal_number = watch.signal_number
                    else:
                        return_code = process.wait(timeout=duration)
                except subprocess.TimeoutExpired:
                    pass
                except _StopSignal as stop:
                    stop_signal_number = stop.signal_number
                except BaseException as error:
                    original_error = error
                finally:
                    watch.disarm()
            finally:
                if process is not None and return_code is None:
                    try:
                        return_code, escalation = _bounded_stop_and_reap(process)
                    except BaseException as error:
                        cleanup_error = error
    except _StopSignal as stop:
        # A stop signal delivered in the few bytecodes between the wait returning and the watch
        # disarming raises out of that finally: clause. The child has already been stopped and
        # reaped by the inner finally above, so nothing leaks -- this only stops the signal from
        # escaping run_recording as an unhandled BaseException. It is reported as the cause only
        # if the child was in fact still running (escalation ran); if the recording had already
        # finished on its duration, the signal simply arrived too late to have caused anything.
        if stop_signal_number is None and escalation != 'none':
            stop_signal_number = stop.signal_number
    finally:
        try:
            session.close()
        except BaseException as error:
            if cleanup_error is None:
                cleanup_error = error
            else:
                cleanup_error.add_note(
                    'recording-session descriptor cleanup also encountered: {}'.format(error))

    if original_error is not None:
        if cleanup_error is not None:
            original_error.add_note(
                'bounded recorder cleanup also encountered: {}'.format(cleanup_error))
        raise original_error
    if cleanup_error is not None:
        raise cleanup_error
    if return_code != 0:
        raise RecorderError('ros2 bag record exited with code {}'.format(return_code))
    result = {
        'dry_run': False,
        'duration_seconds': duration,
        'name': name,
        'ok': True,
        'shutdown': escalation,
        'topics': list(topics),
    }
    if arm_mode != 'dual':
        result['arm_mode'] = arm_mode
    # stopped_by appears only for a signal-initiated stop, so a --duration recording keeps the
    # exact key set it had before F-10i. duration_seconds stays the REQUESTED bound: a signal
    # stop ends the recording early and stopped_by is what says so.
    if stop_signal_number is not None:
        result['stopped_by'] = _stop_signal_label(stop_signal_number)
    return result


def _json_line(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Record the fixed operator topic set for the given arm mode')
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--name', required=True)
    parser.add_argument('--duration', required=True, type=int)
    parser.add_argument(
        '--arm-mode', choices=sorted(ARM_MODE_TOPICS), default='dual',
        help='Fixed topic set to record: the dual-Panda set (default) or the single-Panda set')
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args(argv)
    try:
        if arguments.dry_run:
            result = dry_run_plan(
                arguments.output_root, arguments.name, arguments.duration, arguments.arm_mode)
        else:
            result = run_recording(
                arguments.output_root, arguments.name, arguments.duration,
                arm_mode=arguments.arm_mode)
    except RecorderError:
        print(_json_line({'error': 'recording request failed', 'ok': False}), file=sys.stderr)
        return 2
    except (Exception, KeyboardInterrupt):
        print(_json_line({'error': 'recording operation failed', 'ok': False}), file=sys.stderr)
        return 3
    print(_json_line(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
