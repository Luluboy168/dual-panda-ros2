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

"""Bounded, fixed-topic rosbag recording into a pinned private directory."""

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
from typing import Any


ALLOWED_TOPICS = (
    '/controller_manager/activity',
    '/controller_manager/introspection_data',
    '/controller_manager/statistics',
    '/diagnostics',
    '/franka/joint_states',
    '/franka_panda1_robot_state_broadcaster/robot_state',
    '/franka_panda2_robot_state_broadcaster/robot_state',
)
MINIMUM_DURATION_SECONDS = 1
MAXIMUM_DURATION_SECONDS = 3600
SIGINT_FLUSH_TIMEOUT_SECONDS = 10
TERMINATE_TIMEOUT_SECONDS = 5
KILL_TIMEOUT_SECONDS = 5
_SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW


class RecorderError(RuntimeError):
    """A bounded recording request failed validation or shutdown."""


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


def recorder_argv(bag_path: str) -> tuple[str, ...]:
    return (
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--output', bag_path,
        '--disable-keyboard-controls',
        '--topics', *ALLOWED_TOPICS,
    )


def dry_run_plan(output_root: Path, name: str, duration: int) -> dict[str, Any]:
    _validate_duration(duration)
    _validate_name(name)
    descriptor = _open_directory_no_symlinks(output_root)
    os.close(descriptor)
    return {
        'argv': list(recorder_argv('/proc/self/fd/<session-fd>/bag')),
        'dry_run': True,
        'duration_seconds': duration,
        'name': name,
        'ok': True,
        'pass_fds': ['<session-fd>'],
        'topics': list(ALLOWED_TOPICS),
    }


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
) -> dict[str, Any]:
    _validate_duration(duration)
    session = create_pinned_session(output_root, name)
    process = None
    return_code = None
    escalation = 'none'
    original_error = None
    cleanup_error = None
    try:
        argv = recorder_argv(session.bag_path)
        try:
            process = process_factory(
                argv,
                shell=False,
                pass_fds=(session.descriptor,),
            )
        except OSError as error:
            raise RecorderError('unable to start ros2 bag record') from error

        try:
            return_code = process.wait(timeout=duration)
        except subprocess.TimeoutExpired:
            pass
        except BaseException as error:
            original_error = error
    finally:
        if process is not None and return_code is None:
            try:
                return_code, escalation = _bounded_stop_and_reap(process)
            except BaseException as error:
                cleanup_error = error
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
    return {
        'dry_run': False,
        'duration_seconds': duration,
        'name': name,
        'ok': True,
        'shutdown': escalation,
        'topics': list(ALLOWED_TOPICS),
    }


def _json_line(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Record the fixed dual-Panda operator topic set')
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--name', required=True)
    parser.add_argument('--duration', required=True, type=int)
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args(argv)
    try:
        if arguments.dry_run:
            result = dry_run_plan(arguments.output_root, arguments.name, arguments.duration)
        else:
            result = run_recording(arguments.output_root, arguments.name, arguments.duration)
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
