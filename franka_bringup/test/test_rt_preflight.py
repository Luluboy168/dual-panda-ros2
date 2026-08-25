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
"""Unit tests for the read-only host preflight command."""

import importlib.util
from pathlib import Path
import resource
import subprocess


_SCRIPT = Path(__file__).parents[1] / 'scripts' / 'franka_rt_preflight.py'
_SPEC = importlib.util.spec_from_file_location('franka_rt_preflight', _SCRIPT)
preflight = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(preflight)


def test_kernel_check_uses_active_realtime_evidence(monkeypatch):
    monkeypatch.setattr(preflight, '_read_text', lambda path: '1')
    result = preflight.check_realtime_kernel()
    assert result['status'] == 'pass'


def test_limits_fail_when_fifo_priority_is_too_low(monkeypatch):
    def fake_getrlimit(limit):
        if limit == resource.RLIMIT_RTPRIO:
            return (0, 0)
        return (resource.RLIM_INFINITY, resource.RLIM_INFINITY)

    monkeypatch.setattr(resource, 'getrlimit', fake_getrlimit)
    results = preflight.check_limits(50)
    assert results[0]['status'] == 'fail'
    assert results[1]['status'] == 'pass'


def test_limits_accept_unlimited_fifo_priority(monkeypatch):
    monkeypatch.setattr(
        resource,
        'getrlimit',
        lambda limit: (resource.RLIM_INFINITY, resource.RLIM_INFINITY),
    )
    results = preflight.check_limits(50)
    assert results[0]['status'] == 'pass'
    assert results[0]['evidence'] == 'soft=unlimited hard=unlimited'


def test_fifo_probe_reports_permission_error(monkeypatch):
    completed = subprocess.CompletedProcess(
        args=[], returncode=1, stdout='', stderr='PermissionError: denied')
    monkeypatch.setattr(subprocess, 'run', lambda *args, **kwargs: completed)
    result = preflight.check_fifo_capability(50)
    assert result['status'] == 'fail'
    assert 'denied' in result['evidence']


def test_fifo_probe_reports_launch_error(monkeypatch):
    def raise_oserror(*args, **kwargs):
        raise OSError('cannot execute probe')

    monkeypatch.setattr(subprocess, 'run', raise_oserror)
    result = preflight.check_fifo_capability(50)
    assert result['status'] == 'fail'
    assert 'cannot execute probe' in result['evidence']


def test_fifo_probe_reports_empty_abnormal_exit(monkeypatch):
    completed = subprocess.CompletedProcess(args=[], returncode=9, stdout='', stderr='')
    monkeypatch.setattr(subprocess, 'run', lambda *args, **kwargs: completed)
    result = preflight.check_fifo_capability(50)
    assert result['status'] == 'fail'
    assert 'exited 9' in result['evidence']


def test_run_preflight_propagates_worst_status(monkeypatch):
    monkeypatch.setattr(
        preflight,
        'check_realtime_kernel',
        lambda: preflight._result('kernel', 'pass', 'ok'),
    )
    monkeypatch.setattr(
        preflight,
        'check_limits',
        lambda priority: [preflight._result('limits', 'warn', 'finite')],
    )
    monkeypatch.setattr(
        preflight,
        'check_fifo_capability',
        lambda priority: preflight._result('fifo', 'fail', 'denied'),
    )
    monkeypatch.setattr(preflight, 'collect_versions', lambda franka_dir: {})
    monkeypatch.setattr(preflight, 'check_build_environment', lambda versions: [])
    report = preflight.run_preflight(50, None)
    assert report['overall'] == 'fail'
    assert report['read_only'] is True
    assert report['network_access'] is False


def test_missing_build_environment_is_a_failure():
    missing = 'not found in the sourced environment'
    versions = {
        'ros': {'distro': 'not sourced', 'prefix': 'not sourced'},
        'ros2_control': {'version': missing, 'prefix': missing},
        'compiler': {'version': 'unavailable', 'path': 'not found'},
        'cmake': {'version': 'unavailable', 'path': 'not found'},
        'libfranka': {'version': 'Franka_DIR not supplied', 'cmake_dir': 'not supplied'},
        'mvp_packages': {
            name: {'version': missing, 'prefix': missing}
            for name in preflight.MVP_PACKAGES
        },
    }
    checks = preflight.check_build_environment(versions)
    assert checks
    assert all(check['status'] == 'fail' for check in checks)
