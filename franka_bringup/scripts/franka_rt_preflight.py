#!/usr/bin/env python3
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
"""Read-only host preflight for the multipanda ROS 2 control stack."""

import argparse
import json
import os
from pathlib import Path
import platform
import resource
import shutil
import subprocess
import sys
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET


MVP_PACKAGES = (
    'franka_msgs',
    'franka_description',
    'franka_hardware',
    'franka_semantic_components',
    'franka_robot_state_broadcaster',
    'franka_example_controllers',
    'franka_bringup',
)
STATUS_RANK = {'pass': 0, 'warn': 1, 'fail': 2}


def _result(name: str, status: str, summary: str, evidence: str = '') -> Dict[str, str]:
    return {
        'name': name,
        'status': status,
        'summary': summary,
        'evidence': evidence,
    }


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8').strip()
    except (OSError, UnicodeError):
        return ''


def _kernel_config_has_preempt_rt(release: str) -> bool:
    config = _read_text(Path('/boot') / 'config-{}'.format(release))
    return 'CONFIG_PREEMPT_RT=y' in config.splitlines()


def check_realtime_kernel() -> Dict[str, str]:
    """Report whether the active kernel is demonstrably PREEMPT_RT."""
    uname = platform.uname()
    realtime_flag = _read_text(Path('/sys/kernel/realtime'))
    is_realtime = realtime_flag == '1' or _kernel_config_has_preempt_rt(uname.release)
    evidence = '{} {}'.format(uname.release, uname.version)
    if realtime_flag:
        evidence += '; /sys/kernel/realtime={}'.format(realtime_flag)
    if is_realtime:
        return _result('kernel', 'pass', 'active kernel has PREEMPT_RT', evidence)
    return _result('kernel', 'fail', 'active kernel is not proven PREEMPT_RT', evidence)


def _format_limit(value: int) -> str:
    if value == resource.RLIM_INFINITY:
        return 'unlimited'
    return str(value)


def check_limits(required_priority: int) -> List[Dict[str, str]]:
    """Report the effective limits of this process."""
    rt_soft, rt_hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    mem_soft, mem_hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    rt_status = (
        'pass'
        if rt_soft == resource.RLIM_INFINITY or rt_soft >= required_priority
        else 'fail'
    )
    rt_summary = (
        'effective rtprio permits FIFO {}'.format(required_priority)
        if rt_status == 'pass'
        else 'effective rtprio is below required FIFO {}'.format(required_priority)
    )
    mem_status = 'pass' if mem_soft == resource.RLIM_INFINITY else 'warn'
    mem_summary = (
        'effective memlock is unlimited'
        if mem_status == 'pass'
        else 'effective memlock is finite; verify it is sufficient under load'
    )
    return [
        _result(
            'rtprio_limit',
            rt_status,
            rt_summary,
            'soft={} hard={}'.format(_format_limit(rt_soft), _format_limit(rt_hard)),
        ),
        _result(
            'memlock_limit',
            mem_status,
            mem_summary,
            'soft={} bytes hard={} bytes'.format(
                _format_limit(mem_soft), _format_limit(mem_hard)),
        ),
    ]


def _probe_fifo_child(priority: int) -> int:
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except (OSError, PermissionError) as error:
        print('{}: {}'.format(type(error).__name__, error), file=sys.stderr)
        return 1
    return 0


def check_fifo_capability(priority: int) -> Dict[str, str]:
    """Attempt FIFO scheduling only in a short-lived child process."""
    command = [sys.executable, str(Path(__file__).resolve()), '--_probe-fifo', str(priority)]
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.SubprocessError) as error:
        return _result(
            'fifo_capability',
            'fail',
            'could not run the SCHED_FIFO {} capability probe'.format(priority),
            '{}: {}'.format(type(error).__name__, error),
        )
    if completed.returncode == 0:
        return _result(
            'fifo_capability',
            'pass',
            'current session can request SCHED_FIFO {}'.format(priority),
            completed.stderr.strip() or 'short-lived child exited successfully',
        )
    evidence = completed.stderr.strip()
    if not evidence:
        evidence = 'short-lived child exited {} without a diagnostic'.format(
            completed.returncode)
    return _result(
        'fifo_capability',
        'fail',
        'current session cannot request SCHED_FIFO {}'.format(priority),
        evidence,
    )


def _command_version(command: List[str]) -> str:
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.SubprocessError) as error:
        return 'unavailable ({})'.format(error)
    output = completed.stdout.strip() or completed.stderr.strip()
    return output.splitlines()[0] if output else 'unavailable'


def _os_release() -> str:
    values = {}
    for line in _read_text(Path('/etc/os-release')).splitlines():
        key, separator, value = line.partition('=')
        if separator:
            values[key] = value.strip('"')
    return values.get('PRETTY_NAME', 'unknown')


def _package_info(package_name: str) -> Dict[str, str]:
    try:
        from ament_index_python.packages import (
            get_package_prefix,
            get_package_share_directory,
        )

        package_xml = Path(get_package_share_directory(package_name)) / 'package.xml'
        version = ET.parse(package_xml).getroot().findtext('version')
        return {
            'version': version or 'unknown',
            'prefix': get_package_prefix(package_name),
        }
    except (ImportError, ET.ParseError, OSError, LookupError):
        return {
            'version': 'not found in the sourced environment',
            'prefix': 'not found in the sourced environment',
        }


def _franka_version(franka_dir: Optional[str]) -> Tuple[str, str]:
    if not franka_dir:
        return 'Franka_DIR not supplied', 'not supplied'
    directory = Path(franka_dir).expanduser().resolve()
    candidates = (
        directory / 'FrankaConfigVersion.cmake',
        directory / 'franka-config-version.cmake',
    )
    for candidate in candidates:
        for line in _read_text(candidate).splitlines():
            if line.strip().startswith('set(PACKAGE_VERSION'):
                version = line.split('"', 2)
                return version[1] if len(version) > 1 else 'unknown', str(directory)
    return 'version not found', str(directory)


def collect_versions(franka_dir: Optional[str]) -> Dict[str, object]:
    """Collect build/runtime versions without accessing the network."""
    libfranka_version, libfranka_location = _franka_version(franka_dir)
    return {
        'ubuntu': _os_release(),
        'kernel': platform.release(),
        'ros': {
            'distro': os.environ.get('ROS_DISTRO', 'not sourced'),
            'prefix': os.environ.get('AMENT_PREFIX_PATH', 'not sourced').split(':')[-1],
        },
        'ros2_control': _package_info('controller_manager'),
        'compiler': {
            'version': _command_version(['c++', '--version']),
            'path': shutil.which('c++') or 'not found',
        },
        'cmake': {
            'version': _command_version(['cmake', '--version']),
            'path': shutil.which('cmake') or 'not found',
        },
        'libfranka': {
            'version': libfranka_version,
            'cmake_dir': libfranka_location,
        },
        'mvp_packages': {name: _package_info(name) for name in MVP_PACKAGES},
    }


def check_build_environment(versions: Dict[str, object]) -> List[Dict[str, str]]:
    """Turn required build-version facts into pass/warn/fail checks."""
    checks = []
    ros = versions['ros']
    ros_status = 'pass' if ros['distro'] == 'jazzy' else 'fail'
    checks.append(_result(
        'ros_environment',
        ros_status,
        'ROS 2 Jazzy is sourced' if ros_status == 'pass' else 'ROS 2 Jazzy is not sourced',
        'distro={} prefix={}'.format(ros['distro'], ros['prefix']),
    ))

    ros2_control = versions['ros2_control']
    control_status = (
        'pass'
        if ros2_control['version'] != 'not found in the sourced environment'
        else 'fail'
    )
    checks.append(_result(
        'ros2_control',
        control_status,
        'controller_manager is available' if control_status == 'pass'
        else 'controller_manager is missing',
        'version={} prefix={}'.format(ros2_control['version'], ros2_control['prefix']),
    ))

    libfranka = versions['libfranka']
    if libfranka['version'] == '0.9.2':
        franka_status = 'pass'
        franka_summary = 'libfranka 0.9.2 CMake package is available'
    elif libfranka['version'] in ('Franka_DIR not supplied', 'version not found'):
        franka_status = 'fail'
        franka_summary = 'libfranka CMake package is not identified'
    else:
        franka_status = 'warn'
        franka_summary = 'libfranka version differs from the preserved 0.9.2 baseline'
    checks.append(_result(
        'libfranka',
        franka_status,
        franka_summary,
        'version={} cmake_dir={}'.format(
            libfranka['version'], libfranka['cmake_dir']),
    ))

    missing_packages = [
        name for name, info in versions['mvp_packages'].items()
        if info['version'] == 'not found in the sourced environment'
    ]
    package_status = 'fail' if missing_packages else 'pass'
    checks.append(_result(
        'mvp_packages',
        package_status,
        'all seven MVP packages are available' if package_status == 'pass'
        else 'one or more MVP packages are missing',
        'missing={}'.format(','.join(missing_packages) if missing_packages else 'none'),
    ))

    for tool_name in ('compiler', 'cmake'):
        tool = versions[tool_name]
        tool_status = 'pass' if tool['path'] != 'not found' else 'fail'
        checks.append(_result(
            tool_name,
            tool_status,
            '{} is available'.format(tool_name) if tool_status == 'pass'
            else '{} is missing'.format(tool_name),
            'path={} version={}'.format(tool['path'], tool['version']),
        ))
    return checks


def run_preflight(priority: int, franka_dir: Optional[str]) -> Dict[str, object]:
    versions = collect_versions(franka_dir)
    checks = [check_realtime_kernel()]
    checks.extend(check_limits(priority))
    checks.append(check_fifo_capability(priority))
    checks.extend(check_build_environment(versions))
    overall = max((item['status'] for item in checks), key=STATUS_RANK.get)
    return {
        'schema_version': 1,
        'overall': overall,
        'read_only': True,
        'network_access': False,
        'checks': checks,
        'versions': versions,
    }


def _print_text(report: Dict[str, object]) -> None:
    print('multipanda ROS 2 host preflight: {}'.format(str(report['overall']).upper()))
    for item in report['checks']:
        print('[{status}] {name}: {summary}'.format(**item))
        if item['evidence']:
            print('  {}'.format(item['evidence']))
    print('versions:')
    for name, value in report['versions'].items():
        if name == 'mvp_packages':
            print('  mvp_packages:')
            for package_name, package_info in value.items():
                print('    {}={} ({})'.format(
                    package_name, package_info['version'], package_info['prefix']))
        elif isinstance(value, dict):
            details = ' '.join('{}={}'.format(key, item) for key, item in value.items())
            print('  {}: {}'.format(name, details))
        else:
            print('  {}={}'.format(name, value))


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true', help='emit machine-readable JSON')
    parser.add_argument('--priority', type=int, default=50, help='FIFO priority to probe')
    parser.add_argument(
        '--franka-dir',
        default=os.environ.get('Franka_DIR'),
        help='libfranka CMake package directory (defaults to Franka_DIR)',
    )
    parser.add_argument('--_probe-fifo', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args._probe_fifo is not None:
        return _probe_fifo_child(args._probe_fifo)
    if not 1 <= args.priority <= 99:
        parser.error('--priority must be in [1, 99]')

    report = run_preflight(args.priority, args.franka_dir)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_text(report)
    return 2 if report['overall'] == 'fail' else 0


if __name__ == '__main__':
    sys.exit(main())
