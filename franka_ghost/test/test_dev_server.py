# [THROWAWAY] Session C standalone-server tests; franka_web owns merged integration tests.
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

"""Tests for the loopback development server and its C1 oracle."""

from copy import deepcopy
import http.client
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
dev_server = importlib.import_module('franka_ghost.dev_server')
joint_source = importlib.import_module('franka_ghost.joint_source')


class FrozenClock:
    """Clock whose wall and monotonic values only move when a test asks."""

    def __init__(self) -> None:
        self.wall_ns = 1_756_492_871_512_000_000
        self.monotonic_ns = 4_000_000_000

    def time_ns(self) -> int:
        return self.wall_ns

    def monotonic(self) -> float:
        return self.monotonic_ns / 1.0e9

    def monotonic_time_ns(self) -> int:
        return self.monotonic_ns


def _request(server, method, path, payload=None):
    connection = http.client.HTTPConnection(*server.server_address, timeout=2.0)
    headers = {}
    body = None
    if payload is not None:
        body = json.dumps(payload).encode('utf-8')
        headers['Content-Type'] = 'application/json'
    connection.request(method, path, body=body, headers=headers)
    response = connection.getresponse()
    raw = response.read()
    result = response.status, response.getheaders(), raw
    connection.close()
    return result


@pytest.fixture
def running_server(tmp_path):
    web_root = tmp_path / 'web'
    web_root.mkdir()
    (web_root / 'index.html').write_text('<h1>ghost</h1>', encoding='utf-8')
    (web_root / 'module.js').write_text('export {};', encoding='utf-8')
    (web_root / 'secret.txt').write_text('not served', encoding='utf-8')
    clock = FrozenClock()
    server = dev_server.create_server(
        port=0,
        web_root=web_root,
        source=None,
        source_name='demo',
        monotonic=clock.monotonic,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server, clock
    server.shutdown()
    server.server_close()
    thread.join(timeout=2.0)


def _valid_event(arm_index=1, epoch=1):
    arm_id = f'panda{arm_index}'
    return {
        'schema': 'franka.ghost.apply/1',
        'arm_index': arm_index,
        'arm_id': arm_id,
        'joint_names': [f'{arm_id}_joint{joint}' for joint in range(1, 8)],
        'positions': list(joint_source.INITIAL_POSITIONS),
        'fence': {
            'lower': list(joint_source.URDF_LOWER),
            'upper': list(joint_source.URDF_UPPER),
            'source': 'urdf',
        },
        'measured_at_apply': list(joint_source.INITIAL_POSITIONS),
        'ghost_epoch': epoch,
    }


def _post(server, event):
    status, headers, raw = _request(server, 'POST', '/apply', event)
    assert status == 200
    assert int(dict(headers)['Content-Length']) == len(raw)
    return json.loads(raw)


def test_static_allowlist_traversal_and_http11_keepalive(running_server):
    server, _ = running_server
    assert server.server_address[0] == '127.0.0.1'
    connection = http.client.HTTPConnection(*server.server_address, timeout=2.0)
    connection.request('GET', '/')
    response = connection.getresponse()
    assert response.status == 200
    assert response.version == 11
    assert int(response.getheader('Content-Length')) == len(response.read())

    connection.request('GET', '/module.js')
    second = connection.getresponse()
    assert second.status == 200
    assert second.read() == b'export {};'
    connection.close()

    for path in ('/%2e%2e/index.html', '/%2Fetc%2Fpasswd', '/a..b.html'):
        status, _, _ = _request(server, 'GET', path)
        assert status == 403
    status, _, _ = _request(server, 'GET', '/secret.txt')
    assert status == 415

    # A leaf symlink leaving the web root is SERVED: that is exactly the shape
    # `colcon build --symlink-install` produces under share/franka_ghost/web, and refusing it
    # made the installed prototype 403 every page while the whole suite stayed green.
    outside = server.web_root.parent / 'outside.html'
    outside.write_text('served through a leaf symlink', encoding='utf-8')
    (server.web_root / 'escape.html').symlink_to(outside)
    status, _, body = _request(server, 'GET', '/escape.html')
    assert status == 200
    assert body == b'served through a leaf symlink'

    # It may not launder a non-allowlisted file behind an allowlisted name, though.
    (server.web_root / 'laundered.html').symlink_to(server.web_root / 'secret.txt')
    status, _, _ = _request(server, 'GET', '/laundered.html')
    assert status == 403

    # And a DIRECTORY symlink out of the web root is still refused: that would expose a whole
    # foreign tree, which no install layout needs.
    outside_dir = server.web_root.parent / 'outside_dir'
    outside_dir.mkdir()
    (outside_dir / 'page.html').write_text('not served', encoding='utf-8')
    (server.web_root / 'escape_dir').symlink_to(outside_dir, target_is_directory=True)
    status, _, _ = _request(server, 'GET', '/escape_dir/page.html')
    assert status == 403


def test_static_serves_a_symlink_install_shaped_web_root(tmp_path):
    """Serve share/<pkg>/web as `colcon --symlink-install` actually lays it out."""
    source = tmp_path / 'src' / 'web'
    (source / 'ghost').mkdir(parents=True)
    (source / 'index.html').write_text('<h1>installed ghost</h1>', encoding='utf-8')
    (source / 'style.css').write_text('body{}', encoding='utf-8')
    (source / 'ghost' / 'ghost.js').write_text('export {};', encoding='utf-8')

    # --symlink-install keeps the directories real and makes every leaf a symlink back into
    # the source tree; the installed root itself is a normal directory.
    installed = tmp_path / 'install' / 'share' / 'franka_ghost' / 'web'
    (installed / 'ghost').mkdir(parents=True)
    for relative in ('index.html', 'style.css', 'ghost/ghost.js'):
        (installed / relative).symlink_to(source / relative)
    assert (installed / 'index.html').is_symlink()
    assert not (installed / 'ghost').is_symlink()

    server = dev_server.create_server(
        port=0,
        web_root=installed,
        source=None,
        source_name='demo',
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        for path, expected in (
            ('/', b'<h1>installed ghost</h1>'),
            ('/index.html', b'<h1>installed ghost</h1>'),
            ('/style.css', b'body{}'),
            ('/ghost/ghost.js', b'export {};'),
        ):
            status, _, body = _request(server, 'GET', path)
            assert status == 200, f'{path} returned {status} from a symlink-install web root'
            assert body == expected
        # The URL traversal guard is untouched by the symlink allowance.
        for path in ('/%2e%2e/index.html', '/ghost/../../secret.html', '/a..b.html'):
            status, _, _ = _request(server, 'GET', path)
            assert status == 403
        status, _, _ = _request(server, 'GET', '/missing.js')
        assert status == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_state_schema_is_honest_before_and_after_a_sample(running_server):
    server, clock = running_server
    status, _, raw = _request(server, 'GET', '/state.json')
    state = json.loads(raw)
    assert status == 200
    assert state == {
        'schema': 'franka.ghost.state/1',
        'source': 'demo',
        'stamp_ns': None,
        'age_s': 0.0,
        'stale': True,
        'joints': {},
        'seq': 0,
    }

    server.state.update(clock.wall_ns, {'panda1_joint1': 0.25})
    clock.monotonic_ns += 310_000_000
    _, _, raw = _request(server, 'GET', '/state.json')
    fresh = json.loads(raw)
    assert fresh['stamp_ns'] == clock.wall_ns
    assert fresh['joints'] == {'panda1_joint1': 0.25}
    assert fresh['seq'] == 1
    assert fresh['age_s'] == pytest.approx(0.31)
    assert fresh['stale'] is False

    clock.monotonic_ns += 200_000_000
    _, _, raw = _request(server, 'GET', '/state.json')
    assert json.loads(raw)['stale'] is True


def test_demo_source_starts_at_initial_pose_and_stays_inside_urdf_limits():
    clock = FrozenClock()
    source = joint_source.DemoJointSource(
        clock_ns=clock.time_ns,
        monotonic_ns=clock.monotonic_time_ns,
    )
    stamp, joints = source.latest()
    assert stamp == clock.wall_ns
    assert len(joints) == 14
    for arm in (1, 2):
        actual = [joints[f'panda{arm}_joint{joint}'] for joint in range(1, 8)]
        assert actual == pytest.approx(joint_source.INITIAL_POSITIONS)

    clock.monotonic_ns += 123_000_000_000
    _, moved = source.latest()
    for arm in (1, 2):
        for index in range(7):
            value = moved[f'panda{arm}_joint{index + 1}']
            assert joint_source.URDF_LOWER[index] <= value <= joint_source.URDF_UPPER[index]


def test_apply_accepts_and_echoes_a_valid_c1_event(running_server):
    server, _ = running_server
    event = _valid_event()
    response = _post(server, event)
    assert response == {
        'accepted': True,
        'echo': event,
        'note': 'prototype: not published',
    }


def test_apply_epoch_is_strict_within_a_page_mount_and_resets_on_reload(
    running_server,
):
    """I6 is mount-scoped, while the throwaway root page owns one mount."""
    server, _ = running_server
    assert _post(server, _valid_event(epoch=1))['accepted'] is True
    assert _post(server, _valid_event(epoch=2))['accepted'] is True
    repeated = _post(server, _valid_event(epoch=2))
    assert repeated['accepted'] is False
    assert any(error.startswith('I6:') for error in repeated['errors'])

    status, _, _ = _request(server, 'GET', '/')
    assert status == 200
    assert _post(server, _valid_event(epoch=1))['accepted'] is True


@pytest.mark.parametrize(
    ('invariant', 'mutate'),
    [
        ('I1', lambda event: event.update(joint_names=list(reversed(event['joint_names'])))),
        ('I2', lambda event: event['positions'].__setitem__(0, None)),
        (
            'I3',
            lambda event: event['positions'].__setitem__(3, event['fence']['upper'][3] + 0.1),
        ),
        (
            'I4',
            lambda event: event['fence']['lower'].__setitem__(0, -3.0),
        ),
        ('I5', lambda event: event.update(arm_index=2)),
        ('I6', lambda event: event.update(ghost_epoch=1)),
        ('I7', lambda event: event.update(velocities=[])),
    ],
)
def test_apply_reports_each_c1_invariant(running_server, invariant, mutate):
    server, _ = running_server
    assert _post(server, _valid_event(epoch=1))['accepted'] is True
    event = deepcopy(_valid_event(epoch=2))
    mutate(event)
    response = _post(server, event)
    assert response['accepted'] is False
    assert any(error.startswith(f'{invariant}:') for error in response['errors'])


def test_ros_mode_requires_the_assigned_domain():
    with pytest.raises(RuntimeError, match='ROS_DOMAIN_ID=82'):
        joint_source.require_ros_domain_id({})
    with pytest.raises(RuntimeError, match='ROS_DOMAIN_ID=82'):
        joint_source.require_ros_domain_id({'ROS_DOMAIN_ID': '81'})
    joint_source.require_ros_domain_id({'ROS_DOMAIN_ID': '82'})


@pytest.mark.parametrize('domain_id', [None, '81'])
def test_dev_server_cli_refuses_ros_source_outside_interactive_domain(domain_id):
    """The installed-style CLI remains locked to Session C's interactive domain 82."""
    project_root = Path(__file__).resolve().parents[1]
    environment = os.environ.copy()
    if domain_id is None:
        environment.pop('ROS_DOMAIN_ID', None)
    else:
        environment['ROS_DOMAIN_ID'] = domain_id
    python_path = environment.get('PYTHONPATH')
    environment['PYTHONPATH'] = (
        str(project_root)
        if not python_path
        else f'{project_root}{os.pathsep}{python_path}'
    )

    result = subprocess.run(
        [
            sys.executable,
            str(project_root / 'scripts' / 'franka_ghost_dev_server.py'),
            '--source',
            'ros',
            '--web-root',
            str(project_root / 'web'),
        ],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
        timeout=5.0,
    )

    assert result.returncode == 2
    assert 'ROS source requires ROS_DOMAIN_ID=82' in result.stderr


def test_source_pump_promotes_only_distinct_newest_samples(tmp_path):
    class MutableSource:
        source = 'demo'

        def __init__(self):
            self.sample = (10, {'panda1_joint1': 0.0})

        def latest(self):
            return self.sample

        def close(self):
            pass

    web_root = tmp_path / 'web'
    web_root.mkdir()
    (web_root / 'index.html').write_text('ok', encoding='utf-8')
    source = MutableSource()
    server = dev_server.create_server(
        port=0,
        web_root=web_root,
        source=source,
        rate_hz=100.0,
    )
    try:
        deadline = time.monotonic() + 1.0
        while server.state.snapshot()['seq'] < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert server.state.snapshot()['seq'] == 1
        time.sleep(0.03)
        assert server.state.snapshot()['seq'] == 1
        source.sample = (11, {'panda1_joint1': 0.1})
        deadline = time.monotonic() + 1.0
        while server.state.snapshot()['seq'] < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert server.state.snapshot()['seq'] == 2
    finally:
        server.server_close()
