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
The live round trip: a real IK node, the real server, and no robot at all.

Everything else in this package's ghost suite drives a stub solver, which
proves the endpoint's arithmetic and nothing about the service behind it.
This file is the one that proves the two agree -- that the pose the browser
computes and posts really does come back as the joints that reach it.

The target pose is computed by an independent forward-kinematics pass over
the description the build generated, so a wrong answer here cannot be
excused by a wrong oracle: the oracle reads the same file the renderer will.

FAKE HARDWARE IS NOT EVEN INVOLVED
    The IK service is a pure function. It commands nothing, connects to
    nothing, and needs no robot; this test starts it, asks it questions, and
    stops it.

Domain isolation
    A real ROS graph runs here, so the test uses the ID this package
    reserves for it and SKIPS itself when the environment does not say so --
    a bare pytest can never publish onto somebody else's domain.
"""

import http.client
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from support.urdf_fk import (      # noqa: E402, I100
    angle_between, Chain, quaternion, translation)

#: The ID this package reserves for the ghost round trip (CMakeLists table).
REQUIRED_DOMAIN_ID = '228'

_SKIP_REASON = (
    'the ghost IK round trip brings up a real ROS graph and must stay on its '
    'reserved domain: run it through the CMake registration, or export '
    'ROS_DOMAIN_ID={} (plus FASTDDS_BUILTIN_TRANSPORTS=SHM and '
    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) yourself'.format(REQUIRED_DOMAIN_ID))

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != REQUIRED_DOMAIN_ID, reason=_SKIP_REASON)

#: RFC 5737 documentation addresses; no session is ever started here.
DOC_IP_1 = '192.0.2.11'
DOC_IP_2 = '192.0.2.12'

READY = (0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854)

#: A small per-joint offset, the shape a drag actually produces: the client
#: seeds each solve with the pose it last drew, never with a fresh guess.
PERTURBATION = (0.02, -0.03, 0.02, 0.03, -0.02, 0.03, -0.02)

#: The service's own defaults, which is what a zero tolerance selects.
POSITION_TOLERANCE_M = 1.0e-4
ORIENTATION_TOLERANCE_RAD = 1.0e-3


def free_port():
    """Return a loopback port that was free a moment ago."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]
    finally:
        probe.close()


def installed_model_urdf():
    """Return the generated description the console serves, or skip."""
    path = os.path.join(get_package_share_directory('franka_web'), 'static',
                        'ghost', 'assets', 'model.urdf')
    if not os.path.isfile(path):
        pytest.skip('the scene assets are not installed; build the package first')
    return path


def server_executable():
    """Resolve the INSTALLED franka_web_server through the ament index."""
    path = os.path.join(get_package_prefix('franka_web'), 'lib', 'franka_web',
                        'franka_web_server')
    if not os.path.isfile(path):
        pytest.skip('franka_web is not installed; build and source the workspace')
    return path


class IkNode:
    """The standing IK service, started and stopped by this test."""

    def __init__(self, root, environment):
        """Launch the service and wait for it to answer."""
        self.log_path = os.path.join(root, 'franka_ik.log')
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'franka_ik', 'franka_ik.launch.py'],
            env=environment, stdin=subprocess.DEVNULL,
            stdout=self._log, stderr=subprocess.STDOUT,
            start_new_session=True)

    def wait_until_serving(self, environment, timeout_s=60.0):
        """Poll the service list until solve_ik appears."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError(
                    'the IK node exited with {}\n{}'.format(
                        self.process.returncode, self.tail()))
            found = subprocess.run(['ros2', 'service', 'list'], env=environment,
                                   capture_output=True, text=True, timeout=30)
            if '/franka_ik_service/solve_ik' in found.stdout:
                return
            time.sleep(0.5)
        raise AssertionError('solve_ik never appeared\n{}'.format(self.tail()))

    def tail(self, limit=40):
        """Return the tail of the node's own output."""
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as handle:
                return '\n'.join(handle.read().splitlines()[-limit:])
        except OSError:
            return '<the IK log could not be read>'

    def stop(self):
        """Stop the node and everything it launched."""
        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                self.process.wait(timeout=10)
        self._log.close()


class Console:
    """One real franka_web_server subprocess, driven over HTTP only."""

    def __init__(self, root, environment):
        """Write a private configuration and spawn the installed server."""
        self.root = root
        self.port = free_port()
        state_dir = os.path.join(root, 'state')
        recordings = os.path.join(root, 'recordings')
        os.makedirs(state_dir, mode=0o700, exist_ok=True)
        os.makedirs(recordings, mode=0o700, exist_ok=True)
        self.config_path = os.path.join(root, 'config.yaml')
        with open(self.config_path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump({
                'bind': '127.0.0.1',
                'port': self.port,
                'ros_domain_id': int(REQUIRED_DOMAIN_ID),
                'robots': {'panda1': {'ip': DOC_IP_1},
                           'panda2': {'ip': DOC_IP_2}},
                'directories': {'state': state_dir, 'recordings': recordings},
            }, handle, default_flow_style=False, sort_keys=True)
        self.log_path = os.path.join(root, 'server.log')
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            [sys.executable, server_executable(), '--config', self.config_path],
            env=environment, stdin=subprocess.DEVNULL,
            stdout=self._log, stderr=subprocess.STDOUT)

    def wait_until_listening(self, timeout_s=60.0):
        """Poll the port until the server accepts a connection."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError('the server exited with {}\n{}'.format(
                    self.process.returncode, self.tail()))
            try:
                socket.create_connection(('127.0.0.1', self.port), 0.5).close()
                return
            except OSError:
                time.sleep(0.2)
        raise AssertionError('the server never listened\n{}'.format(self.tail()))

    def tail(self, limit=40):
        """Return the tail of the server's own output."""
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as handle:
                return '\n'.join(handle.read().splitlines()[-limit:])
        except OSError:
            return '<the server log could not be read>'

    def request(self, method, path, body=None, expect=200):
        """Send one request and return its decoded JSON body."""
        headers = {'Host': '127.0.0.1:{}'.format(self.port)}
        payload = None
        if body is not None:
            payload = json.dumps(body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        connection = http.client.HTTPConnection('127.0.0.1', self.port,
                                                timeout=20.0)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            raw = response.read()
            status = response.status
        finally:
            connection.close()
        decoded = json.loads(raw.decode('utf-8'))
        assert status == expect, '{} {} answered {}: {}\n{}'.format(
            method, path, status, decoded, self.tail())
        return decoded

    def stop(self):
        """Stop the server; idempotent."""
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=20)
        self._log.close()


def environment_for(root):
    """Return the subprocess environment: this domain, private state only."""
    environment = dict(os.environ)
    environment.update({
        'ROS_DOMAIN_ID': REQUIRED_DOMAIN_ID,
        'ROS_HOME': os.path.join(root, 'ros_home'),
        'ROS_LOG_DIR': os.path.join(root, 'ros_log'),
        'HOME': root,
        'XDG_CONFIG_HOME': os.path.join(root, 'xdg'),
        'PYTHONUNBUFFERED': '1',
    })
    return environment


@pytest.fixture(scope='module')
def live(tmp_path_factory):
    """Bring up the IK node and the console once for the whole module."""
    root = str(tmp_path_factory.mktemp('ghost_ik_live'))
    environment = environment_for(root)
    node = IkNode(root, environment)
    console = None
    try:
        node.wait_until_serving(environment)
        console = Console(root, environment)
        console.wait_until_listening()
        yield console, Chain(installed_model_urdf())
    finally:
        if console is not None:
            console.stop()
        node.stop()


def target_for(chain, arm_id, positions):
    """Return the request target for one arm's flange, in its own base frame."""
    pose = chain.arm_pose(arm_id, positions)
    return {'position': list(translation(pose)),
            'orientation': list(quaternion(pose))}


def perturbed(positions):
    """Return the seed a mid-drag solve would carry."""
    return [value + offset for value, offset in zip(positions, PERTURBATION)]


class TestWithoutTheService:
    """
    What the console says when the standing node is not there.

    Deliberately first in the file: the round trip below starts the IK node
    in a module-scoped fixture that lives until the module ends, so these
    two cases have to run before anything asks for it. Both prove an
    ABSENCE, and an absence is only provable while the thing is absent.
    """

    def test_a_solve_teaches_the_command_that_fixes_it(self, tmp_path):
        """
        No IK node, no pose authoring -- and one line that fixes it.

        This is the state a new operator meets first, so the console must
        answer it with a command rather than a diagnosis.
        """
        root = str(tmp_path)
        console = Console(root, environment_for(root))
        try:
            console.wait_until_listening()
            scene = console.request('GET', '/api/scene')
            assert scene['ik']['available'] is False
            assert scene['ghost_available'] is False
            answer = console.request('POST', '/api/ghost/solve', {
                'arm_id': 'panda1', 'seed': list(READY),
                'target': {'position': [0.3, 0.0, 0.5],
                           'orientation': [0.0, 1.0, 0.0, 0.0]},
                'redundancy': {'mode': 'from_seed'}}, expect=503)
            assert answer['error'] == 'ghost_unavailable'
            assert answer['ik_state'] == 'not_ready'
            assert 'ros2 launch franka_ik franka_ik.launch.py' in answer['detail']
        finally:
            console.stop()

    def test_the_startup_banner_says_so(self, tmp_path):
        """The failure is visible at startup, not at the first drag."""
        root = str(tmp_path)
        console = Console(root, environment_for(root))
        try:
            console.wait_until_listening()
            # The HTTP thread listens before the banner's last line prints
            # (that line waits on the cell-model load, which the mesh-exact
            # fence made slower), so wait for the banner rather than sleep.
            deadline = time.monotonic() + 30.0
            while 'scene:' not in console.tail(200):
                assert time.monotonic() < deadline, (
                    'no scene banner within 30 s:\n' + console.tail(200))
                time.sleep(0.2)
            assert 'IK service not running' in console.tail(200)
        finally:
            console.stop()


class TestLiveRoundTrip:
    """What the browser posts, and what actually comes back."""

    def test_the_scene_reports_the_service_as_running(self, live):
        """
        With the node up, the ghost is editable and says so.

        Bounded rather than immediate: the server's client discovers the
        service through the ROS graph, which takes a moment after either
        process starts. The page re-fetches this payload on reconnect and
        whenever a solve reports the service missing, so a first answer
        taken during discovery corrects itself; a permanent false would not.
        """
        console, _chain = live
        deadline = time.monotonic() + 15.0
        scene = console.request('GET', '/api/scene')
        while not scene['ik']['available'] and time.monotonic() < deadline:
            time.sleep(0.2)
            scene = console.request('GET', '/api/scene')
        assert scene['ik']['available'] is True
        assert scene['ghost_available'] is True
        assert scene['ik']['arm_ids'] == ['panda1', 'panda2']
        assert scene['ik']['tip_frame'] == 'flange'

    def test_a_solved_pose_reaches_the_target_it_was_given(self, live):
        """
        The whole product, end to end, measured against an independent oracle.

        The returned joints are pushed back through the same forward
        kinematics that produced the target; agreement within the service's
        own tolerances is what "the ghost went where you dragged it" means.
        """
        console, chain = live
        target = target_for(chain, 'panda1', READY)
        answer = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda1', 'seed': perturbed(READY), 'target': target,
            'redundancy': {'mode': 'from_seed'}})
        assert answer['ok'] is True
        assert answer['solved'] is True, answer.get('solve_reason')
        reached = chain.arm_pose('panda1', answer['positions'])
        distance = math.dist(translation(reached), target['position'])
        assert distance <= POSITION_TOLERANCE_M, distance
        assert angle_between(quaternion(reached), target['orientation']) \
            <= ORIENTATION_TOLERANCE_RAD

    def test_every_returned_joint_is_inside_the_reviewed_policy(self, live):
        """A pose outside the policy would be one the console cannot command."""
        from franka_web import defaults
        console, chain = live
        answer = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda1', 'seed': perturbed(READY),
            'target': target_for(chain, 'panda1', READY),
            'redundancy': {'mode': 'from_seed'}})
        for index, value in enumerate(answer['positions']):
            assert defaults.POLICY_POSITION_LOWER_RAD[index] <= value
            assert value <= defaults.POLICY_POSITION_UPPER_RAD[index]

    def test_both_arms_answer_the_same_base_relative_pose_alike(self, live):
        """
        The property that actually discriminates a frame bug.

        A pose expressed in <arm>_link0 is base-relative by construction, and
        the service picks its reference frame from the frame_id string alone.
        Under the symmetric mounting both arms therefore return essentially
        the same joint vector for the same base-relative target -- and they
        only do so if BOTH sides really are base-relative.
        """
        console, chain = live
        first = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda1', 'seed': perturbed(READY),
            'target': target_for(chain, 'panda1', READY),
            'redundancy': {'mode': 'from_seed'}})
        second = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda2', 'seed': perturbed(READY),
            'target': target_for(chain, 'panda2', READY),
            'redundancy': {'mode': 'from_seed'}})
        assert first['solved'] and second['solved']
        for left, right in zip(first['positions'], second['positions']):
            assert abs(left - right) < 1e-3

    def test_a_pose_in_the_root_frame_does_not_reproduce_itself(self, live):
        """
        The negative case: a root-relative target is NOT base-relative.

        panda1's base sits half a metre off the description's root, so a
        target computed in the root frame and sent as if it were in the arm
        base misses by that offset -- either refused outright or solved to a
        different point. Either answer proves the two frames are distinct,
        which is what makes the positive case above meaningful.
        """
        console, chain = live
        values = {'panda1_joint{}'.format(index + 1): value
                  for index, value in enumerate(READY)}
        root_pose = chain.forward('base_link', 'panda1_link8', values)
        answer = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda1', 'seed': perturbed(READY),
            'target': {'position': list(translation(root_pose)),
                       'orientation': list(quaternion(root_pose))},
            'redundancy': {'mode': 'from_seed'}})
        if answer['solved']:
            reached = chain.arm_pose('panda1', answer['positions'])
            base_target = target_for(chain, 'panda1', READY)['position']
            assert math.dist(translation(reached), base_target) > 0.1
        else:
            assert answer['solve_reason']

    def test_an_unreachable_target_is_an_ordinary_two_hundred(self, live):
        """Dragging past the workspace edge is interaction, not an error."""
        console, chain = live
        target = target_for(chain, 'panda1', READY)
        target['position'][0] += 1.5
        answer = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda1', 'seed': list(READY), 'target': target,
            'redundancy': {'mode': 'from_seed'}})
        assert answer['ok'] is True
        assert answer['solved'] is False
        assert answer['solve_reason'] == "That point is outside this arm's reach."
        assert answer['positions'] is None
        assert answer['verdict'] is None

    def test_the_redundancy_sweep_holds_the_flange_still(self, live):
        """
        The geometric premise of the elbow ring, measured on the real solver.

        Every row is a solve against the same flange target, so the flange
        must stay put while the elbow swings. A ring that moved the hand
        would be lying about what it does.
        """
        console, chain = live
        target = target_for(chain, 'panda1', READY)
        answer = console.request('POST', '/api/ghost/redundancy', {
            'arm_id': 'panda1', 'seed': list(READY), 'target': target,
            'samples': 25})
        assert answer['samples'] == 25
        assert len(answer['table']) >= 5, answer
        points = [translation(chain.arm_pose('panda1', row['positions']))
                  for row in answer['table']]
        spread = max(math.dist(point, target['position']) for point in points)
        assert spread < 0.002, spread

    def test_the_copy_snippet_names_this_arms_joints(self, live):
        """What the operator pastes is what they just authored."""
        console, chain = live
        answer = console.request('POST', '/api/ghost/solve', {
            'arm_id': 'panda2', 'seed': perturbed(READY),
            'target': target_for(chain, 'panda2', READY),
            'redundancy': {'mode': 'from_seed'}})
        snippet = answer['copy']['snippet']
        assert 'panda2_joint1' in snippet
        assert 'topic pub' not in snippet
