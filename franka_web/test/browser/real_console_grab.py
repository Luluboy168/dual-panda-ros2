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

"""
Grab the ghost's hand on the REAL console page, the way the operator does.

WHY THIS IS NOT A HARNESS CASE
    ``test/browser/cases/drag.js`` mounts the scene module against a fixture
    and proves its geometry. It cannot prove the thing the operator reported,
    because the thing the operator reported only exists at the size the
    console's own panel gives the canvas, with the console's own camera
    framing, after the console's own toolbar button turned a ghost on. So this
    drives the shipped page: a real ``franka_web_server``, a real session, the
    real Scene panel, the real ``Ghost panda1`` button, and real pointer events
    delivered through Chromium's input pipeline -- not synthesised in JS.

WHAT IT OWNS, AND WHY
    It starts its own console and tears it down again: a
    ``franka_web_server`` on port 8770 under a scratch ``XDG_CONFIG_HOME``,
    and the ``franka_ik`` service the ghost cannot solve without. The
    configuration is the OPERATOR'S OWN ``config.yaml`` -- profiles and
    libfranka path verbatim, because a console configured by this file would
    be a different console -- with exactly three keys overridden: the port,
    and the state and recording directories. Everything runs on
    ``ROS_DOMAIN_ID`` 231, so nothing this case starts can be seen by, or can
    see, whatever is on the machine's usual domain. The session is
    **Simulate**: no robot is involved at any point.

WHAT IT ASSERTS, AND WHY THOSE NUMBERS
    The operator's window is 512x597, which gives the scene canvas 445x273 CSS
    px. At that size the whole ghost arm stands 46 px tall and its hand is
    12x10 px, while the drawn handle is a disc of HANDLE_MIN_PX = 9 px radius.
    A press on -- or just off -- that disc must move the hand.

    Before the fix it did not, in two different ways. The hand's pick proxy was
    floored at the DRAWN knob and no further, so a press a few pixels off the
    centre hit nothing at all and fell through to the orbit controller, which
    spun the view instead of moving the hand -- that is what every offset here
    but the first two did. And where the elbow ring's band, sorted
    nearest-first, crossed in FRONT of the knob, a press that DID reach the
    gizmo layer was answered by the ring: the gesture began, the canvas took
    the dragging class, and the ghost did not move a millimetre. Both are "the
    ghost cannot be dragged", and HAND_PICK_PX plus the hand's priority in
    onPointerDown are the two halves of the answer.

    So this presses at each of ``OFFSETS_PX`` around the handle's centre, from
    the centre itself out past the edge of the drawn disc, twice: as the panel
    opens, and again after the view has been orbited -- because WHICH pick
    proxy lies nearest the camera at the flange changes with the angle, and
    the orbit is checked to have actually happened.

    Each press must satisfy two things, and they are not the same thing. It
    must be TAKEN BY THE HAND, which is read from the call the console makes
    while the gesture is held: only a hand drag asks ``/api/ghost/solve`` and
    only an elbow drag asks ``/api/ghost/redundancy``. And it must MOVE the
    ghost's flange by at least ``MIN_MOVE_M``. The first is the rule this fix
    added; the second is what the operator asked for. Distance alone could not
    prove the first, because a ring grab and a hand grab the solver refused
    for reach both measure zero -- and at the ghost's home pose, refusals for
    reach are common enough that the case pulls all four ways before it
    believes one.

RUNNING IT
    No arguments needed; it brings up everything it uses::

        python3 test/browser/real_console_grab.py

    Under pytest it is opt-in -- ``pytest --real-console`` -- because it wants
    a MuJoCo Simulate session, a WebGL context, a fixed port and a minute
    or two, none of which belong in the battery that runs on every build. See
    ``test_browser.py`` for what stands in for it there.

    ``--url`` drives a console that is already running instead of starting
    one. It then claims the operator lock on that server, taking over if it is
    held, and leaves the session up. Point it at a scratch server, never at
    the one an operator is using.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from franka_web import config                                    # noqa: E402
import yaml                                                      # noqa: E402

#: The operator's own window, measured from the browser they reported from.
VIEWPORT = (512, 597)

#: This case's own port and ROS domain. Neither is the machine's usual one:
#: the console an operator has open must not be disturbed by a test, and a
#: Simulate session on its own domain cannot be heard by anything else.
PORT = 8770
DOMAIN_ID = 231

#: Offsets from the drawn handle's centre, in CSS px, as (dx, dy). The centre
#: is the control: it worked before this case existed and must keep working.
#: The other four did not. Against the parent commit's scene, on this page, at
#: this window::
#:
#:      dx=  0 dy=  0   the hand              0.0605 m
#:      dx=  6 dy=  0   the hand              0.0609 m
#:      dx=  0 dy=  9   the ORBIT CONTROLLER  0.0000 m, view spun 9 deg
#:      dx=-12 dy=  0   the ORBIT CONTROLLER  0.0000 m, view spun 9 deg
#:      dx= 12 dy=  0   the ORBIT CONTROLLER  0.0000 m, view spun 10 deg
#:
#: Nine pixels from the centre of a knob whose DRAWN radius is nine, on an arm
#: 46 px tall, and the press missed everything and spun the view instead. All
#: five offsets are inside HAND_PICK_PX's 26 px footprint, which is the
#: promise under test: the hand's own target, sized for a finger rather than
#: for the knob that happens to be drawn.
OFFSETS_PX = ((0, 0), (6, 0), (0, 9), (-12, 0), (12, 0))

#: A grab that worked moves the flange at least this far. Nothing that failed
#: moves it at all, so the bar only has to be above noise.
MIN_MOVE_M = 0.010

#: How far the case turns the view between its two halves, and how much of
#: that turn has to arrive. The point of the second half is that the pick
#: proxies are stacked differently, so a token wobble will not do.
ORBIT_PX = 900
MIN_ORBIT_DEG = 25.0

#: The four pulls, in CSS px from the press point. Which of them the arm can
#: follow from its home pose depends on the camera angle and on the workspace,
#: not on the handle, and at this pose the ghost is close enough to the edge of
#: its reach that most single pulls are refused. So the case tries the four
#: quadrants before it calls a press dead. Twenty-eight pixels, not forty-five:
#: a shorter pull asks the solver for a smaller step, and the whole question
#: here is whether the hand was GRABBED, not how far it can be thrown.
DRAGS = ((-28, 11), (28, 11), (-28, -11), (28, -11))

BROWSERS = ('/usr/bin/google-chrome', '/usr/bin/google-chrome-stable',
            '/usr/bin/chromium', '/usr/bin/chromium-browser')


# ------------------------------------------------------------ the console --

class Console:
    """
    A ``franka_web_server`` of this case's own, and the IK service it needs.

    Two processes, each in its own session so the whole tree can be reaped:
    the shipped server launcher, and ``ros2 launch franka_ik`` -- which the
    console itself never starts, and without which the Ghost buttons stay
    disabled and every solve is refused.
    """

    def __init__(self, port=PORT, domain_id=DOMAIN_ID):
        """Start both processes and wait until the ghost can be solved for."""
        self._root = Path(tempfile.mkdtemp(prefix='.ghost-console-'))
        self.url = 'http://127.0.0.1:{}'.format(port)
        self._children = []
        for name in ('state', 'recordings'):
            # 0700, and not merely inherited: the bounded recorder refuses an
            # output root any other user could read, and a session whose
            # recording is refused stops before it has started.
            (self._root / name).mkdir(mode=0o700)
        config_home = self._root / 'config'
        (config_home / 'franka_web').mkdir(parents=True)
        (config_home / 'franka_web' / 'config.yaml').write_text(
            self._configuration(port))
        environment = dict(os.environ)
        environment['XDG_CONFIG_HOME'] = str(config_home)
        environment['ROS_DOMAIN_ID'] = str(domain_id)
        try:
            self._spawn([sys.executable, str(_server_launcher())], environment)
            self._spawn([_ros2(), 'launch', 'franka_ik', 'franka_ik.launch.py'],
                        environment)
            self._await_ready()
        except BaseException:
            # A console that never came up still started processes, and a
            # server left holding port 8770 would make the NEXT run fail for
            # a reason that is not the ghost's.
            self.close()
            raise

    def _configuration(self, port):
        """
        Return the operator's own config.yaml, on this case's port and dirs.

        The profiles and the libfranka path are the machine's, verbatim: this
        case exists to drive the console the operator runs, and a
        configuration invented here would be a different console. Three keys
        are overridden and no others, so the case can neither collide with a
        server the operator has open nor write into their recordings.
        """
        raw = {}
        source = Path(config.default_config_path(os.environ))
        if source.is_file():
            raw = yaml.safe_load(source.read_text()) or {}
        raw['port'] = port
        directories = dict(raw.get('directories') or {})
        directories['state'] = str(self._root / 'state')
        directories['recordings'] = str(self._root / 'recordings')
        raw['directories'] = directories
        return ('# Written by real_console_grab.py from {}.\n'
                '# Only the port and the two directories differ from it.\n'
                .format(source) + yaml.safe_dump(raw, sort_keys=True))

    def _spawn(self, command, environment):
        """Start one child in its own session, so its tree can be reaped."""
        self._children.append(subprocess.Popen(
            command, env=environment, stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT, start_new_session=True))

    def _await_ready(self, timeout=180.0):
        """Poll until the page can be served AND the ghost can be solved for."""
        deadline = time.time() + timeout
        note = 'nothing answered on {}'.format(self.url)
        while time.time() < deadline:
            for child in self._children:
                if child.poll() is not None:
                    raise RuntimeError(
                        'a console process exited with {} before it was '
                        'ready ({})'.format(child.returncode, note))
            try:
                with urllib.request.urlopen(
                        self.url + '/api/scene', timeout=5) as answer:
                    scene = json.loads(answer.read())
            except (OSError, ValueError) as error:
                note = 'the server did not answer /api/scene: {}'.format(error)
            else:
                if scene.get('ghost_available'):
                    return
                note = 'the IK service has not come up: {}'.format(
                    scene.get('ik'))
            time.sleep(1.0)
        raise RuntimeError('the console never became usable: ' + note)

    def close(self):
        """Reap both process trees and remove the scratch configuration."""
        for child in reversed(self._children):
            _reap(child)
        shutil.rmtree(self._root, ignore_errors=True)


def _server_launcher():
    """Return the installed ``franka_web_server`` the operator runs."""
    from ament_index_python.packages import get_package_prefix
    launcher = (Path(get_package_prefix('franka_web'))
                / 'lib' / 'franka_web' / 'franka_web_server')
    if not launcher.is_file():
        raise RuntimeError('franka_web is not installed: {}'.format(launcher))
    return launcher


def _ros2():
    """Return the ``ros2`` command line, or say the workspace is not sourced."""
    found = shutil.which('ros2')
    if found is None:
        raise RuntimeError('ros2 is not on PATH; source the workspace first')
    return found


def _reap(child):
    """Signal a child's whole process group, then make sure it is gone."""
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except OSError:
        return
    try:
        child.wait(timeout=30)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except OSError:
        pass
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


# ---------------------------------------------------------------- websocket --

class _Socket:
    """The smallest RFC 6455 client that can carry CDP. No dependencies."""

    def __init__(self, url, timeout=90.0):
        rest = url[len('ws://'):]
        hostport, _, path = rest.partition('/')
        host, _, port = hostport.partition(':')
        self._sock = socket.create_connection((host, int(port or 80)), timeout)
        self._sock.settimeout(timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        self._sock.sendall(
            ('GET /{} HTTP/1.1\r\nHost: {}\r\nUpgrade: websocket\r\n'
             'Connection: Upgrade\r\nSec-WebSocket-Key: {}\r\n'
             'Sec-WebSocket-Version: 13\r\n\r\n').format(
                 path, hostport, key).encode())
        buffer = b''
        while b'\r\n\r\n' not in buffer:
            buffer += self._sock.recv(4096)
        self._rest = buffer.split(b'\r\n\r\n', 1)[1]

    def _take(self, count):
        while len(self._rest) < count:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise IOError('the devtools socket closed')
            self._rest += chunk
        out, self._rest = self._rest[:count], self._rest[count:]
        return out

    def send(self, text):
        """Send one masked text frame."""
        payload = text.encode()
        size = len(payload)
        head = bytes([0x81])
        if size < 126:
            head += bytes([0x80 | size])
        elif size < 65536:
            head += bytes([0x80 | 126]) + struct.pack('>H', size)
        else:
            head += bytes([0x80 | 127]) + struct.pack('>Q', size)
        mask = os.urandom(4)
        self._sock.sendall(head + mask + bytes(
            byte ^ mask[index % 4] for index, byte in enumerate(payload)))

    def recv(self):
        """Return the next text frame, answering pings on the way."""
        while True:
            first, second = self._take(2)
            opcode, size = first & 0x0F, second & 0x7F
            if size == 126:
                size = struct.unpack('>H', self._take(2))[0]
            elif size == 127:
                size = struct.unpack('>Q', self._take(8))[0]
            data = self._take(size)
            if opcode == 0x1:
                return data.decode()
            if opcode == 0x8:
                raise IOError('the devtools socket sent a close frame')
            if opcode == 0x9:
                self._sock.sendall(bytes([0x8A, 0x80]) + os.urandom(4))

    def close(self):
        """Drop the socket; errors here cannot matter."""
        try:
            self._sock.close()
        except OSError:
            pass


# ------------------------------------------------------------------ browser --

#: Capture the scene graph the page renders, and the ghost calls it makes,
#: WITHOUT touching the page's code.
#:
#: The renderer wrapper records each frame's scene and camera. three r111 is a
#: UMD that assigns an empty ``window.THREE`` and then fills it, so this polls
#: rather than trusting a property setter.
#:
#: The fetch wrapper records WHICH question the console asked while a gesture
#: was held, and that is the observation this whole case turns on. Distance
#: moved cannot tell a ring grab from a hand grab the solver refused: both are
#: zero. ``/api/ghost/solve`` is asked only by a HAND drag and
#: ``/api/ghost/redundancy`` only by an ELBOW RING drag, so the pair says
#: exactly which gizmo took the press -- and the refusal sentences the server
#: sends back say, in the operator's own words, why a hand that WAS grabbed
#: did not move.
PROBE = """
(() => {
  const original = window.fetch;
  window.__calls = {solve: 0, redundancy: 0, refused: 0, reasons: []};
  window.fetch = function (input) {
    const url = typeof input === "string" ? input : (input && input.url) || "";
    const answer = original.apply(this, arguments);
    const which = /\\/api\\/ghost\\/(solve|redundancy)/.exec(url);
    if (which) {
      window.__calls[which[1]] += 1;
      answer.then(r => r.clone().json()).then(j => {
        if (which[1] === "solve" && j && j.solved === false) {
          window.__calls.refused += 1;
          if (j.solve_reason
              && window.__calls.reasons.indexOf(j.solve_reason) < 0) {
            window.__calls.reasons.push(j.solve_reason);
          }
        }
      }).catch(() => {});
    }
    return answer;
  };
  const install = () => {
    const T = window.THREE;
    if (!T || !T.WebGLRenderer || T.__probed) return !!(T && T.__probed);
    T.__probed = true;
    const Renderer = T.WebGLRenderer;
    function Probed(...args) {
      const instance = new Renderer(...args);
      const render = instance.render.bind(instance);
      instance.render = function (scene, camera) {
        window.__probe = {scene, camera};
        return render(scene, camera);
      };
      return instance;
    }
    Probed.prototype = Renderer.prototype;
    T.WebGLRenderer = Probed;
    return true;
  };
  if (!install()) {
    const timer = setInterval(() => { if (install()) clearInterval(timer); }, 10);
  }
})();
"""

WHERE = """(() => {
  const p = window.__probe; if (!p) return null;
  const T = window.THREE, cam = p.camera;
  const canvas = document.querySelector('canvas.ghost-canvas');
  if (!canvas) return null;
  const rect = canvas.getBoundingClientRect();
  p.scene.updateMatrixWorld(true);
  let group = null;
  p.scene.traverse(o => { if (o.name === 'hand_handle_1') group = o; });
  if (!group || !group.visible) return null;
  const knob = group.children.find(
    c => c.type === 'Mesh' && !c.userData.pickKind);
  const world = new T.Vector3(); group.getWorldPosition(world);
  const scale = new T.Vector3(); knob.getWorldScale(scale);
  const project = v => { const q = v.clone().project(cam);
    return [rect.left + (q.x + 1) / 2 * rect.width,
            rect.top + (1 - q.y) / 2 * rect.height]; };
  const up = new T.Vector3();
  cam.matrixWorld.extractBasis(new T.Vector3(), up, new T.Vector3());
  const centre = project(world);
  const edge = project(world.clone().addScaledVector(up, scale.x));
  return {x: centre[0], y: centre[1],
          knobPx: Math.hypot(edge[0] - centre[0], edge[1] - centre[1]),
          canvas: {w: rect.width, h: rect.height}};
})()"""

FLANGE = """(() => {
  const p = window.__probe; let group = null;
  p.scene.traverse(o => { if (o.name === 'hand_handle_1') group = o; });
  return group ? group.position.toArray() : null;
})()"""

#: Where the eye is, in degrees around the cell's vertical axis. The case
#: reads it to prove the orbit it asks for actually turned the view: without
#: that, the second half of the case is the first half again.
AZIMUTH = """(() => {
  const p = window.__probe; if (!p) return null;
  const eye = p.camera.position;
  return Math.atan2(eye.x, eye.z) * 180 / Math.PI;
})()"""


class Page:
    """A headless Chromium page, driven over CDP."""

    def __init__(self, url, width, height):
        browser = next((b for b in BROWSERS if os.path.exists(b)), None)
        if browser is None:
            raise RuntimeError('no Chromium or Chrome executable was found')
        self._profile = tempfile.mkdtemp(prefix='.ghost-grab-')
        self._process = subprocess.Popen(
            [browser, '--headless=new', '--remote-debugging-port=0',
             '--enable-unsafe-swiftshader', '--disable-background-networking',
             '--disable-default-apps', '--no-first-run', '--no-sandbox',
             '--window-size={},{}'.format(width, height + 120),
             '--user-data-dir={}'.format(self._profile), 'about:blank'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        port = self._await_port()
        request = urllib.request.Request(
            'http://127.0.0.1:{}/json/new?about:blank'.format(port),
            method='PUT')
        target = json.loads(urllib.request.urlopen(request).read())
        self._socket = _Socket(target['webSocketDebuggerUrl'])
        self._next_id = 0
        self.call('Page.enable')
        self.call('Runtime.enable')
        self.call('Emulation.setDeviceMetricsOverride', width=width,
                  height=height, deviceScaleFactor=1, mobile=False)
        self.call('Page.addScriptToEvaluateOnNewDocument', source=PROBE)
        self.call('Page.navigate', url=url)

    def _await_port(self):
        marker = Path(self._profile) / 'DevToolsActivePort'
        for _ in range(600):
            if marker.exists():
                try:
                    return int(marker.read_text().split('\n')[0])
                except (ValueError, IndexError):
                    pass
            time.sleep(0.05)
        raise RuntimeError('Chromium never opened a devtools port')

    def call(self, method, **params):
        """Issue one CDP command and return its result."""
        self._next_id += 1
        mine = self._next_id
        self._socket.send(json.dumps(
            {'id': mine, 'method': method, 'params': params}))
        while True:
            message = json.loads(self._socket.recv())
            if message.get('id') == mine:
                if 'error' in message:
                    raise RuntimeError('{}: {}'.format(method,
                                                       message['error']))
                return message.get('result', {})

    def js(self, expression):
        """Evaluate one expression in the page and return it by value."""
        result = self.call('Runtime.evaluate', expression=expression,
                           returnByValue=True, awaitPromise=True,
                           userGesture=True)
        if result.get('exceptionDetails'):
            raise RuntimeError(json.dumps(result['exceptionDetails'])[:600])
        return result['result'].get('value')

    def until(self, expression, timeout=180.0, note=''):
        """Poll one expression until it is truthy, or fail saying what it was."""
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            last = self.js(expression)
            if last:
                return last
            time.sleep(0.25)
        raise TimeoutError('timed out waiting for {} (last={!r})'.format(
            note or expression, last))

    def pointer(self, kind, x, y, buttons=1):
        """Deliver a real pointer event through Chromium's input pipeline."""
        self.call('Input.dispatchMouseEvent', type=kind, x=float(x),
                  y=float(y), button='left', buttons=buttons, clickCount=1,
                  pointerType='mouse')

    def close(self):
        """Kill the browser and remove its profile."""
        try:
            self._socket.close()
        except Exception:                          # noqa: BLE001 - teardown
            pass
        try:
            self._process.terminate()
            self._process.wait(timeout=15)
        except Exception:                          # noqa: BLE001 - teardown
            try:
                self._process.kill()
            except Exception:                      # noqa: BLE001 - teardown
                pass
        shutil.rmtree(self._profile, ignore_errors=True)


# ------------------------------------------------------- the operator's walk --

SESSION_LIVE = ('(async () => { const r = await fetch("/api/state");'
                ' const j = await r.json();'
                ' return j.state.session.launch_running === true'
                ' && (j.state.session.arm_ids || []).length > 0; })()')

GHOST_READY = ('(() => [...document.querySelectorAll('
               '\'button[data-act="ghost-show"]\')].some(n => !n.disabled))()')


def press_button(page, finder):
    """Click the first visible button the finder expression returns."""
    return page.js("""(() => {
      const n = %s;
      if (!n) return 'missing';
      if (n.disabled) return 'disabled';
      n.click();
      return 'ok';
    })()""" % finder)


def take_over_if_offered(page):
    """Press Take over, twice, when a stale claim offers it."""
    for _ in range(2):
        took = page.js("""(() => {
          const n = [...document.querySelectorAll('button')].find(
            e => (e.textContent || '').trim() === 'Take over'
                 && e.offsetParent !== null);
          if (!n) return false;
          n.click();
          return true;
        })()""")
        if not took:
            return
        time.sleep(1.0)


def walk_to_a_shown_ghost(page):
    """Simulate, both arms, Start, open Scene, press Ghost panda1."""
    page.until("document.readyState === 'complete'", timeout=60)
    time.sleep(2.0)
    take_over_if_offered(page)
    if not page.js(SESSION_LIVE):
        press_button(page, '[...document.querySelectorAll('
                           '\'button[data-act="mode"]\')].find('
                           'e => /Simulate/.test(e.textContent))')
        press_button(page, '[...document.querySelectorAll('
                           '\'button[data-act="arms"]\')].find('
                           'e => /both/.test(e.textContent))')
        time.sleep(0.6)
        take_over_if_offered(page)
        press_button(page, 'document.querySelector("#btnStart")')
        page.until(SESSION_LIVE, timeout=240, note='a live session')
        time.sleep(4.0)
    if page.js('document.querySelector("#sceneBar")'
               '.getAttribute("aria-expanded")') != 'true':
        press_button(page, 'document.querySelector("#sceneBar")')
    page.until('!!document.querySelector("canvas.ghost-canvas")', timeout=240,
               note='the scene canvas')
    page.until(GHOST_READY, timeout=240, note='the ghost toolbar')
    outcome = press_button(page, 'document.querySelector('
                                 '\'button[data-act="ghost-show"]'
                                 '[data-arm="panda1"]\')')
    if outcome != 'ok':
        raise RuntimeError('the Ghost panda1 button was {}'.format(outcome))
    page.until(WHERE, timeout=60, note='a drawn handle')
    # The scene panel opens below the fold in this window; the operator scrolls.
    page.js('document.querySelector("canvas.ghost-canvas")'
            '.scrollIntoView({block: "center"}); 1')
    time.sleep(1.5)


def reset_ghost(page):
    """
    Press the console's own Reset, so each press starts from the same pose.

    Without this the presses compound: two good drags carry the hand to the
    edge of the workspace and the third is refused for being out of reach --
    a true answer to a different question than the one this case asks.
    """
    press_button(page, 'document.querySelector('
                       '\'button[data-act="ghost-reset"][data-arm="panda1"]\')')
    time.sleep(2.0)


def orbit(page, dx, per_pass=300):
    """
    Turn the view, so the case also asks the question from another angle.

    It matters: which pick proxy lies nearest the camera at the flange changes
    with the orbit, and the elbow ring's band crosses in front of the knob from
    some angles and behind it from others. The caller checks the returned turn
    is a real one -- an orbit that silently did nothing would make the second
    half of the case a copy of the first, which is exactly what happened while
    this drag started at the canvas's CENTRE: the ghost stands there, the
    press was a grab, and the view never moved.
    """
    before = page.js(AZIMUTH)
    where = page.js(WHERE)
    start = page.js('(() => { const r = document.querySelector('
                    '"canvas.ghost-canvas").getBoundingClientRect();'
                    ' return [r.left + r.width * 0.12,'
                    ' r.top + r.height * 0.85]; })()')
    away = min(abs(start[0] - where['x']), abs(start[1] - where['y']))
    if away < 60:
        raise RuntimeError('the orbit would start {:.0f} px from the '
                           'handle, near enough to grab it'.format(away))
    # In passes, because one drag is bounded by the width of a 445 px canvas
    # and one canvas-width of drag is only about eight degrees. A turn that
    # small leaves the pick proxies stacked the way they already were.
    remaining = dx
    while remaining > 0:
        span = min(per_pass, remaining)
        remaining -= span
        page.pointer('mouseMoved', start[0], start[1], buttons=0)
        page.pointer('mousePressed', start[0], start[1])
        for step in range(1, 13):
            page.pointer('mouseMoved', start[0] + span * step / 12, start[1])
            time.sleep(0.03)
        page.pointer('mouseReleased', start[0] + span, start[1])
        time.sleep(0.6)
    time.sleep(1.0)
    after = page.js(AZIMUTH)
    return abs((after - before + 180) % 360 - 180)


GRABBED = ('document.querySelector("canvas.ghost-canvas")'
           '.classList.contains("ghost-dragging")')


def grab_and_drag(page, offset, drag):
    """
    Press ``offset`` px from the handle's centre, pull ``drag``, and report.

    Returns the metres the flange moved, one phrase naming what took the
    press, and the refusal sentences the server sent back. The three ways a
    press can end look identical in the distance alone -- a ring grab, a hand
    grab the solver refused, and a press that fell through to the orbit
    controller all move the ghost exactly zero -- so the answer comes from
    the call the console made while the gesture was held, not from the
    distance.
    """
    reset_ghost(page)
    page.js('window.__calls = '
            '{solve: 0, redundancy: 0, refused: 0, reasons: []}; 1')
    where = page.js(WHERE)
    before = page.js(FLANGE)
    eye = page.js(AZIMUTH)
    x, y = where['x'] + offset[0], where['y'] + offset[1]
    page.pointer('mouseMoved', x, y, buttons=0)
    time.sleep(0.2)
    page.pointer('mousePressed', x, y)
    time.sleep(0.3)
    gesture = page.js(GRABBED)
    for step in range(1, 8):
        page.pointer('mouseMoved', x + drag[0] * step / 7,
                     y + drag[1] * step / 7)
        time.sleep(0.14)
    time.sleep(1.4)
    page.pointer('mouseReleased', x + drag[0], y + drag[1])
    time.sleep(2.4)
    after = page.js(FLANGE)
    moved = max(abs(a - b) for a, b in zip(before, after))
    turned = abs((page.js(AZIMUTH) - eye + 180) % 360 - 180)
    calls = page.js('window.__calls') or {}
    if calls.get('redundancy'):
        went = 'the elbow ring'
    elif calls.get('solve'):
        went = 'the hand'
    elif gesture:
        went = 'a gizmo that asked for nothing'
    elif turned >= 1.0:
        went = 'the orbit controller ({:.0f} deg)'.format(turned)
    else:
        went = 'nothing at all'
    return moved, went, calls.get('reasons') or []


def press_the_handle(page, offset):
    """
    Press once at ``offset``, pulling the other way if the pose refuses.

    Every pull is answered by the same gizmo -- that is the property under
    test and it is checked on every attempt -- but not every DIRECTION is
    reachable from the ghost's home pose, and a solver that says "that point
    is outside this arm's reach" is answering a different question truthfully.
    So the case pulls one way, and if the arm cannot go there it pulls
    another, through all four quadrants, exactly as an operator would. It
    stops at the first pull that is answered by anything but the hand, because
    that is the failure this case exists to catch.
    """
    best, reasons = 0.0, []
    for drag in DRAGS:
        moved, went, said = grab_and_drag(page, offset, drag)
        if went != 'the hand':
            return moved, went, said
        best = max(best, moved)
        reasons = said
        if best >= MIN_MOVE_M:
            break
    return best, 'the hand', reasons


def run(url):
    """Drive the console at ``url`` and return the list of failure sentences."""
    page = Page(url, *VIEWPORT)
    failures = []
    try:
        walk_to_a_shown_ghost(page)
        where = page.js(WHERE)
        print('canvas {:.0f}x{:.0f} px, drawn handle radius {:.2f} px'.format(
            where['canvas']['w'], where['canvas']['h'], where['knobPx']),
            flush=True)
        for turn in ('as it opens', 'after orbiting'):
            if turn != 'as it opens':
                turned = orbit(page, ORBIT_PX)
                print('  the view turned {:.1f} degrees'.format(turned),
                      flush=True)
                if turned < MIN_ORBIT_DEG:
                    failures.append(
                        'the orbit turned the view {:.1f} degrees, under the '
                        '{:.0f} this case needs to be asking a second '
                        'question'.format(turned, MIN_ORBIT_DEG))
            print('  {}:'.format(turn), flush=True)
            for offset in OFFSETS_PX:
                moved, went, reasons = press_the_handle(page, offset)
                ok = went == 'the hand' and moved >= MIN_MOVE_M
                print('    press dx={:>3} dy={:>3} -> taken by {}, moved '
                      '{:.4f} m  {}'.format(offset[0], offset[1], went, moved,
                                            'ok' if ok else 'FAILED'),
                      flush=True)
                if went != 'the hand':
                    failures.append(
                        '{}, a press dx={} dy={} from the handle centre was '
                        'taken by {}'.format(turn, offset[0], offset[1], went))
                elif moved < MIN_MOVE_M:
                    failures.append(
                        '{}, a press dx={} dy={} reached the hand but moved '
                        'the ghost {:.4f} m in either direction (needs '
                        '{:.3f} m); the console said: {}'.format(
                            turn, offset[0], offset[1], moved, MIN_MOVE_M,
                            ' / '.join(reasons) or 'nothing'))
    finally:
        page.close()
    return failures


def main(argv=None):
    """Run the case and print one line per press, then a verdict."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default=None,
                        help='drive a console that is already running')
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--domain-id', type=int, default=DOMAIN_ID)
    arguments = parser.parse_args(argv)

    console = None
    if arguments.url is None:
        console = Console(arguments.port, arguments.domain_id)
        url = console.url
        print('console {} on ROS domain {}'.format(url, arguments.domain_id),
              flush=True)
    else:
        url = arguments.url
    try:
        failures = run(url)
    finally:
        if console is not None:
            console.close()
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS: every press on the handle moved the ghost')
    return 0


if __name__ == '__main__':
    sys.exit(main())
