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

WHAT IT ASSERTS, AND WHY THOSE NUMBERS
    The operator's window is 512x597, which gives the scene canvas 445x272 CSS
    px. At that size the whole ghost arm stands 46 px tall and its hand is
    12x10 px, while the drawn handle is a disc of HANDLE_MIN_PX = 9 px radius.
    A press on that disc must move the hand. Before the fix a press 9 px from
    its centre -- ON the blue knob the operator can see -- was taken by the
    ELBOW RING, whose pick band passes in front of the knob: the gesture began,
    the class went on the canvas, and the ghost did not move a millimetre.
    That is "the ghost cannot be dragged", exactly.

    So: press at 0, 9 and 12 px from the handle's centre; every one of them
    must move the ghost's flange by at least ``MIN_MOVE_M``.

RUNNING IT
    It needs a franka_web server with a live session and a reachable IK
    service -- Simulate is enough, and no robot is involved::

        python3 test/browser/real_console_grab.py --url http://127.0.0.1:8770

    It claims the operator lock on that server (taking over if it is held),
    starts a Simulate session if none is running, and leaves the session up.
    Point it at a scratch server, never at the one an operator is using.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

#: The operator's own window, measured from the browser they reported from.
VIEWPORT = (512, 597)

#: Offsets from the drawn handle's centre, in CSS px. 9 is HANDLE_MIN_PX --
#: the edge of the disc the operator can see; 12 is just outside the paint,
#: inside the forgiving target a primary gesture is owed.
OFFSETS_PX = (0, 9, 12)

#: A grab that worked moves the flange at least this far. A refused solve or a
#: ring grab moves it exactly zero, so the bar only has to be above noise.
MIN_MOVE_M = 0.010

BROWSERS = ('/usr/bin/google-chrome', '/usr/bin/google-chrome-stable',
            '/usr/bin/chromium', '/usr/bin/chromium-browser')


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

#: Capture the scene graph the page renders WITHOUT touching the page's code:
#: wrap the renderer so every frame records its scene and camera. three r111
#: is a UMD that assigns an empty ``window.THREE`` and then fills it, so this
#: polls rather than trusting a property setter.
PROBE = """
(() => {
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

SESSION_LIVE = ("(async () => { const r = await fetch('/api/state');"
                " const j = await r.json();"
                " return j.state.session.launch_running === true"
                " && (j.state.session.arm_ids || []).length > 0; })()")

GHOST_READY = ("(() => [...document.querySelectorAll("
               "'button[data-act=\\'ghost-show\\']')].some(n => !n.disabled))()")


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
    """A stale claim offers Take over; the operator presses it, twice."""
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
        press_button(page, "[...document.querySelectorAll("
                           "'button[data-act=\"mode\"]')].find("
                           "e => /Simulate/.test(e.textContent))")
        press_button(page, "[...document.querySelectorAll("
                           "'button[data-act=\"arms\"]')].find("
                           "e => /both/.test(e.textContent))")
        time.sleep(0.6)
        take_over_if_offered(page)
        press_button(page, "document.querySelector('#btnStart')")
        page.until(SESSION_LIVE, timeout=240, note='a live session')
        time.sleep(4.0)
    if page.js("document.querySelector('#sceneBar')"
               ".getAttribute('aria-expanded')") != 'true':
        press_button(page, "document.querySelector('#sceneBar')")
    page.until("!!document.querySelector('canvas.ghost-canvas')", timeout=240,
               note='the scene canvas')
    page.until(GHOST_READY, timeout=240, note='the ghost toolbar')
    outcome = press_button(page, "document.querySelector("
                                 "'button[data-act=\"ghost-show\"]"
                                 "[data-arm=\"panda1\"]')")
    if outcome != 'ok':
        raise RuntimeError('the Ghost panda1 button was {}'.format(outcome))
    page.until(WHERE, timeout=60, note='a drawn handle')
    # The scene panel opens below the fold in this window; the operator scrolls.
    page.js("document.querySelector('canvas.ghost-canvas')"
            ".scrollIntoView({block: 'center'}); 1")
    time.sleep(1.5)


def reset_ghost(page):
    """Press the console's own Reset, so each press starts from the same pose.

    Without this the presses compound: two good drags carry the hand to the
    edge of the workspace and the third is refused for being out of reach --
    a true answer to a different question than the one this case asks.
    """
    press_button(page, "document.querySelector("
                       "'button[data-act=\"ghost-reset\"][data-arm=\"panda1\"]')")
    time.sleep(2.0)


def orbit(page, dx):
    """Turn the view, so the case also asks the question from another angle.

    It matters: which pick proxy lies nearest the camera at the flange changes
    with the orbit, and the elbow ring's band crosses in front of the knob from
    some angles and behind it from others.
    """
    canvas = page.js("(() => { const r = document.querySelector("
                     "'canvas.ghost-canvas').getBoundingClientRect();"
                     " return [r.left + r.width / 2, r.top + r.height / 2]; })()")
    page.pointer('mouseMoved', canvas[0], canvas[1], buttons=0)
    page.pointer('mousePressed', canvas[0], canvas[1])
    for step in range(1, 13):
        page.pointer('mouseMoved', canvas[0] + dx * step / 12, canvas[1])
        time.sleep(0.03)
    page.pointer('mouseReleased', canvas[0] + dx, canvas[1])
    time.sleep(1.0)


def grab_and_drag(page, offset_px):
    """Press `offset_px` from the handle's centre and drag; return metres moved."""
    reset_ghost(page)
    where = page.js(WHERE)
    before = page.js(FLANGE)
    x, y = where['x'] + offset_px, where['y']
    page.pointer('mouseMoved', x, y, buttons=0)
    time.sleep(0.2)
    page.pointer('mousePressed', x, y)
    time.sleep(0.3)
    for step in range(6, 48, 6):
        page.pointer('mouseMoved', x - step, y + step * 0.4)
        time.sleep(0.14)
    time.sleep(1.4)
    page.pointer('mouseReleased', x - 42, y + 17)
    time.sleep(2.4)
    after = page.js(FLANGE)
    moved = max(abs(a - b) for a, b in zip(before, after))
    return where, moved


def main(argv=None):
    """Run the case and print one line per press, then a verdict."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8770')
    parser.add_argument('--keep-open', action='store_true')
    arguments = parser.parse_args(argv)

    page = Page(arguments.url, *VIEWPORT)
    failures = []
    try:
        walk_to_a_shown_ghost(page)
        where = page.js(WHERE)
        print('canvas {:.0f}x{:.0f} px, drawn handle radius {:.2f} px'.format(
            where['canvas']['w'], where['canvas']['h'], where['knobPx']))
        for turn in ('as it opens', 'after orbiting 160 px'):
            if turn != 'as it opens':
                orbit(page, 160)
            print('  {}:'.format(turn))
            for offset in OFFSETS_PX:
                where, moved = grab_and_drag(page, offset)
                verdict = 'ok' if moved >= MIN_MOVE_M else 'FAILED'
                print('    press {:>2} px from the handle centre '
                      '-> ghost moved {:.4f} m  {}'.format(
                          offset, moved, verdict))
                if moved < MIN_MOVE_M:
                    failures.append(
                        '{}, a press {} px from the handle centre moved the '
                        'ghost {:.4f} m (needs {:.3f} m)'.format(
                            turn, offset, moved, MIN_MOVE_M))
    finally:
        if not arguments.keep_open:
            page.close()
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS: every press on the drawn handle moved the ghost')
    return 0


if __name__ == '__main__':
    sys.exit(main())
