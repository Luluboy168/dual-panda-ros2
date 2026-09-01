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
A protocol-faithful Robotiq 2F-85 emulator on a pty.

``FakeGripper`` opens a pty pair, speaks Modbus RTU on one side and hands out
the device path for the other, so the driver opens a real serial port and
writes real bytes. Nothing is mocked: the driver cannot tell it is not talking
to an adapter. This is the only way any of this is provable before the
hardware lands, so the emulator is held to the manual rather than to
convenience -- wrong slave ids are ignored rather than answered, a bad inbound
CRC is silently dropped as a real RTU device drops it, and unsupported
function codes get no reply at all, because S1 section 4.7.1 says this gripper
implements no exception responses.

**Three things here are inferences, not transcriptions, and they are labelled
so a bring-up divergence is read as a finding rather than as a driver bug.**

1. The fake latches fault ``0x07`` when rGTO is written while gSTA == 0, and
   ``0x05`` when it is written while gSTA == 1. S1 documents those codes only
   as *meanings* -- "the activation bit must be set prior to action" and
   "action delayed" -- and never states that the gripper latches them in
   response to such a write. The latching trigger is inferred from the
   meanings.
2. The fake runs auto-release as the two-state sequence ``0x0B`` then
   ``0x0F``, moving the fingers while ``0x0B`` stands. S1 states neither the
   transition nor that ``0x0B`` permits motion; both are inferred from
   "automatic release in progress" and "automatic release completed".
3. The motor current curve is **invented**. The manual publishes no current
   model at all, so only the *shape* is claimed -- idle below moving, moving
   below stalled -- and no test asserts a magnitude.

A real gripper that behaves differently on bring-up day is a finding to
record, not a red test to fix by editing this file until it agrees with one
hardware session.

**A pty is not RS-485.** It ignores the baud rate, has no line turnaround, no
electrical noise and no half-duplex collisions, and its timing is
microseconds where a real frame takes about a millisecond. The driver's 5 ms
inter-frame rule is therefore enforced by the driver against a monotonic
clock, not by this wire, which is what makes it testable here at all.

The mechanism advances **lazily**: every serviced frame first integrates the
fingers from the last update to now. There is no physics thread. That makes
the emulator deterministic under an injected clock and faithful under the real
one, with one code path for both.
"""

import errno
import json
import os
import select
import threading
import time
import tty

from franka_robotiq import protocol
from franka_robotiq import registers
from franka_robotiq import units

#: 3.5 characters at 115200 8N1 is about 0.3 ms. 1 ms is generous enough to
#: keep a chunked write together and short enough that a deliberate split in a
#: test reads as two frames -- which is what makes the "garbage in the buffer"
#: and "partial frame" cases reproducible.
FRAME_GAP_S = 0.001

#: How long the serve loop blocks in select before re-checking the frame gap.
_SELECT_TIMEOUT_S = 0.0005

#: Fault codes that permit rGTO motion while they stand. 0x08 is the minor,
#: self-recovering overheat, which S1 treats as "it resumes by itself once it
#: cools down". Auto-release (0x0B) also moves the fingers, but not through
#: rGTO -- it has its own branch, and gGTO reads 0 throughout.
_MOTION_PERMITTING_FAULTS = (0x00, 0x08)
#: Fault codes cleared by the rACT falling edge: the priority pair and the
#: minor code. A latched major fault survives it, which is what makes the
#: rising edge the one recovery.
_FALLING_EDGE_CLEARS = (0x05, 0x07, 0x08)
#: Priority codes, cleared once the precondition they complain about holds.
_PRIORITY_FAULTS = (0x05, 0x07)

_AUTO_RELEASE_RUNNING = 0x0B
_AUTO_RELEASE_DONE = 0x0F


class FakeGripper:
    """A Robotiq 2F-85 emulator serving Modbus RTU on a pty."""

    def __init__(self, *, stroke_mm=85.0, activation_duration_s=0.75,
                 clock=None, slave_id=registers.SLAVE_ID):
        """Build an emulator. Nothing is opened until :meth:`start`."""
        self._stroke_mm = stroke_mm
        self._activation_duration_s = activation_duration_s
        self._clock = clock if clock is not None else time.monotonic
        self._slave_id = slave_id

        self._master = None
        self._slave = None
        self._wake_r = None
        self._wake_w = None
        self._thread = None
        self._running = False
        self._lock = threading.RLock()

        # Robot output shadow (what the driver last wrote).
        self._r_act = 0
        self._r_gto = 0
        self._r_atr = 0
        self._r_ard = 0
        self._r_pr = 0
        self._r_sp = 0
        self._r_fr = 0

        # Gripper state.
        self._g_sta = registers.GSta.RESET
        self._g_obj = registers.GObj.MOVING
        self._g_flt = 0x00
        self._g_pr = 0
        self._position = 0.0            # float count, 0 open .. 255 closed
        self._activation_started = None
        self._last_update = None

        self._object_count = None
        self._corrupt_mode = None
        self._delay_s = 0.0

        self.stats = {
            'frames_seen': 0,
            'frames_answered': 0,
            'frames_dropped_crc': 0,
            'frames_dropped_short': 0,
            'frames_dropped_fc': 0,
            'frames_ignored_slave': 0,
            'replies_corrupted': 0,
            'replies_dropped': 0,
        }

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        """Open the pty pair and start serving. Idempotent."""
        with self._lock:
            if self._running:
                return
            self._open_pty()
            self._wake_r, self._wake_w = os.pipe()
            self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True,
                                        name='fake-robotiq')
        self._thread.start()

    def _open_pty(self):
        """
        Create one raw pty pair and keep both ends open.

        Raw is not optional: a default pty is canonical, with echo and newline
        translation, so a 0x0A byte inside a Modbus payload would come back as
        0x0D 0x0A and every frame containing one would corrupt. Keeping the
        emulator's own slave fd open for the pty's whole life is what lets the
        driver close and reopen the same path without the master seeing EIO.
        """
        master, slave = os.openpty()
        tty.setraw(master)
        tty.setraw(slave)
        self._master = master
        self._slave = slave

    def stop(self):
        """Stop serving and close every descriptor. Idempotent, never raises."""
        with self._lock:
            if not self._running:
                self._close_all()
                return
            self._running = False
        self._wake()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        with self._lock:
            self._close_all()

    def _close_all(self):
        """Close whichever descriptors are still open."""
        for name in ('_master', '_slave', '_wake_r', '_wake_w'):
            handle = getattr(self, name)
            if handle is None:
                continue
            setattr(self, name, None)
            try:
                os.close(handle)
            except OSError:
                pass

    def _wake(self):
        """Nudge the serve loop out of select."""
        if self._wake_w is not None:
            try:
                os.write(self._wake_w, b'.')
            except OSError:
                pass

    @property
    def port(self):
        """
        Return the device path the driver opens.

        This is the pty's slave-side name. The emulator serves on the master
        side, so closing the master is exactly what an unplugged adapter looks
        like from the driver's fd.
        """
        if self._slave is None:
            raise RuntimeError('the fake gripper is not started')
        return os.ttyname(self._slave)

    def __enter__(self):
        """Start serving and return self."""
        self.start()
        return self

    def __exit__(self, *exc):
        """Stop serving. Never suppresses an exception."""
        self.stop()
        return False

    # -- the wire ----------------------------------------------------------

    def _serve(self):
        """Accumulate bytes into frames and answer them, until stopped."""
        buffer = bytearray()
        last_byte_at = None
        while True:
            with self._lock:
                if not self._running:
                    return
                master = self._master
                wake = self._wake_r
            if wake is None:
                return
            watched = [wake] + ([master] if master is not None else [])
            try:
                readable, _, _ = select.select(watched, [], [],
                                               _SELECT_TIMEOUT_S)
            except (OSError, ValueError):
                buffer.clear()
                last_byte_at = None
                continue
            if wake in readable:
                try:
                    os.read(wake, 4096)
                except OSError:
                    pass
                buffer.clear()
                last_byte_at = None
                continue
            if master is not None and master in readable:
                try:
                    chunk = os.read(master, 4096)
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        chunk = b''
                    else:
                        continue
                if chunk:
                    buffer += chunk
                    last_byte_at = time.monotonic()
            while buffer:
                length = _expected_request_length(buffer)
                if length is None or len(buffer) < length:
                    break
                frame = bytes(buffer[:length])
                del buffer[:length]
                if not buffer:
                    last_byte_at = None
                self._handle_frame(frame)
            if buffer and last_byte_at is not None and \
                    time.monotonic() - last_byte_at >= FRAME_GAP_S:
                frame = bytes(buffer)
                buffer.clear()
                last_byte_at = None
                self._handle_frame(frame)

    def _handle_frame(self, frame):
        """Validate one inbound frame and answer it, or drop it silently."""
        with self._lock:
            self.stats['frames_seen'] += 1
            if len(frame) < 4:
                self.stats['frames_dropped_short'] += 1
                return
            if frame[0] != self._slave_id:
                # Not addressed to this device: ignored, never answered.
                self.stats['frames_ignored_slave'] += 1
                return
            if not protocol.crc_ok(frame):
                # Real RTU devices drop a bad CRC in silence. That silence is
                # what forces the driver's timeout path to exist.
                self.stats['frames_dropped_crc'] += 1
                return
            self._advance(self._clock())
            function_code = frame[1]
            if function_code in (registers.FC_READ_HOLDING,
                                 registers.FC_READ_INPUT):
                reply = self._read_reply(frame, function_code)
            elif function_code == registers.FC_WRITE_SINGLE:
                reply = self._write_single_reply(frame)
            elif function_code == registers.FC_WRITE_MULTIPLE:
                reply = self._write_multiple_reply(frame)
            else:
                self.stats['frames_dropped_fc'] += 1
                return
            if reply is None:
                self.stats['frames_dropped_short'] += 1
                return
            reply, delay_s = self._apply_corruption(reply)
            if reply is None:
                self.stats['replies_dropped'] += 1
                return
            self.stats['frames_answered'] += 1
        if delay_s > 0:
            # The reply was composed from the state as it stood when the
            # request arrived; it lands late, which is exactly the case the
            # driver's resync branch exists for.
            time.sleep(delay_s)
        self._send(reply)

    def _apply_corruption(self, reply):
        """Return the reply to send and any pending delay, consuming both."""
        mode, self._corrupt_mode = self._corrupt_mode, None
        delay_s, self._delay_s = self._delay_s, 0.0
        if mode is None:
            return reply, delay_s
        self.stats['replies_corrupted'] += 1
        if mode == 'drop':
            return None, delay_s
        if mode == 'crc':
            broken = bytearray(reply)
            broken[-1] ^= 0xFF
            return bytes(broken), delay_s
        if mode == 'short':
            return reply[:-2], delay_s
        if mode == 'garbage':
            return b'\xa5' * len(reply), delay_s
        if mode == 'foreign_slave':
            other = bytearray(reply)
            other[0] = (self._slave_id + 1) & 0xFF
            return protocol.append_crc(bytes(other[:-2])), delay_s
        if mode == 'exception':
            # The frame a real 2F-85 never sends: the function code with its
            # high bit set. S1 section 4.7.1 says exception responses are not
            # implemented, so the driver must call this "not from the gripper"
            # rather than decode it.
            odd = bytearray(reply)
            odd[1] |= 0x80
            return protocol.append_crc(bytes(odd[:-2])), delay_s
        raise ValueError('unknown corruption mode {!r}'.format(mode))

    def _send(self, reply):
        """Write one reply on the master side, ignoring a vanished pty."""
        with self._lock:
            master = self._master
        if master is None:
            return
        try:
            os.write(master, reply)
        except OSError:
            pass

    def _read_reply(self, frame, function_code):
        """Build the reply to an FC03/FC04 read, or None if it is malformed."""
        address = (frame[2] << 8) | frame[3]
        count = (frame[4] << 8) | frame[5]
        if count < 1 or count > registers.BLOCK_REGISTERS:
            return None
        if address == registers.IN_FIRST_REGISTER:
            block = self._input_bytes()
        elif address == registers.OUT_FIRST_REGISTER:
            block = self._output_bytes()
        else:
            return None
        payload = block[:2 * count]
        head = bytes((self._slave_id, function_code, len(payload)))
        return protocol.append_crc(head + payload)

    def _write_single_reply(self, frame):
        """Apply an FC06 write and echo it back, as a real device does."""
        address = (frame[2] << 8) | frame[3]
        value = (frame[4] << 8) | frame[5]
        if not self._apply_output(address, protocol.registers_to_bytes([value])):
            return None
        return bytes(frame)

    def _write_multiple_reply(self, frame):
        """Apply an FC16 write and answer with the address/count echo."""
        address = (frame[2] << 8) | frame[3]
        count = (frame[4] << 8) | frame[5]
        byte_count = frame[6]
        if byte_count != 2 * count or len(frame) != 9 + byte_count:
            return None
        payload = frame[7:7 + byte_count]
        if not self._apply_output(address, payload):
            return None
        return protocol.append_crc(
            bytes((self._slave_id, registers.FC_WRITE_MULTIPLE,
                   frame[2], frame[3], frame[4], frame[5])))

    # -- the mechanism -----------------------------------------------------

    def _apply_output(self, address, payload):
        """
        Write ``payload`` into the output block at ``address``.

        Returns False when the write does not land inside the block, which is
        answered with silence rather than with an exception response.
        """
        if address < registers.OUT_FIRST_REGISTER:
            return False
        offset = 2 * (address - registers.OUT_FIRST_REGISTER)
        if offset + len(payload) > registers.BLOCK_BYTES:
            return False
        block = bytearray(self._output_request_bytes())
        block[offset:offset + len(payload)] = payload
        fields = protocol.unpack_output(bytes(block))
        self._set_output(**fields)
        return True

    def _output_request_bytes(self):
        """Return the six ACTION REQUEST bytes as last written."""
        return protocol.pack_output(
            r_act=self._r_act, r_gto=self._r_gto, r_atr=self._r_atr,
            r_ard=self._r_ard, r_pr=self._r_pr, r_sp=self._r_sp,
            r_fr=self._r_fr)

    def _set_output(self, *, r_act, r_gto, r_atr, r_ard, r_pr, r_sp, r_fr):
        """Apply a decoded ACTION REQUEST, running the edges it implies."""
        was_act = self._r_act
        was_atr = self._r_atr
        self._r_act = r_act
        self._r_gto = r_gto
        self._r_atr = r_atr
        self._r_ard = r_ard
        self._r_pr = r_pr
        self._r_sp = r_sp
        self._r_fr = r_fr
        self._g_pr = r_pr               # the echo is immediate, always

        if r_act and not was_act:
            # The rising edge starts a fresh activation and clears every
            # fault class, which is why it is the one documented recovery.
            self._g_flt = 0x00
            self._g_sta = registers.GSta.ACTIVATING
            self._activation_started = self._clock()
        elif was_act and not r_act:
            # The falling edge clears the priority and minor classes only
            # (S1 section 4.3, "clearing rACT also clears fault status"); a
            # latched major fault survives it (S1 section 4.4, "major faults
            # require a reset: a rising edge on rACT").
            if self._g_flt in _FALLING_EDGE_CLEARS:
                self._g_flt = 0x00
            self._g_sta = registers.GSta.RESET
            self._activation_started = None

        if r_atr and not was_atr:
            # S1 section 4.3: rATR overrides everything except rACT. The pair
            # 0x0B -> 0x0F that follows is INFERRED, not documented.
            self._g_flt = _AUTO_RELEASE_RUNNING
            return

        if (r_gto and self._g_sta != registers.GSta.ACTIVATED
                and self._g_flt in (0x00,) + _PRIORITY_FAULTS):
            # INFERRED, not documented: S1 gives 0x05 and 0x07 as meanings and
            # never states that the gripper latches them in response to an
            # early rGTO write. The guard keeps a latched major fault from
            # being overwritten by a priority one -- a major fault must not be
            # downgraded by asking for motion.
            self._g_flt = (0x05 if self._g_sta == registers.GSta.ACTIVATING
                           else 0x07)

    def _motion_allowed(self):
        """Return True when the fingers may follow rGTO right now."""
        return (self._r_gto == 1
                and self._g_sta == registers.GSta.ACTIVATED
                and self._g_flt in _MOTION_PERMITTING_FAULTS)

    def _g_gto(self):
        """Return the gGTO echo: 0 while stopped, activating or releasing."""
        return 1 if self._motion_allowed() else 0

    def _counts_per_second(self):
        """Finger speed in counts per second at the commanded rSP."""
        speed_mm_s = units.count_to_speed_mm_s(self._r_sp)
        return speed_mm_s * units.COUNT_MAX / self._stroke_mm

    def _advance(self, now):
        """Integrate the mechanism from the last update to ``now``."""
        if self._last_update is None:
            self._last_update = now
            return
        delta = now - self._last_update
        self._last_update = now
        if delta <= 0:
            return

        if (self._g_sta == registers.GSta.ACTIVATING
                and self._activation_started is not None
                and now - self._activation_started >= self._activation_duration_s):
            self._g_sta = registers.GSta.ACTIVATED
            if self._g_flt in _PRIORITY_FAULTS:
                # A priority fault says "you asked too early". Once the
                # precondition holds it has nothing left to complain about.
                self._g_flt = 0x00

        step = self._counts_per_second() * delta
        if self._g_flt == _AUTO_RELEASE_RUNNING:
            # Auto-release is a motion that runs while faulted: rATR overrides
            # everything except rACT, and "in progress" would contradict
            # itself if the code forbade motion. INFERRED, see the module
            # docstring.
            target = 0.0 if self._r_ard else float(units.COUNT_MAX)
            self._position = _towards(self._position, target, step)
            self._g_obj = registers.GObj.MOVING
            if self._position == target:
                self._g_flt = _AUTO_RELEASE_DONE
                self._g_sta = registers.GSta.RESET
            return

        if not self._motion_allowed():
            self._g_obj = registers.GObj.MOVING
            return

        target = float(self._r_pr)
        start = self._position
        moved = _towards(start, target, step)
        blocked = None
        if self._object_count is not None:
            obstruction = float(self._object_count)
            if moved > obstruction >= start:
                moved = obstruction
                blocked = registers.GObj.CLOSED_ON_OBJECT
            elif moved < obstruction <= start:
                moved = obstruction
                blocked = registers.GObj.OPENED_ON_OBJECT
        self._position = moved
        if blocked is not None:
            self._g_obj = blocked
        elif self._position == target:
            self._g_obj = registers.GObj.AT_POSITION
        else:
            self._g_obj = registers.GObj.MOVING

    def _g_po(self):
        """Return the reported encoder position, as an integer count."""
        return int(round(self._position))

    def _g_cu(self):
        """Return a plausible motor current. The curve is INVENTED (see above)."""
        if self._g_gto() == 0 and self._g_flt != _AUTO_RELEASE_RUNNING:
            return 2
        if self._g_obj in (registers.GObj.CLOSED_ON_OBJECT,
                           registers.GObj.OPENED_ON_OBJECT):
            return min(255, 24 + self._r_fr // 2)
        if self._g_po() != self._g_pr or self._g_flt == _AUTO_RELEASE_RUNNING:
            return min(255, 8 + self._r_fr // 8)
        return 2

    def _output_bytes(self):
        """Return the output block as a reader of that register would see it."""
        return self._output_request_bytes()

    def _input_bytes(self):
        """Return the six GRIPPER STATUS bytes for the state as it stands."""
        g_gto = self._g_gto()
        g_obj = self._g_obj if g_gto else registers.GObj.MOVING
        status = ((self._r_act & 0x01) << registers.BIT_GACT
                  | (g_gto & 0x01) << registers.BIT_GGTO
                  | (int(self._g_sta) & registers.MASK_GSTA)
                  << registers.SHIFT_GSTA
                  | (int(g_obj) & registers.MASK_GOBJ) << registers.SHIFT_GOBJ)
        return bytes((status, 0x00, self._g_flt & registers.MASK_GFLT,
                      self._g_pr, self._g_po(), self._g_cu()))

    # -- injection surface -------------------------------------------------

    def set_object(self, width_mm=None):
        """
        Place, move or remove an obstruction the fingers cannot pass.

        ``None`` removes it. The obstruction takes effect immediately, with no
        motion required, and it survives activation and fault injection, so a
        caller can seed it once at start-up and never touch it again.
        """
        with self._lock:
            if width_mm is None:
                self._object_count = None
                return
            self._object_count = units.width_mm_to_count(
                width_mm, stroke_mm=self._stroke_mm)

    def inject_fault(self, code):
        """
        Latch one documented fault code. Codes outside the table are refused.

        A fault byte the manual's table does not list is a wire-level concern,
        tested where frames are decoded; this emulator does not pretend a real
        gripper produces one.
        """
        if code not in registers.FAULTS:
            raise ValueError(
                '0x{:02x} is not a documented gFLT code; the fake refuses to '
                'produce a byte the manual does not list'.format(code))
        with self._lock:
            if code == 0x00:
                self._g_flt = 0x00
                return
            self._g_flt = code
            if code == _AUTO_RELEASE_DONE:
                self._g_sta = registers.GSta.RESET

    def clear_fault(self):
        """
        Clear the minor, self-recovering fault (0x08) and nothing else.

        The priority pair clears on the rACT falling edge or once activation
        completes; a latched major fault clears on the rACT rising edge alone.
        Neither is this method's to clear.
        """
        with self._lock:
            if self._g_flt == 0x08:
                self._g_flt = 0x00

    def unplug(self):
        """Close the master side: the driver's fd then behaves as unplugged."""
        with self._lock:
            master, self._master = self._master, None
        if master is not None:
            try:
                os.close(master)
            except OSError:
                pass
        self._wake()

    def replug(self):
        """
        Open a NEW pty and expose its path. PART-A tests only.

        It cannot reuse the old device name, and that is faithful: a replugged
        adapter can come back on a different node, which is exactly why
        reconnection re-resolves the configured name instead of reopening a
        path fixed at construction.

        The mechanism comes back in its power-on state -- rACT clear, gSTA 0,
        no fault, fingers where they were. S1 section 4.3 says a power loss
        sets rACT and that it must then be cleared and set again, so this is
        the case an automatic re-activation policy exists to handle, and the
        emulator gives that policy something real to act on.
        """
        with self._lock:
            stale = [getattr(self, name) for name in ('_master', '_slave')]
            # The new pty is opened BEFORE the old descriptors are closed, so
            # the kernel cannot hand back the number that was just freed. A
            # replug that reused the old device name would quietly make the
            # reconnect path look correct while never exercising it.
            self._open_pty()
            for handle in stale:
                if handle is not None:
                    try:
                        os.close(handle)
                    except OSError:
                        pass
            self._r_act = 0
            self._r_gto = 0
            self._r_atr = 0
            self._r_ard = 0
            self._g_sta = registers.GSta.RESET
            self._g_obj = registers.GObj.MOVING
            self._g_flt = 0x00
            self._activation_started = None
        self._wake()
        return self.port

    def set_activation_duration_s(self, seconds):
        """Set how long the activation routine takes. PART-A tests only."""
        with self._lock:
            self._activation_duration_s = seconds

    def corrupt_next_reply(self, mode):
        """
        Damage exactly the next reply.

        Modes: ``'crc'``, ``'short'``, ``'garbage'``, ``'drop'``,
        ``'foreign_slave'`` and ``'exception'``.
        """
        if mode not in ('crc', 'short', 'garbage', 'drop', 'foreign_slave',
                        'exception'):
            raise ValueError('unknown corruption mode {!r}'.format(mode))
        with self._lock:
            self._corrupt_mode = mode

    def delay_next_reply(self, seconds):
        """Hold the next reply back, so it arrives after the caller gave up."""
        with self._lock:
            self._delay_s = seconds

    # -- introspection for tests ------------------------------------------

    def snapshot(self):
        """Return the emulator's own state as a dict, for assertions in tests."""
        with self._lock:
            self._advance(self._clock())
            return {
                'r_act': self._r_act, 'r_gto': self._r_gto,
                'r_atr': self._r_atr, 'r_ard': self._r_ard,
                'r_pr': self._r_pr, 'r_sp': self._r_sp, 'r_fr': self._r_fr,
                'g_sta': int(self._g_sta), 'g_obj': int(self._g_obj),
                'g_gto': self._g_gto(), 'g_flt': self._g_flt,
                'g_pr': self._g_pr, 'g_po': self._g_po(),
                'g_cu': self._g_cu(),
            }


def _towards(value, target, step):
    """Move ``value`` toward ``target`` by at most ``step``, never past it."""
    if value < target:
        return min(target, value + step)
    if value > target:
        return max(target, value - step)
    return value


def _expected_request_length(buffer):
    """Length of the request at the head of ``buffer``, or None if unknown."""
    if len(buffer) < 2:
        return None
    function_code = buffer[1]
    if function_code in (registers.FC_READ_HOLDING, registers.FC_READ_INPUT,
                         registers.FC_WRITE_SINGLE):
        return 8
    if function_code == registers.FC_WRITE_MULTIPLE:
        if len(buffer) < 7:
            return None
        return 9 + buffer[6]
    return None


class TranscriptRecorder:
    """
    Wrap a serial factory and record every (t, tx, rx) triple as JSON lines.

    The same wrapper records the emulator today and a real gripper on bring-up
    day, through the driver's ``open_serial`` seam. Replaying a recorded
    transcript against the emulator and failing on the first differing reply
    is the only true proof of fidelity there is; it is built now and armed
    later.

    The file is a wire log and may end up attached to a bug report, so it
    carries no device paths, no serial numbers and no host names -- one JSON
    object per line, ``{"t": <seconds since the first frame>, "tx": "<hex>",
    "rx": "<hex>"}``, with ``"rx": null`` for a reply that never came.
    """

    def __init__(self, factory, path):
        """Record the traffic of every port ``factory`` opens into ``path``."""
        self._factory = factory
        self._path = path
        self._handle = None
        self._first_at = None
        self._lock = threading.RLock()

    def __call__(self, *args, **kwargs):
        """Open a port through the wrapped factory and record its traffic."""
        if self._handle is None:
            self._handle = open(self._path, 'a')
        return _RecordingSerial(self._factory(*args, **kwargs), self)

    def _emit(self, started_at, tx, rx):
        """Write one transaction record."""
        with self._lock:
            if self._handle is None:
                return
            if self._first_at is None:
                self._first_at = started_at
            record = {
                't': round(started_at - self._first_at, 6),
                'tx': tx.hex(),
                'rx': rx.hex() if rx else None,
            }
            self._handle.write(json.dumps(record, sort_keys=True) + '\n')
            self._handle.flush()

    def close(self):
        """Close the transcript file. Idempotent."""
        with self._lock:
            handle, self._handle = self._handle, None
        if handle is not None:
            handle.close()


class _RecordingSerial:
    """A serial object that records each write and the reads that follow it."""

    def __init__(self, inner, recorder):
        """Wrap ``inner`` and report each completed transaction to ``recorder``."""
        self._inner = inner
        self._recorder = recorder
        self._tx = None
        self._rx = bytearray()
        self._started_at = None

    def _flush_record(self):
        """Emit the transaction that has just ended, if there is one."""
        if self._tx is None:
            return
        self._recorder._emit(self._started_at, self._tx, bytes(self._rx))
        self._tx = None
        self._rx = bytearray()

    def write(self, data):
        """Record the start of a new transaction and pass the write through."""
        self._flush_record()
        self._tx = bytes(data)
        self._rx = bytearray()
        self._started_at = time.monotonic()
        return self._inner.write(data)

    def read(self, size=1):
        """Pass the read through, accumulating what came back."""
        chunk = self._inner.read(size)
        if chunk:
            self._rx += chunk
        return chunk

    def reset_input_buffer(self):
        """Pass the flush through."""
        return self._inner.reset_input_buffer()

    def close(self):
        """Emit the last transaction and close the wrapped port."""
        self._flush_record()
        return self._inner.close()

    @property
    def is_open(self):
        """Whether the wrapped port is open."""
        return self._inner.is_open
