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
The Robotiq 2F-85 transport and state machine, and nothing else.

This file owns one serial link. It converts no units -- it deals in raw
counts, and the caller converts through ``units.py``. It resolves no device
names -- the caller hands it a path that ``discovery.py`` produced. It writes
no sentence an operator reads -- the node composes those from the fault
record. And it imports ``pyserial`` lazily, inside the two functions that
need it, so the whole protocol layer can be tested on a machine where the
serial module is not installed.

**Link health is decided in exactly one place**, ``_record_failure``, and
there is exactly one failure counter in this system. Three consecutive failed
transactions -- a read timeout, a short frame, a bad CRC, an unexpected reply
or a transport error -- close the port and raise ``LinkDownError``. A caller
must not keep a second "three in a row" counter of its own; the first two
failures arrive as ``TransientReadError`` and mean "stay up, the driver is
counting".

**Reconnect: the driver owns the mechanism, the caller owns the policy.** The
retry interval, the log throttle and any automatic re-activation after a power
loss are the caller's. What this file provides is a ``close()`` that never
raises, an ``open()`` that raises ``LinkDownError`` rather than a bare
transport exception, and a ``connected`` property. There is deliberately no
``reopen()``: after a replug the adapter can come back on a different device
node, so a method operating on the path fixed at construction would faithfully
reopen a node that is gone. Reconnection is re-resolve, then construct a fresh
instance.

Wire behaviour is fixed by the manual (S1, revision 2018/05/23): one command
is an FC16 write of the six ACTION REQUEST bytes; one status read is an FC03
read of the three GRIPPER STATUS registers; and consecutive frames are at
least 5 ms apart, measured against a monotonic clock from the end of the
previous transaction, because a message is not over until its reply has been
read.
"""

from dataclasses import dataclass
import logging
import time

from franka_robotiq import protocol
from franka_robotiq import registers

_LOG = logging.getLogger(__name__)

#: Three consecutive read failures declare the link down.
FAILURE_LIMIT = 3
#: S1 section 4.7.1's >= 5 ms rule, enforced against a monotonic clock and
#: independent of the caller's poll rate.
INTERFRAME_GAP_S = registers.MIN_INTERFRAME_GAP_S
#: Poll interval while waiting for gSTA to change during activation. Internal:
#: nothing outside this module has a reason to set it.
ACTIVATION_POLL_S = 0.05
#: pyserial write timeout: a write that cannot drain must fail, not block the
#: caller's executor forever. Internal, like ACTIVATION_POLL_S.
WRITE_TIMEOUT_S = 0.5

#: The one clock seam. Every clock read in this file goes through the instance
#: attribute set from this name in ``__init__``; it is not a constructor
#: keyword, because the constructor signature is pinned across parts.
_MONOTONIC = time.monotonic


class RobotiqError(Exception):
    """Base class for every failure this driver reports."""


class TransientReadError(RobotiqError):
    """One failed transaction, below the failure limit. The link is still up."""


class LinkDownError(RobotiqError):
    """
    The link is not usable. The port is closed and ``connected`` is False.

    Two occasions produce it: the ``FAILURE_LIMIT``-th consecutive read
    failure, and ``open()`` on a device that cannot be opened.
    """


class ActivationTimeout(RobotiqError):
    """
    ``activate()`` did not reach gSTA == 3 inside its timeout.

    The message names the ``activation_timeout_s`` configuration key, because
    the routine's real duration is not documented anywhere in the manual and
    raising the key is the correct response to a gripper that is simply slow.
    """


def _transport_errors():
    """
    Exception types the transport can raise, resolved lazily.

    Returns ``(OSError,)`` plus pyserial's ``SerialException`` when the module
    is importable, so this driver still imports on a machine with no serial
    module at all.
    """
    try:
        import serial
    except ImportError:
        return (OSError,)
    return (OSError, serial.SerialException)


@dataclass(frozen=True)
class GripperStatus:
    """One decoded GRIPPER STATUS block: raw counts and enums, no millimetres."""

    g_act: int
    g_gto: int
    g_sta: int
    g_obj: int
    g_flt: int
    k_flt: int
    g_pr: int
    g_po: int
    g_cu: int
    stamp: float


class RobotiqGripper:
    """One Robotiq 2F-85 on one serial port."""

    def __init__(self, port, *, slave_id=9, baud=115200, timeout_s=0.1,
                 stroke_mm=85.0, open_serial=None):
        """
        Bind to a device path. Nothing is opened and nothing moves yet.

        ``open_serial`` is the injection seam: it defaults to pyserial's
        ``Serial`` and the tests pass a pty-backed factory. ``stroke_mm`` is
        recorded so a caller can ask what this driver was configured for; the
        driver itself deals only in counts.
        """
        self._port = port
        self._slave_id = slave_id
        self._baud = baud
        self._timeout_s = timeout_s
        self.stroke_mm = stroke_mm
        self._open_serial = open_serial
        self._clock = _MONOTONIC
        self._ser = None
        self._failures = 0
        self._last_frame_at = 0.0
        self._resync_pending = False
        self._out = bytearray(protocol.pack_output())

    @property
    def port(self):
        """Return the device path this driver was constructed for."""
        return self._port

    @property
    def connected(self):
        """Return True when the port is open and usable."""
        return self._ser is not None and self._ser.is_open

    def open(self):        # noqa: A003 - the pinned method name
        """
        Open the serial port. Does NOT activate: activation is a motion.

        Raises ``LinkDownError`` -- never a bare transport exception -- when
        the device cannot be opened, so a caller polling for a replugged
        adapter needs one ``except`` arm and nothing escapes into its
        callback. Idempotent: opening an open port is a no-op.
        """
        if self._ser is not None:
            return
        factory = self._open_serial
        if factory is None:
            import serial                      # lazy: never at module scope
            factory = serial.Serial
        try:
            self._ser = factory(
                self._port, baudrate=self._baud, bytesize=registers.DATA_BITS,
                parity=registers.PARITY, stopbits=registers.STOP_BITS,
                timeout=self._timeout_s, write_timeout=WRITE_TIMEOUT_S,
                exclusive=True)
        except _transport_errors() as exc:
            self._ser = None
            raise LinkDownError(
                'could not open {}: {}'.format(self._port, exc)) from exc
        self._failures = 0
        self._last_frame_at = 0.0
        self._resync_pending = False
        self._out = bytearray(protocol.pack_output())

    def close(self):
        """
        Close the port. Idempotent, and it never raises.

        Called from teardown paths and from the caller's reconnect loop,
        including on a driver whose device has already vanished.
        """
        ser, self._ser = self._ser, None
        if ser is None:
            return
        try:
            ser.close()
        except Exception:
            # A teardown path must not raise: the device may already be gone.
            _LOG.debug('closing %s raised; the port is gone either way',
                       self._port, exc_info=True)

    # -- transactions ------------------------------------------------------

    def _require_open(self):
        """Return the open serial object, or raise LinkDownError."""
        if not self.connected:
            raise LinkDownError('{} is not open'.format(self._port))
        return self._ser

    def _stamp_frame(self):
        """
        Mark the wire free.

        Called after the reply is read AND on every failure path, because a
        timed-out transaction still used the wire.
        """
        self._last_frame_at = self._clock()

    def _wait_for_gap(self):
        """Sleep out the remainder of INTERFRAME_GAP_S since the last frame."""
        remaining = INTERFRAME_GAP_S - (self._clock() - self._last_frame_at)
        if remaining > 0:
            time.sleep(remaining)

    def _read_exactly(self, ser, count):
        """
        Read exactly ``count`` bytes or raise ShortFrameError.

        pyserial's ``read()`` may return short, so this loops against a
        deadline of the configured timeout. A short or empty result is the
        timeout case and raises into ``_transact``'s handler.
        """
        chunks = []
        got = 0
        deadline = self._clock() + self._timeout_s
        while got < count:
            chunk = ser.read(count - got)
            if chunk:
                chunks.append(chunk)
                got += len(chunk)
                continue
            if self._clock() >= deadline:
                break
        reply = b''.join(chunks)
        if len(reply) < count:
            raise protocol.ShortFrameError(
                'expected {} bytes from {}, got {} before the {} s timeout'
                .format(count, self._port, len(reply), self._timeout_s))
        return reply

    def _record_failure(self, exc):
        """Count one failed transaction and raise. NEVER returns."""
        self._failures += 1
        self._resync_pending = True
        if self._failures >= FAILURE_LIMIT:
            self.close()                 # port shut BEFORE the exception escapes
            raise LinkDownError(
                '{} consecutive failed reads on {}; the link is down '
                '({})'.format(self._failures, self._port, exc)) from exc
        raise TransientReadError(str(exc)) from exc

    def _transact(self, request, response_length, *, parse):
        """
        Send one frame, read one reply, parse it. Honours the 5 ms gap.

        Every protocol-level and transport-level failure of this transaction
        is routed through ``_record_failure``, which is the only place link
        health is decided. The parse runs inside the guarded block on purpose:
        a CRC error or an unexpected reply is a link failure, and a parse
        performed after this method returned would sit outside the funnel.
        """
        self._wait_for_gap()
        ser = self._require_open()
        try:
            if self._resync_pending:
                # A previous transaction timed out. Anything sitting in the
                # input buffer now is that transaction's late reply: reading
                # it as THIS reply would desynchronise the link permanently --
                # every subsequent read would return the previous request's
                # answer, one frame behind, which is a gripper that appears to
                # work and reports stale positions.
                try:
                    ser.reset_input_buffer()
                except Exception:
                    # pyserial lets the terminal layer's own error type
                    # escape from the flush when the device has gone away,
                    # and that type is not an OSError, so it would leave this
                    # funnel and reach the caller's callback. The flush is
                    # best-effort; the write below is what decides whether
                    # this link is usable.
                    _LOG.debug('flushing %s before a retry failed',
                               self._port, exc_info=True)
                self._resync_pending = False
            _LOG.debug('%s tx %s', self._port, request.hex(' '))
            ser.write(request)
            reply = self._read_exactly(ser, response_length)
            _LOG.debug('%s rx %s', self._port, reply.hex(' '))
            result = parse(reply)
        except (protocol.ProtocolError,) + _transport_errors() as exc:
            self._stamp_frame()          # the failed frame still used the wire
            self._record_failure(exc)    # always raises
        self._stamp_frame()
        self._failures = 0
        return result

    # -- commands ----------------------------------------------------------

    def _output_fields(self):
        """Return the seven ACTION REQUEST fields currently shadowed."""
        action = self._out[0]
        return {
            'r_act': (action >> registers.BIT_RACT) & 0x01,
            'r_gto': (action >> registers.BIT_RGTO) & 0x01,
            'r_atr': (action >> registers.BIT_RATR) & 0x01,
            'r_ard': (action >> registers.BIT_RARD) & 0x01,
            'r_pr': self._out[3],
            'r_sp': self._out[4],
            'r_fr': self._out[5],
        }

    def _write_output(self, **changes):
        """
        Apply ``changes`` to the shadowed output bytes and write them.

        Validation happens before any byte reaches the wire, so a bad count is
        a ``ValueError`` from the caller's own mistake: no frame is sent and
        the failure counter is untouched.
        """
        fields = self._output_fields()
        fields.update(changes)
        payload = protocol.pack_output(**fields)
        request = protocol.build_command(self._slave_id, payload)
        self._transact(
            request, protocol.expected_write_response_length(),
            parse=lambda reply: protocol.parse_write_response(
                reply, slave=self._slave_id,
                address=registers.OUT_FIRST_REGISTER,
                count=registers.BLOCK_REGISTERS))
        self._out = bytearray(payload)

    def go_to(self, count, speed_count, force_count):
        """
        Write rPR/rSP/rFR and set rGTO. Leaves rACT exactly as it is.

        Setting rACT here would start an unannounced auto-calibration motion
        (S1 section 4.3 Warning). Motion is never a side effect of a position
        command.
        """
        self._write_output(r_pr=count, r_sp=speed_count, r_fr=force_count,
                           r_gto=1)

    def stop(self):
        """Clear rGTO. The fingers hold where they are; nothing opens."""
        self._write_output(r_gto=0)

    def reset(self):
        """
        Write all six output bytes zero: rACT = 0, which clears gFLT.

        This is the falling edge of the activation cycle. A latched major
        fault survives it by design -- only the rising edge clears one.
        """
        self._write_output(r_act=0, r_gto=0, r_atr=0, r_ard=0,
                           r_pr=0, r_sp=0, r_fr=0)

    def auto_release(self, opening=True):
        """
        Emergency auto-release: rATR, with rARD set for the direction.

        EMERGENCY USE ONLY. S1 section 4.3 says rATR overrides everything
        except rACT, and the sequence ends in a fault that only an activation
        cycle clears. This primitive is deliberately not exposed on the ROS
        surface: it exists, and is tested, so that a future emergency path
        does not have to start with untested code.
        """
        self._write_output(r_atr=1, r_ard=1 if opening else 0)

    def _wait_for_gsta(self, target, deadline, timeout_s):
        """Poll until gSTA reaches ``target`` or the deadline passes."""
        last_seen = None
        while True:
            try:
                status = self.read_status()
            except TransientReadError:
                # A transient failure is a retry, not an abort: the driver is
                # still counting toward FAILURE_LIMIT, and if the link is
                # really gone the next failures raise LinkDownError, which is
                # NOT caught here.
                status = None
            if status is not None:
                last_seen = status.g_sta
                if status.g_sta == target:
                    return
            if self._clock() >= deadline:
                raise ActivationTimeout(
                    'the gripper did not finish activating within {} s '
                    '(gSTA={}); raise activation_timeout_s if this gripper is '
                    'slower'.format(timeout_s, last_seen))
            time.sleep(ACTIVATION_POLL_S)

    def activate(self, timeout_s=10.0):
        """
        Run the documented activation cycle: rACT 0, then rACT 1.

        S1 section 4.3: clearing rACT returns gSTA to 0 and clears the fault
        status; setting it starts an auto-calibration motion that ends with
        gSTA == 3. Both waits survive a dropped reply, because this cycle is
        the one documented recovery from a major fault and must not abort on
        one noisy frame.
        """
        deadline = self._clock() + timeout_s
        self._write_output(r_act=0, r_gto=0, r_atr=0, r_ard=0)
        self._wait_for_gsta(registers.GSta.RESET, deadline, timeout_s)
        self._write_output(r_act=1)
        self._wait_for_gsta(registers.GSta.ACTIVATED, deadline, timeout_s)

    # -- status ------------------------------------------------------------

    def read_status(self):
        """One FC03 read of the three input registers, decoded to raw fields."""
        fields = self._transact(
            protocol.build_status_request(self._slave_id),
            protocol.expected_read_response_length(registers.BLOCK_REGISTERS),
            parse=lambda frame: protocol.unpack_input(
                protocol.parse_read_response(
                    frame, slave=self._slave_id,
                    count=registers.BLOCK_REGISTERS)))
        return GripperStatus(
            g_act=fields.g_act, g_gto=fields.g_gto, g_sta=fields.g_sta,
            g_obj=fields.g_obj, g_flt=fields.g_flt, k_flt=fields.k_flt,
            g_pr=fields.g_pr, g_po=fields.g_po, g_cu=fields.g_cu,
            stamp=self._clock())
