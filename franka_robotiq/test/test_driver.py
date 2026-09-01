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
The driver against the emulator, over a real pty.

Nothing here is mocked at the transport: the driver opens a serial port, real
bytes cross a pty, and the emulator answers them. The tests that need control
over time inject a hand-advanced clock into the emulator; the driver keeps its
own real clock, because the 5 ms inter-frame rule is a property of the driver
and not of the wire.

``ManualClock`` is defined here and imported nowhere: this part owns no shared
test helper, so the three-line class is duplicated in the emulator's own test
module. That duplication is deliberate and is recorded so a reviewer does not
read it as an oversight.
"""

import ast
import inspect
import os
import re
import time

from franka_robotiq import discovery
from franka_robotiq import driver as driver_module
from franka_robotiq import protocol
from franka_robotiq import registers
from franka_robotiq import units
from franka_robotiq.driver import (ActivationTimeout, LinkDownError,
                                   RobotiqError, RobotiqGripper,
                                   TransientReadError)
from franka_robotiq.fake import FakeGripper

import pytest

import serial

DOCUMENTED_FAULT_CODES = sorted(registers.FAULTS)


class ManualClock:
    """A monotonic clock a test advances by hand."""

    def __init__(self, now=0.0):
        """Start at ``now`` seconds."""
        self.now = now

    def __call__(self):
        """Return the current time."""
        return self.now

    def advance(self, seconds):
        """Move the clock forward."""
        self.now += seconds


@pytest.fixture()
def fake():
    """Start an emulator with a short activation routine."""
    emulator = FakeGripper(activation_duration_s=0.2)
    emulator.start()
    try:
        yield emulator
    finally:
        emulator.stop()


@pytest.fixture()
def gripper(fake):
    """Open a driver bound to the emulator's pty."""
    device = RobotiqGripper(fake.port)
    device.open()
    try:
        yield device
    finally:
        device.close()


def _drain(device, attempts=8):
    """Read status until the link goes down, returning the fatal error."""
    for _ in range(attempts):
        try:
            device.read_status()
        except LinkDownError as error:
            return error
        except RobotiqError:
            continue
    raise AssertionError('the link never went down')


def _settle(gripper_, seconds=0.5, stop_when=None):
    """Poll status for a while, returning the last status seen."""
    deadline = time.monotonic() + seconds
    status = gripper_.read_status()
    while time.monotonic() < deadline:
        status = gripper_.read_status()
        if stop_when is not None and stop_when(status):
            return status
        time.sleep(0.01)
    return status


# -- activation and lifecycle ---------------------------------------------


def test_open_does_not_activate(gripper):
    """Activation is a motion and must never be a side effect of opening."""
    status = gripper.read_status()
    assert status.g_sta == registers.GSta.RESET
    assert status.g_act == 0


def test_activate_walks_gsta_0_1_3(fake, gripper, monkeypatch):
    """The documented walk, observed through the statuses the driver read."""
    seen = []
    original = RobotiqGripper.read_status

    def recording(self):
        status = original(self)
        seen.append(status.g_sta)
        return status

    monkeypatch.setattr(RobotiqGripper, 'read_status', recording)
    fake.set_activation_duration_s(0.3)
    gripper.activate(timeout_s=5.0)
    assert seen[0] == registers.GSta.RESET
    assert registers.GSta.ACTIVATING in seen
    assert seen[-1] == registers.GSta.ACTIVATED
    assert registers.GSta.UNUSED not in seen
    assert seen.index(registers.GSta.ACTIVATING) < \
        seen.index(registers.GSta.ACTIVATED)


def test_activate_is_idempotent_from_an_already_activated_gripper(gripper):
    """A second activation cycle is a fresh cycle, not a refusal."""
    gripper.activate(timeout_s=5.0)
    gripper.activate(timeout_s=5.0)
    assert gripper.read_status().g_sta == registers.GSta.ACTIVATED


def test_activate_times_out_and_names_the_config_key(fake, gripper):
    """The refusal names the key an operator raises when a gripper is slow."""
    fake.set_activation_duration_s(5.0)
    with pytest.raises(ActivationTimeout) as caught:
        gripper.activate(timeout_s=0.3)
    assert 'activation_timeout_s' in str(caught.value)


def test_activation_survives_one_dropped_reply(fake, gripper, monkeypatch):
    """
    One dropped reply must not abort the one documented recovery.

    ``read_status`` raises on the FIRST failed transaction, so a naive polling
    loop would give up on a single noisy frame -- during the very call that
    most needs to survive noise.
    """
    calls = []
    original = RobotiqGripper.read_status

    def recording(self):
        calls.append(1)
        if len(calls) == 2:
            fake.corrupt_next_reply('drop')
        return original(self)

    monkeypatch.setattr(RobotiqGripper, 'read_status', recording)
    gripper.activate(timeout_s=5.0)
    assert gripper.read_status().g_sta == registers.GSta.ACTIVATED
    assert fake.stats['replies_dropped'] == 1


def test_activation_propagates_link_down(fake, gripper, monkeypatch):
    """A link that fails FAILURE_LIMIT times in a row is down, mid-activation."""
    original = RobotiqGripper.read_status

    def recording(self):
        fake.corrupt_next_reply('drop')
        return original(self)

    monkeypatch.setattr(RobotiqGripper, 'read_status', recording)
    with pytest.raises(LinkDownError):
        gripper.activate(timeout_s=5.0)
    assert gripper.connected is False


def test_reset_clears_a_priority_fault_but_not_a_major_one():
    """
    The falling edge clears row F2 and leaves row F4 latched.

    Two S1 rules meet here and they are about different classes: clearing
    rACT "also clears fault status", while a major fault "requires a reset: a
    rising edge on rACT". Naming the class each half of this test uses is
    what keeps them from being collapsed into one wrong sentence.

    The falling edge needs rACT to have been high, so the emulator runs on a
    hand-advanced clock: the activation is left half-finished on purpose,
    which is the only state in which rGTO latches 0x05 with rACT still set.
    """
    clock = ManualClock()
    emulator = FakeGripper(activation_duration_s=1.0, clock=clock)
    emulator.start()
    device = RobotiqGripper(emulator.port)
    device.open()
    try:
        with pytest.raises(ActivationTimeout):
            device.activate(timeout_s=0.2)      # gSTA is 1, rACT is 1
        device.go_to(200, 128, 64)              # row F2, 0x05
        assert device.read_status().g_flt == 0x05
        device.reset()                          # the rACT falling edge
        assert device.read_status().g_flt == 0x00

        with pytest.raises(ActivationTimeout):
            device.activate(timeout_s=0.2)      # a fresh rising edge
        clock.advance(2.0)                      # the routine now finishes
        assert device.read_status().g_sta == registers.GSta.ACTIVATED

        emulator.inject_fault(0x0E)             # row F4, a latched major fault
        assert device.read_status().g_flt == 0x0E
        device.reset()
        assert device.read_status().g_flt == 0x0E
    finally:
        device.close()
        emulator.stop()


def test_close_is_idempotent(gripper):
    """close() is called from teardown paths and must never raise."""
    gripper.close()
    gripper.close()
    assert gripper.connected is False


def test_open_is_idempotent(gripper):
    """Opening an open port is a no-op, not a second file descriptor."""
    gripper.open()
    assert gripper.connected is True


# -- commanding ------------------------------------------------------------


def test_go_to_writes_rpr_rsp_rfr_and_sets_rgto(fake, gripper):
    """One command writes the three counts and raises rGTO."""
    gripper.activate(timeout_s=5.0)
    gripper.go_to(200, 128, 64)
    state = fake.snapshot()
    assert (state['r_pr'], state['r_sp'], state['r_fr']) == (200, 128, 64)
    assert state['r_gto'] == 1
    assert gripper.read_status().g_pr == 200


def test_go_to_before_activation_yields_fault_0x07(gripper):
    """
    Row F2 of the fault table. INFERRED, not documented.

    S1 gives 0x07 only as the meaning "the activation bit must be set prior to
    action"; it never states that the gripper latches it in response to an
    early rGTO write. The emulator infers the latching trigger, and a real
    gripper that behaves otherwise on bring-up day is a finding to record, not
    a red test. What is NOT an inference is the rule this test really guards:
    go_to does not touch rACT.
    """
    gripper.go_to(200, 128, 64)
    assert gripper.read_status().g_flt == 0x07


def test_go_to_does_not_set_ract(fake, gripper):
    """Setting rACT inside a position command would start a surprise motion."""
    gripper.reset()
    gripper.go_to(200, 128, 64)
    assert fake.snapshot()['r_act'] == 0
    assert gripper.read_status().g_act == 0


def test_stop_clears_rgto_and_the_fingers_hold(fake, gripper):
    """Clearing rGTO holds the fingers where they are; nothing opens."""
    gripper.activate(timeout_s=5.0)
    gripper.go_to(255, 0, 64)
    time.sleep(0.1)
    gripper.stop()
    held = gripper.read_status().g_po
    assert fake.snapshot()['r_gto'] == 0
    time.sleep(0.2)
    assert gripper.read_status().g_po == held


def test_auto_release_sets_ratr_and_ends_in_fault_0x0f(fake, gripper):
    """
    Row F5: 0x0B while it runs, then 0x0F once the fingers reach the stop.

    INFERRED, not documented: S1 states neither the transition nor that 0x0B
    permits motion. Both are read off the meanings "automatic release in
    progress" and "automatic release completed".
    """
    gripper.activate(timeout_s=5.0)
    gripper.go_to(255, 255, 64)
    _settle(gripper, 1.0, stop_when=lambda s: s.g_po >= 250)
    gripper.auto_release(opening=True)
    assert fake.snapshot()['r_atr'] == 1
    assert gripper.read_status().g_flt == 0x0B
    status = _settle(gripper, 2.0, stop_when=lambda s: s.g_flt == 0x0F)
    assert status.g_flt == 0x0F
    assert status.g_po == 0
    assert status.g_sta == registers.GSta.RESET


def test_auto_release_direction_bit_follows_the_argument(fake, gripper):
    """Direction bit rARD is 1 for opening, 0 for closing, set with rATR."""
    gripper.activate(timeout_s=5.0)
    gripper.auto_release(opening=False)
    state = fake.snapshot()
    assert (state['r_atr'], state['r_ard']) == (1, 0)
    gripper.activate(timeout_s=5.0)
    gripper.auto_release(opening=True)
    state = fake.snapshot()
    assert (state['r_atr'], state['r_ard']) == (1, 1)


# -- grip, detect, stall ---------------------------------------------------


def test_closing_on_an_object_reports_gobj_2_short_of_the_request(fake,
                                                                  gripper):
    """The normal "gripped it" outcome: stopped on contact while closing."""
    gripper.activate(timeout_s=5.0)
    fake.set_object(width_mm=30.0)
    gripper.go_to(255, 255, 64)          # fully closed
    status = _settle(gripper, 2.0, stop_when=lambda s: s.g_obj == 2)
    assert status.g_obj == registers.GObj.CLOSED_ON_OBJECT
    assert status.g_po < 255             # short of the request
    assert status.g_po == pytest.approx(units.width_mm_to_count(30.0), abs=1)


def test_opening_into_an_obstruction_reports_gobj_1(fake, gripper):
    """The mirror case: stopped on contact while opening."""
    gripper.activate(timeout_s=5.0)
    gripper.go_to(255, 255, 64)
    _settle(gripper, 2.0, stop_when=lambda s: s.g_po >= 250)
    fake.set_object(width_mm=30.0)
    gripper.go_to(0, 255, 64)            # fully open, through the obstruction
    status = _settle(gripper, 2.0, stop_when=lambda s: s.g_obj == 1)
    assert status.g_obj == registers.GObj.OPENED_ON_OBJECT
    assert status.g_po > 0


def test_reaching_the_request_reports_gobj_3(fake, gripper):
    """At the requested position, nothing detected -- or the object was lost."""
    gripper.activate(timeout_s=5.0)
    fake.set_object(None)
    gripper.go_to(128, 255, 64)
    status = _settle(gripper, 2.0, stop_when=lambda s: s.g_obj == 3)
    assert status.g_obj == registers.GObj.AT_POSITION
    assert status.g_po == 128


def test_gobj_is_zero_while_gto_is_clear(fake, gripper):
    """Detection is meaningless when gGTO is 0, and reads 0 rather than lying."""
    gripper.activate(timeout_s=5.0)
    gripper.go_to(128, 255, 64)
    _settle(gripper, 2.0, stop_when=lambda s: s.g_obj == 3)
    gripper.stop()
    status = gripper.read_status()
    assert status.g_gto == 0
    assert status.g_obj == 0


def test_current_rises_while_stalled_on_an_object(fake, gripper):
    """Shape only: the current curve is invented and no magnitude is claimed."""
    gripper.activate(timeout_s=5.0)
    idle = gripper.read_status().g_cu
    fake.set_object(width_mm=30.0)
    gripper.go_to(255, 255, 200)
    moving = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        status = gripper.read_status()
        if status.g_obj == registers.GObj.CLOSED_ON_OBJECT:
            break
        if status.g_gto == 1 and status.g_po != status.g_pr:
            moving = status.g_cu
        time.sleep(0.005)
    stalled = gripper.read_status()
    assert stalled.g_obj == registers.GObj.CLOSED_ON_OBJECT
    assert moving is not None
    assert idle < moving < stalled.g_cu


# -- faults, one test per row of the fault table --------------------------


@pytest.mark.parametrize('code', DOCUMENTED_FAULT_CODES)
def test_every_documented_fault_code_round_trips_into_status(fake, gripper,
                                                             code):
    """Each of S1's ten codes reaches the status record unaltered."""
    gripper.activate(timeout_s=5.0)
    fake.inject_fault(code)
    assert gripper.read_status().g_flt == code


def test_a_major_fault_clears_only_on_an_ract_rising_edge(fake, gripper):
    """Row F4. This is what makes an activation cycle the only recovery."""
    gripper.activate(timeout_s=5.0)
    fake.inject_fault(0x0E)
    assert gripper.read_status().g_flt == 0x0E
    gripper.reset()
    assert gripper.read_status().g_flt == 0x0E
    gripper.go_to(128, 128, 64)
    assert gripper.read_status().g_flt == 0x0E
    gripper.activate(timeout_s=5.0)
    assert gripper.read_status().g_flt == 0x00


def test_a_priority_fault_clears_when_the_precondition_finally_holds():
    """
    Row F2: "you asked too early" evaporates once the precondition holds.

    Both of the row's clearers are exercised: the rACT falling edge, and the
    completion of a full activation to gSTA == 3 while the fault stands. The
    second is the one that needs a hand-advanced clock, because it happens
    with no write at all -- the emulator's mechanism catches up and the reason
    for the complaint is gone.
    """
    clock = ManualClock()
    emulator = FakeGripper(activation_duration_s=1.0, clock=clock)
    emulator.start()
    device = RobotiqGripper(emulator.port)
    device.open()
    try:
        with pytest.raises(ActivationTimeout):
            device.activate(timeout_s=0.2)
        device.go_to(200, 128, 64)
        assert device.read_status().g_flt == 0x05
        device.reset()
        assert device.read_status().g_flt == 0x00

        # Latch it again mid-activation, then let the activation finish with
        # no further write: the reason for the complaint simply goes away.
        with pytest.raises(ActivationTimeout):
            device.activate(timeout_s=0.2)
        device.go_to(200, 128, 64)
        assert device.read_status().g_flt == 0x05
        clock.advance(2.0)
        status = device.read_status()
        assert status.g_sta == registers.GSta.ACTIVATED
        assert status.g_flt == 0x00
    finally:
        device.close()
        emulator.stop()


def test_asking_for_motion_before_activation_needs_an_activation_to_clear(
        gripper):
    """
    Row F2's 0x07 branch: with rACT already low there is no falling edge.

    A gripper that has never been activated cannot produce one, so reset()
    leaves 0x07 standing and only an activation cycle clears it. That is the
    behaviour the operator-facing rule already assumes -- a priority fault
    tells the reader to run the activation, not to reset.
    """
    gripper.go_to(200, 128, 64)
    assert gripper.read_status().g_flt == 0x07
    gripper.reset()
    assert gripper.read_status().g_flt == 0x07
    gripper.activate(timeout_s=5.0)
    assert gripper.read_status().g_flt == 0x00


def test_a_minor_fault_does_not_stop_the_fingers(fake, gripper):
    """Row F3: overheating is a WARN that resumes by itself; motion continues."""
    gripper.activate(timeout_s=5.0)
    fake.set_object(None)
    fake.inject_fault(0x08)
    gripper.go_to(200, 255, 64)
    status = _settle(gripper, 2.0, stop_when=lambda s: s.g_po >= 200)
    assert status.g_flt == 0x08
    assert status.g_po == 200


def test_the_auto_release_pair_runs_then_latches(fake, gripper):
    """
    Row F5: 0x0B moves, becomes 0x0F once, then latches like row F4.

    INFERRED, not documented: S1 states neither the transition nor that 0x0B
    permits motion.
    """
    gripper.activate(timeout_s=5.0)
    gripper.go_to(128, 255, 64)
    _settle(gripper, 2.0, stop_when=lambda s: s.g_po >= 128)
    gripper.auto_release(opening=True)
    moved = _settle(gripper, 2.0, stop_when=lambda s: s.g_flt == 0x0F)
    assert moved.g_flt == 0x0F
    assert moved.g_po == 0
    gripper.go_to(200, 255, 64)
    held = _settle(gripper, 0.3)
    assert held.g_flt == 0x0F
    assert held.g_po == 0
    gripper.activate(timeout_s=5.0)
    assert gripper.read_status().g_flt == 0x00


def test_a_latched_major_fault_refuses_motion(fake, gripper):
    """Row F4's motion column, as a mutation guard."""
    gripper.activate(timeout_s=5.0)
    fake.set_object(None)
    fake.inject_fault(0x0A)
    start = gripper.read_status().g_po
    gripper.go_to(255, 255, 64)
    status = _settle(gripper, 0.4)
    assert status.g_flt == 0x0A
    assert status.g_po == start
    assert status.g_gto == 0


# -- link failures, one test per row of the classification table -----------


def test_bad_crc_reply_is_a_timeout_not_a_parse(fake, gripper):
    """A reply whose CRC is wrong is refused, never decoded as a status."""
    fake.corrupt_next_reply('crc')
    with pytest.raises(TransientReadError) as caught:
        gripper.read_status()
    assert 'CRC' in str(caught.value) or 'crc' in str(caught.value)
    assert gripper.connected is True
    assert gripper.read_status().g_sta == registers.GSta.RESET


def test_short_reply_raises_transient_then_link_down_on_the_third(fake,
                                                                  gripper):
    """Two transients, then the link is declared down on the third."""
    for _ in range(2):
        fake.corrupt_next_reply('short')
        with pytest.raises(TransientReadError):
            gripper.read_status()
    fake.corrupt_next_reply('short')
    with pytest.raises(LinkDownError):
        gripper.read_status()
    assert gripper.connected is False


def test_three_consecutive_failures_raise_link_down_and_close_the_port(
        fake, gripper):
    """The port is shut BEFORE the exception escapes."""
    for _ in range(driver_module.FAILURE_LIMIT - 1):
        fake.corrupt_next_reply('drop')
        with pytest.raises(TransientReadError):
            gripper.read_status()
        assert gripper.connected is True
    fake.corrupt_next_reply('drop')
    with pytest.raises(LinkDownError) as caught:
        gripper.read_status()
    assert str(driver_module.FAILURE_LIMIT) in str(caught.value)
    assert gripper.connected is False


def test_a_crc_error_counts_toward_the_failure_limit(fake, gripper):
    """The most common real failure must reach the counter, not bypass it."""
    for _ in range(driver_module.FAILURE_LIMIT - 1):
        fake.corrupt_next_reply('crc')
        with pytest.raises(TransientReadError):
            gripper.read_status()
    fake.corrupt_next_reply('crc')
    with pytest.raises(LinkDownError):
        gripper.read_status()
    assert gripper.connected is False


def test_an_unexpected_reply_counts_toward_the_failure_limit(fake, gripper):
    """Something that is not this gripper answering is a link fault."""
    fake.corrupt_next_reply('foreign_slave')
    with pytest.raises(TransientReadError):
        gripper.read_status()
    fake.corrupt_next_reply('exception')
    with pytest.raises(TransientReadError):
        gripper.read_status()
    fake.corrupt_next_reply('garbage')
    with pytest.raises(LinkDownError):
        gripper.read_status()
    assert gripper.connected is False


def test_a_value_error_from_pack_output_does_not_count(fake, gripper):
    """A programming error must not talk a healthy link down."""
    gripper.activate(timeout_s=5.0)
    before = fake.stats['frames_seen']
    with pytest.raises(ValueError):
        gripper.go_to(300, 0, 0)
    assert fake.stats['frames_seen'] == before
    assert gripper.connected is True
    for _ in range(3):
        assert gripper.read_status() is not None


def test_one_success_resets_the_failure_counter(fake, gripper):
    """Two failures either side of a success are not three in a row."""
    for _ in range(driver_module.FAILURE_LIMIT - 1):
        fake.corrupt_next_reply('drop')
        with pytest.raises(TransientReadError):
            gripper.read_status()
    assert gripper.read_status() is not None
    for _ in range(driver_module.FAILURE_LIMIT - 1):
        fake.corrupt_next_reply('drop')
        with pytest.raises(TransientReadError):
            gripper.read_status()
    assert gripper.connected is True


def test_a_late_reply_does_not_desynchronise_the_next_transaction(fake,
                                                                  gripper):
    """
    The subtlest bug in the transport, pinned.

    The first read times out and its reply lands in the input buffer
    afterwards. Reading that as the NEXT transaction's reply would
    desynchronise the link permanently: every later read would return the
    previous request's answer, one frame behind, which is a gripper that
    appears to work and reports stale positions.
    """
    assert gripper.read_status().g_flt == 0x00
    fake.delay_next_reply(0.3)
    with pytest.raises(TransientReadError):
        gripper.read_status()
    fake.inject_fault(0x0E)
    time.sleep(0.35)                    # the stale reply is in the buffer now
    assert gripper.read_status().g_flt == 0x0E


def test_foreign_slave_reply_is_rejected_by_name(fake, gripper):
    """The refusal names the slave id that answered."""
    fake.corrupt_next_reply('foreign_slave')
    with pytest.raises(TransientReadError) as caught:
        gripper.read_status()
    assert 'slave id' in str(caught.value)


def test_exception_style_reply_is_reported_as_not_from_the_gripper(fake,
                                                                   gripper):
    """S1 section 4.7.1: this gripper implements no exception responses."""
    fake.corrupt_next_reply('exception')
    with pytest.raises(TransientReadError) as caught:
        gripper.read_status()
    assert 'did not come from' in str(caught.value)


def test_a_frame_for_another_slave_id_is_never_answered(fake, gripper):
    """Proving silence needs the emulator's own counters, not a timeout guess."""
    before = dict(fake.stats)
    handle = serial.Serial(fake.port, baudrate=115200, timeout=0.2)
    try:
        handle.write(protocol.build_read(10, 0x07D0, 3))
        assert handle.read(11) == b''
    finally:
        handle.close()
    after = fake.stats
    assert after['frames_ignored_slave'] == before['frames_ignored_slave'] + 1
    assert after['frames_answered'] == before['frames_answered']


# -- unplug and reconnect --------------------------------------------------


def test_unplug_mid_session_raises_link_down_within_the_failure_limit(
        fake, gripper):
    """An unplugged adapter is seen, counted and declared down."""
    assert gripper.read_status() is not None
    fake.unplug()
    error = _drain(gripper)
    assert isinstance(error, LinkDownError)
    assert gripper.connected is False


@pytest.mark.parametrize('raised', [OSError(5, 'Input/output error'),
                                    serial.SerialException('gone')])
def test_serial_exception_and_oserror_both_become_link_down(raised):
    """Neither ever escapes into the caller's callback."""
    class Broken:
        is_open = True

        def write(self, data):
            raise raised

        def read(self, size=1):
            raise raised

        def reset_input_buffer(self):
            pass

        def close(self):
            pass

    device = RobotiqGripper('/dev/null', open_serial=lambda *a, **k: Broken())
    device.open()
    for _ in range(driver_module.FAILURE_LIMIT - 1):
        with pytest.raises(TransientReadError):
            device.read_status()
    with pytest.raises(LinkDownError):
        device.read_status()
    assert device.connected is False


def test_open_of_an_absent_device_raises_link_down_not_a_serial_exception():
    """
    An absent adapter is an expected state on the reconnect path.

    The caller polls open() from its own callback every couple of seconds, so
    it must get one exception type it already handles rather than a bare
    transport error.
    """
    device = RobotiqGripper('/dev/franka-robotiq-does-not-exist')
    with pytest.raises(LinkDownError) as caught:
        device.open()
    assert not isinstance(caught.value, serial.SerialException)
    assert device.connected is False
    assert '/dev/franka-robotiq-does-not-exist' in str(caught.value)


def test_reconnect_after_replug_builds_a_new_driver_on_the_new_path(
        fake, tmp_path):
    """
    The whole production reconnect sequence, with no reopen() anywhere.

    Re-resolving is not an extra step. A replugged adapter can come back on a
    different device node, so the path fixed at construction is stale, and
    re-resolution is also where the anti-swap rules get a second chance to
    refuse a swapped adapter.
    """
    root = tmp_path / 'by-id'
    root.mkdir()
    name = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'
    link = root / name
    os.symlink(fake.port, str(link))

    path = discovery.resolve(name, root=str(root))
    device = RobotiqGripper(path)
    device.open()
    device.activate(timeout_s=5.0)
    assert device.read_status().g_sta == registers.GSta.ACTIVATED

    fake.unplug()
    assert isinstance(_drain(device), LinkDownError)
    device.close()

    os.unlink(str(link))
    with pytest.raises(discovery.BindingError):
        discovery.resolve(name, root=str(root))

    new_port = fake.replug()
    assert new_port != path
    deadline = time.monotonic() + 2.0
    while os.path.exists(path) and time.monotonic() < deadline:
        time.sleep(0.01)
    os.symlink(new_port, str(link))
    new_path = discovery.resolve(name, root=str(root))
    assert new_path == new_port

    fresh = RobotiqGripper(new_path)
    fresh.open()
    try:
        status = fresh.read_status()
        assert status.g_sta == registers.GSta.RESET
        assert fresh is not device
    finally:
        fresh.close()


def test_a_stale_driver_cannot_be_revived_after_a_replug(fake):
    """The mutation guard on the rule above: there is no reopen()."""
    device = RobotiqGripper(fake.port)
    device.open()
    old_path = fake.port
    fake.unplug()
    _drain(device)
    fake.replug()
    assert fake.port != old_path
    deadline = time.monotonic() + 2.0
    while os.path.exists(old_path) and time.monotonic() < deadline:
        time.sleep(0.01)          # the kernel removes the node a moment later
    with pytest.raises(LinkDownError):
        device.open()
    assert device.connected is False


def test_exclusive_open_refuses_a_second_owner_of_the_same_port(
        fake, gripper):
    """
    One link, one owner: a second process is refused, not interleaved.

    This is the runtime half of the anti-swap guarantee; the configuration
    half is a startup refusal.
    """
    second = RobotiqGripper(fake.port)
    with pytest.raises(LinkDownError):
        second.open()
    assert second.connected is False
    assert gripper.read_status() is not None


# -- timing and hygiene ----------------------------------------------------


def _timing_factory(events):
    """Build a serial factory that timestamps every write and every read."""
    def factory(*args, **kwargs):
        inner = serial.Serial(*args, **kwargs)

        class Timed:
            def write(self, data):
                events.append(('write', time.monotonic()))
                return inner.write(data)

            def read(self, size=1):
                chunk = inner.read(size)
                events.append(('read', time.monotonic()))
                return chunk

            def reset_input_buffer(self):
                return inner.reset_input_buffer()

            def close(self):
                return inner.close()

            @property
            def is_open(self):
                return inner.is_open

        return Timed()
    return factory


def test_consecutive_frames_are_at_least_five_milliseconds_apart(fake):
    """S1 section 4.7.1's floor, measured in aggregate over 20 transactions."""
    device = RobotiqGripper(fake.port)
    device.open()
    try:
        started = time.monotonic()
        for _ in range(20):
            device.read_status()
        elapsed = time.monotonic() - started
    finally:
        device.close()
    assert elapsed >= 19 * driver_module.INTERFRAME_GAP_S


def test_the_gap_is_measured_from_the_end_of_the_previous_transaction(fake):
    """
    The assertion the aggregate test cannot make.

    A message is not over until its reply has been read, so the silence the
    manual asks for is between the last reply byte and the next request. With
    the wire stamped immediately after write() instead, this test fails while
    the aggregate one still passes -- which is exactly the bug it exists to
    catch.
    """
    events = []
    device = RobotiqGripper(fake.port, open_serial=_timing_factory(events))
    device.open()
    try:
        for _ in range(6):
            device.read_status()
    finally:
        device.close()
    writes = [index for index, (kind, _) in enumerate(events)
              if kind == 'write']
    assert len(writes) >= 6
    gaps = []
    for index in writes[1:]:
        previous_read = max(other for other in range(index)
                            if events[other][0] == 'read')
        gaps.append(events[index][1] - events[previous_read][1])
    assert min(gaps) >= driver_module.INTERFRAME_GAP_S * 0.98


def test_a_failed_transaction_still_stamps_the_wire(fake):
    """A timed-out transaction occupied the wire too."""
    events = []
    device = RobotiqGripper(fake.port, open_serial=_timing_factory(events))
    device.open()
    try:
        device.read_status()
        fake.corrupt_next_reply('drop')
        with pytest.raises(TransientReadError):
            device.read_status()
        marker = len(events)
        device.read_status()
    finally:
        device.close()
    next_write = next(index for index in range(marker, len(events))
                      if events[index][0] == 'write')
    last_before = max(index for index in range(marker)
                      if events[index][0] in ('read', 'write'))
    gap = events[next_write][1] - events[last_before][1]
    assert gap >= driver_module.INTERFRAME_GAP_S * 0.98


def test_status_stamp_is_monotonic_and_advances(gripper):
    """The stamp is taken after the parse, so it never times a bad frame."""
    stamps = [gripper.read_status().stamp for _ in range(5)]
    assert stamps == sorted(stamps)
    assert stamps[-1] > stamps[0]


def test_driver_never_imports_serial_at_module_scope():
    """The pure layers must import on a machine with no serial module."""
    tree = ast.parse(inspect.getsource(driver_module))
    module_level = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            module_level.update(alias.name.split('.')[0]
                                for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            module_level.add(node.module.split('.')[0])
    assert 'serial' not in module_level

    nested = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Import) and any(
                alias.name == 'serial' for alias in node.names):
            nested += 1
    assert nested == 2          # open() and _transport_errors()


def test_driver_module_contains_no_millimetre_arithmetic():
    """Conversion is the caller's job, through the units module."""
    source = inspect.getsource(driver_module)
    tree = ast.parse(source)
    doc_lines = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) \
                and isinstance(node.value.value, str):
            doc_lines.update(range(node.lineno, node.end_lineno + 1))
    banned = re.compile(r'/ *255|\* *255|255 *-|STROKE_MM|width_mm')
    offenders = [line for number, line in enumerate(source.splitlines(), 1)
                 if number not in doc_lines and banned.search(line)]
    assert offenders == []
    assert 'stroke_mm=85.0' in source          # the pinned signature stays
    assert not re.search(r'^\s*(from|import).*\bunits\b', source,
                         flags=re.MULTILINE)
