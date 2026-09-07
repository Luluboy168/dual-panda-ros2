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
Stage 2 end-to-end: the motion subsystem, measured at the controller.

This is the plan's section 8 Stage 2 layers 2 and 3. Layer 1 (the
line-by-line ``accept`` port and its fourteen rejection reasons) is
``test_jog_message_contract.py``; this file drives the REAL server stack over
HTTP against :class:`support.mock_impedance_controller.MockImpedanceController`
on a REAL ROS graph, and asserts almost everything **at the mock** -- its
inbox's verdicts and its own receive timestamps -- never from the server's
bookkeeping. A server that believed it had stopped publishing while messages
kept arriving would pass every self-report and fail every assertion here.

FAKE HARDWARE ONLY
    The only launch this file can cause is ``fake_dual_state_only.launch.py``.
    The supervisor's ``spawn`` seam is faked and its ``argv_builder`` seam
    returns something that is not a launch command line, so the production
    guarded-motion launch the ``('both', 'motion')`` profile really names is
    unreachable from here.

Domain isolation
    This brings up a real ROS graph, so it runs on the ID this package
    reserves for it -- ``ROS_DOMAIN_ID=220``, see the allocation table in
    ``CMakeLists.txt`` -- and SKIPS itself when the environment does not say
    so, exactly like the domain-219 Stage 1 e2e. A bare ``pytest`` run outside
    the CMake registration can never publish onto somebody else's domain.

What is NOT covered here, and where it is
    *Server SIGKILL.* The plan lists a server ``SIGKILL`` among the four things
    that must stop the target stream. This rig runs the server stack
    **in-process**, so killing it would kill the test. That case is covered by
    the domain-219 Stage 1 e2e
    (``test_server_sigkill_does_not_outlive_its_pdeathsig_children``), which
    SIGKILLs the real ``franka_web_server`` subprocess and proves the whole
    child tree dies from ``PR_SET_PDEATHSIG`` -- and a dead process publishes
    nothing, which is the property the motion case needs. The other three
    stop sources (operator release, lock expiry, disable) plus session stop
    are measured here, at the mock, to 100 ms.

    *The recorder.* Covered end-to-end on domain 219 with a real
    ``franka_record`` and a real sealed bag; this rig uses a fake recorder so
    the only subject is motion.

Ordering
    The cases below run in file order against one module-scoped rig, because
    they are one session's story: refuse, verify, start, enable, stream, jog,
    fault, recover, stop. Each case states the state it expects to inherit.
    The layer-3 case runs LAST on purpose: configuring the real controller
    creates ``~/arm_<n>/enable`` services under the same node name the mock
    uses, so it must not exist while the mock is under test.

Recover comes after the fault clears, not before
    Plan section 7.2's own operator text is "release the physical stop FIRST,
    then press Recover", and section 7.3 makes that structural: the session
    leaves ``fault`` only when the recover sequence succeeded **and** the fault
    rules stop firing on the next tick, which the supervisor evaluates in the
    same tick that consumed the recover. Case 9 therefore clears the synthetic
    diagnostic and then presses Recover, which is the operator sequence the UI
    prescribes.
"""

import json
import os
import subprocess
import threading
import time
import warnings

from franka_web import defaults
import pytest
from support.fake_checker import CheckResult, Contact
from support.mock_impedance_controller import (
    ENABLE_DISABLED_MESSAGE, ENABLE_ENABLED_MESSAGE, NO_ERRORS_MESSAGE)
from support.mock_motion_harness import (
    ARM_IDS, collect_sse, CONTROLLER_NAME, describe, HOME_POSE,
    load_validator, MockMotionHarness, own_lineage, POSE_SETTER_NAME, reap,
    scan_processes, schema_errors, TIGHT_JOINT_INDEX, uniform_fences,
    wait_until)

#: The reviewed controller watchdog, read once for the message below.
WATCHDOG_TIMEOUT_S = defaults.REVIEWED_TIMING_S['watchdog_timeout']

#: The ID this package reserves for the Stage 2 e2e (CMakeLists.txt table).
REQUIRED_DOMAIN_ID = '220'

_SKIP_REASON = (
    'the Stage 2 motion e2e brings up a real ROS graph and must stay on its '
    'reserved domain: run it through the CMake registration, or export '
    'ROS_DOMAIN_ID={} (plus FASTDDS_BUILTIN_TRANSPORTS=SHM and '
    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) yourself'.format(REQUIRED_DOMAIN_ID))

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != REQUIRED_DOMAIN_ID, reason=_SKIP_REASON)

#: The enabled window the plan asks for: 30 s, zero rejections.
MEASURE_WINDOW_S = 30.0

#: Plan section 8: 20 Hz +/- 2 Hz per arm, worst gap under the 0.1 s watchdog.
RATE_TOLERANCE_HZ = 2.0
MAX_GAP_S = 0.08

#: How long a stop source may take to reach the wire (plan section 8).
STOP_BOUND_S = 0.1

#: How long to watch for a straggler after a stop source has been asserted.
QUIET_WINDOW_S = 0.5

#: A panda1 pose whose joint 1 leaves the generated fence (which is +/- 0.35
#: rad around the home pose) while every other joint stays inside it.
OUTSIDE_FENCE_JOINT1 = 0.6


def outside_fence_pose():
    """
    Return a pose whose joint 1 leaves the rig's fence.

    Only joint 1 moves; every other joint stays well inside, so a refusal can
    only be about joint 1 and the message can be asserted precisely.
    """
    pose = list(HOME_POSE)
    pose[0] = OUTSIDE_FENCE_JOINT1
    return tuple(pose)


# ----------------------------------------------------------------------
# The rig
# ----------------------------------------------------------------------


@pytest.fixture(scope='module')
def rig(tmp_path_factory):
    """Bring the whole motion rig up once, and prove it leaks nothing."""
    lineage = own_lineage()
    preexisting = scan_processes(lineage, REQUIRED_DOMAIN_ID)
    if preexisting:
        warnings.warn(
            'domain {} was not clean before this test:\n{}'.format(
                REQUIRED_DOMAIN_ID, describe(preexisting)), stacklevel=1)

    root = str(tmp_path_factory.mktemp('motion-rig'))
    harness = MockMotionHarness(
        root, domain_id=int(REQUIRED_DOMAIN_ID), fences=uniform_fences())
    leaked = {}
    try:
        harness.start()
        harness.claim()
        harness.sse_records = []
        yield harness
    finally:
        harness.close()
        leaked = reap({pid: line
                       for pid, line in scan_processes(
                           lineage, REQUIRED_DOMAIN_ID).items()
                       if pid not in preexisting})
    assert not leaked, (
        'the motion rig leaked processes that this test had to kill:\n{}'.format(
            describe(leaked)))


# ----------------------------------------------------------------------
# Shared assertions
# ----------------------------------------------------------------------


def assert_no_rejections(rig, context):
    """Assert the mock has rejected nothing at all, on either arm."""
    for arm_id in ARM_IDS:
        stats = rig.mock.stats(rig.slot(arm_id))
        assert stats['rejected'] == 0 and stats['inactive'] == 0, (
            '{}: the controller rejected {} of {} targets on {} ({}); every '
            'rejection freezes the arm under the watchdog'.format(
                context, stats['rejected'] + stats['inactive'],
                stats['messages'], arm_id, stats['results']))


def assert_stream_stopped(rig, reference, arm_ids, source):
    """
    Assert the target stream stopped within ``STOP_BOUND_S`` of ``reference``.

    ``reference`` is a monotonic reading of the moment the operator's action
    took effect. The verdict is the mock's OWN receive timestamp of its last
    message, and a second quiet window proves nothing arrives afterwards.
    """
    time.sleep(QUIET_WINDOW_S)
    for arm_id in arm_ids:
        last = rig.last_receive_s(arm_id)
        assert last is not None, (
            '{}: no target ever reached {}, so this proves nothing'.format(
                source, arm_id))
        assert last - reference <= STOP_BOUND_S, (
            '{}: {} kept receiving targets {:.3f} s past the stop (limit '
            '{:.3f} s); the controller watchdog freezes at {:.2f} s'.format(
                source, arm_id, last - reference, STOP_BOUND_S,
                WATCHDOG_TIMEOUT_S))
    counts = {arm_id: rig.message_count(arm_id) for arm_id in arm_ids}
    time.sleep(QUIET_WINDOW_S)
    for arm_id in arm_ids:
        assert rig.message_count(arm_id) == counts[arm_id], (
            '{}: {} was still receiving targets a second after the stop'.format(
                source, arm_id))


def enable_both(rig):
    """Enable both arms and wait until the mock is receiving from each."""
    for arm_id in ARM_IDS:
        _, payload = rig.enable(arm_id, True)
        assert payload['enabled'] is True
        assert payload['message'] == ENABLE_ENABLED_MESSAGE, (
            'the enable response must pass the controller message through '
            'verbatim (plan section 6.13), got {!r}'.format(payload['message']))
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 3, timeout_s=5.0), (
            'no targets reached the mock for {} after enable'.format(arm_id))


def assert_settling_is_command_closed(rig, frame):
    """Prove HTTP Enable/Jog cannot mutate the mock before ``running``."""
    assert frame['session']['state'] == 'settling'
    before_counts = {arm_id: rig.message_count(arm_id) for arm_id in ARM_IDS}
    before_generations = {
        arm_id: rig.mock.inbox(rig.slot(arm_id)).enable_generation
        for arm_id in ARM_IDS}

    status, refusal = rig.enable('panda1', True, expect=409)
    assert status == 409
    assert refusal['error'] == 'session_not_running', refusal
    status, refusal = rig.jog('panda1', 6, 1, expect=409)
    assert status == 409
    assert refusal['error'] == 'session_not_running', refusal

    for arm_id in ARM_IDS:
        inbox = rig.mock.inbox(rig.slot(arm_id))
        assert inbox.enabled is False
        assert inbox.enable_generation == before_generations[arm_id], (
            'a refused settling-time command reached the {} enable service'.format(
                arm_id))
        assert rig.message_count(arm_id) == before_counts[arm_id], (
            'a target reached {} while activation settling was closed'.format(
                arm_id))


# ----------------------------------------------------------------------
# Layer 2 -- the motion subsystem over a real graph
# ----------------------------------------------------------------------


def test_02_a_motion_start_captures_the_baseline_and_reports_the_config_fence(rig):
    """
    Motion is ONE GO: no Watch session, no attestation, no prerequisite.

    The pre-activation baseline is captured INSIDE the session, in
    ``starting``, and the frame reports the fence the configuration file
    installed -- exactly, to the last bit, because the jog model clamps to
    those very doubles.

    The impedance controller is ACTIVE before the session even starts here,
    the way the reviewed launch really sequences it, so exactly one restage
    pair -- pause, then hand back -- must have run before ``settling``.
    """
    deactivations_before = len(rig.bridge.switch_deactivate_calls)
    activations_before = len(rig.bridge.switch_activate_calls)
    settling = rig.begin_motion_session()
    steps = {entry['id']: entry for entry in settling['session']['steps']}
    assert steps['baseline']['status'] == 'done'
    assert steps['controller_pause']['status'] == 'done'
    assert steps['controller']['status'] in ('done', 'active')
    assert list(steps) == ['preflight', 'connect:panda1', 'connect:panda2',
                           'health', 'stack_ready', 'controller_pause',
                           'baseline', 'controller', 'settling']
    assert rig.bridge.switch_deactivate_calls[deactivations_before:] == [
        (CONTROLLER_NAME,)], rig.bridge.switch_deactivate_calls
    assert rig.bridge.switch_activate_calls[activations_before:] == [
        (CONTROLLER_NAME,)], rig.bridge.switch_activate_calls

    lower, upper = rig.fences['panda1']
    for arm_id in ARM_IDS:
        motion = settling['arms'][arm_id]['motion']
        assert motion['fence_lower'] == pytest.approx(list(lower), abs=1e-12)
        assert motion['fence_upper'] == pytest.approx(list(upper), abs=1e-12)
    assert_settling_is_command_closed(rig, settling)
    rig.finish_motion_settling()
    rig.stop_session()
    assert rig.state()['session']['state'] == 'stopped'


def test_03_a_pose_outside_the_fence_faults_at_the_baseline_step(rig):
    """
    The fence-vs-pose refusal moved INSIDE the session, before any torque.

    It now happens at the ``baseline`` step in ``starting``, while the
    impedance controller is not yet active, and it FAULTS rather than
    refusing the request. The message names the joint and both bounds in
    degrees, and no enable is ever possible on the way through.
    """
    rig.set_mock_pose({arm_id: outside_fence_pose() for arm_id in ARM_IDS})
    try:
        rig.start_session(arms='both', mode='motion')
        frame = rig.wait_for_frame(
            lambda snapshot: snapshot['session']['state'] == 'fault',
            timeout_s=90.0, description='the baseline check faults the session')
        steps = {entry['id']: entry for entry in frame['session']['steps']}
        assert steps['baseline']['status'] == 'failed'
        detail = steps['baseline']['detail']
        assert 'panda1 J1' in detail, detail
        assert '\u00b0' in detail, detail
        assert frame['session']['last_error']['code'] == 'pose_outside_fence'
        assert frame['fault']['cause'] == 'session_wedged'
        assert frame['fault']['action'] == 'restart'
        for arm_id in ARM_IDS:
            assert frame['arms'][arm_id]['motion']['enabled'] is False
            assert rig.mock.inbox(rig.slot(arm_id)).enabled is False
        status, refusal = rig.enable('panda1', True, expect=409)
        assert refusal['error'] in ('session_faulted', 'session_not_running')
        rig.stop_session()
    finally:
        rig.set_mock_pose({arm_id: HOME_POSE for arm_id in ARM_IDS})
    assert rig.state()['session']['state'] == 'stopped'


def test_04_the_motion_session_starts_and_offers_the_jog_surface(rig):
    """
    With the reviewed fence the motion session reaches ``running``, disabled.

    Every enable is off at session start, always (plan section 6.13), and the
    frame advertises the jog surface through ``motion.available`` rather than
    through the session mode (frame rule 4).
    """
    settling = rig.begin_motion_session()
    # On-the-wire proof that startup sends NO controller-side SetBool AFTER
    # the activation. The snapshot is taken here, immediately past the
    # restage's own re-activation: onActivate() has just disabled and
    # invalidated every inbox and captured the measured pose, and the mock
    # re-seeds its internal target on every enable-generation change exactly
    # as the reviewed controller's RT update does. A redundant false-to-false
    # call from here on would show up as a bumped generation and a rebased
    # target -- the interaction that produced Panda 2's second J2 settling
    # episode in web-20260831-152313.
    after_activation = {
        arm_id: (rig.mock.inbox(rig.slot(arm_id)).enable_generation,
                 rig.mock.internal_target(rig.slot(arm_id)))
        for arm_id in ARM_IDS}
    assert_settling_is_command_closed(rig, settling)
    frame = rig.finish_motion_settling()
    for arm_id in ARM_IDS:
        generation, target = after_activation[arm_id]
        inbox = rig.mock.inbox(rig.slot(arm_id))
        assert inbox.enabled is False
        assert inbox.enable_generation == generation, (
            '{}: a controller-side SetBool reached the enable service after '
            'the activation; the captured activation target was rebased'.format(
                arm_id))
        assert rig.mock.internal_target(rig.slot(arm_id)) == target, (
            '{}: the mock re-seeded its internal target after the '
            'activation'.format(arm_id))
    assert frame['session']['mode'] == 'motion'
    # Neither key survives in v2: the controller is not a request field, and
    # there is no uploaded configuration to identify.
    assert 'controller_name' not in frame['session']
    assert 'gains_sha256' not in frame['session']
    assert frame['fault']['active'] is False
    lower, upper = rig.fences['panda1']
    for arm_id in ARM_IDS:
        motion = frame['arms'][arm_id]['motion']
        assert motion['available'] is True
        assert motion['enabled'] is False, 'enables are off at session start, always'
        assert motion['target'] is None
        assert motion['targets_published'] == 0
        assert motion['enable_service_available'] is True, (
            'the mock controller offers ~/arm_<n>/enable; the bridge did not '
            'find it')
        assert motion['pose_inside_fence'] is True
        assert motion['fence_lower'] == pytest.approx(list(lower), abs=1e-12)
        assert motion['fence_upper'] == pytest.approx(list(upper), abs=1e-12)
    assert rig.message_count('panda1') == 0, (
        'nothing may be published before an Enable (plan section 8, step 4 of '
        'the supervised real session)')

    # FAKE HARDWARE ONLY, made executable: the supervisor was handed exactly
    # one child role and never anything resembling a launch command line.
    assert {entry['name'] for entry in rig.spawned} == {'launch'}, rig.spawned
    for entry in rig.spawned:
        assert 'ros2' not in entry['argv'], (
            'the supervisor was handed a real launch argv: {}'.format(entry['argv']))
        assert entry['options'].get('parent_death_signal') is not None, (
            'every supervised child must carry a parent-death signal: {}'.format(entry))


def test_05_every_target_is_accepted_at_20_hz_for_30_seconds(rig):
    """
    Both arms enabled: zero rejections, 20 Hz +/- 2 Hz, no gap over 0.08 s.

    This is the safety-relevant measurement of the whole stage. Every
    rejection invalidates the controller's buffered target and freezes the arm
    under its 0.1 s watchdog, so "almost always accepted" is not a passing
    grade; and a publish gap over 0.08 s is an arm that stops moving mid-jog.
    Both numbers are counted by the mock, from its own inbox and its own
    receive clock.
    """
    for arm_id in ARM_IDS:
        _, payload = rig.enable(arm_id, True)
        assert payload['enabled'] is True
        assert payload['message'] == ENABLE_ENABLED_MESSAGE
        assert payload['target'] == pytest.approx(list(HOME_POSE), abs=1e-9), (
            'enable must seed the held target from the MEASURED pose')

    # Sample the SSE wire while the motion block is live; case 14 validates it.
    head, records = collect_sse(rig.port, 3.0)
    assert ' 200 ' in head, 'the SSE stream did not answer 200:\n{}'.format(head)
    assert 'text/event-stream' in head.lower(), head
    rig.sse_records = records
    live = [payload for name, payload in records if name == 'state']
    assert len(live) >= 5, 'only {} state events arrived in 3 s'.format(len(live))
    assert any(frame['arms']['panda1']['motion']['enabled'] for frame in live), (
        'no sampled SSE frame showed an enabled arm, so the motion block was '
        'never exercised on the wire')

    rig.mock.reset_observations()
    elapsed = rig.settle(MEASURE_WINDOW_S)

    for arm_id in ARM_IDS:
        assert rig.buffered_target(arm_id) == pytest.approx(
            list(HOME_POSE), abs=1e-9), (
            'the stream must carry the seeded measured pose, not a default; '
            '{} holds {}'.format(arm_id, rig.buffered_target(arm_id)))
        stats = rig.mock.stats(rig.slot(arm_id))
        assert stats['messages'] > 0, 'no targets reached {}'.format(arm_id)
        assert stats['rejected'] == 0, (
            '{} rejected {} of {} targets over {:.1f} s ({}); the plan '
            'requires ZERO'.format(arm_id, stats['rejected'], stats['messages'],
                                   elapsed, stats['results']))
        assert stats['inactive'] == 0
        assert set(stats['results']) == {'Accepted'}, stats['results']
        rate = stats['messages'] / elapsed
        assert abs(rate - defaults.JOG_STREAM_HZ) <= RATE_TOLERANCE_HZ, (
            '{} received {:.2f} Hz over {:.1f} s; the contract is {} Hz '
            '+/- {} Hz'.format(arm_id, rate, elapsed, defaults.JOG_STREAM_HZ,
                               RATE_TOLERANCE_HZ))
        assert stats['max_gap_s'] is not None
        assert stats['max_gap_s'] <= MAX_GAP_S, (
            '{} saw a {:.3f} s gap between targets (limit {:.3f} s, controller '
            'watchdog {:.2f} s)'.format(
                arm_id, stats['max_gap_s'], MAX_GAP_S, WATCHDOG_TIMEOUT_S))


def test_06_a_jog_moves_the_on_wire_target_by_exactly_one_step(rig):
    """
    The reviewed Panda 1 joint-7 positive jog moves exactly one 2-degree step.

    Measured on the wire: the mock's buffered target is what ``accept``
    actually stored, so this compares the controller's view before and after,
    not the server's.  The zero-based API index is 6 and direction ``+1``;
    this is the exact tuple selected for the later supervised real-arm test.
    """
    joint_index = 6
    before = rig.buffered_target('panda1')
    status, payload = rig.jog('panda1', joint_index, 1)
    assert status == 200
    assert payload['clamped'] == [False] * defaults.JOINT_COUNT

    moved = wait_until(
        lambda: rig.buffered_target('panda1')[joint_index] != before[joint_index],
        timeout_s=3.0, poll_s=0.01)
    assert moved, 'the jogged target never reached the controller'
    after = rig.buffered_target('panda1')

    assert after[joint_index] - before[joint_index] == pytest.approx(
        defaults.JOG_STEP_RAD, abs=1e-12), (
        'joint {} moved {!r}, not one JOG_STEP_RAD ({!r})'.format(
            joint_index, after[joint_index] - before[joint_index],
            defaults.JOG_STEP_RAD))
    for index in range(defaults.JOINT_COUNT):
        if index == joint_index:
            continue
        assert after[index] == pytest.approx(before[index], abs=1e-12), (
            'jogging joint {} also moved joint {}'.format(joint_index, index))
    assert payload['target'] == pytest.approx(list(after), abs=1e-12), (
        'the API answered a different target from the one it put on the wire')
    assert_no_rejections(rig, 'after a jog')


def test_07_a_jog_at_the_fence_boundary_clamps_on_the_wire(rig):
    """
    A jog past the fence clamps to the fence and says so; the wire agrees.

    The controller rejects a position outside ``[position_lower,
    position_upper]`` with ``PositionLimitExceeded`` and freezes, so the model
    must clamp rather than emit the out-of-range value -- and must report the
    clamp instead of swallowing the press. The immediately preceding ordered
    case already made and verified the first in-range J7+ press on the wire;
    this case starts from that exact buffered target and makes the crossing
    press once.
    """
    index = TIGHT_JOINT_INDEX
    upper = rig.fences['panda1'][1][index]

    inside = rig.buffered_target('panda1')
    assert inside[index] == pytest.approx(
        HOME_POSE[index] + defaults.JOG_STEP_RAD, abs=1e-12), (
            'the preceding one-step J7+ wire check did not leave the expected '
            'in-fence target: {!r}'.format(inside[index]))
    assert inside[index] < upper

    _, clamped = rig.jog('panda1', index, 1)
    assert clamped['clamped'][index] is True, (
        'the second press crosses the fence and must report the clamp: '
        '{}'.format(clamped))
    assert [value for position, value in enumerate(clamped['clamped'])
            if position != index] == [False] * (defaults.JOINT_COUNT - 1)
    assert clamped['target'][index] == pytest.approx(upper, abs=1e-12)

    reached = wait_until(
        lambda: rig.buffered_target('panda1')[index] == pytest.approx(
            upper, abs=1e-12),
        timeout_s=3.0, poll_s=0.01)
    assert reached, (
        'the clamped target never reached the controller; the wire holds '
        '{!r}'.format(rig.buffered_target('panda1')[index]))
    assert rig.buffered_target('panda1')[index] <= upper + 1e-12, (
        'the on-wire target left the fence')
    assert_no_rejections(rig, 'after a clamped jog')


def test_08_enable_is_refused_while_the_measured_pose_is_outside_the_fence(rig):
    """
    Moving an arm out of its fence makes Enable refuse, live, over HTTP.

    The start-time check of case 3 is not the only guard: the arm may have
    been moved since. This drives the real mock hardware out of the fence with
    the pose-setter controller, so the refusal is computed from a genuinely
    measured ``/franka/joint_states`` sample.
    """
    _, payload = rig.enable('panda2', False)
    assert payload['enabled'] is False
    assert payload['message'] == ENABLE_DISABLED_MESSAGE

    outside = list(HOME_POSE)
    outside[0] = OUTSIDE_FENCE_JOINT1
    rig.set_mock_pose({'panda1': HOME_POSE, 'panda2': outside})
    rig.wait_for_frame(
        lambda frame: frame['arms']['panda2']['motion']['pose_inside_fence'] is False,
        15.0, 'panda2 reporting its pose outside the fence')

    status, refusal = rig.enable('panda2', True, expect=412)
    assert status == 412
    assert refusal['error'] == 'pose_outside_fence', refusal
    assert 'joint1' in refusal['detail'], refusal['detail']
    assert rig.state()['arms']['panda2']['motion']['enabled'] is False

    rig.set_mock_pose({'panda1': HOME_POSE, 'panda2': HOME_POSE})
    rig.wait_for_frame(
        lambda frame: frame['arms']['panda2']['motion']['pose_inside_fence'] is True,
        15.0, 'panda2 back inside the fence')
    _, allowed = rig.enable('panda2', True)
    assert allowed['enabled'] is True
    assert allowed['target'] == pytest.approx(list(HOME_POSE), abs=1e-9)
    assert rig.wait_for_targets('panda2', 3, timeout_s=5.0)
    assert_no_rejections(rig, 'after re-enabling a re-seeded arm')


def test_09_a_diagnostic_error_faults_the_session_and_recover_returns_it(rig):
    """
    F1 fires, both enables are forced off, the stream stops, Recover works.

    The fault input is real: the mock publishes a synthetic ERROR
    ``DiagnosticStatus`` under the canonical panda1 name, the bridge's real
    ``/diagnostics`` subscription caches it, and the fault engine reads it off
    the health projection. The Recover call runs the real section 7.3 sequence
    against the mock's real ``ErrorRecovery`` service.
    """
    enable_both(rig)
    rig.mock.reset_observations()
    assert rig.wait_for_targets('panda1', 3, timeout_s=5.0)

    rig.set_diagnostic('panda1', 2, 'synthetic ERROR from the Stage 2 e2e')
    frame = rig.wait_for_frame(
        lambda current: current['session']['state'] == 'fault',
        5.0, 'the session faulting on a canonical ERROR diagnostic')

    assert frame['fault']['active'] is True
    assert frame['fault']['recoverable'] is True, (
        'a diagnostic error is one the recovery path addresses (plan 7.1)')
    assert frame['fault']['recover_hint']
    codes = [(reason['code'], reason['arm_id']) for reason in frame['fault']['reasons']]
    assert ('diagnostic_error', 'panda1') in codes, codes
    # The plain-words classification the console renders. A diagnostic error
    # is not an external stop and not a protective stop, so it falls through
    # to the session-wide row -- with Recover offered, because it is one the
    # recovery path addresses.
    assert frame['fault']['cause'] == 'session_wedged', frame['fault']
    assert frame['fault']['action'] == 'recover'
    assert frame['fault']['headline'] == (
        'The session stopped and cannot continue.')
    assert frame['fault']['steps'] == [
        'Press Stop, then start a new session.',
        'Open the logs to see what failed.']
    assert frame['hint'] == (
        'Check that nobody pressed a stop, then press Recover.')
    for arm_id in ARM_IDS:
        assert frame['arms'][arm_id]['motion']['enabled'] is False, (
            'entering fault forces every enable off FIRST (plan section 7.1)')

    counts = {arm_id: rig.message_count(arm_id) for arm_id in ARM_IDS}
    time.sleep(QUIET_WINDOW_S)
    for arm_id in ARM_IDS:
        assert rig.message_count(arm_id) == counts[arm_id], (
            '{} was still receiving targets after the fault'.format(arm_id))

    # Plan section 7.2: release the physical stop FIRST, then press Recover.
    rig.set_diagnostic('panda1', 0, 'franka_web motion e2e: nominal')
    rig.wait_for_frame(
        lambda current: current['arms']['panda1']['diagnostic']['level'] == 0,
        5.0, 'the synthetic diagnostic clearing')
    assert rig.state()['session']['state'] == 'fault', (
        'clearing the diagnostic must not silently un-fault the session; only '
        'a successful Recover may (plan section 7.3)')

    recoveries_before = {
        arm_id: rig.mock.error_recovery_calls(arm_id) for arm_id in ARM_IDS}
    enable_calls_before = {
        arm_id: rig.mock.enable_service_calls(arm_id) for arm_id in ARM_IDS}
    deactivations_before = len(rig.bridge.switch_deactivate_calls)
    _, recovered = rig.recover()
    assert recovered['enabled_after'] is False, (
        're-enabling is a fresh authorization, never an automatic continuation')
    steps = recovered['steps']
    assert all(step['ok'] for step in steps), steps
    recovery_steps = [step for step in steps if step['step'] == 'error_recovery']
    assert [step['arm_id'] for step in recovery_steps] == list(ARM_IDS), steps
    assert all(step['detail'] == NO_ERRORS_MESSAGE for step in recovery_steps), (
        'a success=false/"No errors" reply is informational, not a failure '
        '(plan section 0.8): {}'.format(recovery_steps))
    controller_steps = [step for step in steps
                        if step['step'] == 'controller_active']
    assert controller_steps[-1]['controller'] == CONTROLLER_NAME, steps
    # Since-recovery, not since-start: the start path drives its own
    # deactivate/activate pair (the restage) before this session ever ran.
    assert rig.bridge.switch_deactivate_calls[deactivations_before:] == [
        (CONTROLLER_NAME,)], (
        'the active motion controller must be disabled and deactivated before '
        'backend/hardware restoration')
    assert rig.bridge.switch_activate_calls[-1] == (CONTROLLER_NAME,), (
        'the motion controller must be restored last')
    for arm_id in ARM_IDS:
        assert rig.mock.error_recovery_calls(arm_id) == recoveries_before[arm_id] + 1, (
            'Recover did not call {} ErrorRecovery'.format(arm_id))
    # Exactly ONE controller-side SetBool per arm reached the enable SERVICE:
    # the pre-deactivation disable, which is meaningful because the controller
    # really was active and really was enabled. A surviving post-reactivation
    # false call would show up here as a second request and would have rebased
    # the target onActivate() had just captured. The service count is the
    # measurement, not the inbox generation: onActivate() advances that itself.
    for arm_id in ARM_IDS:
        assert rig.mock.enable_service_calls(arm_id) == \
            enable_calls_before[arm_id] + 1, (
                '{}: recovery sent more than the single pre-deactivation '
                'disable'.format(arm_id))
    assert [(step['phase'], step['arm_id']) for step in steps
            if step['step'] == 'controller_disable'] == [
                ('pre', arm_id) for arm_id in ARM_IDS], steps
    assert [step['step'] for step in steps].index(
        'activation_disable_invariant') > max(
            index for index, step in enumerate(steps)
            if step['step'] == 'controller_active')

    settling = rig.wait_for_session_state('settling', 10.0)
    assert_settling_is_command_closed(rig, settling)
    running = rig.finish_motion_settling(10.0)
    assert running['fault']['active'] is False
    assert running['fault']['reasons'] == []
    assert running['fault']['cause'] is None
    assert running['fault']['action'] == 'none'
    for arm_id in ARM_IDS:
        assert running['arms'][arm_id]['motion']['enabled'] is False
        assert running['arms'][arm_id]['motion']['source'] == 'jog', (
            'a recovery is a fresh authorization: every source is back to jog')

    enable_both(rig)
    assert_no_rejections(rig, 'after recovering and re-enabling')


def test_10_disable_stops_the_stream_within_100_ms(rig):
    """``POST enable {"enabled": false}`` stops that arm's stream, at the mock."""
    enable_both(rig)
    rig.mock.reset_observations()
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    _, payload = rig.enable('panda1', False)
    reference = time.monotonic()
    assert payload['enabled'] is False
    assert payload['message'] == ENABLE_DISABLED_MESSAGE
    assert_stream_stopped(rig, reference, ['panda1'], 'disable')

    assert rig.message_count('panda2') > 0
    assert rig.wait_for_targets('panda2', 3, timeout_s=5.0), (
        'disabling one arm must not stop the other')


def test_11_operator_release_stops_the_stream_within_100_ms(rig):
    """``POST /api/operator/release`` stops every arm's stream, at the mock."""
    enable_both(rig)
    rig.mock.reset_observations()
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 5, timeout_s=5.0)

    reference = rig.release()
    assert_stream_stopped(rig, reference, list(ARM_IDS), 'operator release')

    frame = rig.state()
    assert frame['operator']['locked'] is False
    for arm_id in ARM_IDS:
        assert frame['arms'][arm_id]['motion']['enabled'] is False

    rig.claim()
    enable_both(rig)


def test_12_operator_lock_expiry_stops_the_stream_within_100_ms(rig):
    """
    A browser that stops heartbeating loses the arms at the TTL, at the mock.

    This is the abrupt-loss watchdog case of the supervised real session: the
    browser or network disappears without sending the normal pagehide release,
    and the arm must freeze in place at natural TTL expiry. Nothing here
    refreshes the token during the wait -- every mutating request would.
    """
    enable_both(rig)
    rig.mock.reset_observations()
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 5, timeout_s=5.0)

    beat = rig.heartbeat()
    expiry = beat + defaults.OPERATOR_LOCK_TTL_S
    rig.token = None                      # no request from here may refresh it
    while time.monotonic() < expiry + 1.0:
        time.sleep(0.2)

    assert rig.state()['operator']['locked'] is False, (
        'the operator lock did not expire {:.1f} s after the last '
        'heartbeat'.format(defaults.OPERATOR_LOCK_TTL_S))
    assert_stream_stopped(rig, expiry, list(ARM_IDS), 'operator lock expiry')

    for arm_id in ARM_IDS:
        assert rig.state()['arms'][arm_id]['motion']['enabled'] is False

    rig.claim()
    enable_both(rig)


def test_13_session_stop_stops_the_stream_within_100_ms(rig):
    """``POST /api/session/stop`` stops every arm's stream, at the mock."""
    rig.mock.reset_observations()
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 5, timeout_s=5.0)

    status, payload = rig.request('POST', '/api/session/stop')
    reference = time.monotonic()
    assert status == 202, payload
    assert payload['advisory'] == defaults.STOP_ADVISORY
    assert_stream_stopped(rig, reference, list(ARM_IDS), 'session stop')

    frame = rig.wait_for_session_state('stopped', 45.0)
    assert frame['arms'] == {}, 'frame rule 1: arms must be {} in stopped'
    assert frame['controllers'] == []
    assert_no_rejections(rig, 'over the whole session')


def test_14_every_published_state_frame_matches_the_contract(rig):
    """
    Every frame this run produced validates against the frozen schema.

    Both sources are checked: the frames the pump published (which are the
    exact objects the SSE fan-out carries) and the frames actually read off
    the wire during the enabled window in case 5.
    """
    validator = load_validator()
    frames = list(rig.frames)
    assert len(frames) > 100, (
        'only {} frames were published across the whole run'.format(len(frames)))

    for index, frame in enumerate(frames):
        errors = schema_errors(validator, frame)
        assert not errors, (
            'published frame {} of {} (session.state {!r}) is not '
            'contract-shaped:\n{}'.format(
                index + 1, len(frames), frame['session']['state'], errors))

    wire = [payload for name, payload in rig.sse_records if name == 'state']
    assert wire, 'case 5 recorded no SSE state frames'
    for index, frame in enumerate(wire):
        errors = schema_errors(validator, frame)
        assert not errors, (
            'SSE state frame {} of {} is not contract-shaped:\n{}'.format(
                index + 1, len(wire), errors))

    def motion_of(frame, key):
        return [arm['motion'][key] for arm in frame['arms'].values()]

    assert any(any(motion_of(frame, 'available')) for frame in frames), (
        'no frame ever advertised the jog surface')
    assert any(any(motion_of(frame, 'enabled')) for frame in frames), (
        'no frame ever showed an enabled arm')
    assert any(frame['session']['state'] == 'fault' for frame in frames)
    assert any(frame['session']['state'] == 'stopped' for frame in frames)
    assert not rig.tick_errors, (
        'the rig recorded background errors:\n{}'.format(
            '\n'.join(rig.tick_errors[:20])))


# ----------------------------------------------------------------------
# Layer 2, continued -- the per-arm command source and the takeover
# ----------------------------------------------------------------------


def test_16_switching_a_source_to_external_stops_the_stream_and_tracks_a_publisher(
        rig):
    """
    External silences the server's publisher and counts the operator's.

    Both halves matter. The server must stop publishing -- measured at the
    mock, to the same 100 ms bound as every other stop source -- because that
    is what makes every message counted afterwards the operator's own. And
    the rate must actually track a real publisher, because the console's
    "waiting for your publisher / receiving N Hz" hint is built on it.
    """
    rig.hold_lock()
    # Case 13 stopped the session; this one is its own story from the start.
    rig.begin_motion_session()
    rig.finish_motion_settling()
    enable_both(rig)
    rig.mock.reset_observations()
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 5, timeout_s=5.0)

    _, payload = rig.source('panda1', 'external')
    reference = time.monotonic()
    assert payload == {'ok': True, 'arm_id': 'panda1', 'source': 'external'}
    assert_stream_stopped(rig, reference, ['panda1'], 'source switch')
    assert rig.wait_for_targets('panda2', 3, timeout_s=5.0), (
        'switching one arm must not stop the other'

    )

    frame = rig.state()
    motion = frame['arms']['panda1']['motion']
    assert motion['source'] == 'external'
    # 0.0, not null: nothing has arrived yet, which is a different thing from
    # not counting at all.
    assert motion['external_rate_hz'] == 0.0
    assert motion['command_topic'] == '/{}/arm_1/joint_target'.format(
        CONTROLLER_NAME)
    assert motion['command_template_ready'] is True
    template = motion['command_template']
    assert template.startswith(
        '# trajectory_msgs/msg/JointTrajectory — publish at 10 Hz or more')
    measured = frame['arms']['panda1']['positions']
    rendered = ', '.join('{:.3f}'.format(value) for value in measured)
    assert '- positions: [{}]'.format(rendered) in template, template
    assert frame['hint'] == (
        'Waiting for your publisher on {} — 0.0 Hz'.format(
            motion['command_topic']))

    published = rig.publish_external_targets('panda1', hz=20.0, seconds=3.0)
    assert published >= 40, published
    frame = rig.wait_for_frame(
        lambda snapshot:
            (snapshot['arms']['panda1']['motion']['external_rate_hz'] or 0.0)
            >= 18.0,
        timeout_s=10.0, description='the external rate reaching 20 Hz')
    rate = frame['arms']['panda1']['motion']['external_rate_hz']
    assert 18.0 <= rate <= 22.0, rate
    assert frame['hint'] == (
        'Receiving {:.1f} Hz from your node. The watchdog freezes the arm if '
        'the stream stops.'.format(rate))

    before = rig.message_count('panda1')
    rig.source('panda1', 'jog')
    assert rig.wait_for_targets('panda1', before + 3, timeout_s=5.0), (
        'the server did not resume publishing on the way back to Jog')
    assert rig.state()['arms']['panda1']['motion']['external_rate_hz'] is None


def test_17_takeover_revokes_the_incumbent_and_stops_every_stream(rig):
    """
    "Taking over resets every enable" is a promise the endpoint keeps.

    The revocation hook runs synchronously, under the lock's own mutex,
    BEFORE the successor claim exists -- so there is no window in which the
    lock is free and a stale authorization is still live.
    """
    rig.hold_lock()
    if rig.state()['session']['state'] != 'running':
        rig.begin_motion_session()
        rig.finish_motion_settling()
    enable_both(rig)
    rig.mock.reset_observations()
    for arm_id in ARM_IDS:
        assert rig.wait_for_targets(arm_id, 5, timeout_s=5.0)
    incumbent = rig.token
    incumbent_claim = rig.claim_id

    successor = rig.takeover()
    reference = time.monotonic()
    assert successor['token'] != incumbent
    assert successor['claim_id'] != incumbent_claim
    assert_stream_stopped(rig, reference, list(ARM_IDS), 'operator takeover')

    frame = rig.state()
    assert frame['operator']['locked'] is True
    assert frame['operator']['claim_id'] == successor['claim_id']
    for arm_id in ARM_IDS:
        assert frame['arms'][arm_id]['motion']['enabled'] is False
        assert frame['arms'][arm_id]['motion']['source'] == 'jog'
    assert frame['hint'] == 'Enable an arm to allow commands.'

    # The incumbent's token is inert: it can neither enable nor jog.
    status, refusal = rig.request(
        'POST', '/api/arm/panda1/enable', body={'enabled': True},
        token=incumbent)
    assert status == 401 and refusal['error'] == 'operator_token_invalid'

    enable_both(rig)
    assert_no_rejections(rig, 'after the takeover and a fresh enable')
    rig.stop_session()


# ----------------------------------------------------------------------
# Layer 2, continued -- Apply, measured at the controller
# ----------------------------------------------------------------------
#
# Every travel below is deliberately SHORT, and the reason is a property of
# this rig rather than of the feature: the mock hardware does not follow its
# commanded target, so the commanded pose runs away from the measured one at
# exactly the travel's own speed. The lag brake fires at
# APPLY_LAG_LIMIT_RAD, which caps any travel here at about 2.5 s. That is
# itself one of the cases (E29), and it is why the others stay under it.


#: The joint every Apply case moves. Its fence margin is the wide one, so a
#: travel has room; joint 7 is the deliberately tight one and is left alone.
APPLY_JOINT = 3

#: A travel that completes comfortably inside the lag brake.
SHORT_TRAVEL_RAD = 0.12

#: A travel that does NOT: the commanded pose outruns the mock's stationary
#: measured pose past the brake's limit before it arrives.
LONG_TRAVEL_RAD = 0.30


def apply_goal(delta=SHORT_TRAVEL_RAD, joint=APPLY_JOINT):
    """Return HOME_POSE with one joint displaced by ``delta``."""
    pose = list(HOME_POSE)
    pose[joint] += delta
    return tuple(pose)


def controller_step(rig, arm_id='panda1', joint=APPLY_JOINT):
    """
    Return what the CONTROLLER can ramp through in one stream period.

    Read off the frame's own ``max_target_velocity`` -- the array the
    controller was launched with -- and deliberately NOT multiplied by
    APPLY_SPEED_FRACTION: a budget computed from the very constant under test
    would move with it, and a fraction raised past 1.0 would satisfy its own
    assertion while the executed path left the checked line.
    """
    velocity = rig.state()['arms'][arm_id]['motion']['max_target_velocity']
    return velocity[joint] / defaults.JOG_STREAM_HZ


def apply_ready(rig, arm_id='panda1'):
    """
    Bring the rig to a running Motion session with ``arm_id`` on Ghost.

    The source is cycled through Jog on every call, and that is not tidying:
    switching back re-seeds the held target from the MEASURED pose, and on
    this rig the mock hardware never follows, so a previous case leaves the
    held target several degrees away from where the arm reports itself. A
    real arm would have travelled there; this one has not, and the alignment
    gate is right to refuse an Apply planned from a held target that has
    stopped describing its arm. The cycle is what a real operator's arm gets
    for free.
    """
    rig.hold_lock()
    if rig.state()['session']['state'] != 'running':
        rig.begin_motion_session()
        rig.finish_motion_settling()
    if rig.state()['arms'][arm_id]['motion']['enabled'] is not True:
        rig.enable(arm_id, True)
    rig.source(arm_id, 'jog')
    rig.source(arm_id, 'ghost')
    assert rig.wait_for_targets(arm_id, 3, timeout_s=5.0), (
        'the stream is not running for {} before an Apply'.format(arm_id))
    frame = rig.state()
    assert frame['arms'][arm_id]['motion']['target'] == pytest.approx(
        list(HOME_POSE), abs=1e-6), (
        'the held target did not return to the measured pose; every Apply '
        'below would be refused as not settled')


def apply_state(rig, arm_id='panda1'):
    """Return one arm's ``motion.apply`` block from a fresh frame."""
    return rig.state()['arms'][arm_id]['motion']['apply']


def wait_for_idle(rig, arm_id='panda1', timeout_s=15.0):
    """Wait until this arm's travel is over, and return the last frame."""
    return rig.wait_for_frame(
        lambda frame: frame['arms'][arm_id]['motion']['apply']['state'] == 'idle',
        timeout_s, 'the {} travel reaching idle'.format(arm_id))


def wire_deltas(rig, arm_id='panda1'):
    """Return the per-joint absolute deltas between consecutive on-wire targets."""
    log = rig.wire(arm_id)
    return [[abs(later[index] - earlier[index])
             for index in range(defaults.JOINT_COUNT)]
            for earlier, later in zip(log, log[1:])]


def nudge_co_arm(rig, delta=0.05, seconds=0.4):
    """
    Move the OTHER arm without waiting for it to settle.

    ``set_mock_pose`` blocks until the hardware reports the commanded pose,
    which is several travel-seconds on this rig. What a co-arm drift case
    needs is the move to land WHILE a travel runs, so the command is simply
    published and the assertions wait on the frame instead.
    """
    other = list(HOME_POSE)
    other[0] += delta
    flat = [float(value) for value in HOME_POSE] + [float(value) for value in other]
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rig.tools.publish_pose_command(flat)
        time.sleep(0.05)


def test_18_an_apply_travels_the_on_wire_target_from_here_to_there(rig):
    """
    E18. The whole feature, measured at the controller.

    Consecutive published targets trace the checked line; the final on-wire
    target is the goal EXACTLY, because the last waypoint is the goal by
    identity rather than by arithmetic; and the stream keeps publishing
    afterwards, because arriving is a hold, not a stop.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    goal = apply_goal()
    status, payload = rig.apply_start('panda1', goal)
    assert status == 202, payload
    assert payload['goal'] == pytest.approx(list(goal), abs=1e-12)
    assert payload['steps_total'] > 1
    assert payload['duration_s'] > 0.0
    assert payload['checked']['samples_evaluated'] == 143

    wait_for_idle(rig)
    log = rig.wire('panda1')
    assert len(log) > 10, 'too few targets reached the mock to prove anything'
    assert log[-1][APPLY_JOINT] == goal[APPLY_JOINT], (
        'the final on-wire target is not the goal EXACTLY: {} vs {}'.format(
            log[-1][APPLY_JOINT], goal[APPLY_JOINT]))
    assert rig.buffered_target('panda1') == pytest.approx(list(goal), abs=1e-12)
    # Every intermediate target is on the line between the two ends.
    for positions in log:
        assert HOME_POSE[APPLY_JOINT] - 1e-9 <= positions[APPLY_JOINT] \
            <= goal[APPLY_JOINT] + 1e-9, positions[APPLY_JOINT]
        for index in range(defaults.JOINT_COUNT):
            if index == APPLY_JOINT:
                continue
            assert positions[index] == pytest.approx(HOME_POSE[index], abs=1e-9)

    assert rig.wait_for_targets('panda1', 3, timeout_s=5.0), (
        'the stream stopped when the travel arrived; arriving is a hold')
    assert apply_state(rig)['state'] == 'idle'
    assert apply_state(rig)['goal'] is None
    assert rig.state()['arms']['panda1']['motion']['source'] == 'ghost', (
        'arriving is not a reason to change who commands the arm')
    assert_no_rejections(rig, 'across a whole travel')


def test_19_no_two_consecutive_on_wire_targets_exceed_the_slew_budget(rig):
    """
    E19. The per-joint step the controller can actually track, on the wire.

    The 20 % headroom is what keeps the controller's per-joint slew limiter
    from clamping some joints and not others -- which would take the executed
    path off the line the model approved. Measured between consecutive
    messages the mock received, not from the server's own bookkeeping.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    ceiling = controller_step(rig)
    assert defaults.APPLY_SPEED_FRACTION < 1.0, (
        'the headroom is the whole point of the budget')
    rig.apply_start('panda1', apply_goal())
    wait_for_idle(rig)
    deltas = wire_deltas(rig)
    assert deltas, 'no consecutive pair of targets was recorded'
    worst = max(max(row) for row in deltas)
    assert worst < ceiling, (
        'a step of {:.6f} rad went on the wire; the controller can ramp '
        '{:.6f} in one stream period'.format(worst, ceiling))
    assert worst > 0.0, 'nothing moved, so this proves nothing'


def test_20_an_apply_through_a_blocked_path_publishes_nothing(rig):
    """
    E20. A refusal changes nothing on the wire, and the hold continues.

    The check is the last thing that can refuse, and a refusal is never a
    publish: the held target does not move at all, while the stream keeps
    feeding the controller's watchdog.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    assert rig.wait_for_targets('panda1', 3, timeout_s=5.0)
    before = rig.buffered_target('panda1')
    published_before = rig.message_count('panda1')

    rig.cell_model.path_result = CheckResult(
        ok=False, min_clearance=-0.008,
        contacts=(Contact(kind='cross_arm', a='panda1_link5_v1',
                          b='panda2_link6_v1', distance=0.022, required=0.030,
                          arm_id='panda1'),),
        sample_index=40, samples_evaluated=120)
    try:
        status, refusal = rig.apply_start('panda1', apply_goal(), expect=412)
    finally:
        rig.cell_model.path_result = None
    assert refusal['error'] == 'apply_refused'
    assert refusal['reason_code'] == 'contact'
    assert refusal['offending_links'] == ['panda1_link5', 'panda2_link6']
    assert refusal['detail'].startswith('About 34% of the way there: ')

    time.sleep(2.0)
    assert rig.buffered_target('panda1') == pytest.approx(list(before), abs=1e-12), (
        'a refused Apply moved the on-wire target')
    assert rig.message_count('panda1') > published_before, (
        'the hold stopped when an Apply was refused')
    assert apply_state(rig)['state'] == 'idle'


def test_21_apply_on_a_disabled_arm_publishes_nothing(rig):
    """E21. Enable is a precondition, not something an Apply can imply."""
    apply_ready(rig)
    rig.enable('panda1', False)
    time.sleep(2.0 / defaults.STATE_FRAME_HZ)
    rig.mock.reset_observations()
    status, refusal = rig.apply_start('panda1', apply_goal(), expect=409)
    assert refusal['error'] == 'arm_not_enabled'
    time.sleep(QUIET_WINDOW_S)
    assert rig.message_count('panda1') == 0, (
        'a target reached a disabled arm after a refused Apply')
    rig.enable('panda1', True)


def test_22_cancel_freezes_the_on_wire_target_within_100_ms(rig):
    """
    E22. Stop-and-hold, against an IDLE supervisor.

    The last waypoint is a point on the checked line, so holding there is a
    pose the model approved -- and holding keeps the watchdog fed, so the arm
    stays immediately jog-able. Case 34 is the one that matters; this is its
    easy half.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 8, timeout_s=5.0)

    _, payload = rig.apply_cancel('panda1')
    reference = time.monotonic()
    assert payload['was_travelling'] is True
    assert 0.0 < payload['fraction'] < 1.0
    assert payload['stopped_at'] is not None

    # Two ticks for the advance to stop, then the target must never move again.
    time.sleep(2.0 / defaults.JOG_STREAM_HZ)
    frozen = rig.buffered_target('panda1')
    elapsed = time.monotonic() - reference
    assert elapsed < 0.2, elapsed
    counts = rig.message_count('panda1')
    time.sleep(QUIET_WINDOW_S)
    assert rig.buffered_target('panda1') == pytest.approx(list(frozen), abs=1e-12), (
        'the target kept moving after a cancel')
    assert rig.message_count('panda1') > counts, (
        'the stream stopped on a cancel; a cancel is a HOLD')
    assert apply_state(rig)['state'] == 'idle'


def test_34_cancel_stops_the_target_while_the_supervisor_is_blocked(rig):
    """
    E34. The bound that decides whether Apply is safe to demonstrate at all.

    Every other operator command in this server is a queued ``_Command``, and
    ``_submit`` DISCARDS one the supervisor never reached -- so a queued
    Cancel would answer an error after five seconds while the arm kept
    travelling. Before Apply, a blocked supervisor left a jogged arm STATIC. A
    travelling arm is not static, so this is the one stop that comes off the
    queue, and this is where that is measured.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 6, timeout_s=5.0)

    # Hold the supervisor thread inside a real service call, the way a slow
    # controller does, and issue the stop while it is in there.
    rig.bridge.stall_next_enable_s = 3.0
    outcome = {}

    def enable_other():
        """Enable the other arm; the supervisor blocks inside the call."""
        outcome['enable'] = rig.request(
            'POST', '/api/arm/panda2/enable', body={'enabled': True},
            timeout_s=30.0)
    blocked = threading.Thread(target=enable_other, daemon=True)
    blocked.start()
    try:
        # Long enough for the supervisor to take the command and enter the
        # stall (its tick is 0.1 s), short enough to still be inside it.
        time.sleep(0.5)
        started = time.monotonic()
        status, payload = rig.apply_cancel('panda1')
        elapsed = time.monotonic() - started
        assert payload['was_travelling'] is True, payload
        assert elapsed < 0.2, (
            'the cancel took {:.3f} s while the supervisor was blocked; it '
            'was queued behind the enable'.format(elapsed))
        time.sleep(2.0 / defaults.JOG_STREAM_HZ)
        frozen = rig.buffered_target('panda1')
        counts = rig.message_count('panda1')
        time.sleep(QUIET_WINDOW_S)
        assert rig.buffered_target('panda1') == pytest.approx(
            list(frozen), abs=1e-12), 'the travel resumed after the cancel'
        assert rig.message_count('panda1') > counts, (
            'the hold stopped; a cancel keeps feeding the watchdog')
    finally:
        blocked.join(timeout=30)
        rig.bridge.stall_next_enable_s = None
    assert outcome['enable'][0] == 200, outcome['enable']
    rig.enable('panda2', False)


def test_35_a_burst_of_ticks_does_not_advance_the_travel_faster_than_the_stream(
        rig):
    """
    E35. A stalled executor delivers ticks in a BURST when it catches up.

    That burst is the one schedule that could put more than one step's worth
    of command ahead of the controller's ramp -- which is exactly the width of
    the tube the executed path is proved to stay inside. The floor between two
    advances can only ever delay a step, never bring one forward.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 3, timeout_s=5.0)
    try:
        with rig.supervisor._state_lock:
            before = rig.supervisor._arm_travel['panda1'].step
        for _ in range(10):
            rig.supervisor.jog_stream_tick()
        with rig.supervisor._state_lock:
            after = rig.supervisor._arm_travel['panda1'].step
        # At most one advance of our own, plus at most one from the bridge's
        # own timer landing inside the burst. Without the floor it would be
        # ten.
        assert after - before <= 2, (
            'ten ticks inside one stream period advanced the travel {} '
            'steps'.format(after - before))
        ceiling = controller_step(rig)
        worst = max((max(row) for row in wire_deltas(rig)), default=0.0)
        assert worst < ceiling, worst
    finally:
        rig.apply_cancel('panda1')


def test_23_disable_during_a_travel_stops_the_stream_within_100_ms(rig):
    """E23. The sibling of case 10, with a travel running."""
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    _, payload = rig.enable('panda1', False)
    reference = time.monotonic()
    assert payload['enabled'] is False
    assert_stream_stopped(rig, reference, ['panda1'], 'disable during a travel')
    assert apply_state(rig)['state'] == 'idle'
    rig.enable('panda1', True)


def test_27_re_enabling_after_a_disable_mid_travel_does_not_resume(rig):
    """
    E27. Re-enabling is a new authorization, never an automatic continuation.

    The travel does not pick up where it left off, and the goal is never
    reached. What the arm holds afterwards is its own MEASURED pose, because
    the enable path re-seeds from measurement -- which is where the arm
    actually is, and is the pre-existing behaviour a travel does not change.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    goal = apply_goal(LONG_TRAVEL_RAD * 0.5)
    rig.apply_start('panda1', goal)
    assert rig.wait_for_targets('panda1', 6, timeout_s=5.0)
    rig.enable('panda1', False)
    time.sleep(2.0 / defaults.STATE_FRAME_HZ)
    rig.enable('panda1', True)
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    time.sleep(2.0)
    held = rig.buffered_target('panda1')
    assert held[APPLY_JOINT] != pytest.approx(goal[APPLY_JOINT], abs=1e-6), (
        'the travel resumed across a disable and reached its goal')
    settled = rig.buffered_target('panda1')
    time.sleep(1.0)
    assert rig.buffered_target('panda1') == pytest.approx(
        list(settled), abs=1e-12), 'the target is still advancing after a disable'
    assert apply_state(rig)['state'] == 'idle'


def test_36_an_enable_that_fails_on_a_travelling_arm_leaves_no_live_travel(rig):
    """
    E36. The three enable exits that leave the arm ENABLED are the dangerous ones.

    ``model.seed(measured)`` runs before the service call and therefore before
    every failure exit; a plan left live beside a re-seeded model would resume
    from a target that had just been yanked off the checked line. The plan is
    cleared at the TOP of the handler, so the rule holds on every exit.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    ready = rig.bridge.enable_service_ready
    rig.bridge.enable_service_ready = lambda slot: False
    try:
        status, refusal = rig.request(
            'POST', '/api/arm/panda1/enable', body={'enabled': True})
    finally:
        rig.bridge.enable_service_ready = ready
    assert refusal['error'] == 'enable_service_unavailable', refusal
    assert rig.state()['arms']['panda1']['motion']['enabled'] is True, (
        'this exit leaves the arm enabled, which is what makes it dangerous')
    assert apply_state(rig)['state'] == 'idle'

    time.sleep(1.0)
    held = rig.buffered_target('panda1')
    time.sleep(1.0)
    assert rig.buffered_target('panda1') == pytest.approx(list(held), abs=1e-12), (
        'the travel kept advancing after a failed enable')


def test_24_operator_release_during_a_travel_stops_the_stream(rig):
    """E24. The sibling of case 11, with a travel running."""
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    reference = rig.release()
    assert_stream_stopped(rig, reference, ['panda1'], 'release during a travel')
    rig.claim()
    frame = rig.state()
    assert frame['arms']['panda1']['motion']['enabled'] is False
    assert frame['arms']['panda1']['motion']['source'] == 'jog', (
        'authority left, so the source went back with it')
    assert frame['arms']['panda1']['motion']['apply']['state'] == 'idle'


def test_25_takeover_during_a_travel_clears_every_travel(rig):
    """E25. The sibling of case 17: a successor inherits nothing."""
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    rig.takeover()
    reference = time.monotonic()
    assert_stream_stopped(rig, reference, ['panda1'], 'takeover during a travel')
    frame = rig.state()
    assert frame['arms']['panda1']['motion']['apply']['state'] == 'idle'
    assert frame['arms']['panda1']['motion']['source'] == 'jog'


def test_26_a_fault_during_a_travel_stops_the_stream_and_clears_it(rig):
    """
    E26. A disbelieved picture of the cell stops every checked path.

    The fault input is real -- a synthetic ERROR ``DiagnosticStatus`` under
    the canonical name, through the bridge's own subscription -- and the
    travel goes with the enables and the sources, because continuing a
    checked path on a picture the console no longer believes is the worst
    available option.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 5, timeout_s=5.0)

    rig.set_diagnostic('panda1', 2, 'synthetic ERROR during a travel')
    frame = rig.wait_for_frame(
        lambda current: current['session']['state'] == 'fault',
        5.0, 'the session faulting during a travel')
    reference = time.monotonic()
    assert frame['arms']['panda1']['motion']['apply']['state'] == 'idle'
    assert frame['arms']['panda1']['motion']['source'] == 'jog'
    assert_stream_stopped(rig, reference, ['panda1'], 'fault during a travel')

    rig.set_diagnostic('panda1', 0, 'franka_web motion e2e: nominal')
    rig.wait_for_frame(
        lambda current: current['arms']['panda1']['diagnostic']['level'] == 0,
        5.0, 'the synthetic diagnostic clearing')
    rig.recover()
    rig.wait_for_session_state('settling', 10.0)
    running = rig.finish_motion_settling(10.0)
    assert running['fault']['active'] is False
    assert running['arms']['panda1']['motion']['apply']['state'] == 'idle'


def test_28_a_co_arm_that_moves_cancels_the_travel(rig):
    """
    E28. The pair-pose the model approved is the pair-pose that executes.

    Cancel, not re-check: re-deriving authority mid-motion from a pose that
    is itself moving is the kind of cleverness that hides a bug. The accepted
    cost is that a session with a moving co-arm cannot Apply, and the sentence
    the operator reads says exactly that.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    goal = apply_goal(LONG_TRAVEL_RAD * 0.6)
    rig.apply_start('panda1', goal)
    assert rig.wait_for_targets('panda1', 4, timeout_s=5.0)
    before = rig.logs_last_seq()
    try:
        nudge_co_arm(rig, delta=0.05, seconds=0.6)
        frame = wait_for_idle(rig, timeout_s=8.0)
        assert frame['arms']['panda1']['motion']['target'][APPLY_JOINT] \
            != pytest.approx(goal[APPLY_JOINT], abs=1e-6), (
            'the travel reached its goal even though the co-arm moved')
        lines = json.dumps(rig.log_window(before))
        assert 'moved while' in lines, lines
    finally:
        rig.set_mock_pose({arm_id: HOME_POSE for arm_id in ARM_IDS})


def test_29_an_arm_that_does_not_follow_cancels_the_travel(rig):
    """
    E29. The lag brake stops the travel before the torque ceilings have to.

    The mock never follows its commanded target, so this is the case this rig
    reproduces most faithfully of all: the commanded pose runs away from the
    measured one at the travel's own speed, and the brake catches it at
    APPLY_LAG_LIMIT_RAD -- with a sentence, rather than as a torque ceiling
    somebody has to infer.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    goal = apply_goal(LONG_TRAVEL_RAD)
    before = rig.logs_last_seq()
    rig.apply_start('panda1', goal)
    frame = wait_for_idle(rig, timeout_s=15.0)
    held = frame['arms']['panda1']['motion']['target'][APPLY_JOINT]
    lag = abs(held - HOME_POSE[APPLY_JOINT])
    assert lag < LONG_TRAVEL_RAD, 'the travel reached a goal it could not track'
    assert lag > defaults.APPLY_LAG_LIMIT_RAD * 0.9, lag
    lines = json.dumps(rig.log_window(before))
    assert 'not following its commanded pose' in lines, lines


def test_31_the_co_arm_pose_comes_from_measurement_not_the_request(rig):
    """
    E31. The request carries seven floats and nothing else that matters.

    The solve endpoint may take the client's scene verbatim because there it
    decides only a tint. Here it would decide motion, and only the server's
    own measurements will do -- so a fabricated co-arm pose in the body must
    reach nothing at all.
    """
    apply_ready(rig)
    rig.cell_model.paths = []
    fabricated = [value + 1.0 for value in HOME_POSE]
    status, payload = rig.request(
        'POST', '/api/arm/panda1/apply',
        body={'action': 'start', 'positions': list(apply_goal()),
              'scene': {'panda2': fabricated}, 'co_arm': fabricated})
    assert status == 202, payload
    checked = rig.cell_model.paths[-1]
    assert len(checked) == 3, checked
    for point in checked:
        assert point['panda2'] == pytest.approx(list(HOME_POSE), abs=1e-6), (
            'the check was given a co-arm pose that came from the request')
    rig.apply_cancel('panda1')


def test_37_the_checked_path_is_the_three_waypoint_one(rig):
    """
    E37. Measured, then held, then goal -- and the server chose all three.

    The wire-level twin of the unit case: a two-waypoint call would leave the
    segment the arm physically closes at the start of a travel unchecked, and
    that segment is real -- the arm is at its measured pose while it is
    commanded from its held target.
    """
    apply_ready(rig)
    rig.cell_model.paths = []
    rig.cell_model.path_flags = []
    goal = apply_goal()
    held = rig.state()['arms']['panda1']['motion']['target']
    rig.apply_start('panda1', goal)
    try:
        checked = rig.cell_model.paths[-1]
        assert len(checked) == 3
        assert checked[0]['panda1'] == pytest.approx(list(HOME_POSE), abs=1e-6)
        assert checked[1]['panda1'] == pytest.approx(list(held), abs=1e-9)
        assert checked[2]['panda1'] == pytest.approx(list(goal), abs=1e-12)
        # Whole-path evaluation, so the refusal can say WHERE on the way.
        assert rig.cell_model.path_flags[-1] is False
    finally:
        rig.apply_cancel('panda1')


def test_32_a_second_apply_is_refused_while_one_travels(rig):
    """
    E32. One travel at a time, session-wide, and the running one is untouched.

    Two independently timed checked paths do not compose: each was approved
    against the other arm held at a measured constant, and during execution
    neither assumption holds.
    """
    apply_ready(rig)
    rig.enable('panda2', True)
    rig.source('panda2', 'ghost')
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal(LONG_TRAVEL_RAD * 0.5))
    assert rig.wait_for_targets('panda1', 4, timeout_s=5.0)
    try:
        status, refusal = rig.apply_start('panda2', apply_goal(), expect=409)
        assert refusal['error'] == 'apply_in_progress'
        assert refusal['arm_id'] == 'panda1'
        assert apply_state(rig, 'panda1')['state'] == 'travelling'
        assert apply_state(rig, 'panda2')['state'] == 'idle'
    finally:
        rig.apply_cancel('panda1')
        rig.source('panda2', 'jog')
        rig.enable('panda2', False)


def test_30_every_published_state_frame_matches_the_v5_contract(rig):
    """
    E30. The sibling of case 14, over a session that includes a travel.

    Both halves of the schema delta are exercised: the third source value and
    the always-present apply block, in both of its states.
    """
    apply_ready(rig)
    validator = load_validator()
    seen = set()
    rig.apply_start('panda1', apply_goal())
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        frame = rig.state()
        errors = schema_errors(validator, frame)
        assert not errors, errors
        assert frame['schema_version'] == 5
        block = frame['arms']['panda1']['motion']['apply']
        seen.add(block['state'])
        if 'idle' in seen and 'travelling' in seen:
            break
        time.sleep(0.1)
    assert seen == {'travelling', 'idle'}, seen
    wait_for_idle(rig)


def test_33_every_message_a_travel_publishes_is_accepted_by_the_mock(rig):
    """
    E33. Zero rejections across a whole travel, at a port of ``accept``.

    A rejected message freezes the arm SILENTLY: the controller drops the
    target it had buffered and says nothing on the wire. This is the offline
    proof that a travel never produces one -- the message shape, the stamp
    freshness, the joint names and the fence all still hold under the new way
    the held target changes.
    """
    apply_ready(rig)
    rig.mock.reset_observations()
    rig.apply_start('panda1', apply_goal())
    wait_for_idle(rig)
    stats = rig.mock.stats(rig.slot('panda1'))
    assert stats['messages'] > 10, stats
    assert stats['rejected'] == 0, stats['results']
    assert stats['inactive'] == 0, stats['results']
    assert set(stats['results']) == {'Accepted'}, stats['results']
    rig.stop_session()


# ----------------------------------------------------------------------
# Layer 3 -- the negative test against the REAL controller
# ----------------------------------------------------------------------


def test_15_the_real_impedance_controller_cannot_activate_on_mock_hardware(rig):
    """
    Activating the REAL ``dual_arm_joint_impedance_controller`` must FAIL.

    Plan section 0.2 as an executable fact, named specifically so nobody later
    assumes fake motion works: ``mock_components/GenericSystem`` exports
    position, velocity and effort per joint and nothing else, while
    ``DualArmJointImpedanceController`` also claims the ``<arm>/robot_state``
    and ``<arm>/robot_model`` semantic interfaces of the real Franka hardware.
    It loads, and it CONFIGURES -- the config this rig uploaded is a valid one
    -- and then activation is refused.

    This case runs last: configuring the real controller creates
    ``~/arm_<n>/enable`` under the same node name the mock answers on, so it
    must not exist while the mock is the thing under test.

    The stack must survive the attempt: a failed activation that wrecked the
    controller_manager would be a much worse finding than the refusal itself.
    """
    before = rig.tools.controller_states()
    assert before is not None, 'the controller_manager did not answer'
    assert before.get('joint_state_broadcaster') == 'active'
    assert CONTROLLER_NAME not in before, (
        'the real controller was already loaded before this case: {}'.format(before))

    result = subprocess.run(
        ['ros2', 'run', 'controller_manager', 'spawner', CONTROLLER_NAME,
         '--param-file', rig.materialized_profile_path(),
         '--controller-manager', '/controller_manager',
         '--controller-manager-timeout', '30'],
        env=rig.child_environment(), cwd=rig.root,
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=120, check=False)

    assert result.returncode != 0, (
        'the real {} ACTIVATED on mock hardware. Fake motion is supposed to be '
        'impossible (plan section 0.2) and every Stage 2 decision rests on '
        'that; spawner output:\n{}'.format(CONTROLLER_NAME, result.stdout))

    after = rig.tools.controller_states()
    assert after is not None
    assert after.get(CONTROLLER_NAME) != 'active', (
        '{} reached {!r}; it must not be active on mock hardware'.format(
            CONTROLLER_NAME, after.get(CONTROLLER_NAME)))
    assert after.get('joint_state_broadcaster') == 'active', (
        'the failed activation deactivated joint_state_broadcaster; the stack '
        'was left worse than it was found: {}'.format(after))
    assert after.get(POSE_SETTER_NAME) == 'active', (
        'the failed activation disturbed the other running controller: '
        '{}'.format(after))

    tail = rig.launch_tail(200)
    assert CONTROLLER_NAME in tail and 'robot_state' in tail, (
        'the controller_manager did not refuse for the documented reason (a '
        'missing <arm>/robot_state state interface); launch tail:\n{}'.format(
            tail[-4000:]))
