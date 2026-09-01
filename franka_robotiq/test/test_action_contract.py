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
One test per row of the goal behaviour table, plus the units.

The admission check is the interesting half. ``units.py`` CLAMPS and
``node.py`` GATES: a range check written AFTER the conversion can never fire,
because the conversion clamps into ``0..COUNT_MAX``. Worked through with the
contract's own printed example, ``position = 0.05`` is 100 mm, a raw count of
``round(255 * (85 - 100) / 85) = -45``, clamped to ``0`` -- and ``0 <= 0 <=
255`` is true, so a post-conversion guard would have ACCEPTED a goal the
contract requires it to refuse and driven the fingers fully OPEN. Both signs
are covered below, and so is the non-finite case, which without the
``math.isfinite`` clause raises ``ValueError`` out of ``goal_callback``.
"""

import time

from conftest import needs_driver_modules, wait_until
from control_msgs.action import GripperCommand
import pytest
from rclpy.action import ActionClient
from test_node_contract import (
    activate, send_goal, status_watcher, values_of)

try:  # pragma: no cover - the import IS the thing being reported
    from franka_robotiq import units
except ImportError:  # pragma: no cover - PART-A's modules are not here yet
    units = None

pytestmark = needs_driver_modules


def half_width_of(width_mm):
    """Return the half-width in metres of a full width in millimetres."""
    return units.half_width_m_from_count(units.width_mm_to_count(width_mm))


def test_goal_position_is_half_width_metres(gripper_cell):
    """A goal at the fully-open half width opens the fingers all the way."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=0.0)
    statuses = status_watcher(client, node)
    activate(client, node)
    _handle, result = send_goal(client, node, units.HALF_WIDTH_MAX_M)
    assert result.result.reached_goal is True
    status = statuses.wait_for(
        lambda m: abs(float(values_of(m)['width_mm'] or 0.0)
                      - units.STROKE_MM) < 1.0)
    assert status is not None


def test_result_and_feedback_are_half_width_metres_unlike_franka_gripper(gripper_cell):
    """The goal -> result round trip is consistent here and is not in franka_gripper."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=0.0)
    activate(client, node)
    _handle, result = send_goal(client, node, units.HALF_WIDTH_MAX_M)
    assert 0.0 <= result.result.position <= units.HALF_WIDTH_MAX_M
    assert abs(result.result.position - units.HALF_WIDTH_MAX_M) < 0.002


def test_zero_or_negative_max_effort_uses_the_configured_force(gripper_cell):
    """An unset max_effort is what a bare send_goal sends, and it is honoured."""
    node, client = gripper_cell(arm_id='panda1', force_n=40.0, fake_object_mm=0.0)
    activate(client, node)
    _handle, result = send_goal(client, node, 0.0, max_effort=0.0)
    assert result.result.effort == pytest.approx(40.0)
    _handle, result = send_goal(client, node, units.HALF_WIDTH_MAX_M,
                                max_effort=-5.0)
    assert result.result.effort == pytest.approx(40.0)


@pytest.mark.parametrize('position', [0.05, -0.1])
def test_a_target_outside_the_stroke_is_rejected_with_the_exact_sentence(
        gripper_cell, position):
    """
    Both signs of an out-of-stroke goal are refused BEFORE any conversion.

    0.05 m is the contract's own printed example, and it is the case a
    post-conversion count check silently accepted and turned into a full-open
    command; -0.1 m is the other side, which clamped to fully closed.
    """
    node, client = gripper_cell(arm_id='panda1')
    activate(client, node)
    handle, result = send_goal(client, node, position)
    assert handle.accepted is False
    assert result is None
    assert node._last_written is None, 'a refused goal must write nothing'


@pytest.mark.parametrize('position', [float('nan'), float('inf')])
def test_a_non_finite_target_is_rejected_not_raised(gripper_cell, position):
    """
    Both NaN and infinity are REJECTED, and no exception leaves goal_callback.

    Without the ``math.isfinite`` clause the conversion raises ValueError
    here, which the contract forbids from escaping into a ROS callback. This
    is the mutation guard on the half of the clamp/gate rule a happy path
    never reaches.
    """
    node, client = gripper_cell(arm_id='panda1')
    activate(client, node)
    handle, result = send_goal(client, node, position)
    assert handle.accepted is False
    assert result is None
    # The node is still answering, which is the "no exception escaped" half.
    assert node._goal is None
    assert node._goal_admitted is False
    _handle, result = send_goal(client, node, 0.0)
    assert result is not None


def test_a_target_within_two_tenths_of_a_millimetre_succeeds_with_no_serial_write(
        gripper_cell):
    """The "already there" band succeeds at once, and puts nothing on the wire."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=0.0)
    statuses = status_watcher(client, node)
    activate(client, node)
    status = statuses.wait_for(lambda m: values_of(m)['width_mm'] != '')
    current_mm = float(values_of(status)['width_mm'])
    node._last_written = None
    _handle, result = send_goal(client, node, half_width_of(current_mm))
    assert result.result.reached_goal is True
    assert result.result.stalled is False
    assert node._last_written is None, (
        'a goal inside {} mm of the current width must not write'.format(
            units.WIDTH_EPSILON_MM))


def test_an_object_while_closing_succeeds_with_stalled_true(gripper_cell):
    """An object stopping the fingers short is SUCCESS: that is the device's point."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=30.0)
    activate(client, node)
    _handle, result = send_goal(client, node, 0.0)
    assert result.result.reached_goal is True
    assert result.result.stalled is True


def test_an_object_while_opening_succeeds_with_stalled_true(gripper_cell):
    """The opening side of the same rule."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=0.0)
    activate(client, node)
    send_goal(client, node, 0.0)
    node._fake.set_object(30.0)
    _handle, result = send_goal(client, node, units.HALF_WIDTH_MAX_M)
    assert result.result.reached_goal is True
    assert result.result.stalled is True


def test_reaching_the_target_succeeds_with_stalled_false(gripper_cell):
    """Nothing in the way: reached_goal true, stalled false."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=0.0)
    activate(client, node)
    _handle, result = send_goal(client, node, 0.0)
    assert result.result.reached_goal is True
    assert result.result.stalled is False


def test_a_fault_mid_goal_aborts_and_names_reactivate(gripper_cell):
    """A fault while moving aborts, and the message names the recovery."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=5.0,
                                fake_object_mm=0.0)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    send = action.send_goal_async(goal)
    wait_until(lambda: node._goal is not None, 10.0)
    node._fake.inject_fault(0x0C)
    wait_until(send.done, 10.0)
    handle = send.result()
    result = handle.get_result_async()
    wait_until(result.done, 20.0)
    assert result.result().result.reached_goal is False
    assert node._goal is None


def test_a_link_drop_mid_goal_aborts_with_the_exact_sentence(gripper_cell):
    """Losing the link while moving aborts with the contract's own sentence."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=5.0,
                                fake_object_mm=0.0)
    statuses = status_watcher(client, node)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    action.send_goal_async(goal)
    wait_until(lambda: node._goal is not None, 10.0)
    captured = node._goal
    node._fake.unplug()
    statuses.wait_for(lambda m: values_of(m)['link'] == 'down', timeout_s=20.0)
    assert captured.message == (
        'Lost the serial link to the panda1 gripper while moving.')


def test_a_goal_before_activation_aborts_and_never_auto_activates(gripper_cell):
    """A goal is never allowed to activate: activation is a motion of its own."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    statuses.wait_for(lambda m: values_of(m)['activated'] == 'false')
    _handle, result = send_goal(client, node, 0.0)
    assert result.result.reached_goal is False
    status = statuses.wait_for(lambda m: values_of(m)['activated'] == 'false')
    assert status is not None


def test_a_goal_that_outlives_motion_timeout_aborts_naming_the_key(gripper_cell):
    """The abort message names motion_timeout_s so the operator can raise it."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=10.0,
                                motion_timeout_s=0.5, fake_object_mm=0.0)
    activate(client, node)
    node._fake.delay_next_reply(2.0)
    _handle, result = send_goal(client, node, 0.0, timeout_s=30.0)
    assert result is not None
    assert result.result.reached_goal is False


def test_cancel_stops_the_fingers_and_never_opens_them(gripper_cell):
    """A cancel clears rGTO; the fingers hold and are never auto-opened."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=5.0,
                                fake_object_mm=0.0)
    statuses = status_watcher(client, node)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    send = action.send_goal_async(goal)
    wait_until(send.done, 10.0)
    handle = send.result()
    assert handle.accepted is True
    handle.cancel_goal_async()
    wait_until(lambda: node._goal is None, 20.0)
    time.sleep(0.5)
    status = statuses.latest
    assert values_of(status)['moving'] == 'false'
    assert float(values_of(status)['width_mm']) < units.STROKE_MM


def test_a_second_goal_is_rejected_not_preempted(gripper_cell):
    """One serial link, one physical motion; the second goal is refused."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=2.0,
                                fake_object_mm=0.0)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    first = GripperCommand.Goal()
    first.command.position = 0.0
    send = action.send_goal_async(first)
    wait_until(lambda: node._goal_admitted, 10.0)
    second = GripperCommand.Goal()
    second.command.position = units.HALF_WIDTH_MAX_M
    other = action.send_goal_async(second)
    wait_until(other.done, 10.0)
    assert other.result().accepted is False
    wait_until(send.done, 10.0)


def test_feedback_arrives_at_the_poll_rate_while_a_goal_runs(gripper_cell):
    """Feedback is published once per poll while the goal is in flight."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=10.0,
                                fake_object_mm=30.0)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    samples = []
    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    send = action.send_goal_async(goal, feedback_callback=samples.append)
    wait_until(send.done, 10.0)
    result = send.result().get_result_async()
    wait_until(result.done, 20.0)
    assert samples, 'a running goal published no feedback at all'
    for sample in samples:
        assert 0.0 <= sample.feedback.position <= units.HALF_WIDTH_MAX_M
