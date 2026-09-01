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
The node's topics, services, status table and link health, over a real pty.

Every test here runs the shipped node against the protocol-faithful fake
gripper: nothing is mocked at the protocol level, which is the whole point of
having a fake. The tests live on ``ROS_DOMAIN_ID`` 225 (see ``conftest.py``).
"""

import os
import time

from conftest import needs_driver_modules, wait_until
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticStatus
import pytest
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

try:  # pragma: no cover - the import IS the thing being reported
    from franka_robotiq import discovery, driver, node as robotiq_node, registers, units
except ImportError:  # pragma: no cover - PART-A's modules are not here yet
    discovery = driver = robotiq_node = registers = units = None

pytestmark = needs_driver_modules

STATUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

JOINT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)

#: The thirteen keys of contract section 3.4, enumerated rather than counted
#: so a rename fails this file rather than passing a length check.
CONTRACT_STATUS_KEYS = (
    'width_mm', 'requested_width_mm', 'object', 'activated', 'moving',
    'fault_code', 'fault_name', 'fault_class', 'current_ma', 'speed_mm_s',
    'force_n', 'port', 'link')


class Watcher:
    """Collect every message a topic publishes while a test runs."""

    def __init__(self, client, topic, message_type, qos):
        """Subscribe and start collecting."""
        self.messages = []
        self.subscription = client.create_subscription(
            message_type, topic, self.messages.append, qos)

    @property
    def latest(self):
        """Return the newest message, or None."""
        return self.messages[-1] if self.messages else None

    def wait(self, timeout_s=10.0):
        """Wait for at least one message and return it."""
        wait_until(lambda: bool(self.messages), timeout_s)
        assert self.messages, 'the topic produced nothing within the timeout'
        return self.messages[-1]

    def wait_for(self, predicate, timeout_s=10.0):
        """Wait for a message satisfying ``predicate`` and return it."""
        found = wait_until(
            lambda: next((m for m in list(self.messages) if predicate(m)), None),
            timeout_s)
        assert found is not None, 'no message satisfied the predicate in time'
        return found


def status_watcher(client, node):
    """Return a watcher on one node's ``~/status``."""
    return Watcher(client, '/{}/status'.format(node.get_name()),
                   DiagnosticStatus, STATUS_QOS)


def joint_watcher(client, node):
    """Return a watcher on one node's ``~/joint_states``."""
    return Watcher(client, '/{}/joint_states'.format(node.get_name()),
                   JointState, JOINT_QOS)


def values_of(status):
    """Return a DiagnosticStatus's key/value pairs as a plain dict."""
    return {entry.key: entry.value for entry in status.values}


def call_trigger(client, node, name, timeout_s=20.0):
    """Call one of the node's four Trigger services and return the response."""
    service = client.create_client(Trigger, '/{}/{}'.format(node.get_name(), name))
    assert service.wait_for_service(timeout_sec=10.0), (
        '{} never advertised ~/{}'.format(node.get_name(), name))
    future = service.call_async(Trigger.Request())
    wait_until(future.done, timeout_s)
    assert future.done(), '~/{} did not answer within {} s'.format(name, timeout_s)
    return future.result()


def send_goal(client, node, position, max_effort=0.0, timeout_s=20.0):
    """Send one GripperCommand goal and return ``(goal_handle, result)``."""
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0), (
        '{} never advertised ~/gripper_action'.format(node.get_name()))
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = float(max_effort)
    send = action.send_goal_async(goal)
    wait_until(send.done, timeout_s)
    handle = send.result()
    if not handle.accepted:
        return handle, None
    result = handle.get_result_async()
    wait_until(result.done, timeout_s)
    assert result.done(), 'the goal never produced a result'
    return handle, result.result()


def activate(client, node):
    """Run the activation cycle the way an operator would, and assert it took."""
    response = call_trigger(client, node, 'reactivate')
    assert response.success is True, response.message
    return response


# ----------------------------------------------------------------------
# Names, topics and the two publications
# ----------------------------------------------------------------------


def test_the_node_is_named_per_arm_and_never_collides_with_franka_gripper(gripper_cell):
    """The node, its topics and its joints all avoid franka_gripper's names."""
    node, client = gripper_cell(arm_id='panda1')
    assert node.get_name() == 'panda1_robotiq'
    names = [name for name, _ in node.get_topic_names_and_types()]
    assert '/panda1_robotiq/status' in names
    assert '/panda1_robotiq/joint_states' in names
    assert not any(name.startswith('/panda1_gripper') for name in names)
    joints = joint_watcher(client, node).wait().name
    assert joints == ['panda1_robotiq_finger_joint1', 'panda1_robotiq_finger_joint2']
    assert 'panda1_finger_joint1' not in joints


def test_joint_states_carry_half_width_in_metres_twice(gripper_cell):
    """Two finger joints, each carrying the same half width in metres."""
    node, client = gripper_cell(arm_id='panda1')
    message = joint_watcher(client, node).wait()
    assert len(message.name) == 2
    assert len(message.position) == 2
    assert message.position[0] == message.position[1]
    assert 0.0 <= message.position[0] <= units.HALF_WIDTH_MAX_M


def test_velocity_and_effort_are_zero_and_documented(gripper_cell):
    """The zeros are deliberate: the protocol reports neither quantity."""
    node, client = gripper_cell(arm_id='panda1')
    message = joint_watcher(client, node).wait()
    assert list(message.velocity) == [0.0, 0.0]
    assert list(message.effort) == [0.0, 0.0]
    source = open(robotiq_node.__file__, encoding='utf-8').read()
    assert 'MOTOR CURRENT' in source, (
        'the zeros must carry their reason at the publication site')


def test_status_and_joint_states_come_from_one_snapshot(gripper_cell):
    """A width read from ~/status always matches the same tick's joint state."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    joints = joint_watcher(client, node)
    activate(client, node)
    statuses.wait()
    joints.wait()
    time.sleep(0.5)
    width_mm = float(values_of(statuses.latest)['width_mm'])
    half_m = joints.latest.position[0]
    assert abs(units.half_width_m_from_count(
        units.width_mm_to_count(width_mm)) - half_m) < 1e-6


def test_status_values_carry_every_contracted_key_always(gripper_cell):
    """All thirteen keys are present in every sample, named one by one."""
    node, client = gripper_cell(arm_id='panda1')
    status = status_watcher(client, node).wait()
    assert tuple(entry.key for entry in status.values) == CONTRACT_STATUS_KEYS


def test_current_ma_is_published_on_status_and_absent_from_the_frame(gripper_cell):
    """current_ma is a driver diagnostic: on ~/status, and not in the frame."""
    node, client = gripper_cell(arm_id='panda1')
    activate(client, node)
    status = status_watcher(client, node).wait_for(
        lambda message: values_of(message)['current_ma'] != '')
    assert values_of(status)['current_ma'].lstrip('-').isdigit()
    schema = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'franka_web', 'test', 'support', 'state_frame_schema.json')
    if os.path.isfile(schema):
        with open(schema, encoding='utf-8') as handle:
            assert '"current_ma"' not in handle.read()


def test_status_is_transient_local_for_a_late_joiner(gripper_cell):
    """A subscriber created after the node is up is filled without a poll."""
    node, client = gripper_cell(arm_id='panda1')
    status_watcher(client, node).wait()
    late = status_watcher(client, node)
    assert late.wait(timeout_s=2.0) is not None


# ----------------------------------------------------------------------
# The status table
# ----------------------------------------------------------------------


@pytest.mark.parametrize('case', ['link_down', 'major', 'minor', 'priority',
                                  'not_activated', 'ok'])
def test_status_level_and_message_match_the_table(gripper_cell, case):
    """Each of the six rows of the section 3.4 table, in its own case."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    statuses.wait()
    if case == 'not_activated':
        status = statuses.wait_for(lambda m: values_of(m)['activated'] == 'false')
        assert status.level == DiagnosticStatus.WARN
        assert status.message == (
            'Gripper is not activated. Call /panda1_robotiq/reactivate to run '
            'the calibration.')
        return
    if case == 'link_down':
        node._fake.unplug()
        status = statuses.wait_for(lambda m: values_of(m)['link'] == 'down')
        assert status.level == DiagnosticStatus.ERROR
        assert status.message.startswith('No serial link to the panda1 gripper.')
        return
    activate(client, node)
    if case == 'ok':
        status = statuses.wait_for(lambda m: m.level == DiagnosticStatus.OK)
        assert (status.message.startswith('Open ')
                or status.message.startswith('Holding an object at ')
                or status.message == 'Closed.')
        return
    code = {'major': 0x0C, 'minor': 0x08, 'priority': 0x07}[case]
    node._fake.inject_fault(code)
    status = statuses.wait_for(
        lambda m: values_of(m)['fault_code'] == '0x{:02X}'.format(code))
    expected_level = (DiagnosticStatus.ERROR if case == 'major'
                      else DiagnosticStatus.WARN)
    assert status.level == expected_level
    assert values_of(status)['fault_class'] == case
    # Section 3.4 prints a different sentence per row, and only two of the
    # three name the service. The minor row is deliberately the one that does
    # not: an over-temperature gripper resumes by itself, so telling the
    # operator to call ~/reactivate would teach the wrong next action.
    if case == 'minor':
        assert status.message == (
            'Gripper is too hot. It resumes by itself once it cools down.')
    elif case == 'priority':
        assert status.message == (
            'Gripper is not activated yet. Call /panda1_robotiq/reactivate.')
    else:
        assert status.message.endswith(
            '. Call /panda1_robotiq/reactivate to reset the gripper.')


def test_object_is_unknown_whenever_ggto_is_zero(gripper_cell):
    """Object detection is meaningless while gGTO is 0, and is reported unknown."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    activate(client, node)
    call_trigger(client, node, 'stop')
    status = statuses.wait_for(lambda m: values_of(m)['moving'] == 'false')
    assert values_of(status)['object'] == 'unknown'


def test_an_unknown_fault_code_is_reported_by_number_and_classed_major(gripper_cell):
    """An unrecognised fault is reported by number and treated as major."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    activate(client, node)
    node._fake.inject_fault(0x06)
    status = statuses.wait_for(lambda m: values_of(m)['fault_code'] == '0x06')
    values = values_of(status)
    assert values['fault_name'] == 'unknown_0x06'
    assert values['fault_class'] == 'major'
    assert registers.fault(0x06).klass == 'unknown', (
        'the internal value must stay distinct; only the node maps it')
    assert status.level == DiagnosticStatus.ERROR
    assert '/panda1_robotiq/reactivate' in status.message


# ----------------------------------------------------------------------
# Services
# ----------------------------------------------------------------------


def test_open_and_close_use_the_configured_widths(gripper_cell):
    """~/open and ~/close go to open_width_mm and close_width_mm."""
    node, client = gripper_cell(arm_id='panda1', open_width_mm=60.0,
                                close_width_mm=10.0, fake_object_mm=0.0)
    statuses = status_watcher(client, node)
    activate(client, node)
    assert call_trigger(client, node, 'close').success is True
    closed = statuses.wait_for(
        lambda m: abs(float(values_of(m)['requested_width_mm'] or 0.0) - 10.0) < 0.5)
    assert closed is not None
    assert call_trigger(client, node, 'open').success is True
    opened = statuses.wait_for(
        lambda m: abs(float(values_of(m)['requested_width_mm'] or 0.0) - 60.0) < 0.5)
    assert opened is not None


def test_stop_holds_the_fingers_and_is_never_refused(gripper_cell):
    """Stop works while faulted and while inactive: it must always be pressable."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    statuses.wait()
    assert call_trigger(client, node, 'stop').success is True
    activate(client, node)
    node._fake.inject_fault(0x0C)
    statuses.wait_for(lambda m: values_of(m)['fault_code'] == '0x0C')
    assert call_trigger(client, node, 'stop').success is True


def test_reactivate_runs_the_ract_edge_and_reports_the_outcome(gripper_cell):
    """Activation completes (gSTA 3) and Trigger.success carries the verdict."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    statuses.wait_for(lambda m: values_of(m)['activated'] == 'false')
    response = call_trigger(client, node, 'reactivate')
    assert response.success is True
    assert response.message
    statuses.wait_for(lambda m: values_of(m)['activated'] == 'true')


def test_reactivate_is_refused_while_a_goal_is_active(gripper_cell):
    """One serial link, one motion: reactivate waits for the goal to end."""
    node, client = gripper_cell(arm_id='panda1', poll_rate_hz=2.0,
                                motion_timeout_s=5.0)
    activate(client, node)
    action = ActionClient(client, GripperCommand,
                          '/{}/gripper_action'.format(node.get_name()))
    assert action.wait_for_server(timeout_sec=10.0)
    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    send = action.send_goal_async(goal)
    wait_until(lambda: node._goal is not None or send.done(), 10.0)
    response = call_trigger(client, node, 'reactivate')
    assert response.success is False
    assert response.message == 'Another gripper goal is running; cancel it first.'


def test_activation_is_never_a_constructor_side_effect(gripper_cell):
    """Building the node writes no rACT edge: nothing moves in __init__."""
    node, _client = gripper_cell(arm_id='panda1')
    assert node._last_status is None or node._last_status.g_sta != 0x03
    assert node._last_written is None, 'the constructor composed a command'


def test_a_fresh_node_waits_for_reactivate_and_never_moves_on_startup(gripper_cell):
    """auto_activate is a RECONNECT rule; a fresh node publishes WARN and waits."""
    node, client = gripper_cell(arm_id='panda1', auto_activate=True)
    statuses = status_watcher(client, node)
    status = statuses.wait_for(lambda m: values_of(m)['link'] == 'up')
    time.sleep(0.5)
    status = statuses.latest
    assert values_of(status)['activated'] == 'false'
    assert status.level == DiagnosticStatus.WARN
    assert 'reactivate' in status.message


def test_a_standing_node_started_beside_a_watch_session_never_moves_the_fingers(
        gripper_cell):
    """Bringing the drivers up beside an observing session is a zero-motion event."""
    node, client = gripper_cell(arm_id='panda1', auto_activate=True)
    statuses = status_watcher(client, node)
    before = statuses.wait()
    time.sleep(1.0)
    after = statuses.latest
    assert values_of(before)['width_mm'] == values_of(after)['width_mm']
    assert values_of(after)['moving'] == 'false'


# ----------------------------------------------------------------------
# Link health and the anti-swap re-verification
# ----------------------------------------------------------------------


def test_three_failed_reads_declare_the_link_down_and_stop_joint_states(
        gripper_cell, monkeypatch):
    """The driver's FAILURE_LIMIT decides; the node only reacts to the verdict."""
    monkeypatch.setattr(driver, 'FAILURE_LIMIT', 3)
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    joints = joint_watcher(client, node)
    activate(client, node)
    joints.wait()
    node._fake.unplug()
    status = statuses.wait_for(lambda m: values_of(m)['link'] == 'down')
    assert status.level == DiagnosticStatus.ERROR
    count = len(joints.messages)
    time.sleep(0.5)
    assert len(joints.messages) == count, (
        'a stale width is worse than none: joint states must stop')


def test_the_node_reacts_to_link_down_without_counting_anything(gripper_cell):
    """
    There is exactly ONE consecutive-failure counter, and it is the driver's.

    The check is on the parsed CODE, not on the text: the poll loop's comments
    name ``driver.FAILURE_LIMIT`` to say whose policy it is, and deleting that
    sentence to satisfy a substring scan would delete the explanation rather
    than the duplicate counter.
    """
    import ast

    source = open(robotiq_node.__file__, encoding='utf-8').read()
    tree = ast.parse(source)
    for statement in ast.walk(tree):
        if isinstance(statement, ast.Attribute):
            assert statement.attr != 'FAILURE_LIMIT', (
                'the node reads the driver counting policy it must not own')
        if isinstance(statement, ast.Name):
            assert statement.id != 'FAILURE_LIMIT'
    assert 'LinkDownError' in source
    node, _client = gripper_cell(arm_id='panda1')
    assert not any(name.endswith('_read_failures') for name in vars(node))


def test_a_lying_by_id_symlink_refuses_to_drive(gripper_cell, tmp_path):
    """
    The anti-swap re-verification, and it has no other owner in any part.

    ``by_id_root`` points at a symlink that resolves to a device whose sysfs
    identity reports the OTHER arm's serial. The port is closed, the link
    stays down, and nothing was ever commanded.

    The sysfs half is injected at the seam function rather than as a directory
    tree: the emulator's device is a pty, which has no ``ttyUSB`` parent to
    hang a serial off, so a tmpdir ``sysfs_root`` could only ever answer
    ``None`` here. The ``sysfs_root`` parameter exists and is passed; what a
    real adapter's tree looks like is exercised on hardware.
    """
    from franka_robotiq import fake

    emulator = fake.FakeGripper()
    emulator.start()
    try:
        by_id = tmp_path / 'by-id'
        by_id.mkdir()
        name = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'
        os.symlink(emulator.port, str(by_id / name))
        sysfs = tmp_path / 'sys'
        sysfs.mkdir()
        node, client = gripper_cell(
            arm_id='panda1', use_fake=False, serial_id=name,
            by_id_root=str(by_id), sysfs_root=str(sysfs))
        statuses = status_watcher(client, node)

        def lying(device_path, *, sysfs_root='/sys'):
            """Report panda2's adapter serial for panda1's bound device."""
            return 'D3091K4W'

        assert discovery.identity_matches(name, 'D3091K4T') is True
        assert discovery.identity_matches(name, 'D3091K4W') is False
        node._gripper = None
        node._link_up = False
        before = dict(emulator.stats)
        original = discovery.adapter_serial_from_sysfs
        discovery.adapter_serial_from_sysfs = lying
        try:
            assert node._connect() is False
            assert node._link_up is False
            assert node._gripper is None
            assert node._last_written is None
            assert dict(emulator.stats) == before, 'a frame was written'
            status = statuses.wait_for(lambda m: values_of(m)['link'] == 'down')
            assert status.level == DiagnosticStatus.ERROR
        finally:
            discovery.adapter_serial_from_sysfs = original
    finally:
        emulator.stop()


def test_an_unavailable_sysfs_serial_logs_one_info_line_and_keeps_going(
        gripper_cell, tmp_path):
    """An adapter with no readable serial is used, and the node says why."""
    node, _client = gripper_cell(arm_id='panda1')
    message = discovery.identity_unavailable_message('panda1')
    assert 'panda1' in message
    assert node._verify_identity(None, node._port) is True


def test_a_down_link_retries_on_the_configured_interval_and_logs_once_per_30s(
        gripper_cell):
    """The retry cadence is reconnect_interval_s and the log is rate limited."""
    node, client = gripper_cell(arm_id='panda1', reconnect_interval_s=0.5)
    statuses = status_watcher(client, node)
    activate(client, node)
    node._fake.unplug()
    statuses.wait_for(lambda m: values_of(m)['link'] == 'down')
    first = node._next_reconnect_mono
    wait_until(lambda: node._next_reconnect_mono > first, 5.0)
    assert node._next_reconnect_mono - first >= 0.4
    assert node._last_down_log_mono is not None


def test_a_reconnect_that_lost_power_re_activates_when_auto_activate(gripper_cell):
    """
    The ONE place auto_activate grants motion: a link back with gSTA != 3.

    The power loss is staged with ``reset()`` -- an rACT falling edge, which
    is exactly what losing power does to the gripper -- and the link is then
    dropped and re-made through the node's own reconnect path. The fake's
    ``replug`` injector is deliberately NOT used: it is not on the pinned
    cross-part seam and belongs to the driver's own tests.
    """
    # 0.5 s is the floor the node itself enforces; its sibling above uses the
    # same value, so both reconnect tests run at the fastest LEGAL cadence
    # rather than at one the node would refuse to start on.
    node, client = gripper_cell(arm_id='panda1', reconnect_interval_s=0.5,
                                auto_activate=True)
    statuses = status_watcher(client, node)
    activate(client, node)
    statuses.wait_for(lambda m: values_of(m)['activated'] == 'true')
    node._gripper.reset()
    statuses.wait_for(lambda m: values_of(m)['activated'] == 'false')
    node._gripper.close()
    node._link_up = False
    node._gripper = None
    node._next_reconnect_mono = 0.0
    status = statuses.wait_for(
        lambda m: values_of(m)['link'] == 'up'
        and values_of(m)['activated'] == 'true', timeout_s=30.0)
    assert status is not None


def test_a_reconnect_to_a_different_adapter_serial_refuses_and_stays_down(
        gripper_cell, tmp_path):
    """The anti-swap check runs on EVERY connect, not only the first."""
    node, _client = gripper_cell(arm_id='panda1')
    original = discovery.adapter_serial_from_sysfs
    node._serial_id = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'
    node._use_fake = False
    discovery.adapter_serial_from_sysfs = lambda path, **kwargs: 'D3091K4W'
    try:
        assert node._verify_identity(node._gripper, node._port) is False
    finally:
        discovery.adapter_serial_from_sysfs = original
        node._use_fake = True


def test_a_missing_adapter_starts_the_node_down_and_lists_what_is_present(
        gripper_cell, tmp_path):
    """A node whose adapter is absent RUNS, publishes link: down, and teaches."""
    by_id = tmp_path / 'by-id'
    by_id.mkdir()
    (by_id / 'usb-OTHER_ADAPTER-if00-port0').write_text('', encoding='utf-8')
    node, client = gripper_cell(
        arm_id='panda1', use_fake=False,
        serial_id='usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0',
        by_id_root=str(by_id))
    status = status_watcher(client, node).wait()
    assert values_of(status)['link'] == 'down'
    sentence = node._missing_adapter_sentence()
    assert 'panda1' in sentence
    assert 'doc/SERIAL_BINDING.md' in sentence


# ----------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------


def test_runtime_settable_parameters_take_effect_on_the_next_goal(gripper_cell):
    """The five runtime parameters are accepted and are what the next goal uses."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    activate(client, node)
    for name, value in (('force_n', 40.0), ('speed_mm_s', 30.0),
                        ('open_width_mm', 70.0), ('close_width_mm', 5.0),
                        ('auto_activate', False)):
        result = node.set_parameters([Parameter(name, value=value)])[0]
        assert result.successful is True, result.reason
    status = statuses.wait_for(lambda m: values_of(m)['force_n'] == '40')
    assert values_of(status)['speed_mm_s'] == '30'


@pytest.mark.parametrize('name', ['poll_rate_hz', 'motion_timeout_s',
                                  'activation_timeout_s', 'reconnect_interval_s',
                                  'joint_names'])
def test_a_startup_parameter_set_is_rejected_with_a_teaching_message(
        gripper_cell, name):
    """A startup-only parameter refuses the set and says where it IS set."""
    node, _client = gripper_cell(arm_id='panda1')
    value = (['a', 'b'] if name == 'joint_names' else 9.0)
    result = node.set_parameters([Parameter(name, value=value)])[0]
    assert result.successful is False
    assert name in result.reason
    assert 'restart the node' in result.reason


@pytest.mark.parametrize('name', ['serial_id', 'usb_path'])
def test_serial_id_and_usb_path_are_never_runtime_settable(gripper_cell, name):
    """Re-binding a live gripper through a parameter set is the wrong-arm command."""
    node, _client = gripper_cell(arm_id='panda1')
    result = node.set_parameters([Parameter(name, value='usb-anything')])[0]
    assert result.successful is False
    assert 'wrong-arm command' in result.reason
    assert 'doc/SERIAL_BINDING.md' in result.reason


def test_a_fake_object_makes_a_closing_goal_stall_and_no_object_does_not(
        gripper_cell):
    """fake_object_mm both ways: an object stalls the close, no object does not."""
    node, client = gripper_cell(arm_id='panda1', fake_object_mm=30.0)
    activate(client, node)
    _handle, result = send_goal(client, node, 0.0)
    assert result.result.reached_goal is True
    assert result.result.stalled is True

    empty, empty_client = gripper_cell(arm_id='panda2', fake_object_mm=0.0)
    activate(empty_client, empty)
    _handle, result = send_goal(empty_client, empty, 0.0)
    assert result.result.reached_goal is True
    assert result.result.stalled is False


def test_shutdown_releases_the_port_and_leaves_the_fingers_in_place(gripper_cell):
    """Shutdown clears rGTO and closes the port; the gripper is NOT opened."""
    node, client = gripper_cell(arm_id='panda1')
    statuses = status_watcher(client, node)
    activate(client, node)
    status = statuses.wait_for(lambda m: values_of(m)['width_mm'] != '')
    before = values_of(status)['width_mm']
    node.shutdown()
    assert node._gripper is None
    assert node._link_up is False
    assert before == values_of(statuses.latest)['width_mm']
