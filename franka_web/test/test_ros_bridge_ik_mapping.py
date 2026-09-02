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
The fourteen-field copy into the IK request message, and its three branches.

This file exists because nothing else can see this code. The endpoint above
it speaks plain dataclasses and never names a message type; the live round
trip below it needs a running node and skips on any machine without one. A
transposed position.y / position.z would therefore ship green everywhere
else -- and would put the hand a hand's width away from where the operator
dragged it, in a direction that looks plausible.
"""

import pytest

pytest.importorskip('franka_ik_interfaces',
                    reason='the IK interfaces are not built in this workspace')

from franka_ik_interfaces.srv import SolveIk       # noqa: E402, I100
from franka_web import ros_bridge                  # noqa: E402
from franka_web.ghost import IkCall                # noqa: E402

CALL = IkCall(
    frame_id='panda1_link0',
    arm_id='panda1',
    tip_frame=0,
    position=(0.11, 0.22, 0.33),
    orientation=(0.44, 0.55, 0.66, 0.77),
    seed_positions=(0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854),
    redundancy_mode=1,
    redundancy_value=0.25,
    max_solutions=1,
    solver=0,
    position_tolerance=0.0,
    orientation_tolerance=0.0,
    joint_limit_margin=0.0)


class FakeClient:
    """Stands in for the service client, recording what it was handed."""

    def __init__(self, response=None):
        """Answer with ``response``, or with nothing at all."""
        self.response = response
        self.requests = []

    def service_is_ready(self):
        """Report the service as reachable."""
        return True

    def call_async(self, request):
        """Record the request and return an already-completed future."""
        self.requests.append(request)
        return _Future(self.response)


class _Future:
    """The two future methods the bounded call uses."""

    def __init__(self, value):
        """Hold the value this future has already resolved to."""
        self.value = value

    def add_done_callback(self, callback):
        """Fire immediately: this future was never pending."""
        callback(self)

    def result(self):
        """Return the resolved value."""
        return self.value

    def cancel(self):
        """Do nothing; nothing here is ever cancelled."""


class Bridge:
    """The bridge's IK methods, lifted off the Node they normally live on."""

    def __init__(self, client):
        """Hold one fake client in the field the real bridge uses."""
        self._solve_ik = client

    _bounded_call = ros_bridge.FrankaWebBridge._bounded_call
    call_solve_ik = ros_bridge.FrankaWebBridge.call_solve_ik
    ik_service_ready = ros_bridge.FrankaWebBridge.ik_service_ready


def response_with(positions=(0.1,) * 7, result=0, message='ok'):
    """Build a real SolveIk response carrying one solution."""
    response = SolveIk.Response()
    response.result.result = result
    response.result.message = message
    solution = type(response.result.solutions)  # the typed sequence's class
    assert solution is not None
    entry = _solution(positions)
    response.result.solutions = [entry]
    return response


def _solution(positions):
    """Build one IkSolution with distinguishable error values."""
    from franka_ik_interfaces.msg import IkSolution
    solution = IkSolution()
    solution.positions = [float(value) for value in positions]
    solution.redundancy_value = 0.25
    solution.position_error = 4.2e-8
    solution.orientation_error = 3.1e-5
    return solution


class TestFieldMapping:
    """Every field, at its own destination, asserted one at a time."""

    def sent(self):
        """Run one call and return the request message it produced."""
        client = FakeClient(response_with())
        Bridge(client).call_solve_ik(CALL, 0.25)
        return client.requests[0].request

    def test_the_identity_fields(self):
        """Naming the arm base explicitly deletes a whole class of frame bug."""
        message = self.sent()
        assert message.frame_id == 'panda1_link0'
        assert message.arm_id == 'panda1'
        assert message.tip_frame == 0

    def test_each_position_component_reaches_its_own_axis(self):
        """
        Asserted one component at a time, on purpose.

        A transposed y and z passes any assertion that compares the three as
        a set or a sum, and puts the hand somewhere plausible but wrong.
        """
        message = self.sent()
        assert message.target_pose.position.x == 0.11
        assert message.target_pose.position.y == 0.22
        assert message.target_pose.position.z == 0.33

    def test_each_orientation_component_reaches_its_own_axis(self):
        """The w component is last in the tuple and last in the message."""
        message = self.sent()
        assert message.target_pose.orientation.x == 0.44
        assert message.target_pose.orientation.y == 0.55
        assert message.target_pose.orientation.z == 0.66
        assert message.target_pose.orientation.w == 0.77

    def test_the_seed_and_the_redundancy_knob(self):
        """The seed is always required, and the value is finite in both modes."""
        message = self.sent()
        assert list(message.seed_positions) == list(CALL.seed_positions)
        assert message.redundancy_mode == 1
        assert message.redundancy_value == 0.25

    def test_the_solver_choices_and_the_three_tolerances(self):
        """Zero selects the node's own defaults; one solution is all v1 gives."""
        message = self.sent()
        assert message.max_solutions == 1
        assert message.solver == 0
        assert message.position_tolerance == 0.0
        assert message.orientation_tolerance == 0.0
        assert message.joint_limit_margin == 0.0


class TestBranches:
    """The three ways this method can answer."""

    def test_a_solution_is_copied_field_by_field(self):
        """The four numbers the endpoint reads, and no others."""
        client = FakeClient(response_with(positions=(0.5,) * 7))
        reply = Bridge(client).call_solve_ik(CALL, 0.25)
        assert reply.result == 0
        assert reply.message == 'ok'
        assert reply.positions == (0.5,) * 7
        assert reply.redundancy_value == 0.25
        assert reply.position_error == 4.2e-8
        assert reply.orientation_error == 3.1e-5

    def test_no_client_answers_none(self):
        """A workspace without the interfaces has no client at all."""
        assert Bridge(None).call_solve_ik(CALL, 0.25) is None
        assert Bridge(None).ik_service_ready() is False

    def test_no_response_answers_none(self):
        """A timed-out call is the same "no answer" the endpoint refuses on."""
        client = FakeClient(None)
        assert Bridge(client).call_solve_ik(CALL, 0.25) is None

    def test_a_declined_result_with_no_solutions_keeps_its_message(self):
        """
        An empty solutions array must not become a null dereference.

        Every declined result arrives this way, so this is the ordinary
        path, not an edge case.
        """
        response = SolveIk.Response()
        response.result.result = 6
        response.result.message = 'unreachable'
        client = FakeClient(response)
        reply = Bridge(client).call_solve_ik(CALL, 0.25)
        assert reply.result == 6
        assert reply.message == 'unreachable'
        assert reply.positions == ()
        assert reply.redundancy_value == 0.0
