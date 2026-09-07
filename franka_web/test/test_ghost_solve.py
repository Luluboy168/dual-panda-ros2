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
The ghost solve surface: refusals, sentences, the Copy text, and no motion.

Everything here runs with no ROS and no robot: the solver is a callable and
the checker is an object, which is the whole reason those two are injected.
The last class in this file is the one that matters most -- it proves the
ghost cannot command anything, in four independent ways.
"""

import ast
import importlib.util
import json
import math
import os
import re
import threading
import time

from franka_web import defaults, ghost, ghost_copy, http_api, workspace
from franka_web.ghost import GhostError, GhostService, IkReply, TokenBucket
import pytest
from support import fake_checker, sample_cell
from support.fake_checker import (
    checker_holding, collision, Contact, FakeCellModel, FakeChecker)
from support.fake_clock import FakeClock
from support.ghost_server import GhostServer
from support.stub_ik import failure, StubSolver

READY_POSE = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
TARGET = {'position': [0.42, 0.51, 0.38], 'orientation': [0.0, 1.0, 0.0, 0.0]}

#: The eight ways the solver can decline, and the sentence each produces.
FAILURE_SENTENCES = {
    5: 'The ghost is in a pose the solver will not start from. Press Reset ghost.',
    6: "That point is outside this arm's reach.",
    7: 'Reaching that point would push a joint past its limit.',
    8: 'The solver could not settle on that point. Try moving in a smaller step.',
    9: 'The solver could not settle on that point. Try moving in a smaller step.',
    10: 'No arm posture reaches that point from here. Try moving in a '
        'smaller step.',
    1: 'The IK service refused this request.',
    11: 'The IK service refused this request.',
}

#: The five results that mean a server bug or a mismatched description.
REFUSALS = (1, 2, 3, 4, 11)


def body(**overrides):
    """Return a valid solve request body, with overrides applied."""
    request = {'arm_id': 'panda1', 'seed': list(READY_POSE),
               'target': dict(TARGET), 'redundancy': {'mode': 'from_seed'}}
    request.update(overrides)
    return request


def service(solver=None, checker=None, arm_ids=(), topics=None, ready=True,
            clock=None, log_bus=None):
    """Return a GhostService over doubles, with an injectable clock."""
    view = {'arm_ids': list(arm_ids), 'command_topics': dict(topics or {})}
    return GhostService(
        solver=solver or StubSolver(),
        checker=checker if checker is not None else FakeChecker(),
        session_view=lambda: view,
        ik_ready=lambda: ready,
        monotonic=clock.monotonic if clock is not None else time.monotonic,
        log_bus=log_bus)


class Bus:
    """A log bus that records, so a warn line can be asserted."""

    def __init__(self):
        """Start with nothing recorded."""
        self.lines = []

    def emit(self, level, message):
        """Record one line."""
        self.lines.append((level, message))


class TestSolve:
    """The endpoint's answers: successes, declines, and refusals."""

    def test_a_successful_solve_reports_every_field(self):
        """The whole payload, including the two error floats."""
        solver = StubSolver()
        result = service(solver=solver).solve(body())
        assert result['solved'] is True
        assert result['arm_id'] == 'panda1'
        assert result['positions'] == pytest.approx(READY_POSE)
        assert result['positions_deg'] == [
            round(math.degrees(value), 2) for value in READY_POSE]
        assert result['solve_reason'] is None
        assert result['position_error'] == 0.0
        assert result['orientation_error'] == 0.0
        assert result['redundancy_value'] == pytest.approx(READY_POSE[6])
        assert set(result['copy']) == {'joints_deg', 'joints_rad', 'snippet'}

    @pytest.mark.parametrize('code', sorted(FAILURE_SENTENCES))
    def test_each_decline_is_a_200_with_its_own_sentence(self, code):
        """
        An unreachable pose is ordinary interaction, not a request error.

        A refusal envelope would flash a scary bar on every drag past the
        workspace edge.
        """
        solver = StubSolver().script(failure(code))
        result = service(solver=solver).solve(body())
        assert result['solved'] is False
        assert result['solve_reason'] == FAILURE_SENTENCES[code]
        assert result['positions'] is None
        assert result['positions_deg'] is None
        assert result['copy'] is None
        # There is no new pose to judge, so there is no verdict at all.
        assert result['verdict'] is None

    @pytest.mark.parametrize('code', REFUSALS)
    def test_a_refusal_puts_the_services_own_words_on_the_log_bus(self, code):
        """The operator gets a sentence; the drawer gets the detail."""
        bus = Bus()
        solver = StubSolver().script(failure(code, 'unknown arm panda9'))
        result = service(solver=solver, log_bus=bus).solve(body())
        assert result['solve_reason'] == 'The IK service refused this request.'
        assert ('warn', 'unknown arm panda9') in bus.lines

    def test_success_with_no_solution_is_treated_as_a_refusal(self):
        """A service bug must not become a null dereference in the happy path."""
        bus = Bus()
        solver = StubSolver().script(IkReply(result=0, message='', positions=()))
        result = service(solver=solver, log_bus=bus).solve(body())
        assert result['solved'] is False
        assert result['solve_reason'] == 'The IK service refused this request.'
        assert bus.lines and bus.lines[0][0] == 'warn'

    def test_an_unknown_arm_names_the_two_that_exist(self):
        """The page renders the detail verbatim, so it teaches."""
        with pytest.raises(GhostError) as raised:
            service().solve(body(arm_id='panda9'))
        assert raised.value.code == 'invalid_arms'
        assert raised.value.detail == 'arm_id must be one of: panda1, panda2'

    @pytest.mark.parametrize('overrides', [
        {'seed': [0.0] * 6},
        {'seed': [0.0] * 6 + [float('nan')]},
        {'seed': [0.0] * 6 + [True]},
        {'target': {'position': [0.1, 0.2], 'orientation': [0, 0, 0, 1]}},
        {'target': {'position': [0.1, 0.2, 0.3], 'orientation': [0, 0, 0, 0]}},
        {'target': {'position': [0.1, 0.2, 0.3],
                    'orientation': [0, 0, 0, float('inf')]}},
        {'target': None},
        {'redundancy': {'mode': 'sideways'}},
        {'redundancy': {'mode': 'fixed'}},
        {'redundancy': {'mode': 'fixed', 'value': float('nan')}},
        {'scene': {'panda1': [0.0] * 6}},
        {'scene': 7},
    ])
    def test_every_malformed_field_is_a_400_that_names_it(self, overrides):
        """A field named in the detail is a field the operator can fix."""
        with pytest.raises(GhostError) as raised:
            service().solve(body(**overrides))
        assert raised.value.code == 'invalid_json'
        assert raised.value.detail

    def test_a_body_that_is_not_an_object_is_refused(self):
        """The transport allows only an object; the service says so too."""
        with pytest.raises(GhostError) as raised:
            service().solve([1, 2, 3])
        assert raised.value.code == 'invalid_json'

    def test_a_null_arm_in_scene_degrades_rather_than_refusing(self):
        """
        A drag must never 400 because one arm's pose was momentarily unknown.

        Clients omit the key rather than send null; the server accepts both
        so a client bug costs a verdict, not the whole gesture.
        """
        checker = FakeChecker(
            triple=(workspace.NOTE_SCENE_INCOMPLETE, 'other_arm_pose_unknown'))
        result = service(checker=checker).solve(
            body(scene={'panda1': list(READY_POSE), 'panda2': None}))
        assert result['solved'] is True
        assert result['verdict']['status'] == 'unchecked'
        assert result['verdict']['reason_code'] == 'other_arm_pose_unknown'
        assert checker.seen[0] == {'panda1': tuple(READY_POSE)}

    def test_an_unknown_arm_inside_scene_is_ignored(self):
        """The checker is asked about the arms it declares, and no others."""
        checker = FakeChecker()
        service(checker=checker).solve(
            body(scene={'panda1': list(READY_POSE), 'panda9': [0.0] * 7}))
        assert set(checker.seen[0]) == {'panda1'}

    def test_the_ik_service_being_absent_is_a_503_that_teaches(self):
        """One command fixes this state, and the detail is that command."""
        solver = StubSolver().script(None)
        with pytest.raises(GhostError) as raised:
            service(solver=solver, ready=False).solve(body())
        assert raised.value.code == 'ghost_unavailable'
        assert raised.value.payload == {'ik_state': 'not_ready'}
        assert 'ros2 launch franka_ik franka_ik.launch.py' in raised.value.detail

    def test_a_silent_ik_service_is_distinguished_from_an_absent_one(self):
        """The panel teaches a different sentence for each, so the code must too."""
        solver = StubSolver().script(None)
        with pytest.raises(GhostError) as raised:
            service(solver=solver, ready=True).solve(body())
        assert raised.value.code == 'ghost_unavailable'
        assert raised.value.payload == {'ik_state': 'timeout'}

    def test_the_field_mapping_is_exactly_the_service_contract(self):
        """Every value the solver is handed, asserted where it is decided."""
        solver = StubSolver()
        service(solver=solver).solve(body())
        call = solver.calls[0]
        assert call.frame_id == 'panda1_link0'
        assert call.arm_id == 'panda1'
        assert call.tip_frame == 0
        assert call.max_solutions == 1
        assert call.solver == 0
        assert call.position_tolerance == 0.0
        assert call.orientation_tolerance == 0.0
        assert call.joint_limit_margin == 0.0
        assert call.redundancy_mode == 0
        # Finite even in from-seed mode: the service refuses a non-finite
        # value before it looks at the mode.
        assert math.isfinite(call.redundancy_value)
        assert call.position == tuple(TARGET['position'])
        assert call.seed_positions == tuple(READY_POSE)
        assert solver.timeouts[0] == defaults.GHOST_SOLVE_TIMEOUT_S

    def test_a_fixed_redundancy_value_reaches_the_solver(self):
        """The elbow ring's reconciling solve pins joint 7 explicitly."""
        solver = StubSolver()
        service(solver=solver).solve(
            body(redundancy={'mode': 'fixed', 'value': 0.25}))
        assert solver.calls[0].redundancy_mode == 1
        assert solver.calls[0].redundancy_value == 0.25

    def test_the_orientation_is_renormalised_defensively(self):
        """A client's float drift must not become a solver refusal."""
        solver = StubSolver()
        service(solver=solver).solve(
            body(target={'position': [0.1, 0.2, 0.3],
                         'orientation': [0.0, 2.0, 0.0, 0.0]}))
        assert solver.calls[0].orientation == (0.0, 1.0, 0.0, 0.0)


class TestRateLimit:
    """The one global bucket, driven on a fake clock rather than by sleeping."""

    def test_a_bucket_refills_at_its_stated_rate(self):
        """Sixty tokens, thirty a second, and no wall-clock luck anywhere."""
        clock = FakeClock()
        bucket = TokenBucket(defaults.GHOST_RATE_CAPACITY,
                             defaults.GHOST_RATE_REFILL_HZ,
                             monotonic=clock.monotonic)
        for _ in range(defaults.GHOST_RATE_CAPACITY):
            assert bucket.take(1) is None
        wait = bucket.take(1)
        assert isinstance(wait, int) and wait >= 1
        clock.advance(wait / 1000.0)
        assert bucket.take(1) is None

    def test_the_wait_is_computed_for_the_refused_requests_own_cost(self):
        """A refused sweep waits about twenty-five times a refused solve."""
        clock = FakeClock()
        bucket = TokenBucket(60, 30.0, monotonic=clock.monotonic)
        for _ in range(60):
            bucket.take(1)
        assert bucket.take(25) > 20 * bucket.take(1)

    def test_the_sixty_first_solve_in_one_window_is_refused(self):
        """The backstop fires where the client's own discipline did not."""
        clock = FakeClock()
        ghost_service = service(clock=clock)
        for _ in range(defaults.GHOST_RATE_CAPACITY):
            ghost_service.solve(body())
        with pytest.raises(GhostError) as raised:
            ghost_service.solve(body())
        assert raised.value.code == 'ghost_rate_limited'
        assert raised.value.detail == 'too many ghost requests; slow the drag'
        wait = raised.value.payload['retry_after_ms']
        assert isinstance(wait, int) and wait >= 1
        clock.advance(wait / 1000.0)
        assert ghost_service.solve(body())['solved'] is True

    def test_a_sweep_costs_one_token_per_sample(self):
        """It is literally that many solves, so it is charged as such."""
        clock = FakeClock()
        ghost_service = service(clock=clock)
        ghost_service.redundancy(body(samples=33))
        ghost_service.redundancy(body(samples=27))
        with pytest.raises(GhostError) as raised:
            ghost_service.redundancy(body(samples=33))
        assert raised.value.code == 'ghost_rate_limited'

    def test_validation_happens_before_the_charge(self):
        """A malformed request gets the 400 that names its field, not a 429."""
        clock = FakeClock()
        ghost_service = service(clock=clock)
        for _ in range(defaults.GHOST_RATE_CAPACITY):
            ghost_service.solve(body())
        with pytest.raises(GhostError) as raised:
            ghost_service.solve(body(arm_id='panda9'))
        assert raised.value.code == 'invalid_arms'


class TestCopyGolden:
    """The pasted text, byte for byte, in its three shapes."""

    POSITIONS = [0.011702, -0.443234, 0.008976, -2.187531, 0.004312,
                 1.746233, 0.785398]
    TOPIC = '/dual_arm_joint_impedance_controller/arm_1/joint_target'

    HEAD = ('# Ghost pose for panda1, authored in the Franka console.\n'
            '# Degrees: [0.67, -25.40, 0.51, -125.34, 0.25, 100.05, 45.00]\n')
    BODY = (
        '# trajectory_msgs/msg/JointTrajectory — publish at 10 Hz or more\n'
        '# topic: {topic}\n'
        '# Stamp each message with the time you send it: a stamp of zero, one\n'
        '# older than a second, or one more than 0.1 s ahead is refused, and\n'
        '# frame_id stays empty.\n'
        'header:\n'
        '  stamp: now\n'
        'joint_names: [panda1_joint1, panda1_joint2, panda1_joint3, panda1_joint4,\n'
        '              panda1_joint5, panda1_joint6, panda1_joint7]\n'
        'points:\n'
        '- positions: [0.011702, -0.443234, 0.008976, -2.187531, 0.004312, '
        '1.746233, 0.785398]\n'
        '  time_from_start: {{sec: 0, nanosec: 0}}')

    def test_a_motion_session_names_the_arms_own_topic(self):
        """The shape mirrors the External panel's, so it is recognised at once."""
        payload = ghost_copy.build_copy(
            'panda1', self.POSITIONS, {'status': 'clear'}, self.TOPIC)
        assert payload['snippet'] == self.HEAD + self.BODY.format(topic=self.TOPIC)
        assert payload['joints_rad'] == self.POSITIONS
        assert payload['joints_deg'] == [0.67, -25.4, 0.51, -125.34, 0.25,
                                         100.05, 45.0]

    def test_without_a_session_the_topic_line_says_so(self):
        """The slot depends on the arm selection, so it is genuinely unknown."""
        payload = ghost_copy.build_copy(
            'panda1', self.POSITIONS, {'status': 'clear'}, None)
        assert payload['snippet'] == self.HEAD + self.BODY.format(
            topic="set by your session's arm selection — see the External "
                  'panel when a Motion session runs')

    def test_an_unchecked_pose_carries_its_warning_into_the_paste(self):
        """That is where the claim will later be believed."""
        payload = ghost_copy.build_copy(
            'panda1', self.POSITIONS, {'status': 'unchecked'}, self.TOPIC)
        expected = (
            self.HEAD
            + '# NOT collision-checked: the workspace model is not loaded.\n'
            + self.BODY.format(topic=self.TOPIC))
        assert payload['snippet'] == expected

    def test_a_checked_pose_carries_no_warning(self):
        """The line appears in exactly one case and no other."""
        for status in ('clear', 'collision'):
            payload = ghost_copy.build_copy(
                'panda1', self.POSITIONS, {'status': status}, self.TOPIC)
            assert 'NOT collision-checked' not in payload['snippet']

    def test_the_snippet_never_offers_a_one_shot_command_line(self):
        """A pasteable one-shot publish is a motion path with a prompt on it."""
        for verdict in ({'status': 'clear'}, {'status': 'unchecked'}):
            payload = ghost_copy.build_copy(
                'panda1', self.POSITIONS, verdict, self.TOPIC)
            assert 'topic pub' not in json.dumps(payload)

    def test_the_snippet_ends_without_a_trailing_newline(self):
        """Pasted into a file among other lines, a stray blank line is noise."""
        payload = ghost_copy.build_copy(
            'panda1', self.POSITIONS, {'status': 'clear'}, self.TOPIC)
        assert not payload['snippet'].endswith('\n')

    def test_the_snippet_teaches_the_rule_that_makes_it_work(self):
        """
        A stamped message is accepted and an unstamped one is ignored.

        The controller refuses a target whose header stamp is zero, older
        than a second, or more than 0.1 s in the future. A snippet that
        produced silently-ignored commands would be worse than none.
        """
        snippet = ghost_copy.build_copy(
            'panda1', self.POSITIONS, {'status': 'clear'}, self.TOPIC)['snippet']
        assert 'stamp: now' in snippet
        assert 'a stamp of zero' in snippet
        assert 'frame_id stays empty' in snippet

    def test_the_solve_response_carries_the_session_topic(self):
        """The topic comes from the state frame, through a read-only peek."""
        result = service(topics={'panda1': self.TOPIC}).solve(body())
        assert self.TOPIC in result['copy']['snippet']

    def test_the_joint_names_agree_with_the_canonical_spelling(self):
        """
        health.py owns the spelling; this module holds a second copy.

        The copy exists so that importing the ghost cannot drag in rclpy.
        This case is what stops the two drifting.
        """
        from franka_web import health
        for arm_id in defaults.ARM_IDS:
            assert (ghost_copy.joint_names_for(arm_id)
                    == tuple(health.joint_names_for(arm_id)))
            assert ghost.joint_names_for(arm_id) == health.joint_names_for(arm_id)


def contact(kind, a, b, distance=-0.012, required=0.0, arm_id='panda1'):
    """Return one contact of the given kind."""
    return Contact(kind=kind, a=a, b=b, distance=distance, required=required,
                   arm_id=arm_id)


class TestVerdict:
    """The six sentences, the link names, and every unchecked branch."""

    def verdict_for(self, result=None, triple=None, request=None):
        """Solve once against a scripted checker and return the verdict."""
        checker = FakeChecker(result=result, triple=triple)
        return service(checker=checker).solve(request or body())['verdict']

    def test_a_clear_pose_reports_the_checker_that_answered(self):
        """Something looked at this pose, and the field says which."""
        verdict = self.verdict_for(
            result=fake_checker.CheckResult(ok=True, min_clearance=0.041))
        assert verdict == {'status': 'clear', 'min_clearance': 0.041,
                           'offending_links': [], 'reason': None,
                           'reason_code': None, 'contacts': [],
                           'arms': {'panda1': {'status': 'clear',
                                               'reason': None,
                                               'offending_links': []}},
                           'checker': 'cell_model'}

    @pytest.mark.parametrize('kind,a,b,sentence', [
        ('joint_limit', 'panda1_joint4', '',
         'Panda 1 joint 4 is 0.7° past its limit.'),
        ('self', 'panda1_link5_v1', 'panda1_link0_v0',
         "Panda 1's forearm would hit its own base — 12 mm too close."),
        ('cross_arm', 'panda1_link6_v0', 'panda2_link5_v1',
         "Panda 1's wrist would hit Panda 2's forearm — 12 mm too close."),
        ('containment', 'panda1_link8_v0', 'work_area.z_min',
         "Panda 1's flange would leave the work area through the table top "
         'by 12 mm.'),
        ('environment', 'panda1_link5_v0', 'pedestal',
         "Panda 1's forearm would hit pedestal — 12 mm too close."),
        ('keep_out', 'panda1_link6_v0', 'operator_side',
         "Panda 1's wrist would enter the keep-out zone operator_side."),
    ])
    def test_one_sentence_per_contact_kind(self, kind, a, b, sentence):
        """Six kinds, six sentences, each written out where it is authored."""
        distance = -0.012 if kind != 'joint_limit' else -0.0122173
        verdict = self.verdict_for(
            result=collision(contact(kind, a, b, distance=distance)))
        assert verdict['status'] == 'collision'
        assert verdict['reason'] == sentence
        assert verdict['reason_code'] == 'contact'
        assert verdict['checker'] == 'cell_model'

    @pytest.mark.parametrize('face,phrase', [
        ('x_min', 'past the near edge'),
        ('x_max', 'past the far edge'),
        ('y_min', 'past the panda 2 side'),
        ('y_max', 'past the panda 1 side'),
        ('z_min', 'through the table top'),
        ('z_max', 'through the ceiling'),
    ])
    def test_one_phrase_per_containment_face(self, face, phrase):
        """Six faces, six plain phrases, and no compass directions anywhere."""
        verdict = self.verdict_for(result=collision(contact(
            'containment', 'panda1_link8_v0', 'work_area.{}'.format(face))))
        assert phrase in verdict['reason']

    def test_a_gap_that_rounds_to_zero_says_just_touching(self):
        """Nobody says nought millimetres; they say the two things touch."""
        verdict = self.verdict_for(result=collision(
            contact('self', 'panda1_link5_v1', 'panda1_link0_v0',
                    distance=-0.0001)))
        assert verdict['reason'].endswith('— just touching.')

    def test_the_min_clearance_is_copied_through_unchanged(self):
        """It mixes metres and radians; converting it would be a lie."""
        verdict = self.verdict_for(result=collision(
            contact('self', 'panda1_link5_v1', 'panda1_link0_v0'),
            min_clearance=-0.0123))
        assert verdict['min_clearance'] == -0.0123

    @pytest.mark.parametrize('volume,expected', [
        ('panda1_link5_v1', 'panda1_link5'),
        ('panda1_link5_v10', 'panda1_link5'),
        ('panda1_link5', 'panda1_link5'),
        ('panda1_pedestal', 'panda1_pedestal'),
        ('work_area.z_min', 'work_area.z_min'),
    ])
    def test_volume_ids_become_link_names(self, volume, expected):
        """The renderer tints links; the model reports volumes."""
        assert ghost.link_of(volume) == expected

    @pytest.mark.parametrize('kind,a,b,expected', [
        ('joint_limit', 'panda1_joint4', '', []),
        ('containment', 'panda1_link8_v0', 'work_area.z_min', ['panda1_link8']),
        ('environment', 'panda1_link5_v0', 'pedestal', ['panda1_link5']),
        ('keep_out', 'panda1_link6_v0', 'operator_side', ['panda1_link6']),
        ('self', 'panda1_link5_v1', 'panda1_link0_v0',
         ['panda1_link0', 'panda1_link5']),
        ('self', 'panda1_pedestal', 'panda1_link5_v1', ['panda1_link5']),
        ('cross_arm', 'panda1_link6_v0', 'panda2_link5_v1',
         ['panda1_link6', 'panda2_link5']),
    ])
    def test_offending_links_holds_link_names_and_nothing_else(
            self, kind, a, b, expected):
        """
        A joint name is neither a link name nor a volume id.

        The renderer feeds this list to a link-mesh lookup, so a blanket
        union over both identifier fields would hand it panda1_joint4 for a
        joint-limit contact and work_area.z_min for a containment one.
        """
        links = ghost.offending_links_for([contact(kind, a, b)])
        assert links == expected
        assert all(re.match(r'^panda[12]_link[0-8]$', name) for name in links)

    @pytest.mark.parametrize('code,sentence', [
        ('checker_absent', workspace.NOTE_PACKAGE_ABSENT),
        ('other_arm_pose_unknown', workspace.NOTE_SCENE_INCOMPLETE),
        ('interlock_mismatch', workspace.NOTE_INTERLOCK_MISMATCH),
        ('profile_arm_mismatch', workspace.NOTE_PROFILE_ARM_MISMATCH),
        ('checker_error', 'a configuration maps arm_id to seven joint positions'),
    ])
    def test_every_unchecked_branch_says_absent_and_carries_a_sentence(
            self, code, sentence):
        """The field answers "did a cell model look at this pose?" -- none did."""
        verdict = self.verdict_for(triple=(sentence, code))
        # No cell model answered, so there is no attribution to carry: an
        # empty map is "nothing was attributed", never "every arm is clear".
        assert verdict == {'status': 'unchecked', 'min_clearance': None,
                           'offending_links': [], 'reason': sentence,
                           'reason_code': code, 'contacts': [], 'arms': {},
                           'checker': 'absent'}

    def test_the_checked_scene_carries_the_solved_joints(self):
        """
        The verdict describes the pose the user is ABOUT to see.

        Checking the pose the client sent would tint the arm one drag frame
        late in both directions: green while the ghost enters the table, red
        after it has left.
        """
        solved = [0.1] * 7
        solver = StubSolver().script(IkReply(result=0, positions=tuple(solved)))
        checker = FakeChecker()
        service(solver=solver, checker=checker).solve(
            body(scene={'panda1': list(READY_POSE),
                        'panda2': list(READY_POSE)}))
        asked = checker.seen[0]
        assert asked['panda1'] == tuple(solved)
        # Every OTHER arm comes from the client verbatim.
        assert asked['panda2'] == tuple(READY_POSE)

    def test_no_verdict_is_produced_for_a_pose_that_was_never_solved(self):
        """Emitting one would describe a pose that does not exist."""
        solver = StubSolver().script(failure(6))
        checker = FakeChecker()
        result = service(solver=solver, checker=checker).solve(body())
        assert result['verdict'] is None
        assert checker.seen == []

    def test_a_checker_that_raises_is_never_reported_as_clear(self):
        """The real wrapper turns an exception into a sentence, not a lie."""
        error = workspace.WorkspaceModelError('unknown arm_id')
        model = FakeCellModel(raises=error)
        checker = checker_holding(model)
        result, sentence, code = checker.check('dual', {
            'panda1': READY_POSE, 'panda2': READY_POSE})
        assert result is None
        assert code == 'checker_error'
        assert sentence == 'unknown arm_id'
        verdict = service(checker=checker).solve(
            body(scene={'panda1': list(READY_POSE),
                        'panda2': list(READY_POSE)}))['verdict']
        assert verdict['status'] == 'unchecked'
        assert verdict['checker'] == 'absent'

    def test_a_partial_scene_reports_the_incomplete_sentence(self):
        """A dual profile with one arm is a state, not an exception."""
        checker = checker_holding(FakeCellModel())
        result, sentence, code = checker.check('dual', {'panda1': READY_POSE})
        assert result is None
        assert code == 'other_arm_pose_unknown'
        assert sentence == workspace.NOTE_SCENE_INCOMPLETE

    def test_a_single_profile_that_does_not_cover_this_arm(self):
        """Report unchecked rather than check the wrong frame."""
        checker = checker_holding(FakeCellModel(arms=('panda1',)))
        result, sentence, code = checker.check('single', {'panda2': READY_POSE})
        assert result is None
        assert code == 'profile_arm_mismatch'
        assert sentence == workspace.NOTE_PROFILE_ARM_MISMATCH

    def test_the_checker_is_asked_only_about_the_arms_it_declares(self):
        """A model that declares one arm is never handed two."""
        model = FakeCellModel(arms=('panda1',))
        checker = checker_holding(model)
        checker.check('single', {'panda1': READY_POSE, 'panda2': READY_POSE})
        assert set(model.seen[0]) == {'panda1'}

    @pytest.mark.parametrize('interlock', ['ok', 'mismatch', 'not_checked'])
    @pytest.mark.parametrize('arms,arm_id', [
        (('panda1', 'panda2'), 'panda1'),
        (('panda2',), 'panda1'),
    ])
    def test_the_cheap_note_agrees_with_the_scene_status_block(
            self, interlock, arms, arm_id):
        """
        ``apply_note`` is a cheaper route to one answer, not a second opinion.

        The frame asks it once per arm per tick and must not pay for the cell
        volume to do it, so it reads the cache and nothing else. That makes it
        a second implementation of three of ``status``'s rows, which is legal
        only while something compares the two.
        """
        checker = checker_holding(FakeCellModel(arms=arms))
        checker.set_interlock(interlock)
        status = checker.status('dual')
        if not status['available']:
            expected = (status['cell_note'] or workspace.NOTE_PACKAGE_ABSENT,
                        'absent')
        elif status['interlock'] == 'mismatch':
            expected = (status['checker_note'], 'mismatch')
        elif arm_id not in arms:
            expected = (workspace.NOTE_PROFILE_ARM_MISMATCH, 'absent')
        else:
            expected = None
        assert checker.apply_note('dual', arm_id) == expected

    def test_the_cheap_note_reports_a_profile_that_would_not_load(self):
        """A checker with nothing loaded answers the loader's own sentence."""
        checker = workspace.WorkspaceChecker(cell_path='/nowhere/at/all.yaml')
        sentence, code = checker.apply_note('dual', 'panda1')
        assert code == 'absent'
        assert sentence == checker.status('dual')['cell_note']


class TestVerdictContacts:
    """
    The itemised half of a verdict, which each arm's own line comes from.

    The whole-scene fields answer "is this cell allowed". They cannot answer
    "what is wrong with Panda 1", and a console that draws a row per arm has
    to answer that -- which is how Panda 2's joint limit came to be printed
    above Panda 1's degrees and stay there. These cases pin the list that
    fixed it: every contact, in the checker's order, with the same sentence
    the one-line `reason` would have made of it.
    """

    def verdict_for(self, result, scene=None):
        """
        Solve once against a scripted checker and return the verdict.

        The scene names BOTH arms by default, because the per-arm half of a
        verdict has one entry per arm that was checked and a one-arm scene
        would make the two-arm cases below assert nothing.
        """
        request = body(scene=scene or {'panda1': list(READY_POSE),
                                       'panda2': list(READY_POSE)})
        return service(checker=FakeChecker(result=result),
                       arm_ids=('panda1', 'panda2')).solve(request)['verdict']

    def test_a_clear_verdict_carries_an_empty_list(self):
        """Nothing is wrong, so there is nothing to itemise."""
        verdict = self.verdict_for(
            fake_checker.CheckResult(ok=True, min_clearance=0.041))
        assert verdict['contacts'] == []

    def test_every_contact_carries_its_own_sentence_and_its_own_names(self):
        """One entry per contact, each field the page reads spelled out."""
        one = contact('joint_limit', 'panda2_joint4', '',
                      distance=-0.0698132, arm_id='panda2')
        two = contact('cross_arm', 'panda1_link6_v0', 'panda2_link5_v1',
                      distance=-0.012, arm_id='panda1')
        result = fake_checker.CheckResult(
            ok=False, min_clearance=-0.0698132, contacts=(one, two))
        contacts = self.verdict_for(result)['contacts']
        assert contacts == [
            {'kind': 'joint_limit', 'arm_id': 'panda2', 'a': 'panda2_joint4',
             'b': '', 'distance': -0.0698132,
             'sentence': 'Panda 2 joint 4 is 4.0\u00b0 past its limit.'},
            {'kind': 'cross_arm', 'arm_id': 'panda1', 'a': 'panda1_link6_v0',
             'b': 'panda2_link5_v1', 'distance': -0.012,
             'sentence': "Panda 1's wrist would hit Panda 2's forearm "
                         '\u2014 12 mm too close.'},
        ]

    def test_every_sentence_is_the_one_the_single_reason_would_have_used(self):
        """
        Same builder, one contact at a time -- so the words cannot drift.

        A second spelling of these sentences is the failure this asserts
        against: the page renders them verbatim and has no copy of any of
        them, so a list whose words differed from `reason`'s would put two
        vocabularies on one panel.
        """
        made = [contact('self', 'panda1_link5_v1', 'panda1_link0_v0'),
                contact('containment', 'panda2_link8_v0', 'work_area.z_min',
                        arm_id='panda2'),
                contact('keep_out', 'panda1_link6_v0', 'operator_side'),
                contact('environment', 'panda2_link5_v0', 'pedestal',
                        arm_id='panda2')]
        result = fake_checker.CheckResult(ok=False, min_clearance=-0.012,
                                          contacts=tuple(made))
        contacts = self.verdict_for(result)['contacts']
        assert [entry['sentence'] for entry in contacts] == [
            ghost.verdict_sentence(item) for item in made]

    def test_the_first_entry_is_the_sentence_the_verdict_already_reported(self):
        """
        `contacts[0].sentence` IS `reason`, since nothing here re-sorts.

        The checker returns its contacts most-violating first and `reason` is
        built from the head of that tuple. A second sort in the payload
        builder would be a second opinion about which contact is the worst,
        and the two fields would disagree about the same answer.
        """
        made = (contact('containment', 'panda1_link8_v0', 'work_area.z_min',
                        distance=-0.067),
                contact('self', 'panda1_link5_v1', 'panda1_link0_v0',
                        distance=-0.012),
                contact('joint_limit', 'panda2_joint4', '',
                        distance=-0.0698132, arm_id='panda2'))
        verdict = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.067, contacts=made))
        assert verdict['contacts'][0]['sentence'] == verdict['reason']
        assert [entry['a'] for entry in verdict['contacts']] == [
            item.a for item in made]

    def test_the_list_is_bounded_and_keeps_the_worst_end(self):
        """
        A deeply folded pose reports many contacts; a drag route carries few.

        The bound cuts the TAIL, never the head, so what survives is the
        worst end -- which is what a reader wants first. Nothing is
        ATTRIBUTED from this list; see the case below for why that matters.
        """
        made = tuple(
            contact('self', 'panda1_link{}_v0'.format(index % 8),
                    'panda1_link0_v0', distance=-0.05 + index * 0.001)
            for index in range(20))
        verdict = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.05, contacts=made))
        assert ghost.CONTACT_LIMIT == 8
        assert len(verdict['contacts']) == 8
        assert [entry['a'] for entry in verdict['contacts']] == [
            item.a for item in made[:8]]

    def test_an_arm_whose_only_contact_sits_past_the_bound_is_still_named(self):
        """
        THE BOUND IS NOT ALLOWED TO DECIDE WHAT IS TRUE.

        One folded arm alone can report a dozen contacts -- self-collision
        reports one per capsule pair below margin, not one per arm -- so the
        neighbour's single containment breach lands past the eighth entry and
        falls off the wire. For as long as the page attributed from this
        list, that arm read "Clear of everything in the cell model." while it
        was outside the work area. `arms` is built over the COMPLETE tuple
        before the truncation, so it names the arm the list cannot.

        The shape is the measured one: twelve panda2 contacts sorted ahead of
        panda1's one containment breach.
        """
        crowd = tuple(
            contact('self', 'panda2_link{}_v0'.format(index % 8),
                    'panda2_link0_v0', distance=-0.05 + index * 0.001,
                    arm_id='panda2')
            for index in range(12))
        last = contact('containment', 'panda1_link7_v0', 'work_area.x_max',
                       distance=-0.004, arm_id='panda1')
        verdict = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.05, contacts=crowd + (last,)))

        assert len(verdict['contacts']) == ghost.CONTACT_LIMIT
        assert not any('panda1' in entry['arm_id'] or 'panda1' in entry['a']
                       or 'panda1' in entry['b']
                       for entry in verdict['contacts']), (
            'the fixture no longer crowds panda1 off the list, so it proves '
            'nothing')
        assert verdict['arms']['panda1'] == {
            'status': 'collision',
            'reason': ghost.verdict_sentence(last),
            'offending_links': ['panda1_link7'],
        }
        assert verdict['arms']['panda2']['status'] == 'collision'

    def test_an_arm_no_contact_names_is_the_only_arm_called_clear(self):
        """
        The per-arm answer, in both directions, from one whole-cell check.

        A cross-arm pair names one arm in `arm_id` and the other in `b` and
        belongs to BOTH rows; a self-contact belongs to one. Each arm's
        `offending_links` carry only that arm's own parts, so the tint says
        the same thing the sentence does.
        """
        pair = contact('cross_arm', 'panda1_link6_v0', 'panda2_link5_v1',
                       distance=-0.012, arm_id='panda1')
        verdict = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.012, contacts=(pair,)))
        assert verdict['arms']['panda1']['offending_links'] == ['panda1_link6']
        assert verdict['arms']['panda2']['offending_links'] == ['panda2_link5']
        assert verdict['arms']['panda1']['reason'] == verdict['reason']
        assert verdict['arms']['panda2']['reason'] == verdict['reason']

        alone = contact('self', 'panda2_link5_v1', 'panda2_link0_v0',
                        arm_id='panda2')
        verdict = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.012, contacts=(alone,)))
        assert verdict['arms']['panda1'] == {
            'status': 'clear', 'reason': None, 'offending_links': []}
        assert verdict['arms']['panda2']['status'] == 'collision'

    def test_a_clear_cell_says_so_for_each_arm_by_name(self):
        """
        A clear answer carries the same per-arm shape a refused one does.

        The page reads one key for every row it draws, so an answer that
        carried the map only when something was wrong would make "no entry"
        mean "clear" in one case and "not attributed" in the other.
        """
        verdict = self.verdict_for(
            fake_checker.CheckResult(ok=True, min_clearance=0.041))
        assert verdict['arms'] == {
            arm: {'status': 'clear', 'reason': None, 'offending_links': []}
            for arm in ('panda1', 'panda2')}

    def test_the_ghost_check_asks_for_every_violation_not_the_first(self):
        """
        The flag the attribution rests on, asserted where it is passed.

        `first_violation=True` makes the model stop at the first violation it
        finds, so `contacts` holds exactly one entry however many arms are in
        trouble -- and an answer with one contact cannot say whose fault a
        two-arm refusal is. travel.py made the same call for the same reason.
        """
        model = FakeCellModel()
        checker = checker_holding(model)
        service(checker=checker, arm_ids=('panda1', 'panda2')).solve(body(
            scene={'panda1': list(READY_POSE), 'panda2': list(READY_POSE)}))
        assert model.config_flags == [False]

    def test_a_distance_the_model_never_measured_is_said_in_words(self):
        """
        A non-finite distance is a sentence, never a 500 on every drag frame.

        `int(round(nan))` raises, and this builder is called for every
        contact of every kind -- on a route a drag calls thirty times a
        second, and again from travel.py on Apply. A degenerate capsule pair
        that produced one would have taken the whole panel down rather than
        cost one number.
        """
        for value in (float('nan'), float('inf')):
            touching = contact('self', 'panda1_link5_v1', 'panda1_link0_v0',
                               distance=value)
            sentence = ghost.verdict_sentence(touching)
            assert sentence.startswith("Panda 1's forearm would hit its own base")
            assert sentence.endswith('.')
            assert 'nan' not in sentence and 'inf' not in sentence

            limit = contact('joint_limit', 'panda2_joint4', '',
                            distance=value, arm_id='panda2')
            assert (ghost.verdict_sentence(limit)
                    == 'Panda 2 joint 4 is past its limit.')

            edge = contact('containment', 'panda1_link7_v0', 'work_area.x_max',
                           distance=value)
            assert edge and ghost.verdict_sentence(edge).endswith(
                'would leave the work area past the far edge.')

    def test_such_a_verdict_still_leaves_the_endpoint_answering(self):
        """The whole answer, end to end, over a contact nobody could measure."""
        verdict = self.verdict_for(collision(
            contact('self', 'panda1_link5_v1', 'panda1_link0_v0',
                    distance=float('nan'))))
        assert verdict['status'] == 'collision'
        assert verdict['contacts'][0]['distance'] is None
        assert verdict['arms']['panda1']['status'] == 'collision'
        assert json.dumps(verdict, allow_nan=False)

    def test_the_whole_verdict_still_fits_in_strict_json(self):
        """
        The envelope is JSON, and JSON has no NaN.

        `distance` is the first number this payload carries that came
        straight off a contact, so it goes through the same finite-or-null
        gate `min_clearance` already used.
        """
        verdict = self.verdict_for(collision(
            contact('self', 'panda1_link5_v1', 'panda1_link0_v0')))
        assert verdict['contacts'][0]['distance'] == -0.012
        assert json.dumps(verdict, allow_nan=False)
        # `distance` goes through the same finite-or-null gate `min_clearance`
        # has always gone through, so this field can never be the one that
        # puts a NaN literal in the envelope. There is no case here for a
        # non-finite distance ARRIVING, because such a contact cannot reach
        # this function: `verdict_sentence` raises on one, and has since
        # before this list existed.

    def test_both_arms_are_findable_in_one_answer(self):
        """
        The property the page's attribution rests on.

        Every contact names its arm in at least one of `arm_id`, `a` and `b`,
        and a cross-arm pair names both arms -- which is what lets one
        whole-scene answer fill one line per arm without the page guessing.
        """
        made = (contact('joint_limit', 'panda2_joint4', '',
                        distance=-0.0698132, arm_id='panda2'),
                contact('cross_arm', 'panda1_link6_v0', 'panda2_link5_v1'),
                contact('self', 'panda1_link5_v1', 'panda1_link0_v0'))
        contacts = self.verdict_for(fake_checker.CheckResult(
            ok=False, min_clearance=-0.0698132, contacts=made))['contacts']

        def names(entry, arm_id):
            return (entry['arm_id'] == arm_id
                    or entry['a'].startswith(arm_id + '_')
                    or entry['b'].startswith(arm_id + '_'))

        assert [names(entry, 'panda1') for entry in contacts] == [
            False, True, True]
        assert [names(entry, 'panda2') for entry in contacts] == [
            True, True, False]


class TestConcurrency:
    """Two viewers dragging at once must not serialise behind each other."""

    def test_several_threads_are_inside_the_check_at_the_same_moment(self):
        """
        The check is called unsynchronised, and that is a stated guarantee.

        The parsed cell structure is immutable after load and the collision
        core is numpy-only, so a lock here would buy nothing and cost every
        second viewer their drag. The barrier is what makes this a real
        proof: six threads must all be INSIDE the check before any of them
        may leave, so a wrapper that serialised them would hang here rather
        than pass quietly.
        """
        threads_wanted = 6
        barrier = threading.Barrier(threads_wanted)
        model = FakeCellModel(barrier=barrier)
        ghost_service = service(checker=checker_holding(model),
                                arm_ids=('panda1', 'panda2'))
        failures = []

        def drag():
            try:
                result = ghost_service.solve(body(scene={
                    'panda1': list(READY_POSE), 'panda2': list(READY_POSE)}))
                assert result['verdict']['status'] == 'clear'
            except Exception as error:      # noqa: BLE001 - reported below
                failures.append(error)

        threads = [threading.Thread(target=drag) for _ in range(threads_wanted)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30)
        assert failures == [], failures
        assert len(model.seen) == threads_wanted


class TestRedundancy:
    """The q7 sweep: its clamp, its deadline, and what it reports."""

    @pytest.mark.parametrize('asked,expected', [
        (None, defaults.GHOST_REDUNDANCY_SAMPLES),
        (1, defaults.GHOST_REDUNDANCY_SAMPLES_MIN),
        (999, defaults.GHOST_REDUNDANCY_SAMPLES_MAX),
        (17, 17),
    ])
    def test_the_sample_count_is_clamped(self, asked, expected):
        """Nine is too few to lerp through; thirty-three is a full second."""
        request = body()
        if asked is not None:
            request['samples'] = asked
        result = service().redundancy(request)
        assert result['samples'] == expected
        assert len(result['table']) == expected

    def test_every_sample_pins_joint_seven_at_the_same_target(self):
        """The hand is frozen for the whole gesture; only the elbow moves."""
        solver = StubSolver()
        service(solver=solver).redundancy(body(samples=9))
        assert len(solver.calls) == 9
        assert {call.redundancy_mode for call in solver.calls} == {1}
        assert {call.position for call in solver.calls} == {
            tuple(TARGET['position'])}
        assert {call.seed_positions for call in solver.calls} == {
            tuple(READY_POSE)}

    def test_the_sweep_spans_joint_sevens_whole_range(self):
        """Both endpoints are included, so the ring covers the real freedom."""
        solver = StubSolver()
        result = service(solver=solver).redundancy(body(samples=9))
        values = [row['q7'] for row in result['table']]
        assert values[0] == pytest.approx(defaults.POLICY_POSITION_LOWER_RAD[6])
        assert values[-1] == pytest.approx(defaults.POLICY_POSITION_UPPER_RAD[6])
        assert values == sorted(values)

    def test_unsolved_samples_are_omitted_while_samples_reports_the_request(self):
        """
        The two numbers genuinely differ, and the client compares them.

        Reporting len(table) here would hide exactly the condition the ring's
        fallback exists to detect.
        """
        solver = StubSolver().script(failure(6), None, failure(10))
        result = service(solver=solver).redundancy(body(samples=9))
        assert result['samples'] == 9
        assert len(result['table']) == 6

    def test_the_batch_deadline_bounds_the_whole_sweep(self):
        """Twenty-five calls at a per-call budget would be a 37 second request."""
        clock = FakeClock()

        def slow_solver(call, timeout_s):
            clock.advance(0.4)
            return IkReply(result=0, positions=tuple(call.seed_positions))

        result = service(solver=slow_solver, clock=clock).redundancy(
            body(samples=25))
        assert result['samples'] == 25
        assert 0 < len(result['table']) < 25

    def test_a_sample_never_waits_longer_than_the_deadline_allows(self):
        """The last sample's budget is what is left, not a fresh quarter second."""
        clock = FakeClock()

        def solver(call, timeout_s):
            clock.advance(0.2)
            return IkReply(result=0, positions=tuple(call.seed_positions))

        recorded = []

        def recording(call, timeout_s):
            recorded.append(timeout_s)
            return solver(call, timeout_s)

        service(solver=recording, clock=clock).redundancy(body(samples=9))
        assert all(value <= defaults.GHOST_SOLVE_TIMEOUT_S for value in recorded)
        assert recorded[-1] <= defaults.GHOST_REDUNDANCY_TIMEOUT_S

    def test_an_absent_ik_service_is_refused_rather_than_answered_empty(self):
        """An empty table would look like a table; a 503 teaches."""
        with pytest.raises(GhostError) as raised:
            service(ready=False).redundancy(body())
        assert raised.value.code == 'ghost_unavailable'
        assert raised.value.payload == {'ik_state': 'not_ready'}

    def test_the_table_carries_joint_vectors_and_no_geometry(self):
        """The elbow is derived by the renderer's own kinematics, not here."""
        result = service().redundancy(body(samples=9))
        for row in result['table']:
            assert set(row) == {'q7', 'positions'}
            assert len(row['positions']) == defaults.JOINT_COUNT


class TestOverTheSocket:
    """The three routes as bytes: statuses, envelopes and the inert token."""

    @pytest.fixture()
    def running(self, tmp_path):
        """Serve one app whose ghost is wired to doubles."""
        server = GhostServer(tmp_path, service())
        yield server
        server.close()

    def test_a_solve_is_a_200_with_the_success_envelope(self, running):
        """The envelope is the same one every other endpoint uses."""
        response = running.post('/api/ghost/solve', body())
        assert response.status == 200
        payload = response.json()
        assert payload['ok'] is True
        assert payload['solved'] is True

    def test_a_solve_is_byte_identical_with_and_without_the_token(self, running):
        """
        The route is token-free server-side; the page's header is ignored.

        A ghost that required the operator token would let a passive viewer
        take control by opening a 3D view.
        """
        plain = running.post('/api/ghost/solve', body()).body
        tokened = running.post(
            '/api/ghost/solve', body(),
            headers={'X-Operator-Token': 'nonsense'}).body
        assert plain == tokened

    def test_an_unreachable_pose_is_not_an_error_envelope(self, tmp_path):
        """200 with ok true, every time, so no scary bar flashes on a drag."""
        server = GhostServer(tmp_path, service(
            solver=StubSolver().script(failure(6))))
        try:
            payload = server.post('/api/ghost/solve', body()).json()
            assert payload['ok'] is True
            assert payload['solved'] is False
        finally:
            server.close()

    def test_a_rate_limited_solve_is_a_429_carrying_its_wait(self, tmp_path):
        """429 is new to this server; the framing and guards do not care."""
        clock = FakeClock()
        server = GhostServer(tmp_path, service(clock=clock))
        try:
            for _ in range(defaults.GHOST_RATE_CAPACITY):
                server.post('/api/ghost/solve', body())
            response = server.post('/api/ghost/solve', body())
            assert response.status == 429
            payload = response.json()
            assert payload['ok'] is False
            assert payload['error'] == 'ghost_rate_limited'
            assert payload['retry_after_ms'] >= 1
        finally:
            server.close()

    def test_an_absent_ik_service_is_a_503_carrying_its_state(self, tmp_path):
        """The panel picks its sentence from this discriminator."""
        server = GhostServer(tmp_path, service(
            solver=StubSolver().script(None), ready=False))
        try:
            response = server.post('/api/ghost/solve', body())
            assert response.status == 503
            payload = response.json()
            assert payload['error'] == 'ghost_unavailable'
            assert payload['ik_state'] == 'not_ready'
        finally:
            server.close()

    def test_a_malformed_body_is_a_400(self, running):
        """The detail names the field, and the page renders it verbatim."""
        response = running.post('/api/ghost/solve', body(seed=[0.0]))
        assert response.status == 400
        assert response.json()['error'] == 'invalid_json'

    def test_the_redundancy_route_answers_its_table(self, running):
        """One call at pointer-down, and no network traffic during the drag."""
        payload = running.post('/api/ghost/redundancy', body()).json()
        assert payload['ok'] is True
        assert payload['samples'] == defaults.GHOST_REDUNDANCY_SAMPLES


# ----------------------------------------------------------------------
# The gate that matters: the ghost cannot command anything
# ----------------------------------------------------------------------

PACKAGE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'franka_web')

#: Words that would mean this code can make something move.
FORBIDDEN = ('publish', 'enable', 'joint_target', 'JointTrajectory',
             'check_jog', 'check_path', 'operator_lease', 'X-Operator-Token',
             'rclpy', 'ros2 topic pub')

#: The whole import closure the three ghost modules are allowed to reach.
ALLOWED_IMPORTS = {
    'dataclasses', 'json', 'math', 'os', 're', 'threading', 'time', 'typing',
    'yaml', 'franka_web', 'franka_web.defaults', 'franka_web.ghost',
    'franka_web.ghost_copy', 'franka_web.workspace', 'franka_workspace_model',
    'franka_workspace_model.model',
}


def module_source(name):
    """Return one shipped module's source text."""
    with open(os.path.join(PACKAGE, name), encoding='utf-8') as handle:
        return handle.read()


def imports_of(tree):
    """Return every module name one parsed module imports."""
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module)
            for alias in node.names:
                names.add('{}.{}'.format(node.module, alias.name))
    return names


class TestNoMotionPath:
    """
    Four independent proofs that no ghost request can move a robot.

    The greps catch a copied line; the import closure catches a new
    dependency; the handler allowlist catches a new call in a handler; the
    exploding doubles catch everything else, including a call made through a
    helper the other three never look at.

    Apply exists now, and it is not here. It lives in ``travel.py``, behind a
    token-gated route, executed by the jog stream's own producer, and it has
    its own gate. What this class proves is unchanged and still worth proving:
    the ghost's three modules cannot express motion, so no ghost request --
    which is to say, no request from a viewer holding no operator token -- can
    move a robot. That property is what lets the ghost routes stay token-free.
    """

    # -- layer 1: the vocabulary ---------------------------------------

    @pytest.mark.parametrize('name', ['ghost.py', 'workspace.py'])
    def test_the_ghost_modules_contain_no_motion_vocabulary(self, name):
        """Zero occurrences, no allowance, in the two modules that decide."""
        source = module_source(name)
        found = [word for word in FORBIDDEN if word in source]
        assert found == [], '{} names {}'.format(name, found)

    def test_the_copy_module_has_exactly_one_stated_allowance(self):
        """
        Two words, once each, inside the two templates and nowhere else.

        The snippet the operator pastes must name the message type it is,
        and say what to do with it. That text is documentation, not a
        capability this server has -- which is why it lives in its own
        module, so the two modules above can stay at zero rather than the
        gate growing an allowance every time it complains.
        """
        source = module_source('ghost_copy.py')
        assert source.count('JointTrajectory') == 1
        assert source.count('publish') == 1
        for word in FORBIDDEN:
            if word in ('JointTrajectory', 'publish'):
                continue
            assert word not in source, 'ghost_copy.py names {}'.format(word)

    # -- layer 2: the import closure, walked transitively ---------------

    def test_the_import_closure_reaches_no_ros_and_no_session(self):
        """
        Walked TRANSITIVELY, which is the whole point of this layer.

        health.py imports franka_bringup.status, which imports rclpy and
        three controller_manager services. A closure test that read only
        each module's own import lines would pass green while the property
        it advertises was false.
        """
        seen = set()
        pending = ['franka_web.ghost', 'franka_web.ghost_copy',
                   'franka_web.workspace']
        closure = set()
        while pending:
            name = pending.pop()
            if name in seen:
                continue
            seen.add(name)
            path = os.path.join(PACKAGE, name.split('.')[-1] + '.py')
            if not os.path.isfile(path):
                continue
            with open(path, encoding='utf-8') as handle:
                tree = ast.parse(handle.read())
            for imported in imports_of(tree):
                closure.add(imported)
                if imported.startswith('franka_web.'):
                    pending.append(imported)
        assert 'rclpy' not in closure
        assert 'franka_bringup' not in closure
        assert 'franka_web.health' not in closure
        assert 'franka_web.session' not in closure
        assert 'franka_web.jog' not in closure
        assert 'franka_web.ros_bridge' not in closure
        assert 'franka_web.lock' not in closure
        unexpected = {name for name in closure
                      if name.split('.')[0] not in
                      {part.split('.')[0] for part in ALLOWED_IMPORTS}}
        assert unexpected == set(), 'the ghost grew a dependency: {}'.format(
            sorted(unexpected))

    def test_importing_the_ghost_needs_no_ros_at_all(self):
        """
        The honest form of "nothing here imports rclpy".

        Every module in the closure is imported in a subprocess whose import
        of rclpy is made to fail, so a transitive reach would raise rather
        than quietly succeed on a machine that happens to have ROS.
        """
        import subprocess
        script = (
            'import sys\n'
            'class Block:\n'
            '    def find_module(self, name, path=None):\n'
            '        if name.split(".")[0] in ("rclpy", "franka_bringup"):\n'
            '            raise ImportError("blocked: " + name)\n'
            '        return None\n'
            'sys.meta_path.insert(0, Block())\n'
            'import franka_web.ghost, franka_web.ghost_copy, '
            'franka_web.workspace\n'
            'print("ok")\n')
        result = subprocess.run(
            [os.sys.executable, '-c', script],
            capture_output=True, text=True,
            cwd=os.path.dirname(PACKAGE), timeout=120)
        assert result.returncode == 0, result.stderr
        assert 'ok' in result.stdout

    # -- layer 3: the handler allowlist --------------------------------

    def test_the_ghost_handlers_touch_nothing_but_the_ghost(self):
        """
        Every attribute chain rooted at the app object, enumerated.

        A future edit that reaches a session command from a ghost handler
        fails here with a readable message rather than at a live check.
        """
        with open(os.path.join(PACKAGE, 'http_api.py'), encoding='utf-8') as handle:
            tree = ast.parse(handle.read())
        wanted = {'handle_scene', 'handle_ghost_solve', 'handle_ghost_redundancy'}
        chains = set()
        found = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef) or node.name not in wanted:
                continue
            found.add(node.name)
            # Only MAXIMAL chains: `app.ghost.solve` walks past `app.ghost`,
            # and counting the fragment would say the handler touches
            # something it only ever reads through.
            inner_nodes = list(ast.walk(node))
            nested = {id(item.value) for item in inner_nodes
                      if isinstance(item, ast.Attribute)}
            for inner in inner_nodes:
                if isinstance(inner, ast.Attribute) and id(inner) not in nested:
                    chain = _chain(inner)
                    if chain and chain.startswith('app.'):
                        chains.add(chain)
        assert found == wanted, 'a ghost handler is missing: {}'.format(
            sorted(wanted - found))
        assert chains == {'app.ghost.scene', 'app.ghost.solve',
                          'app.ghost.redundancy', 'app.static_root'}

    def test_the_three_ghost_routes_require_no_token(self):
        """The security property, read off the routing table itself."""
        rows = [route for route in http_api.ROUTES
                if route.path.startswith('/api/ghost/')
                or route.path == '/api/scene']
        assert len(rows) == 3
        assert all(route.needs_token is False for route in rows)

    # -- layer 4: the exploding doubles --------------------------------

    def test_a_whole_request_reaches_neither_supervisor_nor_bridge(self, tmp_path):
        """
        The layer that catches what greps cannot.

        Anything beyond the one read-only frame call and the two IK methods
        raises inside the handler, and surfaces as a failed assertion rather
        than as a 500 nobody reads.

        The statuses are COLLECTED here and asserted only after the trespass
        lists, deliberately. A trespass raises inside the handler, and the
        HTTP layer turns any handler exception into a 500 -- so asserting the
        statuses first would report `500 != 200` and bury the one line that
        says what was actually reached for. The docstring above is a claim
        about the failure a future breaker reads, and the order below is what
        makes it true.
        """
        supervisor = ExplodingSupervisor()
        bridge = ExplodingBridge()
        ghost_service = GhostService(
            solver=bridge.call_solve_ik,
            checker=FakeChecker(),
            session_view=ghost.session_view_from(supervisor.frame),
            ik_ready=bridge.ik_service_ready)
        server = GhostServer(tmp_path, ghost_service, supervisor=supervisor)
        try:
            statuses = [
                server.request('GET', '/api/scene').status,
                server.post('/api/ghost/solve', body()).status,
                server.post('/api/ghost/redundancy', body(samples=9)).status,
                server.post('/api/ghost/solve', body(arm_id='x')).status,
            ]
        finally:
            server.close()
        assert supervisor.trespasses == [], (
            'a ghost request reached the supervisor: '
            + ', '.join(supervisor.trespasses))
        assert bridge.trespasses == [], (
            'a ghost request reached the bridge: '
            + ', '.join(bridge.trespasses))
        assert supervisor.frames > 0, 'the read-only frame was never read'
        assert statuses == [200, 200, 200, 400], statuses


def _chain(node):
    """Return a dotted attribute chain, or None when it is not a plain one."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name):
        return None
    parts.append(node.id)
    return '.'.join(reversed(parts))


CANNED_FRAME = {
    'session': {'arm_ids': ['panda1', 'panda2']},
    'arms': {'panda1': {'motion': {'command_topic': '/topic/arm_1/target'}},
             'panda2': {'motion': {'command_topic': None}}},
}


class ExplodingSupervisor:
    """Everything except the read-only frame is a contract violation."""

    def __init__(self):
        """Start with nothing read and nothing trespassed."""
        self.frames = 0
        self.trespasses = []

    def frame(self):
        """Return the ONE thing a ghost request is permitted to ask for."""
        self.frames += 1
        return CANNED_FRAME

    def __getattr__(self, name):
        """Fail loudly on anything else, naming what was reached for."""
        self.__dict__.setdefault('trespasses', []).append(name)
        raise AssertionError('a ghost request reached the supervisor: ' + name)


class ExplodingBridge:
    """Only the two IK methods exist; anything else fails the test."""

    def __init__(self):
        """Start with nothing trespassed."""
        self.trespasses = []

    def ik_service_ready(self):
        """Report the service as reachable."""
        return True

    def call_solve_ik(self, call, timeout_s):
        """Answer with the seed, which is all the endpoint needs."""
        return IkReply(result=0, positions=tuple(call.seed_positions))

    def __getattr__(self, name):
        """Fail loudly on anything else, naming what was reached for."""
        self.__dict__.setdefault('trespasses', []).append(name)
        raise AssertionError('a ghost request reached the bridge: ' + name)


# ----------------------------------------------------------------------
# The real checker, on the real corpus
# ----------------------------------------------------------------------


def load_corpus_entries():
    """
    Return the model package's hand-derived corpus, or an empty list.

    Loaded by file path rather than by import: the corpus lives in another
    package's test directory, which is not importable as a module and whose
    name would collide with the standard library's own ``test`` package.
    """
    directory = sample_cell.corpus_directory()
    if directory is None:
        return []
    loader_path = os.path.join(os.path.dirname(str(directory)),
                               'corpus_loader.py')
    if not os.path.isfile(loader_path):
        return []
    try:
        spec = importlib.util.spec_from_file_location(
            'franka_workspace_model_corpus_loader', loader_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return list(module.load_corpus(directory))
    except Exception:               # noqa: BLE001 - reported as a skip
        return []


class TestVerdictAgainstCorpus:
    """
    The endpoint's verdict against expectations nobody derived from it.

    Every entry in that corpus was worked out by hand, or by an independent
    kinematic chain, before the checker existed. Running the WHOLE endpoint
    over it is the only place the adapter, the sentence builder and the real
    collision core are asked the same question at once.
    """

    @pytest.fixture(scope='class')
    def real_checker(self, tmp_path_factory):
        """Return a checker holding the shipped cell model, or skip."""
        pytest.importorskip('franka_workspace_model.model',
                            reason='the workspace model is not installed')
        path = sample_cell.write_sample_cell(
            str(tmp_path_factory.mktemp('cell')))
        if path is None:
            pytest.skip('the cell model and its sources are not both present')
        checker = workspace.WorkspaceChecker(cell_path=path)
        if checker.model_for('dual') is None:
            pytest.skip('the shipped cell model did not load here')
        return checker

    @pytest.fixture(scope='class')
    def corpus(self):
        """Return the corpus entries, or skip when they are not reachable."""
        entries = load_corpus_entries()
        if not entries:
            pytest.skip('the validation corpus is not in this workspace')
        return entries

    def test_every_corpus_entry_gets_the_verdict_it_expects(self, real_checker,
                                                            corpus):
        """
        One solve per entry, with the solver returning the entry's own pose.

        The stub hands back exactly what it was seeded with, so the pose the
        checker sees is the corpus pose and the answer is the corpus answer.
        """
        wrong = []
        for entry in corpus:
            if set(entry.q) != {'panda1', 'panda2'}:
                continue
            request = body(arm_id='panda1', seed=list(entry.q['panda1']),
                           scene={arm: list(values)
                                  for arm, values in entry.q.items()})
            answer = service(checker=real_checker,
                             arm_ids=('panda1', 'panda2')).solve(request)
            verdict = answer['verdict']
            expected = 'clear' if entry.ok else 'collision'
            if verdict['status'] != expected:
                wrong.append((entry.id, expected, verdict['status'],
                              verdict['reason']))
        assert wrong == [], wrong

    def test_no_forbidden_contact_kind_ever_appears(self, real_checker, corpus):
        """A verdict naming the wrong reason is worse than no verdict."""
        for entry in corpus:
            if entry.ok or set(entry.q) != {'panda1', 'panda2'}:
                continue
            result, _sentence, _code = real_checker.check('dual', entry.q)
            assert result is not None
            kinds = {item.kind for item in result.contacts}
            assert not kinds & set(entry.forbidden_kinds), entry.id

    def test_every_reported_contact_produces_a_sentence(self, real_checker,
                                                        corpus):
        """No contact the real model can report is left without words."""
        seen = set()
        for entry in corpus:
            if entry.ok or set(entry.q) != {'panda1', 'panda2'}:
                continue
            result, _sentence, _code = real_checker.check('dual', entry.q)
            for item in result.contacts:
                sentence = ghost.verdict_sentence(item)
                assert sentence and sentence.endswith('.'), (entry.id, sentence)
                seen.add(item.kind)
            for name in ghost.offending_links_for(result.contacts):
                assert re.match(r'^panda[12]_link[0-8]$', name), (entry.id, name)
        assert seen, 'the corpus produced no contacts at all'

    #: A scene in which BOTH arms are at fault for reasons that do not depend
    #: on the fence's geometry: panda1's joint 1 and panda2's joint 4 are each
    #: past a joint limit. (An earlier version put panda1 4 mm outside the
    #: work area under the padded capsule model; the mesh-exact fence
    #: measures that pose as inside, which is the honest answer, so a
    #: geometry-independent fault keeps this test about attribution.)
    BOTH_IN_TROUBLE = {
        'panda1': [3.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
        'panda2': [0.0, -0.7854, 0.0, -3.2, 0.0, 1.5708, 0.7854],
    }

    def test_two_faulted_arms_are_each_named_in_one_answer(self, real_checker):
        """
        THE BLOCKER, through the real model: neither arm may read clear.

        Every other case in this file scripts the checker, and a scripted
        checker returns the contact tuple the test wrote -- so no test could
        see that the production call asked for the FIRST violation only, and
        that a one-entry list leaves the second faulted arm unnamed. The
        console reads one row per arm out of this answer, so an arm nothing
        names is an arm told, in green, that it is clear.
        """
        request = body(arm_id='panda1',
                       seed=list(self.BOTH_IN_TROUBLE['panda1']),
                       scene={arm: list(values)
                              for arm, values in self.BOTH_IN_TROUBLE.items()})
        verdict = service(checker=real_checker,
                          arm_ids=('panda1', 'panda2')).solve(request)['verdict']
        assert verdict['status'] == 'collision'
        assert verdict['arms']['panda1']['status'] == 'collision'
        assert verdict['arms']['panda2']['status'] == 'collision'
        assert 'Panda 1' in verdict['arms']['panda1']['reason']
        assert 'Panda 2' in verdict['arms']['panda2']['reason']

    def test_the_check_behind_that_answer_itemised_every_violation(
            self, real_checker):
        """
        And the reason it can: the ghost check does not stop at the first one.

        Asserted on the checker itself rather than through the service, so
        this reads as the property it is -- the itemised list the attribution
        is built from is the whole list.
        """
        result, _sentence, _code = real_checker.check(
            'dual', self.BOTH_IN_TROUBLE)
        arms = {item.arm_id for item in result.contacts}
        assert arms == {'panda1', 'panda2'}, [
            (item.kind, item.arm_id) for item in result.contacts]

    def test_the_cell_the_scene_draws_is_the_cell_that_was_checked(
            self, real_checker):
        """One model answers both questions, so the box cannot disagree."""
        status = real_checker.status('dual')
        assert status['cell_source'] == 'cell_model'
        assert status['cell']['x_min'] < status['cell']['x_max']
        assert status['cell']['y_min'] < status['cell']['y_max']
        assert status['cell']['z_min'] < status['cell']['z_max']
        assert status['model']['model_sha256']


class TestRealCheckerDegradedModes:
    """The wrapper's job is to answer, never to raise; here is each answer."""

    def test_a_missing_cell_file_says_so_and_names_what_was_tried(self):
        """The most misleading state is the one that blames the wrong thing."""
        checker = workspace.WorkspaceChecker(cell_path='/nowhere/cell.yaml')
        status = checker.status('dual')
        assert status['cell'] is None
        assert status['cell_source'] == 'unavailable'
        assert status['cell_note'].startswith(workspace.NOTE_LOAD_FAILED)
        assert '/nowhere/cell.yaml' in status['cell_note']
        assert 'cell model not loaded' in checker.banner()

    def test_a_corrupt_cell_file_carries_the_loaders_own_sentence(self, tmp_path):
        """
        The second line is the loader's words, not a paraphrase of them.

        Whoever wrote the cell file needs to know which key it choked on, and
        only the loader knows that.
        """
        pytest.importorskip('franka_workspace_model.model',
                            reason='the workspace model is not installed')
        path = sample_cell.write_sample_cell(str(tmp_path),
                                             text='schema_version: 1\n')
        if path is None:
            pytest.skip('the cell model and its sources are not both present')
        bus = Bus()
        checker = workspace.WorkspaceChecker(cell_path=path, log_bus=bus)
        status = checker.status('dual')
        assert status['cell_note'].startswith(workspace.NOTE_LOAD_FAILED)
        assert '\n' in status['cell_note']
        assert status['cell_note'].splitlines()[1]
        assert bus.lines and bus.lines[0][0] == 'warn'

    def test_a_failure_is_logged_once_and_not_once_per_drag(self, tmp_path):
        """A wedged cell file must not fill the drawer during a drag."""
        pytest.importorskip('franka_workspace_model.model',
                            reason='the workspace model is not installed')
        path = sample_cell.write_sample_cell(str(tmp_path),
                                             text='schema_version: 1\n')
        if path is None:
            pytest.skip('the cell model and its sources are not both present')
        bus = Bus()
        checker = workspace.WorkspaceChecker(cell_path=path, log_bus=bus)
        for _ in range(5):
            checker.check('dual', {'panda1': READY_POSE, 'panda2': READY_POSE})
        assert len(bus.lines) == 1

    def test_an_interlock_mismatch_stops_the_check_before_it_runs(self):
        """A model built for another description must not answer at all."""
        checker = checker_holding(FakeCellModel())
        checker.set_interlock('mismatch')
        result, sentence, code = checker.check(
            'dual', {'panda1': READY_POSE, 'panda2': READY_POSE})
        assert result is None
        assert code == 'interlock_mismatch'
        assert sentence == workspace.NOTE_INTERLOCK_MISMATCH
        assert checker.status('dual')['checker_note'] == (
            workspace.NOTE_INTERLOCK_MISMATCH)

    def test_an_unknown_interlock_state_is_ignored(self):
        """Only the three states the payload declares may ever be reported."""
        checker = checker_holding(FakeCellModel())
        checker.set_interlock('probably fine')
        assert checker.status('dual')['interlock'] == 'not_checked'

    def test_the_cell_path_prefers_the_operators_own_key(self):
        """One configuration surface, and it wins over every default."""
        assert workspace.resolve_cell_path('/lab/cell.yaml') == '/lab/cell.yaml'

    def test_no_environment_variable_resolves_the_cell_path(self, monkeypatch):
        """
        There is no environment variable, here or anywhere.

        The package's one configuration surface is its file; a variable would
        be a second one, invisible to the operator reading that file.
        """
        monkeypatch.setenv('FRANKA_WEB' + '_CELL_MODEL', '/nowhere/else.yaml')
        assert workspace.resolve_cell_path() != '/nowhere/else.yaml'
