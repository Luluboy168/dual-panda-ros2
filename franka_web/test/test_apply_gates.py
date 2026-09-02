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
The structural gates that keep Apply the shape its safety argument assumes.

Every test here reads the tree rather than running it. Behaviour is proved
elsewhere -- ``test_travel.py`` for the decision, ``test_session_state_machine``
for the lifecycle, ``e2e_fake_motion_mock_test`` on the wire. What these prove
is that a future edit cannot quietly move the pieces:

* there is exactly ONE door into a travel, and it is locked behind a path
  check (G3-G1);
* the stops that must be bounded take no queue and no supervisor lock
  (G3-G1b);
* the ghost surface gained no motion seam (G3-G2);
* the decider cannot publish and the publisher does not decide (G3-G5,
  G3-G6);
* and the external contract this whole argument rests on is really doing what
  it says (G3-G7, G3-G10).
"""

import ast
import os
import subprocess
import sys
import tempfile
import time

from franka_web import defaults, travel
import pytest
from support import sample_cell

PACKAGE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'franka_web')
STATIC_GHOST = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'static', 'ghost')

#: The Franka home pose, the start of every fixture below.
HOME = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)


def module_source(name):
    """Return one shipped module's source text."""
    with open(os.path.join(PACKAGE, name), encoding='utf-8') as handle:
        return handle.read()


def module_tree(name):
    """Return one shipped module's parsed AST."""
    return ast.parse(module_source(name))


def python_modules():
    """Return every shipped module name in the package directory."""
    return sorted(name for name in os.listdir(PACKAGE) if name.endswith('.py'))


def functions_of(tree):
    """Return ``{qualified name: node}`` for every function in one module."""
    found = {}

    def walk(node, prefix):
        """Record every function definition under ``node``."""
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                name = prefix + child.name
                found[name] = child
                walk(child, name + '.')
            elif isinstance(child, ast.ClassDef):
                walk(child, prefix + child.name + '.')
            else:
                walk(child, prefix)
    walk(tree, '')
    return found


def function_named(tree, name):
    """Return the one function or method with this bare name, or raise."""
    matches = [node for key, node in functions_of(tree).items()
               if key.rpartition('.')[2] == name]
    assert len(matches) == 1, 'expected one {}, found {}'.format(
        name, len(matches))
    return matches[0]


def enclosing_function(tree, target):
    """Return the name of the function ``target`` sits inside, or None."""
    for name, node in functions_of(tree).items():
        if any(item is target for item in ast.walk(node)):
            return name.rpartition('.')[2]
    return None


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


def calls_named(tree, attribute):
    """Return every Call node whose callee attribute is ``attribute``."""
    return [node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == attribute]


def subscript_targets(tree, attribute):
    """Return every assignment whose target is ``self.<attribute>[...]``."""
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (isinstance(target, ast.Subscript)
                    and isinstance(target.value, ast.Attribute)
                    and target.value.attr == attribute):
                found.append(node)
    return found


class TestG3G1TheCheckCannotBeSkipped:
    """One door into a travel, and the check is the lock on it."""

    def test_check_path_is_called_from_exactly_one_function(self):
        """
        The whole package has one caller, and it is ``plan_travel``.

        This is the gate the ``check_path`` prohibition on the ghost surface
        turned into: the word is still forbidden in ghost.py and workspace.py,
        and here it is permitted in exactly one place, which is that module's
        entire reason to exist.
        """
        homes = []
        for name in python_modules():
            tree = module_tree(name)
            for call in calls_named(tree, 'check_path'):
                homes.append((name, enclosing_function(tree, call)))
        assert homes == [('travel.py', 'plan_travel')], homes

    def test_only_travel_py_reaches_for_the_attribute_at_all(self):
        """
        Not even a bound reference lives elsewhere -- prose aside.

        Read off the AST rather than the text, so the several modules that
        EXPLAIN the check in a comment stay legal while a module that reached
        for it does not.
        """
        naming = []
        for name in python_modules():
            for node in ast.walk(module_tree(name)):
                if isinstance(node, ast.Attribute) and node.attr == 'check_path':
                    naming.append(name)
        assert sorted(set(naming)) == ['travel.py'], sorted(set(naming))

    def test_plan_travel_cannot_return_without_a_passing_check(self):
        """
        The check is the LAST statement that can refuse.

        Every ``return`` in the function is downstream of the ``check_path``
        call, so there is no early exit that builds a plan around it.
        """
        function = function_named(module_tree('travel.py'), 'plan_travel')
        call = calls_named(function, 'check_path')
        assert len(call) == 1
        checked_at = call[0].lineno
        # Its OWN returns, not those of the nested helper that builds one
        # waypoint mapping: that helper returns a dict, never a plan.
        nested = set()
        for inner in functions_of(function).values():
            nested.update(id(item) for item in ast.walk(inner))
        returns = [node.lineno for node in ast.walk(function)
                   if isinstance(node, ast.Return) and node.value is not None
                   and id(node) not in nested]
        assert returns and min(returns) > checked_at, returns

    def test_a_plan_is_installed_in_exactly_two_places_and_named_ones(self):
        """
        Every write of ``self._arm_travel[...]`` is enumerated here.

        Clears (``= None``) are free -- a clear can only stop a travel. The
        two writes that install one are the commit in the Apply handler and
        the tick's own store-back, and nothing else in the package may.
        """
        tree = module_tree('session.py')
        installs = []
        for node in subscript_targets(tree, '_arm_travel'):
            if isinstance(node.value, ast.Constant) and node.value.value is None:
                continue
            installs.append((enclosing_function(tree, node),
                             getattr(node.value, 'id', ast.dump(node.value))))
        assert sorted(installs) == [
            ('_accept_arm_apply', 'plan'),
            ('_advance_travel_locked', 'following')], installs

    def test_no_other_module_writes_the_travel_dictionary(self):
        """A second writer would be a second door with no check on it."""
        writers = [name for name in python_modules()
                   if name != 'session.py' and '_arm_travel' in module_source(name)]
        assert writers == [], writers

    def test_the_committed_plan_is_the_one_plan_travel_returned(self):
        """The name installed in the handler is bound from ``plan_travel``."""
        function = function_named(module_tree('session.py'), '_accept_arm_apply')
        sources = [node for node in ast.walk(function)
                   if isinstance(node, ast.Assign)
                   and any(getattr(target, 'id', None) == 'plan'
                           for target in node.targets)]
        assert len(sources) == 1
        assert isinstance(sources[0].value, ast.Call)
        assert sources[0].value.func.attr == 'plan_travel'


class TestG3G1bTheStopsAreLockFreeAndUnqueued:
    """The bounded stops are structural, not incidental."""

    def test_the_cancel_never_touches_the_command_queue(self):
        """
        ``_submit`` DISCARDS a command the supervisor never reached.

        For a stop that would mean an error response while the arm kept
        moving, so the cancel path may not name the queue at all.
        """
        function = function_named(module_tree('session.py'), '_cancel_travel_now')
        text = ast.dump(function)
        assert '_commands' not in text
        assert not [node for node in ast.walk(function)
                    if isinstance(node, ast.Call)
                    and getattr(node.func, 'attr', None) == '_submit']

    @pytest.mark.parametrize('name', ['_force_sources_jog_lockfree',
                                      '_clear_every_travel_lockfree',
                                      'revoke_operator_authorization'])
    def test_the_lock_free_paths_take_no_supervisor_lock(self, name):
        """
        They run inside the operator lock's own mutex and may not block.

        Taking ``_state_lock`` there would create the lock-order inversion the
        supervisor's own order forbids, and would stall every heartbeat behind
        a supervisor critical section.
        """
        function = function_named(module_tree('session.py'), name)
        # Read off the attribute accesses, not the text: these functions
        # EXPLAIN at length why they may not take that lock, and the
        # explanation must not be what trips the gate.
        reached = [node.attr for node in ast.walk(function)
                   if isinstance(node, ast.Attribute)]
        assert '_state_lock' not in reached

    def test_the_ticks_store_back_is_guarded_by_an_identity_check(self):
        """
        The compare-and-set, read off the source.

        A blind store-back would resurrect a plan a lock-free clear had
        already removed, and nothing else would ever clear it again.
        """
        function = function_named(
            module_tree('session.py'), '_advance_travel_locked')
        guarded = False
        for node in ast.walk(function):
            if not isinstance(node, ast.If):
                continue
            test = node.test
            if (isinstance(test, ast.Compare)
                    and len(test.ops) == 1 and isinstance(test.ops[0], ast.Is)
                    and getattr(test.comparators[0], 'id', None) == 'plan'):
                guarded = True
        assert guarded, 'the tick stores an advanced plan without an is-check'

    def test_the_advanced_plan_is_built_before_the_identity_check(self):
        """
        Nothing but the store may sit between the check and the store.

        Building the successor after the check would reopen exactly the window
        the check exists to close.
        """
        function = function_named(
            module_tree('session.py'), '_advance_travel_locked')
        built = [node.lineno for node in ast.walk(function)
                 if isinstance(node, ast.Call)
                 and getattr(node.func, 'attr', None) == 'advanced']
        # The CAS specifically: `<something> is plan`. The other identity
        # comparisons in this function ask whether a value is None, and those
        # legitimately come first.
        checks = [node.lineno for node in ast.walk(function)
                  if isinstance(node, ast.Compare)
                  and len(node.ops) == 1 and isinstance(node.ops[0], ast.Is)
                  and getattr(node.comparators[0], 'id', None) == 'plan']
        assert len(built) == 1 and len(checks) == 1, (built, checks)
        assert built[0] < checks[0], (built, checks)


class TestG3G2TheGhostSurfaceIsUnchanged:
    """The scene panel never gained a motion button, and cannot have."""

    @pytest.mark.parametrize('name', ['ghost.py', 'ghost_copy.py',
                                      'workspace.py'])
    def test_the_ghost_modules_name_none_of_applys_vocabulary(self, name):
        """
        Apply is a different module with its own gate, and stays there.

        The existing no-motion gate forbids ``check_path`` in two of these
        three; this adds the rest of the Apply surface, so a helper reaching
        back into the ghost modules fails here rather than at a live check.
        """
        source = module_source(name)
        for word in ('travel', 'TravelPlan', 'plan_travel', '_arm_travel',
                     'apply_refused', 'apply_unavailable', 'apply_in_progress',
                     '/api/arm/'):
            assert word not in source, '{} names {}'.format(name, word)

    def test_the_scene_module_still_refuses_an_apply_seam(self):
        """
        ``onApply`` is not a seam that was removed; it is one that throws.

        The scene keeps its own refusal, in its own words, so a future edit
        that hands it a motion callback fails loudly at mount time.
        """
        with open(os.path.join(STATIC_GHOST, 'ghost.js'), encoding='utf-8') as handle:
            source = handle.read()
        assert 'onApply is not part of this seam' in source
        assert 'throw new TypeError' in source

    def test_no_scene_module_reaches_the_apply_route_or_the_frame(self):
        """
        The panel gained no seam, no callback and no request.

        Its own purity gate already forbids ``/api/`` and the ROS vocabulary;
        this names the Apply surface specifically, so the intent survives a
        future edit to that list.
        """
        for name in sorted(os.listdir(STATIC_GHOST)):
            if not name.endswith('.js'):
                continue
            with open(os.path.join(STATIC_GHOST, name), encoding='utf-8') as handle:
                source = handle.read()
            for word in ('/api/', 'motion.apply', 'apply_refused',
                         'arm/panda'):
                assert word not in source, '{} names {}'.format(name, word)


class TestG3G5OnlyOnePublisher:
    """The central claim of the safety argument, read off the tree."""

    def test_publish_target_has_exactly_one_call_site(self):
        """
        Anything that made Apply its own publisher would destroy the argument.

        The argument is that every byte reaching the controller is built by
        one function, from one held target, under one set of gates. One call
        site is what makes that checkable by reading.
        """
        sites = []
        for name in python_modules():
            tree = module_tree(name)
            for call in calls_named(tree, 'publish_target'):
                sites.append((name, enclosing_function(tree, call)))
        assert sites == [('session.py', 'jog_stream_tick')], sites


class TestG3G6TheDeciderCannotPublish:
    """travel.py is pure, and the purity is enforced three ways."""

    #: Words that would mean this module can make something move.
    FORBIDDEN = ('rclpy', 'JointTrajectory', 'publish', 'joint_target',
                 'X-Operator-Token', 'franka_web.session',
                 'franka_web.ros_bridge')

    @pytest.mark.parametrize('word', FORBIDDEN)
    def test_travel_names_no_motion_vocabulary(self, word):
        """Zero occurrences, and no allowance."""
        assert word not in module_source('travel.py')

    def test_the_import_closure_is_the_three_modules_it_declares(self):
        """A new dependency here is a new way for the decider to reach out."""
        allowed = {'dataclasses', 'dataclasses.dataclass', 'dataclasses.replace',
                   'math', 'franka_web', 'franka_web.defaults',
                   'franka_web.ghost', 'franka_web.ghost.offending_links_for',
                   'franka_web.ghost.verdict_sentence'}
        assert imports_of(module_tree('travel.py')) <= allowed

    def test_it_imports_where_the_ros_client_library_does_not_exist(self):
        """
        The honest form of "nothing here imports rclpy".

        Imported in a subprocess whose import of the ROS client library is
        made to fail, so a transitive reach raises rather than quietly
        succeeding on a machine that happens to have ROS.
        """
        script = (
            'import sys\n'
            'class Block:\n'
            '    def find_module(self, name, path=None):\n'
            '        if name.split(".")[0] in ("rclpy", "franka_bringup"):\n'
            '            raise ImportError("blocked: " + name)\n'
            '        return None\n'
            'sys.meta_path.insert(0, Block())\n'
            'import franka_web.travel\n'
            'print("ok")\n')
        result = subprocess.run(
            [sys.executable, '-c', script], capture_output=True, text=True,
            cwd=os.path.dirname(PACKAGE), timeout=120)
        assert result.returncode == 0, result.stderr
        assert 'ok' in result.stdout


class TestG3G3TheNoMotionGateIsUnchanged:
    """The G1/G2 deferral gate did not rot into an allowance."""

    def test_the_forbidden_vocabulary_still_names_check_path(self):
        """
        The prohibition that made Apply a separate module is still in force.

        G3 satisfies it rather than relaxing it: the word is still zero in
        ghost.py and workspace.py, and lives only in travel.py.
        """
        from test_ghost_solve import ALLOWED_IMPORTS, FORBIDDEN
        assert 'check_path' in FORBIDDEN
        assert 'publish' in FORBIDDEN
        assert 'franka_web.travel' not in ALLOWED_IMPORTS
        assert 'franka_web.jog' not in ALLOWED_IMPORTS


class TestG3G7TheCheckIsReallySwept:
    """
    The external contract this whole argument rests on, exercised.

    If this test ever passes trivially -- because ``check_path`` stopped
    resampling and became a two-endpoint check in disguise -- G3's entire
    safety argument is void. It is written to fail loudly rather than to pass
    quietly, which is why both endpoints are asserted CLEAR before the path is
    asserted refused.
    """

    #: Two panda1 poses that are each individually allowed, and between which
    #: the straight line in joint space is not. Found by search against the
    #: shipped cell model and pinned here so the case is deterministic.
    CLEAR_A = (0.8703, -0.8353, -0.3782, -0.9314, 0.9376, 1.3517, 1.6014)
    CLEAR_B = (-1.182, -1.4377, 0.8119, -2.3209, -0.9559, 1.976, 1.0812)

    @pytest.fixture(scope='class')
    def model(self):
        """Load the shipped cell model, or skip: this cannot be invented."""
        workspace_model = pytest.importorskip('franka_workspace_model.model')
        directory = tempfile.mkdtemp()
        path = sample_cell.write_sample_cell(directory)
        if path is None:
            pytest.skip('the workspace model package installs no cell file here')
        return workspace_model.CellModel.load(path, profile='dual')

    def test_both_endpoints_are_allowed_on_their_own(self, model):
        """The premise. Without this the refusal below proves nothing."""
        for pose in (self.CLEAR_A, self.CLEAR_B):
            result = model.check_configuration(
                {'panda1': pose, 'panda2': HOME})
            assert result.ok, result.contacts
            swept = model.check_path([{'panda1': pose, 'panda2': HOME}])
            assert swept.ok, swept.contacts

    def test_the_straight_line_between_them_is_refused(self, model):
        """
        The conclusion: the samples BETWEEN two allowed poses are checked.

        The violating sample is strictly inside the path, so this cannot be
        satisfied by a checker that looks only at the endpoints.
        """
        result = model.check_path(
            [{'panda1': self.CLEAR_A, 'panda2': HOME},
             {'panda1': self.CLEAR_B, 'panda2': HOME}], first_violation=False)
        assert result.ok is False
        assert result.samples_evaluated > 2, (
            'check_path evaluated {} samples for a path this long; it is not '
            'resampling, and G3 rests on the fact that it does'.format(
                result.samples_evaluated))
        assert 0 < result.sample_index < result.samples_evaluated - 1, (
            result.sample_index, result.samples_evaluated)

    def test_the_refusal_places_itself_on_the_path_honestly(self, model):
        """
        The prefix the console shows, computed from the model's own numbers.

        This is why the call passes ``first_violation=False``: with True the
        model stops at the first violation and reports ``samples_evaluated``
        as the count it got through, so the percentage would always be 100 and
        every refusal would read "At the pose you drew".
        """
        result = model.check_path(
            [{'panda1': self.CLEAR_A, 'panda2': HOME},
             {'panda1': self.CLEAR_B, 'panda2': HOME}], first_violation=False)
        prefix = travel.refusal_prefix(result.sample_index,
                                       result.samples_evaluated)
        assert prefix.startswith('About ')
        assert prefix.endswith('% of the way there: ')
        stopped = model.check_path(
            [{'panda1': self.CLEAR_A, 'panda2': HOME},
             {'panda1': self.CLEAR_B, 'panda2': HOME}], first_violation=True)
        assert stopped.sample_index == stopped.samples_evaluated - 1, (
            'the early-exit call no longer reports a truncated count; the '
            'reason for evaluating the whole path may have gone away')


class TestG3G10TheChecksCostIsMeasured:
    """A budget assertion, not a performance test."""

    def test_the_longest_legal_travel_is_checked_inside_its_budget(self, capsys):
        """
        The supervisor stall an Apply can cause is a known quantity.

        The number is printed, so a reader of the test output sees what this
        host actually spends rather than only that it was under a ceiling.
        """
        workspace_model = pytest.importorskip('franka_workspace_model.model')
        directory = tempfile.mkdtemp()
        path = sample_cell.write_sample_cell(directory)
        if path is None:
            pytest.skip('the workspace model package installs no cell file here')
        model = workspace_model.CellModel.load(path, profile='dual')
        # The longest path the module will accept: the whole excursion budget
        # spent on one joint, which maximises the resampled sample count.
        goal = list(HOME)
        goal[0] += 2.8
        goal[4] -= min(2.8, defaults.APPLY_MAX_PATH_RAD - 2.8)
        started = time.perf_counter()
        try:
            travel.plan_travel(
                arm_id='panda1', q_held=HOME, q_measured=HOME,
                q_goal=tuple(goal),
                fence_lower=defaults.POLICY_POSITION_LOWER_RAD,
                fence_upper=defaults.POLICY_POSITION_UPPER_RAD,
                max_target_velocity=(0.1,) * defaults.JOINT_COUNT,
                model=model, co_arm_id='panda2', co_arm_q=HOME,
                enable_epoch=0, cancel_gen=0)
        except travel.TravelError as error:
            # A refusal is a legitimate outcome for this pose; the SUBJECT is
            # what the check cost, not whether the cell allows the move.
            assert error.reason_code in ('contact', 'too_far'), error.reason_code
        elapsed = time.perf_counter() - started
        with capsys.disabled():
            print('\nplan_travel over the longest legal path: '
                  '{:.3f} s (budget {:.2f} s)'.format(
                      elapsed, defaults.APPLY_CHECK_BUDGET_S))
        assert elapsed < defaults.APPLY_CHECK_BUDGET_S, elapsed


class TestG3G9NothingShippedNamesTheNotesTree:
    """The new files are inside the scans that already forbid it."""

    def test_the_new_files_are_in_the_scanned_set(self):
        """
        A gate nothing scans is a gate.

        ``test_review_regressions`` walks the shipped package; this asserts
        the two files G3 adds are in that walk, so their contents really are
        covered by the notes-tree and environment-prefix scans.
        """
        from test_review_regressions import walk_package_files
        scanned = {relative for relative, _text in walk_package_files()}
        assert 'franka_web/travel.py' in scanned
        assert 'franka_web/session.py' in scanned

    def test_the_new_module_reads_no_environment_at_all(self):
        """
        Belt to the braces above, in a form that scans no forbidden literal.

        The prefix and notes-tree scans are ``test_review_regressions``'s and
        must stay its alone -- a second copy of either needle would make this
        very file the thing that trips them.
        """
        for node in ast.walk(module_tree('travel.py')):
            if isinstance(node, ast.Attribute):
                assert node.attr != 'environ'
                assert node.attr != 'getenv'


def app_js():
    """Return the shipped console script's source text."""
    path = os.path.join(os.path.dirname(PACKAGE), 'static', 'app.js')
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def js_function(source, name):
    """Return one JavaScript function's text, by brace matching."""
    start = source.index('function {}('.format(name))
    depth = 0
    for index in range(source.index('{', start), len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError('{} never closes'.format(name))


class TestG3G8TheElevenConditions:
    """
    The Apply button is live under exactly eleven conditions, and no others.

    A STRUCTURAL gate, and the reason is worth stating rather than hiding:
    ``app.js`` is a classic script, not a module, so the browser harness --
    which imports the scene modules -- cannot import it, and there is no DOM
    rig here to drive the card in. The card's BEHAVIOUR is proved where it
    matters more anyway, on the wire, by the motion e2e battery: what a robot
    would receive is a stronger fact than what a button looks like.

    What this class pins is the part a wire test cannot see: WHICH frame key
    each condition reads, and WHICH of the two treatments it gets. Rows 1-6
    and 11 leave the button visible and disabled with a reason under it,
    because a control that vanishes teaches nothing; rows 7-10 HIDE it,
    exactly as the scene's Copy button is hidden until the ghost differs --
    there is nothing to apply, so an affordance would be a lie.
    """

    #: (row, the key the condition reads, the treatment its failure gets).
    ROWS = (
        (1, 'motion.available !== true', 'blocked'),
        (2, 'lockIsElsewhere(frame)', 'blocked'),
        (3, 'motion.enabled !== true', 'blocked'),
        (4, "motion.source !== 'ghost'", 'blocked'),
        (5, 'apply.note', 'blocked'),
        (6, 'travellingArm(frame)', 'blocked'),
        (7, 'ui.ghostShown[armId] !== true', 'hidden'),
        (8, 'ui.ghostDiffers[armId] !== true', 'hidden'),
        (9, 'scene.solved[index]', 'hidden'),
        (10, "verdict.status !== 'clear'", 'hidden'),
        (11, "ui.pending['apply:' + armId] === true", 'blocked'),
    )

    def verdict_source(self):
        """Return the decision function's own text, split at its two halves."""
        body = js_function(app_js(), 'applyVerdict')
        marker = '// 1-6 and 11:'
        assert marker in body, 'the two halves of the ladder are not marked'
        head, _, tail = body.partition(marker)
        return head, tail

    @pytest.mark.parametrize('row,needle,treatment', ROWS,
                             ids=[str(row[0]) for row in ROWS])
    def test_each_condition_is_read_and_gets_its_treatment(self, row, needle,
                                                           treatment):
        """One row, one key, one treatment -- and the treatment is asserted."""
        hidden_half, blocked_half = self.verdict_source()
        half = hidden_half if treatment == 'hidden' else blocked_half
        assert needle in half, (
            'row {} reads {!r} in the wrong half of the ladder'.format(
                row, needle))

    def test_the_hidden_rows_are_decided_before_the_blocked_ones(self):
        """
        Nothing to apply outranks something stopping it.

        An arm with no ghost drawn must not be told to enable itself: the
        honest answer is that there is nothing to apply yet, and a reason
        under an invisible button is a reason nobody reads.
        """
        hidden_half, _blocked = self.verdict_source()
        assert "return {mode: 'hidden'}" in hidden_half
        assert "mode: 'blocked'" not in hidden_half

    def test_a_travelling_arm_short_circuits_every_other_row(self):
        """While a travel runs, the panel shows progress and Cancel."""
        body = js_function(app_js(), 'applyVerdict')
        first = body.index("apply.state === 'travelling'")
        assert first < body.index('ui.ghostShown'), (
            'the travelling branch is not the first thing decided')

    def test_the_checkers_sentence_is_rendered_and_never_authored(self):
        """
        Row 5 shows the SERVER's words: the page holds no copy of them.

        The four checker sentences have one author, in the checker module, and
        this is the one place they reach a screen through the arm card.
        """
        _hidden, blocked = self.verdict_source()
        assert 'reason: apply.note' in blocked
        source = app_js()
        for sentence in ('The workspace model is not installed',
                         'The cell model could not be loaded',
                         'was built for a different robot description'):
            assert sentence not in source, sentence

    def test_cancel_is_not_routed_through_the_pending_gate(self):
        """
        A stop control that can be greyed out is not a stop control.

        Every other action funnels through ``runAction``, which disables its
        control for the life of the request. Cancel deliberately does not: it
        is idempotent by design, so a double press is free.
        """
        handler = app_js()
        start = handler.index("'apply-cancel': function")
        end = handler.index('\n  },', start)
        # Comments out: the handler EXPLAINS why it does not use the pending
        # gate, and the explanation must not be what satisfies the gate.
        body = '\n'.join(line for line in handler[start:end].splitlines()
                         if not line.strip().startswith('//'))
        assert 'runAction' not in body
        assert "api('POST'" in body
        assert 'withLock(' in body

    def test_the_source_segment_offers_exactly_three_values(self):
        """A fourth would be a source the server's closed set does not know."""
        source = app_js()
        assert "['jog', 'external', 'ghost']" in source
        assert ("var SOURCE_LABELS = {jog: 'Jog', external: 'External', "
                "ghost: 'Ghost'};") in source

    def test_the_page_compares_the_schema_version_exactly_once(self):
        """
        A second comparison is a second place to forget on the next bump.

        Asserted here as well as in the gripper e2e, because this build is the
        one that moved the number.
        """
        import re
        versions = re.findall(r'schema_version !== (\d+)', app_js())
        assert versions == [str(defaults.SCHEMA_VERSION)], versions
