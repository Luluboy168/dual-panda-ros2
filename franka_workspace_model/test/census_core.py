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
The acceptance census, as a function so that its numbers can be re-measured.

The test file next door pins the constants and asserts the criteria; this file
is the machinery, kept separate for one reason: a number that can only be
produced by running the whole test suite is a number nobody re-measures.

WHAT THE CENSUS IS FOR.  It is the test that would have caught
``fix/true-clearance``, which made the fence looser and passed every test in the
package.  Every criterion here is designed so that it cannot be satisfied by a
fence that refuses everything, cannot be satisfied by a fence that accepts
everything, and cannot be weakened without the weakening appearing as a diff in
a named constant.

IT LOOKS AT ALL THIRTY-SIX LINK PAIRS, not the sixteen the SRDF leaves enabled.
An allowed-collision matrix hole is invisible to a census that only asks about
the pairs the matrix already allows, and this description has one:
``link2``/``link6`` is disabled as ``reason="Never"`` and it is not never.
"""

import math
import time

import mesh_oracle

import numpy as np


#: Pairs that reach contact, or nearly, because the two castings are joined at a
#: joint housing.  Each is written with the minimum measured over the single
#: joint it depends on, so that "adjacent" is a measurement and not an opinion.
#: Four of the eight genuinely reach 0.0000 mm; the other four never do and are
#: exempt anyway, because a casting touching its own joint housing is not a
#: collision.
ADJACENT_BY_CONSTRUCTION = {
    ('link0', 'link1'): 0.0009931,
    ('link1', 'link2'): 0.0,
    ('link2', 'link3'): 0.0009729,
    ('link3', 'link4'): 0.0,
    ('link4', 'link5'): 0.0009905,
    ('link5', 'link6'): 0.0,
    ('link6', 'link7'): 0.0006027,
    ('link7', 'link8'): 0.0,
}
#: NOT adjacent, and exempt for a different reason that is written down: the
#: link8 body is an ENVELOPE built from URDF primitives, and it overlaps link6
#: because of that rather than because the metal does.  Giving link8 a real body
#: later removes this exemption instead of leaving it unexamined.
ENVELOPE_EXEMPT = {('link6', 'link8'): (
    'link8_flange is the hull of the URDF primitives at metal radius; it '
    'provably overlaps link6 and link7, which is a property of the envelope '
    'and not of the metal. Retire this exemption when the flange is measured '
    'with a caliper.')}

#: A reported distance below this is treated as "too close to call apart from
#: contact" and is counted separately from a certain contact.  It is two orders
#: of magnitude below the tightest margin in the model, so a value here is a
#: finding under every criterion either way.
UNRESOLVED_CONTACT_M = 1e-4

LINKS = ('link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7',
         'link8')


def link_pairs():
    """Return all 36 intra-arm link pairs, adjacent ones included."""
    return tuple((LINKS[i], LINKS[j])
                 for i in range(len(LINKS)) for j in range(i + 1, len(LINKS)))


def non_adjacent_pairs():
    """Return the 28 pairs criterion (d) covers: everything not adjacent."""
    return tuple(pair for pair in link_pairs()
                 if pair not in ADJACENT_BY_CONSTRUCTION)


def cross_arm_pairs():
    """Return the 81 cross-arm link pairs, excluding the two link0 castings."""
    return tuple((first, second) for first in LINKS for second in LINKS)


def joint_limits(oracle):
    """
    Read the joint box from the ARTEFACT's own parent_joint limits.

    Never a restated constant, and never ``mj_dual.xml``: that file declares
    ``j6`` in [0.5445, 4.5169], which are the FR3's ranges on a Panda chain,
    while the URDF, the joint-limit policy and the loaded model all agree on
    [-0.0175, 3.7525].  A census drawn over the wrong box would measure a robot
    that is not in this lab.
    """
    lower = [None] * 7
    upper = [None] * 7
    for link, joint in oracle.joints.items():
        if joint['type'] != 'revolute':
            continue
        if not link.startswith(oracle.arm_ids[0] + '_'):
            continue
        index = int(joint['name'].rsplit('joint', 1)[1]) - 1
        lower[index] = float(joint['limit_lower'])
        upper[index] = float(joint['limit_upper'])
    if any(value is None for value in lower + upper):
        raise mesh_oracle.OracleError(
            'the artefact does not declare all seven joint limits')
    return np.array(lower), np.array(upper)


def _pair_table(oracle, configuration, thresholds,
                gate=mesh_oracle.ORACLE_EXACT_GATE_M):
    """
    Every link pair's true distance at one configuration, from the oracle.

    Above ``gate`` the entry is a certified LOWER BOUND rather than an exact
    distance; every margin in this model is at most 50 mm and
    ``swept_path_extra`` is 10 mm, so no census decision turns on the difference
    between 61 mm and 400 mm, and a bound is conservative in the direction that
    matters.
    """
    placed = oracle.place(configuration)
    capsules = oracle.capsule_bounds(configuration)
    self_table = {}
    for arm_id in oracle.arm_ids:
        for pair in link_pairs():
            self_table[(arm_id, pair)] = oracle.link_pair_distance(
                placed, (arm_id, pair[0]), (arm_id, pair[1]), gate, capsules,
                thresholds)
    cross_table = {}
    first_arm, second_arm = oracle.arm_ids
    for pair in cross_arm_pairs():
        cross_table[pair] = oracle.link_pair_distance(
            placed, (first_arm, pair[0]), (second_arm, pair[1]), gate, capsules,
            thresholds)
    return self_table, cross_table, placed


def enabled_link_pairs(model):
    """Return the link pairs the loaded model actually evaluates, per arm."""
    enabled = {}
    for volume_a, volume_b, arm_id in model._intra_pairs:
        link_a = model._volume_index[volume_a][0][len(arm_id) + 1:]
        link_b = model._volume_index[volume_b][0][len(arm_id) + 1:]
        enabled.setdefault(arm_id, set()).add(tuple(sorted((link_a, link_b))))
    return enabled


def in_force_self_margins(model):
    """Return every self margin actually in force, at link-pair granularity."""
    base = model._margins['self_collision']
    margins = {}
    for arm_id, pairs in enabled_link_pairs(model).items():
        for pair in pairs:
            margins[(arm_id, pair)] = getattr(model, '_pair_margins', {}).get(
                (arm_id,) + pair, base)
    return margins


def run_census(model, oracle, draws, seed, gate=mesh_oracle.ORACLE_EXACT_GATE_M,
               progress=None):
    """
    Draw uniformly over the joint box and record what the fence did about it.

    Returns a report dictionary.  Nothing here asserts; the test next door does
    that, so that a re-measurement is a script run and not a suite run.
    """
    lower, upper = joint_limits(oracle)
    rng = np.random.default_rng(seed)
    arm_ids = oracle.arm_ids
    margins = in_force_self_margins(model)
    cross_margin = model._margins['cross_arm']
    # The values the census DECIDES against, and only those.  A pair whose
    # bracket straddles one of them is escalated rather than decided.
    #
    # Zero is deliberately NOT here.  A bracket that straddles zero means the
    # two bodies are somewhere between touching and a few millimetres apart -
    # which already puts them far inside every margin in this model, so the
    # verdict is settled without spending a deep solve on the last digit.  What
    # the census reports is therefore a CERTAIN contact (upper bound at or below
    # zero) and, separately, an unresolved one (the bracket contains zero); the
    # criteria are asserted against both, so nothing is decided on a bracket
    # that does not decide it.
    thresholds = tuple(sorted(set(list(margins.values()) + [cross_margin])))

    accepted = 0
    self_cross_accepted = 0
    oracle_calls = 0
    accepted_below_margin = 0
    accepted_metal_at_or_below_zero = 0
    accepted_metal_unresolved_at_zero = 0
    accepted_non_adjacent_contact = 0
    accepted_non_adjacent_unresolved = 0
    accepted_cross_below_margin = 0
    tightest_accepted_enabled = math.inf
    worst_accepted_below_margin = math.inf
    disabled_report = {}
    hazards = []
    started = time.perf_counter()

    for index in range(draws):
        configuration = {arm_id: rng.uniform(lower, upper) for arm_id in arm_ids}
        query = {arm_id: list(values) for arm_id, values in configuration.items()}
        verdict = model.check_configuration(query)
        self_table, cross_table, oracle_placed = _pair_table(
            oracle, configuration, thresholds, gate)
        oracle_calls += len(self_table) + len(cross_table)

        # The SELF-AND-CROSS statistic, recomputed from the oracle's own table
        # with containment, the pedestal, environment and keep-out excluded.  It
        # is a SEPARATE quantity from the model's verdict and it is named
        # separately, so that a containment regression cannot mask a margin
        # regression or the reverse.
        self_cross_ok = True
        for (arm_id, pair), value in self_table.items():
            if (arm_id, pair) in margins and value < margins[(arm_id, pair)]:
                self_cross_ok = False
                break
        if self_cross_ok:
            for value in cross_table.values():
                if value < cross_margin:
                    self_cross_ok = False
                    break
        if self_cross_ok:
            self_cross_accepted += 1

        # The disabled-pair report: a finding, on every draw, accepted or not.
        for (arm_id, pair), value in self_table.items():
            if pair in ADJACENT_BY_CONSTRUCTION or pair in ENVELOPE_EXEMPT:
                continue
            if (arm_id, pair) in margins:
                continue
            record = disabled_report.setdefault(pair, {'minimum': math.inf,
                                                       'below_margin': 0,
                                                       'contacts': 0})
            record['minimum'] = min(record['minimum'], value)
            if value < model._margins['self_collision']:
                record['below_margin'] += 1
            if value <= 0.0:
                record['contacts'] += 1

        if not verdict.ok:
            if progress is not None and index % progress == 0:
                progress_report(index, draws, started)
            continue
        accepted += 1

        tightest = math.inf
        tightest_key = None
        for (arm_id, pair), value in self_table.items():
            if (arm_id, pair) not in margins:
                continue
            if value < tightest:
                tightest, tightest_key = value, (arm_id, pair)
            if value < margins[(arm_id, pair)]:
                accepted_below_margin += 1
                # A number the acceptance table shows a person is re-solved at
                # full precision, never left at the cheap bracket's upper end.
                refined = oracle.refine(
                    oracle_placed, (arm_id, pair[0]), (arm_id, pair[1]))
                worst_accepted_below_margin = min(worst_accepted_below_margin,
                                                  refined)
                hazards.append((index, arm_id, pair, refined,
                                margins[(arm_id, pair)]))
            if value <= 0.0:
                accepted_metal_at_or_below_zero += 1
            elif value < UNRESOLVED_CONTACT_M:
                accepted_metal_unresolved_at_zero += 1
        if tightest_key is not None and tightest < 0.030:
            tightest = oracle.refine(oracle_placed, (tightest_key[0],
                                                     tightest_key[1][0]),
                                     (tightest_key[0], tightest_key[1][1]))
        tightest_accepted_enabled = min(tightest_accepted_enabled, tightest)
        for value in cross_table.values():
            if value < cross_margin:
                accepted_cross_below_margin += 1
        for (arm_id, pair), value in self_table.items():
            if pair in ADJACENT_BY_CONSTRUCTION or pair in ENVELOPE_EXEMPT:
                continue
            if value <= 0.0:
                accepted_non_adjacent_contact += 1
            elif value < UNRESOLVED_CONTACT_M:
                accepted_non_adjacent_unresolved += 1
        if progress is not None and index % progress == 0:
            progress_report(index, draws, started)

    return {
        'draws': draws,
        'seed': seed,
        'accepted': accepted,
        'accepted_fraction': accepted / draws,
        'self_cross_accepted': self_cross_accepted,
        'self_cross_fraction': self_cross_accepted / draws,
        'oracle_calls': oracle_calls,
        'accepted_below_margin': accepted_below_margin,
        'worst_accepted_below_margin': worst_accepted_below_margin,
        'accepted_metal_at_or_below_zero': accepted_metal_at_or_below_zero,
        'accepted_metal_unresolved_at_zero': accepted_metal_unresolved_at_zero,
        'accepted_non_adjacent_contact': accepted_non_adjacent_contact,
        'accepted_non_adjacent_unresolved': accepted_non_adjacent_unresolved,
        'accepted_cross_below_margin': accepted_cross_below_margin,
        'tightest_accepted_enabled': tightest_accepted_enabled,
        'disabled_report': disabled_report,
        'hazards': hazards,
        'seconds': time.perf_counter() - started,
        'oracle_deep_solves': oracle.deep,
        'oracle_program_escalations': oracle.escalations,
        'smallest_in_force_self_margin': min(margins.values()) if margins else None,
    }


def progress_report(index, draws, started):
    """Print how far the census has got; CI shows a live line, not a hang."""
    elapsed = time.perf_counter() - started
    print('census {}/{} draws, {:.1f} s elapsed'.format(index, draws, elapsed),
          flush=True)


def format_report(report):
    """Render the census report as the lines the test emits to its log."""
    lines = [
        'census: {} draws, seed {}, {:.1f} s'.format(
            report['draws'], report['seed'], report['seconds']),
        '  whole-fence accepted      {} ({:.2%})'.format(
            report['accepted'], report['accepted_fraction']),
        '  self-and-cross accepted   {} ({:.2%})'.format(
            report['self_cross_accepted'], report['self_cross_fraction']),
        '  oracle link-pair queries  {}'.format(report['oracle_calls']),
        '  accepted below margin     {}'.format(report['accepted_below_margin']),
        '  accepted at metal <= 0    {}'.format(
            report['accepted_metal_at_or_below_zero']),
        '  accepted within 0.1 mm    {}'.format(
            report['accepted_metal_unresolved_at_zero']),
        '  accepted cross < margin   {}'.format(
            report['accepted_cross_below_margin']),
        '  tightest accepted metal   {:.4f} mm'.format(
            report['tightest_accepted_enabled'] * 1000.0),
    ]
    if report['worst_accepted_below_margin'] < math.inf:
        lines.append('  worst accepted hazard     {:.4f} mm'.format(
            report['worst_accepted_below_margin'] * 1000.0))
    lines.append('  DISABLED, NON-ADJACENT PAIRS (a report, not a failure):')
    for pair in sorted(report['disabled_report']):
        record = report['disabled_report'][pair]
        lines.append(
            '    {:>7}-{:<7} min {:9.4f} mm  below 20 mm on {:5d} draws  '
            'contacts {}'.format(pair[0], pair[1], record['minimum'] * 1000.0,
                                 record['below_margin'], record['contacts']))
    return '\n'.join(lines)
