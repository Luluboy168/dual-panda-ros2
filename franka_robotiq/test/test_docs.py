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
Keep this package's operator documents honest.

Documents rot in ways code does not: a service gets renamed and the runbook
that names it stays green forever. Every test here pins a document against a
source that can move underneath it -- the node's own surface, the driver's
message text, the sibling packages this package cites -- or against a rule the
documents must not break.

Pure standard library plus pytest. It imports no ROS and nothing from
``franka_robotiq``, so it runs with nothing sourced.
"""

from pathlib import Path
import re

import pytest

PKG = Path(__file__).resolve().parents[1]          # franka_robotiq/
REPO = PKG.parent                                   # repository root, when present
SELF = Path(__file__).resolve()

# Assembled from fragments so this file does not itself contain the tokens it
# bans. The self-exclusion in _package_text_files() is then belt-and-braces
# rather than load-bearing, and the removal greps in the build checklist stay
# clean even before their --exclude flag is applied.
#
# These five are exactly the contract's rule-1 list. A sixth token -- a bare
# 'notes/' -- is deliberately NOT here: it is in neither the contract nor the
# checklist, and it would ban a substring that any innocent path ending in a
# notes directory could carry. A test that bans a token the checklist never
# checks is not stricter, it is divergent.
BANNED = ('multipanda_ros2_jazzy' + '_notes',
          'plans/' + 'gripper',
          'plans/' + 'post_mvp',
          'WORKSPACE_' + 'MODEL_',
          'CELL_' + 'MODEL_DRAFT')

PIP = ('pip ' + 'install', 'pip3 ' + 'install')

# The complete set of files this part owns. Listed so that a deletion during a
# later refactor is caught here rather than at install time.
OWNED = (
    'README.md',
    'doc/CAPSULE.md',
    'doc/MOUNTING.md',
    'doc/SERIAL_BINDING.md',
    'doc/UNITS.md',
    'udev/99-franka-robotiq.rules',
    'test/test_docs.py',
)

# The operator-facing prose. udev rules and this file are not doc surface.
DOCS = ('README.md', 'doc/CAPSULE.md', 'doc/MOUNTING.md',
        'doc/SERIAL_BINDING.md', 'doc/UNITS.md')

# The three names section 3.1 deliberately does NOT ship. They appear in
# doc/UNITS.md and README.md precisely to say they are absent, so a blanket
# "every quoted name exists in node.py" check would fail on correct documents.
# The exemption is written as a second assertion rather than as a hole -- see
# test_docs_quote_only_real_ros_names.
ABSENT_BY_DESIGN = ('homing', 'move', 'grasp')

NODE = PKG / 'franka_robotiq' / 'node.py'
UNITS_PY = PKG / 'franka_robotiq' / 'units.py'
DISCOVERY = PKG / 'franka_robotiq' / 'discovery.py'
DRIVER = PKG / 'franka_robotiq' / 'driver.py'


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _package_text_files():
    """Yield (path, text) for every readable text file under the package."""
    for path in sorted(PKG.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        if path.resolve() == SELF:
            continue                                # the scanner is not doc surface
        try:
            yield path, path.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            continue                                # binaries are not doc surface


def _read(relative):
    return (PKG / relative).read_text(encoding='utf-8')


def _repo_is_present():
    """Say whether this package sits inside the source tree, not an install."""
    return (REPO / 'franka_hardware').is_dir()


def _require_repo():
    if not _repo_is_present():
        pytest.skip('the repository root is not present; this is an installed tree')


_FENCE = re.compile(r'^\s*```\s*([A-Za-z0-9_+-]*)\s*$')
REFUSAL = 'refusal'


def _fenced_blocks(text):
    """Yield (info_string, block_text) for every fenced block in ``text``."""
    info = None
    buf = []
    for line in text.splitlines():
        match = _FENCE.match(line)
        if match and info is None:
            info, buf = match.group(1), []
        elif match:
            yield info, '\n'.join(buf)
            info = None
        elif info is not None:
            buf.append(line)


def _without_refusal_blocks(text):
    """``text`` with every ```refusal fenced block removed, fences included."""
    kept = []
    inside = False
    for line in text.splitlines():
        match = _FENCE.match(line)
        if match and not inside and match.group(1) == REFUSAL:
            inside = True
            continue
        if match and inside:
            inside = False
            continue
        if not inside:
            kept.append(line)
    return '\n'.join(kept)


def _swap_arms(text):
    return re.sub(r'panda([12])',
                  lambda m: 'panda2' if m.group(1) == '1' else 'panda1', text)


# --------------------------------------------------------------------------
# rules that bind every shipped file
# --------------------------------------------------------------------------

def test_no_notes_paths():
    """
    Nothing shipped may name the planning tree.

    Mutation proof: put one of the banned paths into README.md and this test
    fails. The converse matters just as much and the old form could not give
    it -- with the self-exclusion in place, this file's own text does not trip
    it, which is why the tokens above are assembled from fragments.
    """
    hits = [(str(path.relative_to(PKG)), token)
            for path, text in _package_text_files()
            for token in BANNED if token in text]
    assert hits == [], 'planning-tree references in shipped files: {}'.format(hits)


def test_no_pip_instructions():
    """No shipped file may tell anyone to install a dependency with pip."""
    hits = [(str(path.relative_to(PKG)), token)
            for path, text in _package_text_files()
            for token in PIP if token in text.lower()]
    assert hits == [], 'pip instructions in shipped files: {}'.format(hits)


def test_docs_are_installed_set():
    """Every file this part owns still exists."""
    missing = [name for name in OWNED if not (PKG / name).is_file()]
    assert missing == [], 'owned files are gone: {}'.format(missing)


# --------------------------------------------------------------------------
# the udev rule
# --------------------------------------------------------------------------

def test_udev_creates_no_alias():
    """
    The rule file grants access and silences ModemManager. Nothing else.

    A stable /dev alias would be a second naming system that can disagree with
    /dev/serial/by-id, which is the one the configuration names.
    """
    rules = _read('udev/99-franka-robotiq.rules')
    assert 'SYMLINK' not in rules, 'the udev rule creates a device alias'
    assert 'NAME=' not in rules, 'the udev rule renames a device'
    assert 'ID_MM_DEVICE_IGNORE' in rules, 'ModemManager is not kept off the link'
    assert 'dialout' in rules, 'group access is not granted'


# --------------------------------------------------------------------------
# the capsule
# --------------------------------------------------------------------------

def _capsule_block():
    for info, block in _fenced_blocks(_read('doc/CAPSULE.md')):
        if 'robotiq_2f85_v0' in block:
            return block
    raise AssertionError('doc/CAPSULE.md carries no paste-ready capsule block')


def _value_of(block, key):
    """
    Return the value on ``key``'s line, with any trailing comment stripped.

    The mandated block carries explanatory comments on the very lines this
    test reads -- the arithmetic behind ``radius``, the page behind ``d`` --
    so a rule phrased as "no digit on this line" fails on the content it
    guards. The value is what stands before the first '#'.
    """
    for line in block.splitlines():
        stripped = line.strip()
        if stripped.startswith(key):
            return stripped[len(key):].split('#', 1)[0].strip()
    raise AssertionError('no {} line in the capsule block'.format(key))


def test_capsule_blanks_unfilled():
    """
    The two mounting-day blanks are still blank, and the rest is computed.

    This is the test that stops a helpful future edit from filling ``a`` and
    ``b`` with a plausible number. It is aimed only at the values that are
    genuinely unknown; the radius the desk settled is asserted to be a real
    number with its page beside it.
    """
    block = _capsule_block()
    assert _value_of(block, 'a:') == '[0.0, 0.0, null]'
    assert _value_of(block, 'b:') == '[0.0, 0.0, null]'
    assert float(_value_of(block, 'radius:')) > 0.0
    assert float(_value_of(block, 'containment_margin:')) > 0.0
    assert 'derivation_status: to_be_derived' in block
    assert 'TBC-6' in block, 'the coupling measurement is no longer named'

    lines = block.splitlines()
    index = next(i for i, line in enumerate(lines) if line.strip().startswith('radius:'))
    context = '\n'.join(lines[max(0, index - 3):index])
    assert re.search(r'page 119', context), (
        "the depth's provenance no longer rides beside the number it justifies")


def test_capsule_provenance_is_specific():
    """
    The provenance string names a revision, a section, a figure and a page.

    "Robotiq datasheet" alone is not provenance; it cannot be re-checked.
    """
    block = _capsule_block()
    provenance = block.split('provenance:', 1)
    assert len(provenance) == 2, 'the capsule block carries no provenance string'
    text = ' '.join(provenance[1].split())
    for pattern, what in ((r'2018/05/23', 'the document revision'),
                          (r'section \d', 'a section number'),
                          (r'Fig\. \d|table', 'a figure or table'),
                          (r'page \d+', 'a page number')):
        assert re.search(pattern, text), 'the provenance omits {}'.format(what)


# --------------------------------------------------------------------------
# the runbook's citations
# --------------------------------------------------------------------------

def test_mounting_records_inertia_provenance():
    """
    The inertia matrix keeps its revision, figure and page.

    This cannot detect a transcription error -- only a second reader can --
    but it stops the source from being dropped by a later edit, which is the
    failure that makes a transcription error undetectable.
    """
    lines = _read('doc/MOUNTING.md').splitlines()
    exact = [i for i, line in enumerate(lines)
             if all(value in line for value in ('0.002768', '0.003149', '0.000564'))]
    assert exact, 'doc/MOUNTING.md no longer prints the exact inertia matrix'
    for index in exact:
        context = ' '.join(lines[index:index + 2])
        assert 'Fig. 6-20' in context and 'page 142' in context, (
            'the inertia matrix on line {} has lost its source'.format(index + 1))


def test_set_load_citation_is_real():
    """The runbook's load-declaration citation still matches this repository."""
    _require_repo()
    srv = REPO / 'franka_msgs' / 'srv' / 'SetLoad.srv'
    assert srv.is_file(), 'franka_msgs/srv/SetLoad.srv is gone'
    fields = srv.read_text(encoding='utf-8')
    for field in ('mass', 'center_of_mass', 'load_inertia'):
        assert field in fields, 'SetLoad.srv no longer declares {}'.format(field)

    server = REPO / 'franka_hardware' / 'src' / 'real' / 'franka_param_service_server.cpp'
    assert server.is_file(), 'the param service server is gone'
    assert '"~/set_load"' in server.read_text(encoding='utf-8'), (
        'the load-declaration service was renamed')

    mounting = _read('doc/MOUNTING.md')
    for arm in ('panda1', 'panda2'):
        name = '/{}_param_service_server/set_load'.format(arm)
        assert name in mounting, 'doc/MOUNTING.md does not spell {}'.format(name)


def test_state_topic_citation_is_real():
    """
    Both session shapes, because the runbook prints both.

    A two-arm session spawns per-arm broadcasters; a one-arm session spawns
    one under a fixed name that carries no arm id. Checking only the two-arm
    file is what would let the runbook be wrong for every single-arm session
    while this test stayed green. It is the one citation an operator
    copy-pastes under time pressure.
    """
    _require_repo()
    mounting = _read('doc/MOUNTING.md')
    launch = REPO / 'franka_bringup' / 'launch' / 'real'

    dual = (launch / 'dual_franka.launch.py')
    assert dual.is_file(), 'the two-arm launch file is gone'
    dual_text = dual.read_text(encoding='utf-8')
    for arm in ('panda1', 'panda2'):
        spawner = 'franka_{}_robot_state_broadcaster'.format(arm)
        assert spawner in dual_text, '{} is no longer spawned'.format(spawner)
        assert '/{}/robot_state'.format(spawner) in mounting, (
            'doc/MOUNTING.md does not quote the {} topic'.format(spawner))

    single = (launch / 'one_arm_franka.launch.py')
    assert single.is_file(), 'the one-arm launch file is gone'
    assert 'franka_robot_state_broadcaster' in single.read_text(encoding='utf-8'), (
        'the one-arm broadcaster was renamed')
    assert '/franka_robot_state_broadcaster/robot_state' in mounting, (
        'doc/MOUNTING.md does not quote the one-arm topic; a single-arm session '
        'would follow it into silence')


# --------------------------------------------------------------------------
# the per-arm house rule
# --------------------------------------------------------------------------

def _symmetry_tokens(text):
    """
    Collect example commands and configuration keys, with two exemptions.

    (a) Quoted refusal messages name one arm because the software printed it
        that way; balancing them would distort a verbatim quote, which
        test_refusal_messages_match_source enforces. They are skipped by their
        fence marker.
    (b) The runbook's bring-up table, whose commands stay INLINE in table
        cells so that step 8.9 can collapse the second arm into "repeat
        8.2-8.8". This extractor reads fenced command blocks only, which is
        what makes that exemption work -- and the runbook states the rule at
        the table so a later editor who fences those commands knows the
        exemption has to move with them.
    """
    tokens = set()
    for info, block in _fenced_blocks(text):
        if info == REFUSAL:
            continue
        for line in block.splitlines():
            if line.strip().startswith('ros2 '):
                tokens.add(' '.join(line.split()))
    tokens.update(re.findall(r'grippers\.panda[12]\.[a-z_]+',
                             _without_refusal_blocks(text)))
    return tokens


def test_per_arm_symmetry():
    """
    Every example that names one arm has a twin that names the other.

    A symmetry rule, not a word count: counting arm names is unsatisfiable
    against content this package mandates, and satisfying a count would mean
    distorting the verbatim refusals.
    """
    for name in DOCS:
        tokens = _symmetry_tokens(_read(name))
        swapped = {_swap_arms(token) for token in tokens}
        assert swapped == tokens, '{} has unbalanced examples: {}'.format(
            name, sorted(swapped.symmetric_difference(tokens)))


# --------------------------------------------------------------------------
# seams with the other parts (deferred until they land)
# --------------------------------------------------------------------------

def test_docs_quote_only_real_ros_names():
    """
    The documents cannot name a service nobody serves.

    And the three names that are absent by design really are absent: the
    allow-list proves a fact instead of punching a hole.
    """
    if not NODE.is_file():
        pytest.skip('franka_robotiq/node.py has not landed yet')
    node_text = NODE.read_text(encoding='utf-8')

    quoted = set()
    for name in DOCS:
        text = _read(name)
        quoted.update(re.findall(r'`~/([a-z_]+)`', text))
        quoted.update(re.findall(r'/panda[12]_robotiq/([a-z_]+)', text))

    unknown = sorted(name for name in quoted
                     if name not in ABSENT_BY_DESIGN and name not in node_text)
    assert unknown == [], 'documents name endpoints node.py does not serve: {}'.format(
        unknown)

    # The absence half is written against the SERVICE form, with the '~/'
    # prefix, and that is not a stylistic choice. A bare substring check for
    # 'move' is red against correct code: node.py legitimately carries the
    # mandated 'moving' status key, the "...while moving." abort sentence and
    # the "Nothing moves until a human calls ~/reactivate" startup log.
    for name in ABSENT_BY_DESIGN:
        service = '~/' + name
        assert service not in node_text, (
            '{} is declared in node.py; the surface ships without it '
            'deliberately, and two documents say so'.format(service))


def test_units_constants_match_units_py():
    """The endpoints in doc/UNITS.md are the endpoints units.py converts with."""
    if not UNITS_PY.is_file():
        pytest.skip('franka_robotiq/units.py has not landed yet')
    source = UNITS_PY.read_text(encoding='utf-8')
    doc = _read('doc/UNITS.md')
    for constant in ('85.0', '20.0', '150.0', '235.0'):
        assert constant in source, 'units.py no longer carries {}'.format(constant)
        assert constant in doc, 'doc/UNITS.md no longer carries {}'.format(constant)


# The variable parts of a refusal message: an arm id, a device name, an
# adapter serial, a configuration key, and the pinned serial-line constants.
# Everything between them is text a document quotes word for word, and the
# operator's ability to match a message against doc/SERIAL_BINDING.md rests on
# it. Longest alternatives first, so a device name is consumed whole.
_VARIABLE = re.compile(
    r'/dev/serial/by-id/\S*'
    r'|usb-\S*'
    r'|\bD[0-9A-Z]{7}\b'
    r'|panda[12]'
    r'|serial_id|usb_path'
    r'|slave ID \d+'
    r'|115200'
    r'|8N1')

MIN_FRAGMENT = 16


def _normalise(text):
    r"""
    Flatten a message so a document quote and a format template compare.

    Three things stand between the two and none of them is content: line
    breaks, Python's implicit concatenation of adjacent string literals, and
    the quoting itself. Escapes become spaces, adjacent literals are joined,
    and quote characters and backslashes are dropped -- so a sentence written
    across three source lines reads as one sentence, and ``Robotiq\\'s`` and
    ``Robotiq's`` are the same word.
    """
    text = text.replace('\\n', ' ').replace('\\t', ' ')
    text = ' '.join(text.split())
    text = re.sub(r"['\"]\s*\+?\s*[fFrRbBuU]{0,2}['\"]", '', text)
    text = re.sub(r"[\\'\"]", '', text)
    return ' '.join(text.split())


def _invariant_fragments(block):
    """Split out the parts of a refusal that no interpolation can change."""
    return [part.strip() for part in _VARIABLE.split(_normalise(block))
            if len(part.strip()) >= MIN_FRAGMENT]


def _message_source(paths):
    """Read the message-owning sources, normalised the same way."""
    return _normalise(' '.join(path.read_text(encoding='utf-8') for path in paths))


def test_refusal_messages_match_source():
    """
    The quoted refusals still match the code that prints them.

    This is the seam the anti-swap operator story rests on, and it is the one
    with a different owner at each end: the message text lives in the package,
    doc/SERIAL_BINDING.md quotes it word for word, and nothing else would
    notice a rewording.

    Gated on both discovery.py and node.py: three of the four messages are
    composed in discovery.py, but the connect-time serial re-verification is
    driven from node.py, so a discovery-only gate would un-skip in the window
    between the two merges and fail on a file that does not exist yet.
    driver.py joins the haystack when it exists, because the timeout message
    is emitted on the transport boundary and may legitimately be composed
    there.
    """
    for required in (DISCOVERY, NODE):
        if not required.is_file():
            pytest.skip('franka_robotiq/{} has not landed yet'.format(required.name))

    haystack = _message_source([path for path in (DISCOVERY, NODE, DRIVER)
                                if path.is_file()])
    missing = []
    for info, block in _fenced_blocks(_read('doc/SERIAL_BINDING.md')):
        if info != REFUSAL:
            continue
        for fragment in _invariant_fragments(block):
            if fragment not in haystack:
                missing.append(fragment)
    assert missing == [], (
        'doc/SERIAL_BINDING.md quotes text no source prints any more. Either '
        'the message was reworded and the document must follow it word for '
        'word, or the document drifted. Fragments not found: {}'.format(missing))
