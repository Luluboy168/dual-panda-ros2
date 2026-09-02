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
The anti-swap rule, at mutation level, rooted in a temporary directory.

This is the safety-relevant file of the package: a wrong binding is not an
inconvenience, it is one arm commanding the other arm's gripper. Every branch
below has its own case.

**These tests prove the four identity functions, not the refusal.** Closing
the port on a mismatch and commanding nothing is the node's obligation, and
the test that proves it lives with the node. This module must not be read as
covering that.
"""

import ast
import inspect
import os

from franka_robotiq import discovery

import pytest

REAL_NAME = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'
OTHER_NAME = 'usb-FTDI_FT230X_Basic_UART_D3091K4W-if00-port0'


@pytest.fixture()
def by_id(tmp_path):
    """Build a by-id root holding one symlink to a real pty."""
    root = tmp_path / 'by-id'
    root.mkdir()
    master, slave = os.openpty()
    device = os.ttyname(slave)
    os.symlink(device, str(root / REAL_NAME))
    try:
        yield str(root), device
    finally:
        os.close(master)
        os.close(slave)


def test_resolve_returns_the_symlink_target(by_id):
    """The configured name resolves to the device it points at."""
    root, device = by_id
    assert discovery.resolve(REAL_NAME, root=root) == device


def test_resolve_by_path_uses_the_by_path_root(by_id):
    """The by-path fallback is the same resolution against the other root."""
    root, device = by_id
    assert discovery.resolve_by_path(REAL_NAME, root=root) == device


@pytest.mark.parametrize('name', [
    'sub/dir-name',
    'usb-FTDI_*-if00-port0',
    'usb-FTDI_?-if00-port0',
    '',
    '.',
    '..',
    '/dev/ttyUSB0',
])
def test_resolve_refuses_anything_that_is_not_a_bare_entry_name(name, by_id):
    """
    Separators, patterns, dot names and absolute paths are all refused.

    The absolute-path refusal is what keeps emulator mode out of the binding
    path: a fake gripper's device path goes straight to the driver and never
    through this module.
    """
    root, _ = by_id
    with pytest.raises(discovery.BindingError):
        discovery.resolve(name, root=root)


def test_resolve_refuses_a_symlink_pointing_outside_dev(tmp_path):
    """A symlink to a regular file elsewhere is not a device."""
    root = tmp_path / 'by-id'
    root.mkdir()
    decoy = tmp_path / 'not-a-device'
    decoy.write_text('')
    os.symlink(str(decoy), str(root / REAL_NAME))
    with pytest.raises(discovery.BindingError) as caught:
        discovery.resolve(REAL_NAME, root=str(root))
    assert 'not a device' in str(caught.value)


def test_resolve_refuses_a_plain_file_that_is_not_a_symlink(tmp_path):
    """A hand-made regular file with the right name is not an adapter."""
    root = tmp_path / 'by-id'
    root.mkdir()
    (root / REAL_NAME).write_text('')
    with pytest.raises(discovery.BindingError):
        discovery.resolve(REAL_NAME, root=str(root))


def test_resolve_never_picks_the_only_adapter_present(by_id):
    """
    The single most important test in this package.

    The root holds exactly one adapter, under a DIFFERENT name. Resolution
    must fail rather than open the one it found -- picking "the only one
    present" is precisely how one arm ends up driving the other arm's
    gripper.
    """
    root, device = by_id
    with pytest.raises(discovery.BindingError) as caught:
        discovery.resolve(OTHER_NAME, root=root)
    message = str(caught.value)
    assert OTHER_NAME in message
    assert REAL_NAME in message          # it says what IS there
    assert device not in message         # and it did not open it


def test_resolve_does_not_touch_a_device_directory_of_its_own(by_id,
                                                              monkeypatch):
    """Nothing outside the configured root is ever listed."""
    root, _ = by_id
    seen = []
    real_listdir = os.listdir

    def watched(path, *args, **kwargs):
        seen.append(path)
        return real_listdir(path, *args, **kwargs)

    monkeypatch.setattr(discovery.os, 'listdir', watched)
    discovery.resolve(REAL_NAME, root=root)
    with pytest.raises(discovery.BindingError):
        discovery.resolve(OTHER_NAME, root=root)
    assert seen == [root]


def test_discovery_imports_no_globber():
    """A globber in this module would be an enumeration-order fallback."""
    tree = ast.parse(inspect.getsource(discovery))
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert roots == {'errno', 'os'}


def test_missing_adapter_message_lists_what_is_present(by_id):
    """The operator's next action is a copy-paste of a name that exists."""
    root, _ = by_id
    message = discovery.missing_adapter_message('panda1', OTHER_NAME,
                                                root=root)
    assert 'panda1: no gripper at' in message
    assert 'Adapters present now:' in message
    assert '  ' + REAL_NAME in message


def test_missing_adapter_message_when_no_adapters_exist_at_all(tmp_path):
    """An empty heading followed by nothing would read as a bug."""
    message = discovery.missing_adapter_message(
        'panda2', REAL_NAME, root=str(tmp_path / 'absent'))
    assert 'No USB serial adapters are present at all.' in message
    assert 'Adapters present now:' not in message


def test_missing_adapter_message_names_the_config_key_and_the_doc():
    """It names the key to edit and the document that explains the rule."""
    message = discovery.missing_adapter_message('panda1', REAL_NAME,
                                                root='/nowhere')
    assert 'grippers.panda1.serial_id' in message
    assert 'franka_robotiq/doc/SERIAL_BINDING.md' in message
    by_path = discovery.missing_adapter_message('panda1', REAL_NAME,
                                                root='/nowhere',
                                                key='usb_path')
    assert 'grippers.panda1.usb_path' in by_path


def test_no_message_contains_a_notes_tree_path():
    """Operator-facing text points at the installed docs, never at a notes tree."""
    messages = [
        discovery.missing_adapter_message('panda1', REAL_NAME, root='/nowhere'),
        discovery.identity_mismatch_message('panda1', REAL_NAME, 'D3091K4W',
                                            'panda2'),
        discovery.identity_unavailable_message('panda1'),
    ]
    for arm, serial_id, usb_path in [('panda1', '', ''),
                                     ('panda1', 'a', 'b')]:
        with pytest.raises(discovery.BindingError) as caught:
            discovery.check_binding(arm, serial_id, usb_path)
        messages.append(str(caught.value))
    # The banned string is built from two halves on purpose: this file is
    # itself inside the package the rule covers, so spelling it whole would
    # make the check its own first violation.
    banned = 'multipanda_ros2_jazzy' + '_notes'
    for message in messages:
        assert banned not in message


def test_enabled_without_a_binding_is_refused():
    """Neither serial_id nor usb_path set is a startup error, not a default."""
    with pytest.raises(discovery.BindingError) as caught:
        discovery.check_binding('panda1', '', '')
    message = str(caught.value)
    assert 'grippers.panda1.serial_id' in message
    assert 'grippers.panda1.usb_path' in message


def test_both_bindings_set_is_refused():
    """Two ways of naming one adapter must not both be given."""
    with pytest.raises(discovery.BindingError) as caught:
        discovery.check_binding('panda1', REAL_NAME, 'pci-0000:00:14.0-usb-0:2')
    assert 'both' in str(caught.value)


def test_one_binding_each_way_is_accepted():
    """Exactly one of the two is the whole rule."""
    assert discovery.check_binding('panda1', REAL_NAME, '') is None
    assert discovery.check_binding('panda1', '', 'pci-0000:00:14.0-usb-0:2') \
        is None


def test_two_arms_naming_the_same_adapter_is_refused():
    """A shared binding leaves one arm unbound and the other doubly owned."""
    with pytest.raises(discovery.BindingError) as caught:
        discovery.check_cross_arm({'panda1': (REAL_NAME, ''),
                                   'panda2': (REAL_NAME, '')})
    message = str(caught.value)
    assert 'panda1' in message and 'panda2' in message
    assert REAL_NAME in message
    assert 'One adapter cannot drive two grippers.' in message


def test_two_arms_with_different_adapters_are_accepted():
    """The ordinary case passes silently."""
    assert discovery.check_cross_arm({'panda1': (REAL_NAME, ''),
                                      'panda2': (OTHER_NAME, '')}) is None


def test_two_arms_resolving_to_the_same_device_is_refused(tmp_path):
    """Two names, two symlinks, one real device is still one adapter."""
    root = tmp_path / 'by-id'
    root.mkdir()
    master, slave = os.openpty()
    device = os.ttyname(slave)
    os.symlink(device, str(root / REAL_NAME))
    os.symlink(device, str(root / OTHER_NAME))
    try:
        with pytest.raises(discovery.BindingError) as caught:
            discovery.check_cross_arm(
                {'panda1': (REAL_NAME, ''), 'panda2': (OTHER_NAME, '')},
                resolver=lambda name: discovery.resolve(name, root=str(root)))
        message = str(caught.value)
        assert device in message
        assert REAL_NAME in message and OTHER_NAME in message
    finally:
        os.close(master)
        os.close(slave)


def test_cross_arm_ignores_an_absent_adapter():
    """A resolver that raises means "absent", and absence is not a collision."""
    def absent(name):
        raise discovery.BindingError('not plugged in')

    assert discovery.check_cross_arm(
        {'panda1': (REAL_NAME, ''), 'panda2': (OTHER_NAME, '')},
        resolver=absent) is None


def test_cross_arm_ignores_an_arm_with_no_binding():
    """An arm whose gripper is not configured cannot collide with anything."""
    assert discovery.check_cross_arm({'panda1': (REAL_NAME, ''),
                                      'panda2': ('', '')}) is None


def test_check_cross_arm_takes_the_pinned_mapping_shape():
    """
    A mapping of arm id to a (serial_id, usb_path) pair, and nothing else.

    A list of pairs or two positional arguments must raise, so a consumer
    cannot drift the shape and have it silently half-work.
    """
    assert discovery.check_cross_arm(
        {'panda1': (REAL_NAME, ''),
         'panda2': ('', 'pci-0000:00:14.0-usb-0:2:1.0-port0')}) is None
    with pytest.raises(discovery.BindingError):
        discovery.check_cross_arm([('panda1', (REAL_NAME, '')),
                                   ('panda2', (OTHER_NAME, ''))])
    with pytest.raises(TypeError):
        discovery.check_cross_arm(REAL_NAME, OTHER_NAME)


def test_binding_error_is_not_a_value_error():
    """A ValueError base would be swallowed by a generic validation handler."""
    assert issubclass(discovery.BindingError, Exception)
    assert not issubclass(discovery.BindingError, ValueError)


def _fake_sysfs(tmp_path, tty_name='ttyUSB0', serial='D3091K4T',
                write_serial=True):
    """Build a plausible sysfs tree for one FTDI adapter and return its root."""
    sysfs = tmp_path / 'sys'
    device = sysfs / 'devices' / 'usb1' / '1-1' / '1-1:1.0' / tty_name
    device.mkdir(parents=True)
    usb_device = sysfs / 'devices' / 'usb1' / '1-1'
    (usb_device / 'idVendor').write_text('0403\n')
    if write_serial:
        (usb_device / 'serial').write_text(serial + '\n')
    tty_class = sysfs / 'class' / 'tty' / tty_name
    tty_class.mkdir(parents=True)
    os.symlink(str(device), str(tty_class / 'device'))
    return str(sysfs)


def test_sysfs_serial_is_read_from_the_first_parent_holding_idvendor_and_serial(
        tmp_path):
    """The walk looks for the pair rather than counting a fixed number of levels."""
    sysfs = _fake_sysfs(tmp_path)
    assert discovery.adapter_serial_from_sysfs(
        '/dev/ttyUSB0', sysfs_root=sysfs) == 'D3091K4T'


def test_sysfs_serial_is_none_when_the_tree_is_absent(tmp_path):
    """A pty has no such tree; the caller must get None, not an exception."""
    assert discovery.adapter_serial_from_sysfs(
        '/dev/pts/7', sysfs_root=str(tmp_path / 'sys')) is None
    assert discovery.adapter_serial_from_sysfs('') is None


def test_sysfs_serial_is_none_when_the_adapter_reports_no_serial(tmp_path):
    """An FTDI part with a blank EEPROM serial yields None, not a guess."""
    sysfs = _fake_sysfs(tmp_path, write_serial=False)
    assert discovery.adapter_serial_from_sysfs(
        '/dev/ttyUSB0', sysfs_root=sysfs) is None


def test_identity_matches_by_containment_not_by_parsing():
    """
    Containment, because a false negative refuses a correctly bound gripper.

    A model string with an underscore in it would defeat "the token after the
    last underscore", and the consequence of that mistake is a refusal the
    operator would fix by deleting the check.
    """
    assert discovery.identity_matches(REAL_NAME, 'D3091K4T')
    assert not discovery.identity_matches(REAL_NAME, 'D3091K4W')
    assert discovery.identity_matches(
        'usb-Some_Vendor_Model_X_2_ABC123-if00-port0', 'ABC123')
    assert not discovery.identity_matches(REAL_NAME, '')
    assert not discovery.identity_matches('', 'D3091K4T')


def test_serial_hint_is_used_only_for_wording():
    """The hint is a best guess and never decides anything."""
    assert discovery.serial_hint_from_by_id(REAL_NAME) == 'D3091K4T'
    assert discovery.serial_hint_from_by_id('') == ''


def test_identity_mismatch_message_names_both_serials_and_says_nothing_ran():
    """It names what was found, what was expected, and that nothing moved."""
    message = discovery.identity_mismatch_message('panda1', REAL_NAME,
                                                  'D3091K4W', 'panda2')
    assert 'D3091K4W' in message
    assert 'D3091K4T' in message
    assert "panda2's adapter" in message
    assert message.endswith('Nothing was commanded.')


def test_identity_mismatch_message_without_an_other_arm_still_refuses():
    """
    With no second arm to name, the sentence still refuses and still teaches.

    The branch is pinned so nobody deletes it as dead code: a single-arm cell,
    or an arm id outside the closed pair, has no second arm to name and the
    refusal must not fabricate one.
    """
    message = discovery.identity_mismatch_message('panda1', REAL_NAME,
                                                  'D3091K4W', '')
    assert "another arm's adapter" in message
    assert 'panda2' not in message
    assert message.endswith('Nothing was commanded.')


def test_identity_unavailable_message_admits_it_did_not_verify():
    """An unavailable check is reported as unavailable, never as a pass."""
    message = discovery.identity_unavailable_message('panda1')
    assert 'could not be re-verified' in message
    assert 'physical port alone' in message
