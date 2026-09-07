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
Build a real ``Settings`` for a test, from a real configuration file.

Every rig in this suite used to assemble ``Settings`` out of an environment.
There is no environment any more, so they write a small ``config.yaml`` into
their own temporary directory and load it through the shipped loader --
which keeps the tests exercising the real parsing, unit conversion and
defaulting rather than a hand-built dataclass.

Angles here are DEGREES, because that is what the file speaks.
"""

import math
import os

from franka_web import config, defaults

import yaml

#: Documentation addresses (RFC 5737 TEST-NET-1). No real robot lives here.
DOC_IP_1 = '192.0.2.11'
DOC_IP_2 = '192.0.2.12'

#: SI field name -> the degree key the configuration file speaks. The three
#: keys that are already in seconds or counts map to themselves.
_SETTLING_KEYS = {
    'drift_limit_rad': 'drift_limit_deg',
    'span_limit_rad': 'span_limit_deg',
    'velocity_limit_rad_s': 'velocity_limit_deg_s',
    'fence_margin_rad': 'fence_margin_deg',
}


def _degrees(values):
    """Render a radian scalar or 7-sequence as degrees for the file."""
    if isinstance(values, (list, tuple)):
        return [math.degrees(float(value)) for value in values]
    return math.degrees(float(values))


def config_document(*, port=None, bind=None, domain_id=None, state_dir=None,
                    recording_root=None, franka_dir=None, robot_ips=None,
                    recording_enabled=None, jog_step_deg=None,
                    settling_rad=None, fences_rad=None, profiles=None,
                    grippers=None, max_total_gb=None):
    """
    Return the configuration mapping these keyword arguments describe.

    ``settling_rad`` and ``fences_rad`` are given in RADIANS for the caller's
    convenience -- the rigs think in the units the controller does -- and are
    converted to the degrees the file speaks on the way out.

    ``grippers`` is emitted verbatim as the file's ``grippers:`` block, in the
    millimetres, newtons and seconds that block speaks. Without it no test
    could build a ``Settings`` with a gripper enabled at all, because this
    function is keyword-only with an explicit parameter list and no
    ``**overrides``.
    """
    document = {}
    if port is not None:
        document['port'] = int(port)
    if bind is not None:
        document['bind'] = str(bind)
    if domain_id is not None:
        document['ros_domain_id'] = int(domain_id)
    if robot_ips is None:
        robot_ips = {'panda1': DOC_IP_1, 'panda2': DOC_IP_2}
    document['robots'] = {arm_id: {'ip': address}
                          for arm_id, address in robot_ips.items()}
    directories = {}
    if state_dir is not None:
        directories['state'] = str(state_dir)
    if recording_root is not None:
        directories['recordings'] = str(recording_root)
    if franka_dir is not None:
        directories['franka_dir'] = str(franka_dir)
    if directories:
        document['directories'] = directories
    if recording_enabled is not None:
        document['recording'] = {'enabled': bool(recording_enabled)}
    if max_total_gb is not None:
        # A number or the documented "unlimited" spelling, verbatim.
        document['recordings'] = {'max_total_gb': max_total_gb}
    if jog_step_deg is not None:
        document['jog'] = {'step_deg': float(jog_step_deg)}
    if settling_rad:
        settling = {}
        for key, value in settling_rad.items():
            target = _SETTLING_KEYS.get(key, key)
            settling[target] = (value if target == key
                                else _degrees(value))
        document['settling'] = settling
    if fences_rad:
        document['fence'] = {
            arm_id: {'enabled': True,
                     'lower_deg': _degrees(lower),
                     'upper_deg': _degrees(upper)}
            for arm_id, (lower, upper) in fences_rad.items()}
    if profiles:
        document['profiles'] = profiles
    if grippers:
        document['grippers'] = {
            arm_id: dict(block) for arm_id, block in grippers.items()}
    return document


#: A binding that satisfies the basename-syntax rule without naming a real
#: adapter. Nothing resolves it until a session opens the device, which is
#: exactly how the row is demonstrated before the hardware arrives.
PLACEHOLDER_SERIAL_ID = 'usb-PLACEHOLDER_ADAPTER_0000-if00-port0'


def gripper_block(*, enabled=True, serial_id=PLACEHOLDER_SERIAL_ID, **overrides):
    """Return one arm's ``grippers.<arm>`` mapping for ``config_document``."""
    block = {'enabled': bool(enabled)}
    if serial_id is not None:
        block['serial_id'] = serial_id
    block.update(overrides)
    return block


def write_config(path, document):
    """Write a configuration mapping to ``path`` and return the path as a str."""
    path = str(path)
    os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(document, handle, default_flow_style=False,
                       sort_keys=True)
    return path


def make_settings(tmp_path, *, domain_id=80, **overrides):
    """
    Build ``Settings`` over private temporary directories under ``tmp_path``.

    The state and recording directories are created 0700 the way the loader
    would; the file is written next to them so a test can read it back.
    """
    tmp_path = str(tmp_path)
    state_dir = os.path.join(tmp_path, 'state')
    recording_root = os.path.join(tmp_path, 'recordings')
    os.makedirs(state_dir, mode=0o700, exist_ok=True)
    os.makedirs(recording_root, mode=0o700, exist_ok=True)
    os.chmod(tmp_path, 0o700)
    os.chmod(state_dir, 0o700)
    os.chmod(recording_root, 0o700)
    overrides.setdefault('state_dir', state_dir)
    overrides.setdefault('recording_root', recording_root)
    document = config_document(domain_id=domain_id, **overrides)
    path = write_config(os.path.join(tmp_path, 'config.yaml'), document)
    return config.load(path, environ={'HOME': tmp_path}, make_dirs=True)


def settling_config(**overrides):
    """Return a ``SettlingConfig`` built from the SI defaults plus overrides."""
    values = dict(defaults.DEFAULT_SETTLING)
    values.update(overrides)
    return config.SettlingConfig(**values)
