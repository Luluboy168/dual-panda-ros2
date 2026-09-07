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
The Copy payload: the exact snippet text the operator pastes.

The templates live apart from ghost.py because the snippet must name the
message type it is, and say what to do with it -- two of the words the ghost
modules' vocabulary scan forbids. That scan covers this file too, with one
stated allowance for those two words inside the templates below; every other
forbidden word is forbidden here as well. This module still commands
nothing: it formats a string.

The text is computed here rather than in the browser so that the slot
arithmetic lives in one place -- and so that it is provable by an exact
golden-string test in Python.
"""

import math

from franka_web import defaults

_COPY_HEAD = (
    '# Ghost pose for {arm_id}, authored in the Franka console.\n'
    '# Degrees: [{degrees}]\n')

#: Inserted directly after the degrees line, and only when the pose was not
#: collision-checked. It goes into the pasted code deliberately: that is
#: where the claim will later be believed.
_COPY_UNCHECKED = '# NOT collision-checked: the workspace model is not loaded.\n'

#: The message body. `stamp: now` is an instruction to the reader's own code,
#: not a literal a shell would send: the impedance controller refuses a
#: target whose header stamp is zero, older than a second, or more than
#: 0.1 s ahead, and it wants an empty frame_id. Saying so here is the
#: difference between a snippet that works and one that is silently ignored.
_COPY_BODY = (
    '# trajectory_msgs/msg/JointTrajectory — publish at 10 Hz or more\n'
    '# topic: {topic}\n'
    '# Stamp each message with the time you send it: a stamp of zero, one\n'
    '# older than a second, or one more than 0.1 s ahead is refused, and\n'
    '# frame_id stays empty.\n'
    'header:\n'
    '  stamp: now\n'
    'joint_names: [{n0}, {n1}, {n2}, {n3},\n'
    '              {n4}, {n5}, {n6}]\n'
    'points:\n'
    '- positions: [{radians}]\n'
    '  time_from_start: {{sec: 0, nanosec: 0}}')

#: Replaces the topic when no Motion session is running: the slot depends on
#: the session's arm selection, so the topic is genuinely unknowable then.
_COPY_NO_TOPIC = ("set by your session's arm selection — see the External "
                  'panel when a Motion session runs')


def joint_names_for(arm_id):
    """
    Return this arm's joint names, joint1 first.

    Deliberately duplicated from health.py, which owns the canonical
    spelling: importing that module here would pull the whole ROS client
    library in through franka_bringup, and cost the ghost its ability to be
    driven from a unit test on a machine with no ROS at all. The two are asserted equal by a
    test, so the duplication cannot drift.
    """
    return tuple(
        '{}_joint{}'.format(arm_id, index)
        for index in range(1, defaults.JOINT_COUNT + 1))


def build_copy(arm_id, positions, verdict, topic):
    """
    Return the ``copy`` block: degrees, radians and the pasteable snippet.

    ``verdict`` is the verdict object this solve produced, or None; only its
    status is read, and only to decide whether the pose carries the
    not-checked warning.
    """
    names = joint_names_for(arm_id)
    degrees = [round(math.degrees(float(value)), 2) for value in positions]
    radians = [round(float(value), 6) for value in positions]
    head = _COPY_HEAD.format(
        arm_id=arm_id,
        degrees=', '.join('{:.2f}'.format(value) for value in degrees))
    if verdict is not None and verdict.get('status') == 'unchecked':
        head += _COPY_UNCHECKED
    body = _COPY_BODY.format(
        topic=topic if topic else _COPY_NO_TOPIC,
        n0=names[0], n1=names[1], n2=names[2], n3=names[3],
        n4=names[4], n5=names[5], n6=names[6],
        radians=', '.join('{:.6f}'.format(value) for value in positions))
    return {'joints_deg': degrees, 'joints_rad': radians,
            'snippet': head + body}
