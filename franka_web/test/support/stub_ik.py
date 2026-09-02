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
A deterministic stand-in for the IK service, with no ROS anywhere.

The endpoint under test performs no kinematics at all, so the default
success answer returns the seed verbatim: an FK-consistent stub would assert
nothing the identity does not, and FK truth is proved where it exists --
against the real service, in the live gate.

Every call is recorded, which is how the field-mapping table is asserted.
"""

from franka_web.ghost import IkReply, RESULT_SUCCESS


class StubSolver:
    """A callable ``(IkCall, timeout_s) -> IkReply | None`` with a script."""

    def __init__(self, ready=True):
        """Start ready, answering every call with the seed it was given."""
        self.ready = ready
        self.calls = []
        self.timeouts = []
        self.replies = []          # scripted answers, consumed in order
        self.answer = None         # None means "the identity success"

    def script(self, *replies):
        """Queue answers for the next calls; None means "no answer"."""
        self.replies.extend(replies)
        return self

    def __call__(self, call, timeout_s):
        """Record one call and return the next scripted or default answer."""
        self.calls.append(call)
        self.timeouts.append(timeout_s)
        if self.replies:
            return self.replies.pop(0)
        if self.answer is not None:
            return self.answer
        return IkReply(result=RESULT_SUCCESS, message='',
                       positions=tuple(call.seed_positions),
                       redundancy_value=call.redundancy_value,
                       position_error=0.0, orientation_error=0.0)

    def is_ready(self):
        """Return the readiness the service client would report right now."""
        return self.ready


def failure(result, message='the service said so'):
    """Return one declined answer with the given result code."""
    return IkReply(result=result, message=message)
