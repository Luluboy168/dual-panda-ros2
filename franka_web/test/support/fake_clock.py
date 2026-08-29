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

"""A deterministic monotonic clock for timing-sensitive unit tests."""


class FakeClock:
    """
    Injectable stand-in for ``time.monotonic``.

    Pass ``clock.monotonic`` wherever production code accepts a monotonic
    callable, then drive time explicitly with :meth:`advance`.
    """

    def __init__(self, start=1000.0):
        """Start the clock at ``start`` seconds."""
        self._now = float(start)

    def monotonic(self):
        """Return the current fake time in seconds."""
        return self._now

    def monotonic_ns(self):
        """Return the current fake time in nanoseconds."""
        return int(self._now * 1e9)

    def advance(self, seconds):
        """Move the clock forward by ``seconds`` (never backward)."""
        if seconds < 0:
            raise ValueError('a monotonic clock cannot go backward')
        self._now += float(seconds)
