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
The mesh fence's runtime: convex bodies, a certified broad phase, and GJK.

Nothing in this subpackage reads a mesh file.  It reads one pinned artefact,
``cell/mesh_bodies_v1.yaml``, whose vertices were derived offline by
``generate_mesh_bodies.py`` - the single sanctioned reader of
``franka_description/mujoco`` and of any ``.stl`` / ``.obj`` / ``.dae`` bytes in
this package.  Nothing here imports ROS, and nothing here imports scipy: the
runtime is numpy plus this package's own strict YAML reader, and
``test_purity_and_performance.py`` asserts it.

The one sentence that matters for a reader of results:

    clearance = gjk(body_a, body_b)

Nothing is subtracted from it.  There is no undercut term because the bodies
contain the visual shell by construction, and there is no capsule radius in it
because the bounding-capsule radii ``r_a`` and ``r_b`` belong to the broad phase
alone.  What the broad phase computes is a certified LOWER BOUND on that same
quantity::

    seg_seg(A, B) - r_a - r_b   <=   gjk(body_a, body_b)

so a pair whose bound already exceeds its own margin cannot violate it and need
not be evaluated.  Writing ``gjk(...) - r_a - r_b`` would subtract the capsule
radii a second time; on ``link5``/``link7`` at the ready pose that is worth
-107 mm and would refuse the home pose.
"""

from .bodies import (certified_lower_bounds, load_mesh_bodies,
                     MAXIMUM_MESH_BODIES_BYTES, MAXIMUM_MESH_BODIES_DEPTH,
                     MAXIMUM_MESH_BODIES_SCALARS, MeshBody, MeshBodySet)
from .gjk import (distance, GJK_DEGENERACY_REL, GJK_ITERATION_CAP, GJK_TOL_ABS_M,
                  GJK_TOL_REL)

__all__ = [
    'GJK_DEGENERACY_REL',
    'GJK_ITERATION_CAP',
    'GJK_TOL_ABS_M',
    'GJK_TOL_REL',
    'MAXIMUM_MESH_BODIES_BYTES',
    'MAXIMUM_MESH_BODIES_DEPTH',
    'MAXIMUM_MESH_BODIES_SCALARS',
    'MeshBody',
    'MeshBodySet',
    'certified_lower_bounds',
    'distance',
    'load_mesh_bodies',
]
