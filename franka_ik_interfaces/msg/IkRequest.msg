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

# One IK query.

# "" or "<arm_id>_link0" selects the arm base; the URDF root is also accepted.
string<=128 frame_id

# Must be one of the arm identifiers configured by the service node.
string<=64 arm_id

uint8 TIP_FLANGE=0
uint8 TIP_HAND_TCP=1
uint8 tip_frame 0

# Pose of tip_frame expressed in frame_id.
geometry_msgs/Pose target_pose

# Radians in canonical joint1..joint7 order. The seed is always required.
float64[7] seed_positions

uint8 REDUNDANCY_FROM_SEED=0
uint8 REDUNDANCY_FIXED=1
uint8 redundancy_mode 0
float64 redundancy_value 0.0

# Zero requests a reachability probe; one through four bounds returned solutions.
uint8 max_solutions 1

uint8 SOLVER_DEFAULT=0
uint8 SOLVER_ANALYTIC=1
uint8 SOLVER_NUMERIC=2
uint8 solver 0

# Zero selects the corresponding node default.
float64 position_tolerance 0.0
float64 orientation_tolerance 0.0
float64 joint_limit_margin 0.0
