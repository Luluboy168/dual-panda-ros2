<!--
Copyright 2026 The multipanda_ros2 Authors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Franka IK frozen contract

This is the Session A integration contract for `franka_ik` version 0.1.0. The blocks under
"Canonical rosidl sources" are byte-for-byte copies of the corresponding `.msg` and `.srv`
files, including comments and copyright headers. They are structured so a test can extract and
compare each block directly to its source file.

## Canonical rosidl sources

### franka_ik_interfaces/msg/IkRequest.msg

```text
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
```

### franka_ik_interfaces/msg/IkSolution.msg

```text
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

# Radians in canonical joint1..joint7 order.
float64[7] positions
float64 redundancy_value
float64 position_error
float64 orientation_error
float64 seed_distance
uint8 BRANCH_NUMERIC=255
uint8 branch
```

### franka_ik_interfaces/msg/IkResult.msg

```text
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

uint8 RESULT_SUCCESS=0
uint8 RESULT_BAD_REQUEST=1
uint8 RESULT_UNKNOWN_ARM=2
uint8 RESULT_UNKNOWN_FRAME=3
uint8 RESULT_UNSUPPORTED_TIP=4
uint8 RESULT_SEED_OUT_OF_LIMITS=5
uint8 RESULT_UNREACHABLE=6
uint8 RESULT_LIMITS_VIOLATED=7
uint8 RESULT_TOLERANCE_NOT_MET=8
uint8 RESULT_ITERATION_BUDGET_EXHAUSTED=9
uint8 RESULT_NO_ACCEPTABLE_SOLUTION=10
uint8 RESULT_INTERNAL_ERROR=11
uint8 result

string<=256 message
franka_ik_interfaces/IkSolution[<=4] solutions
uint8 solver_used
uint16 iterations
builtin_interfaces/Duration solve_time
```

### franka_ik_interfaces/msg/ChainInfo.msg

```text
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

string<=64 arm_id
string base_frame
string flange_frame
string hand_tcp_frame
string[7] joint_names
float64[7] position_lower
float64[7] position_upper
float64[7] velocity_limit
geometry_msgs/Transform root_to_base
```

### franka_ik_interfaces/srv/SolveIk.srv

```text
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

franka_ik_interfaces/IkRequest request
---
franka_ik_interfaces/IkResult result
```

### franka_ik_interfaces/srv/GetChainInfo.srv

```text
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

---
string urdf_root_frame
string<=64 urdf_sha256
franka_ik_interfaces/ChainInfo[<=4] chains
uint8 default_solver
bool analytic_backend_available
string<=64 package_version
```

## Node identity and resolved names

```yaml
executable: franka_ik_service_node
node_name: franka_ik_service
namespace: /
services:
  solve_ik: /franka_ik_service/solve_ik
  chain_info: /franka_ik_service/chain_info
service_qos: rclcpp::ServicesQoS()
callback_group: MutuallyExclusive
executor: SingleThreadedExecutor
lifecycle: plain_rclcpp_node
```

Both services use the default reliable service QoS. URDF parsing happens during node construction;
an invalid description or invalid configuration throws and the process exits non-zero.

## Read-only parameters

```yaml
robot_description:
  type: string
  default: ""
  constraint: required non-empty URDF XML
arm_ids:
  type: string_array
  default: [panda1, panda2]
  constraint: 1..4 unique entries matching "[A-Za-z][A-Za-z0-9_]*", each present in the URDF
default_solver:
  type: string
  default: numeric
  allowed: [analytic, numeric]
  note: analytic is a startup error in this numeric-only build
position_tolerance:
  type: double
  default: 1.0e-4
  unit: m
  interval: "(0, 1.0e-2]"
orientation_tolerance:
  type: double
  default: 1.0e-3
  unit: rad
  interval: "(0, 1.0e-1]"
numeric_max_iterations:
  type: integer
  default: 40
  interval: "[10, 2000]"
numeric_eps:
  type: double
  default: 1.0e-6
  interval: "(0, 1.0e-2]"
joint_limit_margin_max:
  type: double
  default: 0.100
  unit: rad
  interval: "[0, 0.5]"
```

Every parameter is declared read-only and read once during construction. An invalid value is a
hard startup failure whose message names the parameter; values are never silently clamped.

## Invariants

1. The service is a pure function of `(robot_description, parameters, IkRequest)`. It has no
   internal mutable state between calls beyond scratch buffers.
2. Every returned `positions` vector satisfies the margin-adjusted URDF limits, inclusive.
3. Every returned `positions` vector satisfies `position_error <= position_tolerance` and
   `orientation_error <= orientation_tolerance`, as measured by the service's own FK.
4. `solutions` is ordered by `seed_distance` ascending, ties broken by `branch` ascending.
5. `result == RESULT_SUCCESS` means the backend found at least one valid witness. If
   `max_solutions == 0`, that witness is deliberately not serialized and `solutions` is empty;
   otherwise `solutions` is non-empty. A failed reachability probe never becomes success merely
   because `max_solutions == 0`.
6. Identical requests produce byte-identical responses apart from `solve_time`.
7. The service never blocks on anything but computation: no file I/O, no network, no logging
   inside the solve path above `RCLCPP_DEBUG`.

## Frame and solver invariants

- For arm id `A`, the flange chain is `A_link0 -> A_link8`; joint names are exactly
  `A_joint1` through `A_joint7` in order.
- `frame_id == ""` and `frame_id == A_link0` select the arm base. The only other accepted frame
  is the parsed URDF root.
- A URDF-root target is converted as
  `base_T_target = inverse(root_T_base) * root_T_target` with the parsed transform.
- `TIP_HAND_TCP` is supported only when the loaded URDF includes `A_hand_tcp`.
- V1 has one available backend: `SOLVER_NUMERIC` (KDL LMA). `SOLVER_DEFAULT` resolves to numeric;
  `SOLVER_ANALYTIC` returns `RESULT_BAD_REQUEST`, and `analytic_backend_available` is false.
- Joint 7 is fixed to `seed_positions[6]` for `REDUNDANCY_FROM_SEED` or to
  `redundancy_value` for `REDUNDANCY_FIXED`.
- The numeric backend is bounded only by `numeric_max_iterations`, never wall time. `solve_time`
  measures the backend call only and is the sole response field excluded from determinism.
