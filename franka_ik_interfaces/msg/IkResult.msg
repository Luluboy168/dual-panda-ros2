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
