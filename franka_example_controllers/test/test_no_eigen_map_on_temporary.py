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

import argparse
from pathlib import Path
import re


UNSAFE_MAP = re.compile(
    r'(?:'
    r'(?:const\s+)?Eigen::Map\s*<[^;]+?>\s+\w+\s*'
    r'|'
    r'(?:const\s+)?auto\s+\w+\s*=\s*Eigen::Map\s*<[^;]+?>\s*'
    r')'
    r'[({]\s*franka_robot_model_->get'
    r'(?:PoseMatrix|MassMatrix|CoriolisForceVector|ZeroJacobian|BodyJacobian)'
    r'\s*\((?:[^()]|\([^()]*\))*\)\s*\.data\s*\(\s*\)\s*[)}]',
    re.DOTALL,
)

COMMENTS_AND_LITERALS = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
    re.DOTALL,
)


def strip_cpp_comments(source: str) -> str:
    """Blank C++ comments while preserving code positions and literals."""

    def replace(match: re.Match) -> str:
        text = match.group(0)
        if not text.startswith('/'):
            return text
        return ''.join('\n' if character == '\n' else ' ' for character in text)

    return COMMENTS_AND_LITERALS.sub(replace, source)


def guard_self_test() -> bool:
    """Prove representative unsafe forms match and safe/comment forms do not."""
    unsafe_fixtures = (
        'Eigen::Map<const Vector7d> value('
        'franka_robot_model_->getCoriolisForceVector().data());',
        'Eigen::Map<const Matrix7d> value{'
        'franka_robot_model_->getMassMatrix().data()};',
        'auto value = Eigen::Map<const Matrix4d>('
        'franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());',
        'const auto value = Eigen::Map<const Matrix4d>{'
        'franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data()};',
    )
    safe_fixtures = (
        'const auto array = franka_robot_model_->getMassMatrix();\n'
        'Eigen::Map<const Matrix7d> value(array.data());',
        '// Eigen::Map<const Matrix7d> value('
        'franka_robot_model_->getMassMatrix().data());',
        '/* auto value = Eigen::Map<const Matrix7d>{'
        'franka_robot_model_->getMassMatrix().data()}; */',
    )
    return (
        all(UNSAFE_MAP.search(strip_cpp_comments(fixture)) for fixture in unsafe_fixtures)
        and not any(UNSAFE_MAP.search(strip_cpp_comments(fixture)) for fixture in safe_fixtures)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-root', required=True, type=Path)
    args = parser.parse_args()

    if not guard_self_test():
        print('internal Eigen::Map source-guard fixture failed')
        return 2

    failures = []
    for source_path in sorted(args.source_root.glob('src/**/*.cpp')):
        source = source_path.read_text(encoding='utf-8')
        if UNSAFE_MAP.search(strip_cpp_comments(source)):
            failures.append(source_path.relative_to(args.source_root))

    if failures:
        print('Eigen::Map binds data owned by a temporary getter result:')
        for failure in failures:
            print(f'  {failure}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
