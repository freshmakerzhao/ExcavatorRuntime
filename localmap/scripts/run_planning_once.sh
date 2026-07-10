#!/usr/bin/env bash
# ROS环境适配入口；规划编排与参数校验位于Python应用层。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi

exec /usr/bin/python3 "${AIRY_ROOT}/localmap/apps/planning/run_planning_once.py" "$@"
