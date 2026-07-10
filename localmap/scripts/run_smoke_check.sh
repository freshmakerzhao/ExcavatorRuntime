#!/usr/bin/env bash
# source ROS环境后调用Python smoke check；真正检查逻辑在apps/diagnostics中。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi

exec /usr/bin/python3 "${AIRY_ROOT}/localmap/apps/diagnostics/run_smoke_check.py" "$@"
