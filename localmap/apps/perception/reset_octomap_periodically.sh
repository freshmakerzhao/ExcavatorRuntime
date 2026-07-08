#!/usr/bin/env bash
# 周期性清空octomap_server，让OctoMap更接近“当前实时视野”而不是长期累计地图。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 关键：ROS setup脚本不适合在set -u开启前source。
source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

OCTOMAP_RESET_SERVICE="${OCTOMAP_RESET_SERVICE:-/octomap_server/reset}"
OCTOMAP_RESET_INTERVAL_S="${OCTOMAP_RESET_INTERVAL_S:-1.0}"

echo "OctoMap reset service : ${OCTOMAP_RESET_SERVICE}"
echo "Reset interval        : ${OCTOMAP_RESET_INTERVAL_S} s"
echo "按 Ctrl+C 停止周期reset。"

while true; do
  # 关键：std_srvs/srv/Empty没有请求字段；每次调用都会清空整棵OctoMap树。
  ros2 service call "${OCTOMAP_RESET_SERVICE}" std_srvs/srv/Empty "{}" >/dev/null
  sleep "${OCTOMAP_RESET_INTERVAL_S}"
done
