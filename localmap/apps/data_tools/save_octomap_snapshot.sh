#!/usr/bin/env bash
# 保存当前octomap_server中的地图快照，默认保存为.bt二进制OctoMap。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 关键：octomap_saver_node来自ROS官方octomap_server包。
source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

if ! ros2 pkg executables octomap_server | grep -q 'octomap_saver_node'; then
  cat >&2 <<'EOF'
未检测到 octomap_saver_node。
请先安装官方ROS2包：
  sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-ros
EOF
  exit 1
fi

DEFAULT_OUTPUT="${AIRY_ROOT}/localmap/exports/octomap/airy_octomap_$(date +%Y%m%d_%H%M%S).bt"
OUTPUT_PATH="${1:-${DEFAULT_OUTPUT}}"
mkdir -p "$(dirname "${OUTPUT_PATH}")"

echo "保存OctoMap到: ${OUTPUT_PATH}"
exec ros2 run octomap_server octomap_saver_node --ros-args \
  -p octomap_path:="${OUTPUT_PATH}"
