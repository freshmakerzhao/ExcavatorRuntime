#!/usr/bin/env bash
# 启动OctoMap实时建图：输入必须是已经转换到machine_root的点云。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 关键：使用系统ROS环境和本地rslidar overlay，避免conda Python污染ROS2运行环境。
source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

if ! ros2 pkg executables octomap_server | grep -q 'octomap_server_node'; then
  cat >&2 <<'EOF'
未检测到 octomap_server_node。
请先安装官方ROS2包：
  sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-ros
EOF
  exit 1
fi

# 这些环境变量允许现场快速调参，不需要反复编辑脚本。
OCTOMAP_CLOUD_TOPIC="${OCTOMAP_CLOUD_TOPIC:-/localmap/machine_root_points}"
OCTOMAP_FRAME_ID="${OCTOMAP_FRAME_ID:-machine_root}"
OCTOMAP_RESOLUTION="${OCTOMAP_RESOLUTION:-0.05}"
OCTOMAP_MAX_RANGE="${OCTOMAP_MAX_RANGE:-6.0}"
OCTOMAP_FILTER_GROUND="${OCTOMAP_FILTER_GROUND:-false}"
OCTOMAP_RESET_INTERVAL_S="${OCTOMAP_RESET_INTERVAL_S:-0}"

# 可选：在machine_root中裁剪进入OctoMap的点云，空值表示不设置该参数。
OCTOMAP_POINT_CLOUD_MIN_X="${OCTOMAP_POINT_CLOUD_MIN_X:-}"
OCTOMAP_POINT_CLOUD_MAX_X="${OCTOMAP_POINT_CLOUD_MAX_X:-}"
OCTOMAP_POINT_CLOUD_MIN_Y="${OCTOMAP_POINT_CLOUD_MIN_Y:-}"
OCTOMAP_POINT_CLOUD_MAX_Y="${OCTOMAP_POINT_CLOUD_MAX_Y:-}"
OCTOMAP_POINT_CLOUD_MIN_Z="${OCTOMAP_POINT_CLOUD_MIN_Z:-}"
OCTOMAP_POINT_CLOUD_MAX_Z="${OCTOMAP_POINT_CLOUD_MAX_Z:-}"

echo "OctoMap input topic : ${OCTOMAP_CLOUD_TOPIC}"
echo "OctoMap frame_id    : ${OCTOMAP_FRAME_ID}"
echo "OctoMap resolution  : ${OCTOMAP_RESOLUTION} m"
echo "OctoMap max_range   : ${OCTOMAP_MAX_RANGE} m"
echo "filter_ground_plane : ${OCTOMAP_FILTER_GROUND}"
echo "reset interval      : ${OCTOMAP_RESET_INTERVAL_S} s"
echo "point cloud crop    : x=[${OCTOMAP_POINT_CLOUD_MIN_X:-*}, ${OCTOMAP_POINT_CLOUD_MAX_X:-*}], y=[${OCTOMAP_POINT_CLOUD_MIN_Y:-*}, ${OCTOMAP_POINT_CLOUD_MAX_Y:-*}], z=[${OCTOMAP_POINT_CLOUD_MIN_Z:-*}, ${OCTOMAP_POINT_CLOUD_MAX_Z:-*}]"

PARAM_ARGS=(
  -p "frame_id:=${OCTOMAP_FRAME_ID}"
  -p "resolution:=${OCTOMAP_RESOLUTION}"
  -p "sensor_model.max_range:=${OCTOMAP_MAX_RANGE}"
  -p "filter_ground_plane:=${OCTOMAP_FILTER_GROUND}"
  -p "use_height_map:=true"
)

# 关键：OctoMap仍接收点云topic，但只积分我们关心的工作空间，减少远处墙体/支架/天花板等无关占据块。
[[ -n "${OCTOMAP_POINT_CLOUD_MIN_X}" ]] && PARAM_ARGS+=(-p "point_cloud_min_x:=${OCTOMAP_POINT_CLOUD_MIN_X}")
[[ -n "${OCTOMAP_POINT_CLOUD_MAX_X}" ]] && PARAM_ARGS+=(-p "point_cloud_max_x:=${OCTOMAP_POINT_CLOUD_MAX_X}")
[[ -n "${OCTOMAP_POINT_CLOUD_MIN_Y}" ]] && PARAM_ARGS+=(-p "point_cloud_min_y:=${OCTOMAP_POINT_CLOUD_MIN_Y}")
[[ -n "${OCTOMAP_POINT_CLOUD_MAX_Y}" ]] && PARAM_ARGS+=(-p "point_cloud_max_y:=${OCTOMAP_POINT_CLOUD_MAX_Y}")
[[ -n "${OCTOMAP_POINT_CLOUD_MIN_Z}" ]] && PARAM_ARGS+=(-p "point_cloud_min_z:=${OCTOMAP_POINT_CLOUD_MIN_Z}")
[[ -n "${OCTOMAP_POINT_CLOUD_MAX_Z}" ]] && PARAM_ARGS+=(-p "point_cloud_max_z:=${OCTOMAP_POINT_CLOUD_MAX_Z}")

ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:="${OCTOMAP_CLOUD_TOPIC}" \
  "${PARAM_ARGS[@]}" &
OCTOMAP_PID=$!

cleanup() {
  kill "${OCTOMAP_PID}" >/dev/null 2>&1 || true
  wait "${OCTOMAP_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if [[ "${OCTOMAP_RESET_INTERVAL_S}" != "0" && "${OCTOMAP_RESET_INTERVAL_S}" != "0.0" ]]; then
  echo "周期reset已启用；用于调试近实时当前视野。"
  while kill -0 "${OCTOMAP_PID}" >/dev/null 2>&1; do
    sleep "${OCTOMAP_RESET_INTERVAL_S}"
    # 关键：reset服务由octomap_server_node提供；失败通常只是节点尚未完全启动，下一轮会重试。
    ros2 service call /octomap_server/reset std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
  done
else
  wait "${OCTOMAP_PID}"
fi
