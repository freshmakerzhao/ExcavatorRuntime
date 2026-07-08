#!/usr/bin/env bash
# 一键启动雷达感知栈：rslidar驱动 -> machine_root点云 -> LocalMap -> OctoMap。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 关键：ROS setup脚本不适合在set -u开启前source。
source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

mkdir -p "${AIRY_ROOT}/runtime/logs"

RSLIDAR_CONFIG="${RSLIDAR_CONFIG:-${AIRY_ROOT}/runtime/config_airy_current.yaml}"
EXTRINSICS="${EXTRINSICS:-${AIRY_ROOT}/localmap/config/extrinsics_rslidar_to_machine_root.measured.json}"
RUN_RSLIDAR="${RUN_RSLIDAR:-1}"
RUN_TRANSFORM="${RUN_TRANSFORM:-1}"
RUN_LIVE_LOCAL_MAP="${RUN_LIVE_LOCAL_MAP:-1}"
RUN_OCTOMAP="${RUN_OCTOMAP:-1}"
RUN_REACHABLE_WORKSPACE_MARKERS="${RUN_REACHABLE_WORKSPACE_MARKERS:-1}"
RUN_TRAJECTORY_MARKERS="${RUN_TRAJECTORY_MARKERS:-0}"
TRAJECTORY_JSON="${TRAJECTORY_JSON:-${AIRY_ROOT}/localmap/exports/live_latest/trajectory_command.simple_rrt.json}"
REACHABLE_WORKSPACE_JSON="${REACHABLE_WORKSPACE_JSON:-${AIRY_ROOT}/../shared/reachable_workspaces/scale_excavator_workspace.json}"
WORKSPACE_MODE="${WORKSPACE_MODE:-MoveToDig}"
TARGETS_JSON="${TARGETS_JSON:-${AIRY_ROOT}/localmap/config/targets.mock.json}"
LIVE_LOCAL_MAP_JSON="${LIVE_LOCAL_MAP_JSON:-${AIRY_ROOT}/localmap/exports/live_latest/local_map.live.json}"
LIVE_LOCAL_MAP_BOUNDS="${LIVE_LOCAL_MAP_BOUNDS:--1.5 3.0 -0.70 1.00 -0.5 4.0}"

PIDS=()

start_process() {
  local name="$1"
  shift
  local log_path="${AIRY_ROOT}/runtime/logs/${name}.log"
  echo "启动 ${name}，日志: ${log_path}"
  "$@" >"${log_path}" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  echo "正在关闭感知栈..."
  for pid in "${PIDS[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

if [[ "${RUN_RSLIDAR}" == "1" ]]; then
  start_process rslidar_sdk \
    ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
      -p "config_path:=${RSLIDAR_CONFIG}"
fi

if [[ "${RUN_TRANSFORM}" == "1" ]]; then
  start_process live_cloud_transform \
    /usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/transform_live_cloud_to_base.py" \
      --input-topic /rslidar_points \
      --output-topic /localmap/machine_root_points \
      --extrinsics "${EXTRINSICS}"
fi

if [[ "${RUN_LIVE_LOCAL_MAP}" == "1" ]]; then
  # 关键：run_planning_once.sh会读取这个JSON作为ground/target来源；必须持续刷新到machine_root。
  # shellcheck disable=SC2086
  start_process live_local_map \
    /usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/run_live_local_map_node.py" \
      --input-topic /localmap/machine_root_points \
      --output-json "${LIVE_LOCAL_MAP_JSON}" \
      --targets "${TARGETS_JSON}" \
      --expected-frame machine_root \
      --bounds ${LIVE_LOCAL_MAP_BOUNDS} \
      --write-every 5 \
      --publish-every 10
fi

if [[ "${RUN_OCTOMAP}" == "1" ]]; then
  # 关键：OctoMap调参仍通过环境变量传给run_octomap_mapping.sh，避免这里复制一份参数逻辑。
  start_process octomap \
    "${AIRY_ROOT}/localmap/scripts/run_octomap_mapping.sh"
fi

if [[ "${RUN_REACHABLE_WORKSPACE_MARKERS}" == "1" ]]; then
  start_process reachable_workspace_markers \
    /usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/publish_reachable_workspace_markers.py" \
      --workspace "${REACHABLE_WORKSPACE_JSON}" \
      --mode "${WORKSPACE_MODE}"
fi

if [[ "${RUN_TRAJECTORY_MARKERS}" == "1" ]]; then
  start_process trajectory_markers \
    /usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/publish_trajectory_markers.py" \
      --trajectory "${TRAJECTORY_JSON}"
fi

cat <<EOF

感知栈已启动。
常用检查：
  ros2 topic list | grep -E 'rslidar|machine_root|octomap|occupied|reachable'
  ros2 topic hz /localmap/machine_root_points
  /usr/bin/python3 -m json.tool ${LIVE_LOCAL_MAP_JSON} | sed -n '1,40p'
  ros2 topic hz /occupied_cells_vis_array

日志目录：
  ${AIRY_ROOT}/runtime/logs

按 Ctrl+C 关闭全部子进程。
EOF

while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      echo "子进程 ${pid} 已退出，请查看 runtime/logs。"
      exit 1
    fi
  done
  sleep 2
done
