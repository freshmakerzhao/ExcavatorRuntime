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
RUN_BUCKET_TIP_BRIDGE="${RUN_BUCKET_TIP_BRIDGE:-0}"
TRAJECTORY_JSON="${TRAJECTORY_JSON:-${AIRY_ROOT}/localmap/exports/live_latest/trajectory_command.simple_rrt.json}"
BUCKET_TIP_BRIDGE_CONFIG="${BUCKET_TIP_BRIDGE_CONFIG:-${AIRY_ROOT}/localmap/config/bucket_tip_tf_bridge.machine_root.json}"
BUCKET_TIP_JSON="${BUCKET_TIP_JSON:-${AIRY_ROOT}/localmap/exports/live_latest/bucket_tip.machine_root.live.json}"
REACHABLE_WORKSPACE_JSON="${REACHABLE_WORKSPACE_JSON:-${AIRY_ROOT}/../shared/reachable_workspaces/scale_excavator_workspace.json}"
WORKSPACE_MODE="${WORKSPACE_MODE:-MoveToDig}"
TARGETS_JSON="${TARGETS_JSON:-${AIRY_ROOT}/localmap/config/targets.mock.json}"
LIVE_LOCAL_MAP_JSON="${LIVE_LOCAL_MAP_JSON:-${AIRY_ROOT}/localmap/exports/live_latest/local_map.live.json}"
LIVE_LOCAL_MAP_BOUNDS="${LIVE_LOCAL_MAP_BOUNDS:--1.5 3.0 -0.70 1.00 -0.5 4.0}"

PIDS=()
PID_NAMES=()

config_host_address() {
  /usr/bin/python3 - "$RSLIDAR_CONFIG" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
for line in path.read_text(encoding="utf-8").splitlines():
    stripped = line.split("#", 1)[0].strip()
    if stripped.startswith("host_address:"):
        print(stripped.split(":", 1)[1].strip())
        raise SystemExit(0)
raise SystemExit(1)
PY
}

check_rslidar_host_address() {
  local host_address
  host_address="$(config_host_address || true)"
  if [[ -z "${host_address}" || "${host_address}" == "0.0.0.0" ]]; then
    return 0
  fi

  if ! ip -brief addr | grep -qw "${host_address}"; then
    cat <<EOF
错误：rslidar配置中的 host_address=${host_address} 当前不在本机网卡上。
请先给雷达网口配置这个IP，例如：
  sudo ip link set enp130s0 up
  sudo ip addr add ${host_address}/24 dev enp130s0
  ip -brief addr show enp130s0

当前本机地址：
$(ip -brief addr)
EOF
    exit 1
  fi
}

start_process() {
  local name="$1"
  shift
  local log_path="${AIRY_ROOT}/runtime/logs/${name}.log"
  echo "启动 ${name}，日志: ${log_path}"
  "$@" >"${log_path}" 2>&1 &
  PIDS+=("$!")
  PID_NAMES+=("${name}")
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
  check_rslidar_host_address
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

if [[ "${RUN_BUCKET_TIP_BRIDGE}" == "1" ]]; then
  # 关键：原始TF项目在fk_root发布 /bucket_tip_pose_map；这里桥接到machine_root给RRT读取。
  start_process bucket_tip_tf_bridge \
    /usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/bridge_bucket_tip_from_tf.py" \
      --input-topic /bucket_tip_pose_map \
      --output-topic /localmap/bucket_tip_machine_root_pose \
      --bridge "${BUCKET_TIP_BRIDGE_CONFIG}" \
      --output-json "${BUCKET_TIP_JSON}"
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
  RUN_BUCKET_TIP_BRIDGE=1 时再检查：ros2 topic echo /localmap/bucket_tip_machine_root_pose --once

日志目录：
  ${AIRY_ROOT}/runtime/logs

按 Ctrl+C 关闭全部子进程。
EOF

while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      echo "子进程 ${PID_NAMES[$i]}(${pid}) 已退出，请查看 runtime/logs/${PID_NAMES[$i]}.log。"
      exit 1
    fi
  done
  sleep 2
done
