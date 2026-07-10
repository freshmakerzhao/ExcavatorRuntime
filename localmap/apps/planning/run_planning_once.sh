#!/usr/bin/env bash
# 从当前OctoMap生成一次LocalMap、RRT请求、bucket-tip轨迹和observation切片。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

EXPORT_DIR="${EXPORT_DIR:-${AIRY_ROOT}/localmap/exports/live_latest}"
mkdir -p "${EXPORT_DIR}"

LOCAL_MAP_JSON="${LOCAL_MAP_JSON:-${EXPORT_DIR}/local_map.octomap_obstacles.json}"
BASE_LOCAL_MAP_JSON="${BASE_LOCAL_MAP_JSON:-${EXPORT_DIR}/local_map.live.json}"
RRT_REQUEST_JSON="${RRT_REQUEST_JSON:-${EXPORT_DIR}/rrt_star_request.octomap_obstacles.json}"
TRAJECTORY_JSON="${TRAJECTORY_JSON:-${EXPORT_DIR}/trajectory_command.simple_rrt.json}"
OBS_SLICE_JSON="${OBS_SLICE_JSON:-${EXPORT_DIR}/observation_waypoint_slice.simple_rrt.json}"
LIVE_BUCKET_TIP_JSON="${LIVE_BUCKET_TIP_JSON:-${EXPORT_DIR}/bucket_tip.machine_root.live.json}"
FALLBACK_BUCKET_TIP_JSON="${FALLBACK_BUCKET_TIP_JSON:-${AIRY_ROOT}/localmap/config/bucket_tip.machine_root.measured.json}"
if [[ -z "${BUCKET_TIP_JSON:-}" ]]; then
  if [[ -f "${LIVE_BUCKET_TIP_JSON}" ]]; then
    BUCKET_TIP_JSON="${LIVE_BUCKET_TIP_JSON}"
  else
    BUCKET_TIP_JSON="${FALLBACK_BUCKET_TIP_JSON}"
  fi
fi
REACHABLE_WORKSPACE_JSON="${REACHABLE_WORKSPACE_JSON:-${AIRY_ROOT}/../shared/reachable_workspaces/scale_excavator_workspace.json}"
TASK_MODE="${TASK_MODE:-MoveToDig}"
TARGET_KIND="${TARGET_KIND:-dig}"
TARGET_ID="${TARGET_ID:-mock_dig_001}"
WORKSPACE_MODE="${WORKSPACE_MODE:-${TASK_MODE}}"
USE_REACHABLE_WORKSPACE="${USE_REACHABLE_WORKSPACE:-1}"

PLANNING_BOUNDS="${PLANNING_BOUNDS:--1.5 3.0 -0.7 1.0 -0.5 4.0}"
OBSTACLE_EXPORT_BOUNDS="${OBSTACLE_EXPORT_BOUNDS:--1.5 3.0 -0.42 1.0 -0.5 4.0}"
OCTOMAP_BOX_SIZE="${OCTOMAP_BOX_SIZE:-0.20}"
OCTOMAP_MAX_OBSTACLES="${OCTOMAP_MAX_OBSTACLES:-1000}"
COLLISION_RADIUS="${COLLISION_RADIUS:-0.05}"
MASK_START_RADIUS="${MASK_START_RADIUS:-0.15}"
MASK_GOAL_RADIUS="${MASK_GOAL_RADIUS:-0.45}"
WAYPOINT_COUNT="${WAYPOINT_COUNT:-5}"
MAX_ITERATIONS="${MAX_ITERATIONS:-6000}"
RRT_SEED="${RRT_SEED:-8}"

BASE_LOCAL_MAP_JSON="${BASE_LOCAL_MAP_JSON}" /usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path

path = Path(os.environ["BASE_LOCAL_MAP_JSON"])
if not path.exists():
    raise SystemExit(
        "缺少base LocalMap文件: "
        f"{path}\n"
        "请先重启/启动 localmap/scripts/run_perception_stack.sh，让 live_local_map 节点生成 machine_root 版 LocalMap。"
    )

with path.open("r", encoding="utf-8") as file:
    data = json.load(file)

frame_id = data.get("frame_id")
if frame_id != "machine_root":
    raise SystemExit(
        "base LocalMap frame 不正确: "
        f"{frame_id}，期望 machine_root。\n"
        f"文件: {path}\n"
        "这通常是 live_latest 里残留了 fake_base 阶段旧文件。请重启 run_perception_stack.sh，"
        "或删除 localmap/exports/live_latest 后等待 live_local_map 重新生成。"
    )
PY

echo "1/4 导出OctoMap obstacles -> LocalMap"
# shellcheck disable=SC2086
/usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/export_octomap_markers_to_local_map.py" \
  --base-local-map "${BASE_LOCAL_MAP_JSON}" \
  --expected-frame machine_root \
  --bounds ${OBSTACLE_EXPORT_BOUNDS} \
  --box-size "${OCTOMAP_BOX_SIZE}" \
  --max-obstacles "${OCTOMAP_MAX_OBSTACLES}" \
  --output "${LOCAL_MAP_JSON}"

echo "2/4 生成RRT请求"
echo "    bucket tip: ${BUCKET_TIP_JSON}"
echo "    target: ${TARGET_KIND}:${TARGET_ID}, task_mode=${TASK_MODE}"
/usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/generate_rrt_request_from_local_map.py" \
  --local-map "${LOCAL_MAP_JSON}" \
  --bucket-tip "${BUCKET_TIP_JSON}" \
  --target-kind "${TARGET_KIND}" \
  --target-id "${TARGET_ID}" \
  --task-mode "${TASK_MODE}" \
  --output "${RRT_REQUEST_JSON}"

echo "3/4 生成bucket-tip简单避障轨迹"
WORKSPACE_ARGS=()
if [[ "${USE_REACHABLE_WORKSPACE}" == "1" ]]; then
  WORKSPACE_ARGS=(
    --reachable-workspace "${REACHABLE_WORKSPACE_JSON}"
    --workspace-mode "${WORKSPACE_MODE}"
  )
else
  WORKSPACE_ARGS=(--disable-reachable-workspace)
fi

# shellcheck disable=SC2086
/usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/generate_simple_rrt_trajectory_from_request.py" \
  --request "${RRT_REQUEST_JSON}" \
  --output "${TRAJECTORY_JSON}" \
  --bounds ${PLANNING_BOUNDS} \
  "${WORKSPACE_ARGS[@]}" \
  --collision-radius "${COLLISION_RADIUS}" \
  --mask-start-radius "${MASK_START_RADIUS}" \
  --mask-goal-radius "${MASK_GOAL_RADIUS}" \
  --waypoint-count "${WAYPOINT_COUNT}" \
  --max-iterations "${MAX_ITERATIONS}" \
  --seed "${RRT_SEED}"

echo "4/4 生成ONNX observation waypoint切片"
/usr/bin/python3 "${AIRY_ROOT}/localmap/scripts/generate_observation_waypoint_slice.py" \
  --trajectory "${TRAJECTORY_JSON}" \
  --bucket-tip "${BUCKET_TIP_JSON}" \
  --output "${OBS_SLICE_JSON}"

cat <<EOF

规划链路完成：
  LocalMap:    ${LOCAL_MAP_JSON}
  RRT request: ${RRT_REQUEST_JSON}
  Trajectory:  ${TRAJECTORY_JSON}
  Obs slice:   ${OBS_SLICE_JSON}

如需RViz显示轨迹，请单独运行：
  /usr/bin/python3 localmap/scripts/publish_trajectory_markers.py --trajectory ${TRAJECTORY_JSON}
EOF
