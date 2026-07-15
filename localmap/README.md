# LocalMap：右手 ROS 生产链路

唯一 ROS 坐标根为 `machine_root_ros`：`+X` 前、`+Y` 左、`+Z` 上。雷达、FK、
LocalMap、workspace、轨迹和 RViz 都使用该坐标系。Unity 左手 `machine_root` 不在
ROS TF 或 LocalMap JSON 中出现；它只在 `runtime_bridge/unity_observation_adapter.py`
组装 ONNX 38 维 observation 时转换。

## 启动

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch waji_description display.launch.py
```

另开终端：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
RUN_BUCKET_TIP_BRIDGE=1 \
RUN_TRAJECTORY_MARKERS=1 \
localmap/scripts/run_perception_stack.sh
```

启动脚本总是加载 `config/perception.json`，其中包含唯一的右手 frame、外参、目标、
裁剪边界和 topic。不要用环境变量替换这些坐标语义。

RViz Fixed Frame 使用 `machine_root_ros`，主要话题：

- `/localmap/machine_root_ros_points`
- `/bucket_tip_pose_machine_root_ros`
- `/localmap/bucket_tip_machine_root_ros_pose`
- `/occupied_cells_vis_array`

## 重要边界

- `extrinsics_rslidar_to_machine_root_ros.derived.v1.json` 和右手 target/workspace 由旧
  Unity 数据代数重表达，尚不是新的现场测量；不得把它们当作 Bucket Tip 标定通过证据。
- 真机标定仍以 `EvaluationReport/2026-07-15_bucket_tip_real_machine_coordinate_diagnosis.md`
  的静止姿态 protocol 为准。
- 默认不启动运动。只有冻结 38D observation 与真机静止标定均通过后，才评审运动授权。

## 回归

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=AiryLidar/localmap \
python3 -m pytest AiryLidar/localmap/tests -q
```
