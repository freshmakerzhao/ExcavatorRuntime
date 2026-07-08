# AiryLidar LocalMap 离线开发入口

这个目录只做雷达感知到 `LocalMap` 的最小输入契约和离线工具，不做真机 PWM/UDP 控制，不修改 Unity 旧场景，也不修改 vendor SDK。

## 当前边界

- 原始点云来自 ROS2 topic：`/rslidar_points`
- 原始点云 frame：`rslidar`
- 当前点类型：`XYZIRT`
- 字段：`x, y, z, intensity, ring, timestamp`
- RRT* 不直接消费原始点云；它消费 `LocalMap` 中的 `ground / obstacles / dig_targets / dump_targets`
- `LocalMap` 阶段输出坐标应统一到 `machine_root` 或 Unity 的 `MachineRoot`
- `rslidar -> machine_root` 外参必须显式进入链路，不能靠 RViz 视觉效果代替标定

## 文件说明

当前代码按运行职责分层：

- `apps/perception/`：感知与建图入口，包括实时点云坐标转换、实时 `LocalMap`、OctoMap、OctoMap obstacle 导出和在线几何检查。
- `apps/planning/`：规划入口，包括 RRT 请求生成、bucket-tip 简单 RRT、轨迹命令和 observation waypoint 切片。
- `apps/visualization/`：RViz 可视化入口，包括可达区域 marker 和规划轨迹 marker。
- `apps/data_tools/`：离线/数据工具，包括 bag/npz 导出、离线 pipeline 和 OctoMap 快照保存。
- `scripts/`：兼容入口层，保留旧命令路径；实际实现转发到 `apps/*`。
- `localmap_core/`：感知、规划和可视化共享的核心模块，不直接依赖 ROS 节点生命周期。

- `schemas/local_map.schema.json`：第一版 `LocalMap` JSON Schema
- `examples/mock_local_map.json`：离线 mock 示例，可作为 RRT* 输入样例
- `scripts/inspect_bag_points.py`：兼容入口，检查 bag 中 `/rslidar_points` 的 frame、字段、点数和范围
- `scripts/export_first_cloud.py`：兼容入口，导出第一帧点云到 `localmap/exports/`
- `scripts/export_live_cloud.py`：兼容入口，从在线 `/rslidar_points` 抓一帧并导出为同样格式的 NPZ/CSV
- `scripts/generate_local_map_from_npz.py`：兼容入口，把离线 NPZ 点云、外参和 target 配置合成第一版 `LocalMap`
- `scripts/generate_rrt_request_from_local_map.py`：兼容入口，把 `LocalMap`、bucket tip 和 `machine_profile.json` 组织成 RRT* 请求
- `scripts/bridge_bucket_tip_from_tf.py`：兼容入口，把运动学包输出的 `/bucket_tip_pose_base` 转成 `machine_root` bucket tip JSON
- `scripts/generate_simple_rrt_trajectory_from_request.py`：兼容入口，从 RRT* 请求生成第一版 bucket-tip 简单避障轨迹，默认受 `shared/reachable_workspaces/scale_excavator_workspace.json` 约束
- `scripts/generate_observation_waypoint_slice.py`：兼容入口，生成 38 维 observation 中 `idx 15..26` 的 waypoint 相关切片
- `scripts/run_perception_stack.sh`：兼容入口，一键启动雷达驱动、实时坐标转换、实时LocalMap、OctoMap和可达区域marker，可选轨迹marker
- `scripts/run_planning_once.sh`：兼容入口，从当前 OctoMap 一次性生成 LocalMap、RRT请求、轨迹和observation切片
- `config/extrinsics_rslidar_to_machine_root.measured.json`：当前实测 `rslidar -> machine_root` 外参
- `config/targets.mock.json`：占位 dig/dump target，后续由任务配置或感知模块生成
- `config/bucket_tip.machine_root.measured.json`：占位 bucket tip，真机运行时由状态估计/FK提供
- `config/bucket_tip_tf_bridge.machine_root.json`：`base_link` bucket tip 到 `machine_root` bucket tip 的坐标桥接配置
- `docs/octomap_integration_plan.md`：OctoMap 接入路线、运行命令和后续 LocalMap/RRT* 适配计划
- `tests/test_localmap_pipeline.py`：少量关键行为测试，覆盖外参变换、NaN过滤、LocalMap生成
- `tests/test_trajectory_contracts.py`：少量契约测试，覆盖 RRT* 请求、轨迹命令和 observation waypoint 切片
- `tests/test_reachable_workspace.py`：少量可达区域测试，覆盖 20 点可达体 inside 判断和 RRT waypoint 约束
- `exports/`：本地导出产物，已在 `.gitignore` 中忽略

## 推荐运行方式

ROS2 Jazzy 的 Python 扩展依赖系统 Python 3.12。当前机器如果 shell 在 conda 环境中，建议显式使用 `/usr/bin/python3`。

## 运动学 TF 接入

原 `/home/zhaoshuai/workspace_uinty/RL_prj/TF` 项目已经作为源码子模块纳入：

```text
AiryLidar/kinematics/excavator_kinematics
```

`ros2_ws/src/excavator_kinematics` 是指向该源码目录的软连接，便于在同一个 ROS2 overlay 中编译：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select excavator_kinematics
source install/setup.zsh
```

启动 FK/TF 节点：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

ros2 launch excavator_kinematics excavator_tf.launch.py
```

TF 包输出的 `/bucket_tip_pose_base` 使用 ROS 坐标约定：

```text
X forward, Y left, Z up
```

规划链路使用 Unity/MachineRoot 约定：

```text
X right, Y up, Z forward
```

桥接节点把 bucket tip 位置转换到 `machine_root` 并持续写出 JSON：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_base \
  --output-topic /localmap/bucket_tip_machine_root_pose \
  --bridge localmap/config/bucket_tip_tf_bridge.machine_root.json \
  --output-json localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

`localmap/scripts/run_planning_once.sh` 会优先读取 live JSON：

```text
localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

如果 live JSON 不存在，则回退到：

```text
localmap/config/bucket_tip.machine_root.measured.json
```

一键感知栈默认不启动 bucket tip bridge；如果 TF 节点已经在运行，可以打开：

```bash
RUN_BUCKET_TIP_BRIDGE=1 localmap/scripts/run_perception_stack.sh
```

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj
source /opt/ros/jazzy/setup.zsh

/usr/bin/python3 AiryLidar/localmap/scripts/inspect_bag_points.py \
  AiryLidar/bags/airy_20260706_202359 \
  --frames 3
```

导出第一帧点云：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj
source /opt/ros/jazzy/setup.zsh

/usr/bin/python3 AiryLidar/localmap/scripts/export_first_cloud.py \
  AiryLidar/bags/airy_20260706_202359 \
  --max-csv-points 2000
```

输出：

- `AiryLidar/localmap/exports/rslidar_points_first_frame.npz`
- `AiryLidar/localmap/exports/rslidar_points_first_frame_sample.csv`

从在线 `/rslidar_points` 抓一帧：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/export_live_cloud.py \
  --output-dir localmap/exports/live_latest \
  --timeout-s 5
```

输出：

- `AiryLidar/localmap/exports/live_latest/rslidar_points_live_frame.npz`
- `AiryLidar/localmap/exports/live_latest/rslidar_points_live_frame_sample.csv`

这一步只抓一帧，不替代后续实时 LocalMap 节点；它用于确认在线 topic 可以进入和离线 bag 相同的数据处理入口。

实时发布 `machine_root` 点云：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/transform_live_cloud_to_base.py \
  --input-topic /rslidar_points \
  --output-topic /localmap/machine_root_points \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json
```

该脚本只做在线坐标转换和可视化topic发布：

- 输入：`/rslidar_points`，`frame_id=rslidar`
- 输出：`/localmap/machine_root_points`，`frame_id=machine_root`
- 字段保持 `XYZIRT`：`x, y, z, intensity, ring, timestamp`
- 默认发布静态 TF：`world -> machine_root`，用于让 RViz 识别目标坐标系

RViz 查看转换后点云：

```bash
rviz2 -d /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/rviz/airy_points.rviz
```

在 RViz 中建议：

- `Global Options / Fixed Frame` 改为 `machine_root`
- `PointCloud2 / Topic` 改为 `/localmap/machine_root_points`
- `PointCloud2 / Color Transformer` 可用 `AxisColor`
- `PointCloud2 / Axis` 可设为 `Y`，检查地面高度和上下方向
- `Grid / Plane` 改为 `XZ`，因为 `machine_root` 中 `+Y` 是上方，地面是 XZ 平面
- 可添加 `Axes` display，Reference Frame 设为 `machine_root`，用于看原点和红/绿/蓝轴
- 可添加 `MarkerArray` display，Topic 设为 `/localmap/reachable_workspace_markers`，用于看 bucket tip 可达区域

如果长时间收不到转换后点云，先检查：

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /localmap/machine_root_points
ros2 topic echo /localmap/machine_root_points --once --field header
```

数值检查转换是否成功：

```bash
/usr/bin/python3 localmap/scripts/inspect_live_cloud_geometry.py \
  --topic /localmap/machine_root_points \
  --expected-frame machine_root \
  --up-axis y \
  --frames 3
```

重点看：

- `frame_id` 是否等于 `machine_root`
- `ground_estimate_y_m_p05` 是否接近地面高度，通常应接近 `0m`
- `x/y/z_range_m` 是否符合现场量到的左右、上下、前后方向
- 移动一个已知物体时，数值变化方向是否和 `machine_root` 约定一致

单独发布可达区域 marker：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/publish_reachable_workspace_markers.py \
  --workspace /home/zhaoshuai/workspace_uinty/RL_prj/shared/reachable_workspaces/scale_excavator_workspace.json \
  --mode MoveToDig
```

`run_perception_stack.sh` 默认会启动这个 marker 发布节点；如果只想跑点云和 OctoMap，可以设置 `RUN_REACHABLE_WORKSPACE_MARKERS=0`。

实时生成最小 `LocalMap`：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/run_live_local_map_node.py \
  --input-topic /localmap/machine_root_points \
  --output-json localmap/exports/live_latest/local_map.live.json \
  --targets localmap/config/targets.mock.json \
  --expected-frame machine_root \
  --write-every 10 \
  --publish-every 10
```

第一版实时 `LocalMap` 与 Spinelli 等论文中的模块化路线一致，但只实现最小可验证链路：

- 输入已经统一坐标的实时点云：`/localmap/machine_root_points`
- 在线估计 `ground` 平面
- `dig_targets / dump_targets` 暂时来自配置
- `obstacles` 暂时为空，后续由 heightmap/voxel/OctoMap 风格模块填充
- 输出 JSON 文件：`localmap/exports/live_latest/local_map.live.json`
- 可选 ROS 字符串 topic：`/localmap/local_map_json`

检查实时 `LocalMap`：

```bash
python3 -m json.tool localmap/exports/live_latest/local_map.live.json | sed -n '1,120p'
ros2 topic echo /localmap/local_map_json --once
```

生成第一版 `LocalMap`：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_local_map_from_npz.py
```

如果要临时裁剪 base frame 点云，可以传入：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_local_map_from_npz.py \
  --bounds -2.0 4.0 -3.0 3.0 -0.2 2.0
```

输出：

- `AiryLidar/localmap/exports/local_map_from_npz.mock.json`

检查外参配置：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/inspect_extrinsics.py \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json
```

用 `machine_root` 外参生成 LocalMap：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_local_map_from_npz.py \
  --npz localmap/exports/airy_20260707_095607/rslidar_points_first_frame.npz \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json \
  --bag-path bags/airy_20260707_095607 \
  --output localmap/exports/airy_20260707_095607/local_map_machine_root.measured.json
```

输出 JSON 的 `frame_id` 应为：

```text
machine_root
```

生成 RRT* 请求：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_rrt_request_from_local_map.py
```

输出：

- `AiryLidar/localmap/exports/rrt_star_request.mock.json`

临时生成 mock 轨迹命令：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_mock_trajectory_from_rrt_request.py
```

输出：

- `AiryLidar/localmap/exports/trajectory_command.mock.json`

注意：这个脚本不是 RRT*，只是用直线 waypoint 打通接口。真实接入时替换这一段，不改上下游 JSON 契约。

生成 observation waypoint 切片：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 localmap/scripts/generate_observation_waypoint_slice.py
```

输出：

- `AiryLidar/localmap/exports/observation_waypoint_slice.mock.json`

该文件只覆盖 38 维 observation 的以下位置：

```text
15..23: 未来3个 waypoint 相对 bucket tip 的误差，按 distance_normalizer 归一化
24: progress
25: tube_signed
26: isFinal
```

完整 38 维 observation 仍由部署侧状态估计器按主计划第 2.2 节组装，本目录不改变 ONNX observation 维度。

一键运行离线 pipeline：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh

export BAG_DIR=bags/airy_repositioned_20260707_152018
export EXPORT_DIR=localmap/exports/$(basename "$BAG_DIR")

/usr/bin/python3 localmap/scripts/run_offline_localmap_pipeline.py \
  --bag "$BAG_DIR" \
  --output-dir "$EXPORT_DIR" \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json \
  --bucket-tip localmap/config/bucket_tip.machine_root.measured.json
```

如果已经导出过第一帧 NPZ，只想复用现有点云并重跑 LocalMap/RRT/Observation：

```bash
/usr/bin/python3 localmap/scripts/run_offline_localmap_pipeline.py \
  --bag "$BAG_DIR" \
  --output-dir "$EXPORT_DIR" \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json \
  --bucket-tip localmap/config/bucket_tip.machine_root.measured.json \
  --reuse-export
```

检查将要执行的命令但不写产物：

```bash
/usr/bin/python3 localmap/scripts/run_offline_localmap_pipeline.py \
  --bag "$BAG_DIR" \
  --output-dir "$EXPORT_DIR" \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json \
  --bucket-tip localmap/config/bucket_tip.machine_root.measured.json \
  --dry-run
```

运行关键测试：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

/usr/bin/python3 -m unittest discover -s localmap/tests -p 'test_*.py'
```

## 已验证的离线 bag 摘要

当前录包 `AiryLidar/bags/airy_20260706_202359` 中：

- `/rslidar_points`：196 帧，约 10 Hz
- `/rslidar_imu_data`：3923 帧，约 200 Hz
- 首帧点云 layout：`height=900, width=96, point_step=26`
- 每帧点槽：`86400`
- 前 3 帧有效 XYZ 点数：约 `66583 ~ 66657`
- ring 范围：`0 ~ 95`
- intensity 范围：`0 ~ 255`

## 下一步开发顺序

1. 使用 Unity/真机一致的 `machine_root` 作为唯一感知与规划坐标系。
2. 测量并维护 `extrinsics_rslidar_to_machine_root.measured.json`。
3. 用 measured 外参生成 `frame_id=machine_root` 的 LocalMap。
4. 基于 `machine_root` 点云做最小预处理：过滤 NaN、裁剪工作空间、降采样。
5. 从 `machine_root` 点云估计 `ground`，并生成最小 `obstacles`。
6. 用任务配置或人工标注替换 mock `dig_targets / dump_targets`。
7. 让真实 RRT* 读取 `rrt_star_request.v1`，输出 `trajectory_command.v1` 中的 `waypoints_base`。
8. 后续把这些 waypoint error 填入既有 ONNX 38 维 observation，不改变 observation 维度。

## machine_root 方向约定

短期雷达仍放在固定支架上，但所有点云会通过外参转换到挖机语义坐标 `machine_root`。`machine_root` 必须对齐 Unity `MachineRoot`：

- `+X`：挖机右侧，Unity 红色轴
- `+Y`：竖直向上，Unity 绿色轴
- `+Z`：挖机前方或伸臂方向，Unity 蓝色轴

Unity MCP 只读检查结果：

- `MachineRoot` 是 `TrajectoryEnvironment_01/Machine/ExcavatorScaleDown/Frames/MachineRoot`
- `MachineRoot` 相对 `Frames` 的 localPosition/localRotation 为 0
- `right=(1,0,0)`
- `up=(0,1,0)`
- `forward=(0,0,1)`
- 当前 `BucketTip - MachineRoot ≈ (-0.113, -0.459, +0.714)`

2026-07-08 当前 measured 外参：

```text
translation_m = [1.3, 0.624, 0.3]
rslidar +X -> machine_root +Z
rslidar +Y -> machine_root +X
rslidar +Z -> machine_root -Y
```

该轴映射使用 `axis_mapping_matrix`，不要强行写成 `rotation_rpy_deg`。生成的 measured LocalMap 已校验：

```text
frame_id = machine_root
```

## 当前离线生成结果

使用 `rslidar_points_first_frame.npz` 和 mock 外参生成 `LocalMap` 时：

- 输入有效点：`66583`
- 输出 base frame 点：`66583`
- `frame_id`：`machine_root`
- 外参 ID：`T_base_rslidar.mock.v0`
- 地面模型：`plane`
- `dig_targets`：`1`
- `dump_targets`：`1`

注意：当前外参为零位 mock，所以这个结果只能验证链路，不代表真实挖掘机坐标。

## 2026-07-07 在线抓帧验证

当天新录 bag `bags/airy_20260707_095607`：

- `/rslidar_points`：`197` 帧，约 `10Hz`
- `/rslidar_imu_data`：`3930` 帧，约 `200Hz`
- `frame_id`：`rslidar`
- 字段：`x, y, z, intensity, ring, timestamp`
- `ring`：`0 ~ 95`
- 点云 layout：`width=96` 稳定，`height` 在 `864 ~ 900` 间轻微变化

在线抓帧 `localmap/exports/live_20260707_check/rslidar_points_live_frame.npz`：

- 原始点槽：`86016`
- 有效点：`68046`
- `frame_id`：`rslidar`
- 继续生成的 `LocalMap / RRT* request / trajectory_command / observation waypoint slice` 均通过 schema 校验

说明：height 轻微变化来自 SDK 分帧列数变化；由于 width 仍为 `96`、ring 仍为 `0~95`、topic 频率正常，当前不视为阻塞问题。

继续生成 RRT* 请求、mock 轨迹和 observation waypoint 切片时：

- `rrt_star_request.mock.json` 的 `start_bucket_tip_base` 来自 `config/bucket_tip.machine_root.measured.json`
- `target_threshold`、`tube_radius`、`waypoint_lookahead` 来自 `shared/machine_profile.json`
- `trajectory_command.mock.json` 默认生成 `5` 个 waypoint，但策略每步只看未来 `3` 个
- `observation_waypoint_slice.mock.json` 的 `indices` 固定为 `[15..26]`
