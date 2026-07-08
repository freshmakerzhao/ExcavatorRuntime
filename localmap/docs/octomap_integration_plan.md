# OctoMap 集成计划

本文档记录 AiryLidar 感知链路接入 OctoMap 的最小路线。当前阶段只做雷达、LocalMap、RRT* 输入，不做 PWM/UDP 真机控制，不修改 Unity 旧场景，不修改 vendor SDK。

## 当前结论

- 使用官方 ROS2 Jazzy `octomap_server`，不把 OctoMap 源码复制进 AiryLidar。
- 输入点云使用我们已经验证过的 `/localmap/machine_root_points`。
- OctoMap 的 `frame_id` 第一版设为 `machine_root`。
- 第一版关闭 `filter_ground_plane`，因为 `machine_root` 采用 Unity 风格 `+Y` 向上，而 `octomap_server` 的地面滤波更偏 ROS 常见 `+Z` 向上假设。先建 3D 占据图，再由我们自己的 LocalMap 层解释 ground/obstacle。
- `LocalMap` 仍然是 RRT* 的输入契约；RRT* 不直接消费原始点云。

## 安装

如果还没安装 OctoMap ROS 包，先执行：

```bash
sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-ros
```

这条命令只安装 ROS 包，不需要 `apt update`，也不应触发内核或显卡驱动更新。

## 实时运行顺序

终端 1：启动 Airy 版 rslidar 驱动，输出 `/rslidar_points`。

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=/home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/runtime/config_airy_current.yaml
```

终端 2：把雷达点云转换到 `machine_root`。

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/transform_live_cloud_to_base.py \
  --input-topic /rslidar_points \
  --output-topic /localmap/machine_root_points \
  --extrinsics localmap/config/extrinsics_rslidar_to_machine_root.measured.json
```

终端 3：启动 OctoMap 建图。

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

localmap/scripts/run_octomap_mapping.sh
```

可选调参：

```bash
OCTOMAP_RESOLUTION=0.03 OCTOMAP_MAX_RANGE=4.0 \
  localmap/scripts/run_octomap_mapping.sh
```

如果 RViz 中出现大量远处墙体、支架或顶部无关方块，先裁剪进入 OctoMap 的工作空间。这里的坐标已经是 `machine_root`，其中 `+Y` 向上；当前缩比挖机的 `machine_root` 在空中，地面通常会出现在负 `Y`：

```bash
OCTOMAP_RESOLUTION=0.05 \
OCTOMAP_MAX_RANGE=4.0 \
OCTOMAP_POINT_CLOUD_MIN_X=-1.5 \
OCTOMAP_POINT_CLOUD_MAX_X=3.0 \
OCTOMAP_POINT_CLOUD_MIN_Y=-0.70 \
OCTOMAP_POINT_CLOUD_MAX_Y=1.00 \
OCTOMAP_POINT_CLOUD_MIN_Z=-0.5 \
OCTOMAP_POINT_CLOUD_MAX_Z=4.0 \
  localmap/scripts/run_octomap_mapping.sh
```

这些边界是现场调试起点，不是最终标定值；后续应根据挖机、土堆、铲斗运动范围收紧。

终端 4：RViz 查看。

```bash
rviz2 -d /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/rviz/airy_points.rviz
```

在 RViz 中可以添加：

- `PointCloud2`：`/localmap/machine_root_points`
- `MarkerArray`：通常看 `/occupied_cells_vis_array`
- `MarkerArray`：`/localmap/reachable_workspace_markers`，看 bucket tip 可达区域
- `Map`：如果需要看 2D 投影，可查看 `/projected_map`
- `OccupancyMap`：如果 RViz 插件可用，可查看 `/octomap_binary` 或 `/octomap_full`

## 检查命令

```bash
ros2 topic list | grep -E 'octomap|occupied|projected|machine_root'
ros2 topic hz /localmap/machine_root_points
ros2 topic hz /occupied_cells_vis_array
ros2 topic hz /localmap/reachable_workspace_markers
ros2 topic echo /octomap_binary --once --field header
```

保存当前 OctoMap：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

localmap/scripts/save_octomap_snapshot.sh
```

指定输出路径：

```bash
localmap/scripts/save_octomap_snapshot.sh \
  localmap/exports/octomap/box_scene_test.bt
```

## 累计地图与实时视野

`octomap_server` 默认是累计建图：每一帧点云都会被融合进占据树。这样适合静态场景，但如果现场有人移动物体、铲斗运动、或者点云比较杂，RViz 中会出现“残影”和越来越多的 occupied cells。

如果当前目标是调试“实时当前视野”，可以周期性 reset OctoMap：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

OCTOMAP_RESET_INTERVAL_S=1.0 \
  localmap/scripts/run_octomap_mapping.sh
```

`OCTOMAP_RESET_INTERVAL_S` 可以按需要调小或调大：

- `0.5`：更像实时刷新，但画面可能闪烁，CPU开销更高。
- `1.0`：推荐现场调试起点。
- `2.0`：保留一点短时累计，画面更稳定。

长期方案不是无限累计，也不是简单每帧清空，而是让 RRT* 使用一个局部、短时、裁剪后的占据图；当前 reset 参数只是最快可验证的调试模式。

## 后续接入 LocalMap / RRT*

下一步不是重写 OctoMap，而是增加一个适配层：

```text
octomap_server outputs
-> occupied cells / octomap message
-> LocalMap.obstacles
-> rrt_star_request.obstacles
-> RRT* collision checking
```

适配层需要解决：

- 工作空间裁剪：只保留挖掘机附近、铲斗可能经过的区域。
- 地面解释：`machine_root` 中 `+Y` 向上，地面是 XZ 平面。
- 障碍物表达：第一版可把 occupied cells 转成 `shape=box` 的 obstacle。
- 目标表达：`dig_targets` 和 `dump_targets` 仍先来自任务配置，后续再由土堆/料堆感知生成。

## 第一版 bucket-tip 简单避障链路

当前先不做关节空间 RRT*，只在 `machine_root` 中规划 bucket tip 的 xyz waypoints。完整离线/在线混合调试命令如下：

日常推荐使用两个入口减少终端数量：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

OCTOMAP_RESET_INTERVAL_S=1.0 \
OCTOMAP_RESOLUTION=0.05 \
OCTOMAP_MAX_RANGE=4.0 \
OCTOMAP_POINT_CLOUD_MIN_X=-1.5 \
OCTOMAP_POINT_CLOUD_MAX_X=3.0 \
OCTOMAP_POINT_CLOUD_MIN_Y=-0.70 \
OCTOMAP_POINT_CLOUD_MAX_Y=1.00 \
OCTOMAP_POINT_CLOUD_MIN_Z=-0.5 \
OCTOMAP_POINT_CLOUD_MAX_Z=4.0 \
  localmap/scripts/run_perception_stack.sh
```

这个入口默认会同时启动：

- `rslidar_sdk`：发布 `/rslidar_points`
- `transform_live_cloud_to_base.py`：发布 `/localmap/machine_root_points`
- `run_live_local_map_node.py`：持续写出 `localmap/exports/live_latest/local_map.live.json`
- `run_octomap_mapping.sh`：发布 OctoMap occupied cells
- `publish_reachable_workspace_markers.py`：发布 `/localmap/reachable_workspace_markers`

另开一个终端，需要规划一次时执行：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar

localmap/scripts/run_planning_once.sh
```

下面是展开后的手动命令，便于排查每一步：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

/usr/bin/python3 localmap/scripts/export_octomap_markers_to_local_map.py \
  --bounds -1.5 3.0 -0.42 1.0 -0.5 4.0 \
  --box-size 0.20 \
  --max-obstacles 1000 \
  --output localmap/exports/live_latest/local_map.octomap_obstacles.json

/usr/bin/python3 localmap/scripts/generate_rrt_request_from_local_map.py \
  --local-map localmap/exports/live_latest/local_map.octomap_obstacles.json \
  --bucket-tip localmap/config/bucket_tip.machine_root.measured.json \
  --output localmap/exports/live_latest/rrt_star_request.octomap_obstacles.json

/usr/bin/python3 localmap/scripts/generate_simple_rrt_trajectory_from_request.py \
  --request localmap/exports/live_latest/rrt_star_request.octomap_obstacles.json \
  --output localmap/exports/live_latest/trajectory_command.simple_rrt.json \
  --bounds -1.5 3.0 -0.70 1.00 -0.5 4.0 \
  --reachable-workspace /home/zhaoshuai/workspace_uinty/RL_prj/shared/reachable_workspaces/scale_excavator_workspace.json \
  --workspace-mode MoveToDig \
  --collision-radius 0.05 \
  --mask-start-radius 0.15 \
  --mask-goal-radius 0.45 \
  --waypoint-count 5

/usr/bin/python3 localmap/scripts/generate_observation_waypoint_slice.py \
  --trajectory localmap/exports/live_latest/trajectory_command.simple_rrt.json \
  --bucket-tip localmap/config/bucket_tip.machine_root.measured.json \
  --output localmap/exports/live_latest/observation_waypoint_slice.simple_rrt.json
```

RViz 中查看规划路径时，单独运行轨迹 marker 发布节点：

```bash
/usr/bin/python3 localmap/scripts/publish_trajectory_markers.py \
  --trajectory localmap/exports/live_latest/trajectory_command.simple_rrt.json
```

RViz 中添加 `MarkerArray`：

```text
/localmap/planned_bucket_tip_markers
```

当前 `generate_simple_rrt_trajectory_from_request.py` 默认读取：

```text
/home/zhaoshuai/workspace_uinty/RL_prj/shared/reachable_workspaces/scale_excavator_workspace.json
```

并按 `task_mode` 选择 `MoveToDig` 或 `CarryMaterial` 可达体。RRT 的采样点、扩展边、shortcut 边和最终 bucket-tip waypoints 都会检查是否在这个可达体内。只有排查问题时才建议临时加 `--disable-reachable-workspace`。

注意：这条链路只做简单空间避障。地面和挖掘目标附近必须 mask 掉，否则 OctoMap 会把土堆/地面当作普通障碍，导致起点或目标处于碰撞状态。

## 为什么不马上开启 SLAM

当前雷达固定在支架上，点云通过实测外参进入挖机语义坐标 `machine_root`。只要支架和挖机相对位姿不变，OctoMap 可以在这个局部坐标系内稳定建图，不需要 SLAM。

以后如果雷达装到移动底盘或挖机上，或者需要跨位置累计大范围地图，再引入 SLAM / localization，把 OctoMap 的 `frame_id` 从 `machine_root` 切到 `map`。
