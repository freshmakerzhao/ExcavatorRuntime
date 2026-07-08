# AiryLidar

AiryLidar 是缩比挖掘机真机侧的感知、局部地图、bucket-tip 规划实验工程。

当前只做：

- RoboSense Airy 雷达接入
- 点云从 `rslidar` 转到 `machine_root`
- 实时 `LocalMap` / OctoMap
- 简单 bucket-tip RRT 避障轨迹
- FK bucket tip 坐标桥接
- RViz 可视化

当前不做：

- 不发送 PWM
- 不发送 UDP 真机控制命令
- 不修改 Unity 旧场景或旧 prefab
- 不修改 ONNX observation 维度

## 目录

```text
runtime/                 # 当前雷达运行配置
ros2_ws/                 # ROS2 overlay workspace
rslidar_sdk/             # Airy 版 RoboSense SDK 源码，只读使用
kinematics/              # 挖掘机 FK / TF，输入关节角，输出 bucket tip
localmap/                # 感知、LocalMap、OctoMap、RRT、RViz marker 工具
rviz/                    # RViz 配置
docs/                    # 雷达接线、端口、防火墙等运行笔记
```

## 1. 准备环境

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
```

首次或源码变化后编译 overlay：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

如果 OctoMap 未安装：

```bash
sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-ros
```

## 2. 配置雷达网口

当前 Airy 雷达配置：

```text
雷达 IP: 192.168.1.200
上位机 IP: 192.168.1.103
MSOP: 6700
DIFOP: 7789
IMU: 6688
```

临时配置网口：

```bash
sudo ip link set enp130s0 up
sudo ip addr flush dev enp130s0
sudo ip addr add 192.168.1.103/24 dev enp130s0
ip -brief addr show enp130s0
ping -I 192.168.1.103 -c 3 192.168.1.200
```

如果防火墙拦截 UDP：

```bash
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6700
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6688
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 7789
```

## 3. 启动感知栈

一个终端启动：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

OCTOMAP_RESET_INTERVAL_S=1.0 \
OCTOMAP_RESOLUTION=0.05 \
OCTOMAP_MAX_RANGE=4.0 \
OCTOMAP_POINT_CLOUD_MIN_X=-1.5 \
OCTOMAP_POINT_CLOUD_MAX_X=3.0 \
OCTOMAP_POINT_CLOUD_MIN_Y=-0.42 \
OCTOMAP_POINT_CLOUD_MAX_Y=1.00 \
OCTOMAP_POINT_CLOUD_MIN_Z=-0.5 \
OCTOMAP_POINT_CLOUD_MAX_Z=4.0 \
localmap/scripts/run_perception_stack.sh
```

这个脚本会启动：

```text
rslidar_sdk                         -> /rslidar_points, /rslidar_imu_data
transform_live_cloud_to_base.py     -> /localmap/machine_root_points
run_live_local_map_node.py          -> localmap/exports/live_latest/local_map.live.json
octomap_server_node                 -> /occupied_cells_vis_array
publish_reachable_workspace_markers -> /localmap/reachable_workspace_markers
```

常用检查：

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /localmap/machine_root_points
ros2 topic hz /occupied_cells_vis_array
python3 -m json.tool localmap/exports/live_latest/local_map.live.json | sed -n '1,60p'
```

## 4. 打开 RViz

另一个终端：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh
rviz2 -d rviz/airy_points.rviz
```

主要看这些 display：

```text
/localmap/machine_root_points              # machine_root 下的实时点云
/occupied_cells_vis_array                  # OctoMap 占据栅格
/localmap/reachable_workspace_markers      # bucket tip 可达区域
/localmap/planned_bucket_tip_markers       # 规划后的轨迹
Axes: machine_root                         # 坐标原点和方向
```

## 5. 可选：启动 bucket tip FK

如果只看雷达和 OctoMap，可以跳过本节。

启动 FK / TF：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

ros2 launch excavator_kinematics excavator_tf.launch.py
```

FK 的输入是关节角，不是油缸位移：

```text
/joint_states: sensor_msgs/msg/JointState
swing_joint, boom_joint, arm_joint, bucket_joint
单位: rad
```

测试输入：

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'],
  position: [0.0, 0.3, -0.8, 0.5]
}" -r 10
```

单独启动 bucket tip bridge：

```bash
/usr/bin/python3 localmap/scripts/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_base \
  --output-topic /localmap/bucket_tip_machine_root_pose \
  --bridge localmap/config/bucket_tip_tf_bridge.machine_root.json \
  --output-json localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

或者在感知栈里一起启动：

```bash
RUN_BUCKET_TIP_BRIDGE=1 localmap/scripts/run_perception_stack.sh
```

检查 bucket tip：

```bash
ros2 topic echo /bucket_tip_pose_base --once
ros2 topic echo /localmap/bucket_tip_machine_root_pose --once
python3 -m json.tool localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

## 6. 跑一次规划

感知栈运行后，在新终端执行一次规划：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

PLANNING_BOUNDS="-1.5 3.0 -0.70 1.00 -0.5 4.0" \
OBSTACLE_EXPORT_BOUNDS="-1.5 3.0 -0.42 1.00 -0.5 4.0" \
WORKSPACE_MODE=MoveToDig \
localmap/scripts/run_planning_once.sh
```

输出：

```text
localmap/exports/live_latest/local_map.octomap_obstacles.json
localmap/exports/live_latest/rrt_star_request.octomap_obstacles.json
localmap/exports/live_latest/trajectory_command.simple_rrt.json
localmap/exports/live_latest/observation_waypoint_slice.simple_rrt.json
```

发布轨迹到 RViz：

```bash
/usr/bin/python3 localmap/scripts/publish_trajectory_markers.py \
  --trajectory localmap/exports/live_latest/trajectory_command.simple_rrt.json
```

## 7. 离线 bag

录一段雷达数据：

```bash
mkdir -p bags
timeout --signal=INT 20s ros2 bag record \
  -o bags/airy_$(date +%Y%m%d_%H%M%S) \
  /rslidar_points \
  /rslidar_imu_data
```

检查 bag：

```bash
/usr/bin/python3 localmap/scripts/inspect_bag_points.py \
  bags/<bag_name> \
  --frames 3
```

## 关键坐标系

```text
rslidar       # 原始雷达坐标
machine_root  # 统一感知和规划坐标，对齐 Unity MachineRoot
```

当前外参：

```text
localmap/config/extrinsics_rslidar_to_machine_root.measured.json
```

bucket tip 实时 JSON：

```text
localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

更详细的雷达端口、防火墙、DIFOP、RViz 说明见：

```text
docs/lidar_runtime_notes.md
localmap/README.md
kinematics/README.md
```
