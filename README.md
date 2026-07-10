# ExcavatorRuntime

ExcavatorRuntime 是缩比挖掘机真机侧的感知、局部地图、bucket-tip 规划实验工程。

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
runtime_bridge/          # PC 与 Orin/STM32 的 UDP 状态/动作中转协议
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
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime/ros2_ws
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
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime
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

基础链路与产物结构检查：

```bash
localmap/scripts/run_smoke_check.sh
```

如果还要按 `planning.json` 的时效规则验证当前输入确实可规划：

```bash
localmap/scripts/run_smoke_check.sh --run-planning mock_dig_001
```

## 4. 打开 RViz

另一个终端：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime
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
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime
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

或者启动关节角滑块窗口：

```bash
ros2 run excavator_kinematics joint_slider_publisher \
  --publish-on-change \
  --initial 0.0 0.0 0.0 0.0
```

单独启动 bucket tip bridge：

```bash
python3 localmap/scripts/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_map \
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
ros2 topic echo /bucket_tip_pose_map --once
ros2 topic echo /bucket_tip_pose_unity --once
ros2 topic echo /bucket_tip_observation --once
ros2 topic echo /localmap/bucket_tip_machine_root_pose --once
python3 -m json.tool localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

## 6. 跑一次规划

感知栈运行后，在新终端执行一次规划：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

localmap/scripts/run_planning_once.sh mock_dig_001
```

目标类型和任务模式由 `target_id` 在当前 LocalMap 中的位置自动推导。规划算法参数保存在
`localmap/config/planning.json`；共享的frame、live路径、OctoMap topic和bounds由它引用的
`localmap/config/perception.json` 派生，现场运行不再通过环境变量逐项覆盖。如果要切到左侧远端目标：

```bash
localmap/scripts/run_planning_once.sh mock_dig_left_far
```

只验证 live 输入和展示内部步骤、不发布规划产物：

```bash
localmap/scripts/run_planning_once.sh mock_dig_001 --dry-run
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
python3 localmap/scripts/publish_trajectory_markers.py \
  --trajectory localmap/exports/live_latest/trajectory_command.simple_rrt.json
```

只检查本地 JSON 产物，不检查 ROS topic：

```bash
python3 localmap/apps/diagnostics/run_smoke_check.py --skip-ros
```

## 7. 可选：本机模拟 Orin 中转

终端 1，启动 PC 通信诊断入口：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --reply-zero
```

终端 2，启动 mock Orin relay：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/mock_orin_relay.py
```

如果要把 Orin 状态发布成 ROS2 `/joint_states`，PC 侧加：

```bash
--publish-joint-states
```

连接真实 Orin 时，PC 侧使用：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py --reply-zero
```

这一步只回发零动作，用于验证 Orin -> PC 状态包和 PC -> Orin 动作包通道。网络、动作有效期和日志采样均读取运行配置；不加 `--reply-zero` 时只接收和记录状态。

启动 ONNX policy bridge：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

.venv_runtime/bin/python runtime_bridge/apps/pc_policy_bridge.py
```

网络、模型、机型 profile、waypoint 产物、超时和日志采样统一读取 `runtime_bridge/config/runtime.json`，现场启动不再重复输入。任务切换使用 `--task-mode CarryMaterial`；使用其他部署配置时只需增加 `--config <path>`。

它会接收 Orin `machine_state_v1`，发布 `/joint_states` 给 FK，读取 `/bucket_tip_observation`，组装 38 维 observation 并运行 ONNX。默认不会发送 UDP 动作；完成离线检查和现场安全确认后，必须显式增加 `--enable-motion` 才会向 Orin 发送 `policy_action`。程序不再提供忽略 `control_enabled` 的策略发送参数。
ONNX 输出在 PC 内部仍按训练语义视为 `[-1, 1]` 策略动作；发给 Orin 前会按 `shared/machine_profile.json` 反归一化为物理速度，但 UDP 包里的 `action_type` 仍保持 `normalized_velocity_command` 以兼容 Orin 端解析。其中 `boom/stick/bucket` 单位 m/s，`swing` 单位 rad/s。
默认安全门开启：如果 `estop=true`、`sensor_valid=false`、`stm32_alive=false`、`control_enabled=false` 或存在 `fault_flags`，仍会计算 ONNX 输出，但实际发给 Orin 的动作是零动作。

每个真正成功发往 Orin 的 UDP 动作都会异步追加到本地会话日志：

```text
runtime_bridge/exports/action_journal/<发送入口>.<UTC启动时间>.<PID>.partNNNN.jsonl
```

每条记录包含 PC 记录时间、发送入口、Orin 目标地址、可读 packet、精确 payload 的 Base64、字节数和 SHA-256。日志写盘在线程中完成，不把磁盘延迟加入控制循环；程序启动时会打印本次日志路径。队列满或写盘失败后，下一帧动作会在 `sendto` 之前被拒绝，进程返回错误，不允许长期“继续控制但停止记录”。

容量策略统一位于 `runtime_bridge/config/runtime.json` 的 `action_journal` section。正式配置按每个文件 64 MiB、保留 16 个文件轮转，总量约 1 GiB，超过后删除最旧记录；mock 配置单独写入 `runtime_bridge/exports/action_journal_mock/`。未增加 `--enable-motion`（或诊断入口未增加 `--reply-zero`）时没有实际发送，因此也不会生成发送日志。

到达挖掘点或倾倒点后，可以临时用固定动作脚本执行挖掘/倾倒。该脚本不做路径规划，后续由外部 planner 决定何时启动：

```bash
python3 runtime_bridge/apps/fixed_action_player.py dig
```

倾倒动作：

```bash
python3 runtime_bridge/apps/fixed_action_player.py dump
```

固定动作增益、阈值、超时和网络设置同样读取 `runtime_bridge/config/runtime.json`。配置 schema 已升级为 `runtime_bridge_config_v3`，自定义配置必须包含 `action_journal` section；旧 v2 配置会明确拒绝而不会静默补默认值。上述命令默认只计算和打印，不发送 UDP 动作；完成单轴方向、限位、急停和现场安全检查后，必须显式增加 `--enable-motion` 才会向 Orin 发送动作。`control_enabled=false` 时始终只生成零动作，不提供绕过参数。

首次使用前需要当前 Python 环境安装 ONNX Runtime：

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv_runtime
uv pip install --python .venv_runtime/bin/python onnxruntime
```

协议说明见：

```text
docs/runtime_bridge_protocol.md
```

## 8. 离线 bag

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
python3 localmap/scripts/inspect_bag_points.py \
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
