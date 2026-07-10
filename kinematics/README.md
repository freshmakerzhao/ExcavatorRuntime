# Kinematics 子模块

本目录接收原 `/home/zhaoshuai/workspace_uinty/RL_prj/TF` 项目的源码，用于在 ExcavatorRuntime 统一工程内管理 bucket tip 正运动学。

## 边界

- 输入：`/joint_states`
- 原始输出：`/bucket_tip_pose_base`、`/bucket_tip_pose_map`、`/bucket_tip_pose_unity` 和 TF tree
- 规划输入：优先使用 `/bucket_tip_pose_map`，通过 `localmap/scripts/bridge_bucket_tip_from_tf.py` 转成 `machine_root` bucket tip JSON
- 不发送真机控制命令
- 不直接修改 ONNX observation 维度

## ROS2 包

当前包路径：

```text
kinematics/excavator_kinematics
```

工作空间入口：

```text
ros2_ws/src/excavator_kinematics -> ../../kinematics/excavator_kinematics
```

这个软连接让 `colcon build` 可以在 ExcavatorRuntime 的 `ros2_ws` 中同时编译雷达 SDK 和运动学包，而源码仍按项目职责放在 `kinematics/` 下。

## 原始坐标约定

原 TF 项目使用 ROS 坐标约定：

```text
base_link:
X forward
Y left
Z up
```

关节轴：

```text
swing_joint:  rotate about Z
boom_joint:   rotate about Y
arm_joint:    rotate about Y
bucket_joint: rotate about Y
```

真机/Orin 输入角度进入 FK 前会经过方向适配：

```text
fk_angle = orin_angle * joint_sign + angle_offset
```

当前实测配置：

```text
swing: -1.0
boom:  -1.0
arm:   -1.0
bucket: 1.0
```

如果后续发现某个关节电位计正方向仍与 TF 运动方向相反，只改 `config/excavator_geometry.yaml` 里的 `joint_signs`，不要改 Orin 协议和连杆几何。

规划链路使用 `machine_root`：

```text
machine_root:
X right
Y up
Z forward
```

因此当前基线不是直接改 FK 源码坐标，而是让 FK 内部 `fk_root` 原点与 `machine_root` 重合，轴向保持 ROS/FK 约定，再把 `/bucket_tip_pose_map` 桥接到 `machine_root`：

```text
fk_root +X forward -> machine_root +Z
fk_root +Y left    -> machine_root -X
fk_root +Z up      -> machine_root +Y
```

对应配置文件：

```text
localmap/config/bucket_tip_tf_bridge.machine_root.json
```

运动学节点现在同时发布两组最常用的 bucket tip 坐标：

```text
/bucket_tip_pose_map    # frame_id=fk_root，FK/ROS坐标
/bucket_tip_pose_unity  # frame_id=machine_root，Unity/MachineRoot左手坐标
```

同时发布 observation 需要的 bucket tip 状态：

```text
/bucket_tip_observation  # std_msgs/msg/Float32MultiArray
data[0] = x_machine_root
data[1] = y_machine_root
data[2] = z_machine_root
data[3] = pitch_rad
```

其中 `pitch_rad` 定义为 `bucket_tip` 局部 `+Z` 轴与 `fk_root +Z` 轴的夹角，单位 rad。

## 编译

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime/ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select excavator_kinematics
source install/setup.zsh
```

## 运行

启动运动学节点：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime/ros2_ws/install/setup.zsh

ros2 launch excavator_kinematics excavator_tf.launch.py
```

发布一组测试关节角：

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'],
  position: [0.0, 0.3, -0.8, 0.5]
}" -r 10
```

也可以启动滑块窗口实时控制四个关节角：

```bash
ros2 run excavator_kinematics joint_slider_publisher \
  --publish-on-change \
  --initial 0.0 0.0 0.0 0.0 \
  --min -3.14 -1.6 -1.6 -1.6 \
  --max 3.14 1.6 1.6 1.6
```

检查原始 FK 输出：

```bash
ros2 topic echo /bucket_tip_pose_base --once
ros2 topic echo /bucket_tip_pose_map --once
ros2 topic echo /bucket_tip_pose_unity --once
ros2 topic echo /bucket_tip_observation --once
ros2 run tf2_ros tf2_echo base_link bucket_tip
```

桥接到 `machine_root`：

```bash
python3 localmap/scripts/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_map \
  --output-topic /localmap/bucket_tip_machine_root_pose \
  --bridge localmap/config/bucket_tip_tf_bridge.machine_root.json \
  --output-json localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

`run_planning_once.sh` 会优先读取 `localmap/exports/live_latest/bucket_tip.machine_root.live.json`。如果该文件不存在，则回退到 `localmap/config/bucket_tip.machine_root.measured.json`。
