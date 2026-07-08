# Kinematics 子模块

本目录接收原 `/home/zhaoshuai/workspace_uinty/RL_prj/TF` 项目的源码，用于在 ExcavatorRuntime 统一工程内管理 bucket tip 正运动学。

## 边界

- 输入：`/joint_states`
- 输出：`/bucket_tip_pose_base`、`/bucket_tip_pose_map` 和 TF tree
- 不发送真机控制命令
- 不直接修改 ONNX observation 维度
- 只提供 bucket tip 状态给 `localmap` 规划链路

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

## 坐标桥接

`excavator_kinematics` 使用 ROS 坐标约定：

```text
X forward
Y left
Z up
```

ExcavatorRuntime 当前规划坐标使用 Unity/MachineRoot 约定：

```text
X right
Y up
Z forward
```

因此 bucket tip 位置进入规划前要经过固定轴映射：

```text
machine_root.x = -base_link.y
machine_root.y =  base_link.z
machine_root.z =  base_link.x
```

对应配置文件：

```text
localmap/config/bucket_tip_tf_bridge.machine_root.json
```

如果后续发现 `base_link` 原点与 Unity `MachineRoot` 原点不完全一致，只修改该配置中的 `translation_m`，不要改可达区域或 RRT 输入契约。

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

检查输出：

```bash
ros2 topic echo /bucket_tip_pose_base --once
ros2 run tf2_ros tf2_echo base_link bucket_tip
```

桥接到 `machine_root`：

```bash
/usr/bin/python3 localmap/scripts/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_base \
  --output-topic /localmap/bucket_tip_machine_root_pose \
  --bridge localmap/config/bucket_tip_tf_bridge.machine_root.json \
  --output-json localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

`run_planning_once.sh` 会优先读取 `localmap/exports/live_latest/bucket_tip.machine_root.live.json`。如果该文件不存在，则回退到 `localmap/config/bucket_tip.machine_root.measured.json`。
