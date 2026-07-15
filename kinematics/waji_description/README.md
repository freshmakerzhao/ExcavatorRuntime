# waji_description

此 ROS 2 包以重新测量的 [`urdf/waji.urdf`](urdf/waji.urdf) 为唯一 FK 几何来源。该文件
在测试中必须与工作区权威源 `/home/zhaoshuai/workspace_uinty/RL_prj/urdf/urdf/waji.urdf`
字节一致；不要用旧手写 FK 的 offset、sign 或 tip RPY 覆盖它。

模型本身的 `fk_root` 是右手 ROS 坐标（`+X` 前、`+Y` 左、`+Z` 上）。
`display.launch.py` 通过显式单位 TF 将它置于唯一系统根 `machine_root_ros` 下，启动：

```bash
ros2 launch waji_description display.launch.py
```

它运行 `robot_state_publisher` 与只读 Bucket Tip pose publisher，输入为现有
`/joint_states`，输出为：

```text
/bucket_tip_pose_machine_root_ros
machine_root_ros -> fk_root -> base_link -> swing_link -> boom_link -> arm_link -> bucket_link -> bucket_tip
```

`PoseStamped.header.stamp` 从 TF transform 原样复制，因而保持 `/joint_states` 的源时间。
`/joint_states` 在生产运行时只由 state bridge 提供。`rviz.launch.py` 仅用于可视化。

## 仅模型验证：Tk slider

新 URDF 项目自带的 `slider.launch.py` 已保留，用于验证 URDF 关节轴和 RViz 随动；它会发布
模拟 `/joint_states`，因此必须使用与真机不同的 ROS domain：

```bash
ROS_DOMAIN_ID=221 ros2 launch waji_description display.launch.py
ROS_DOMAIN_ID=221 ros2 launch waji_description slider.launch.py
ROS_DOMAIN_ID=221 ros2 launch waji_description rviz.launch.py
```

三个命令分别在三个终端执行。严禁在真机 ROS domain 中运行 slider。
