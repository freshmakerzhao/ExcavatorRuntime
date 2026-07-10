# excavator_kinematics

This ROS2 package publishes the excavator TF tree and computes the bucket tooth
center pose from four joint angles:

```text
map
└── base_link
    └── swing_link
        └── boom_link
            └── arm_link
                └── bucket_link
                    └── bucket_tip
```

## Coordinate Convention

The model uses the ROS convention:

```text
X forward
Y left
Z up
```

Joint axes:

```text
swing_joint:  rotate about Z
boom_joint:   rotate about Y
arm_joint:    rotate about Y
bucket_joint: rotate about Y
```

This makes the arm motion mostly lie in the `swing_link` X-Z plane.

## Inputs

Publish joint angles to:

```text
/joint_states
```

Required joint names:

```text
swing_joint
boom_joint
arm_joint
bucket_joint
```

Message type:

```text
sensor_msgs/msg/JointState
```

Angles are in radians.

## Outputs

TF:

```text
base_link -> swing_link
swing_link -> boom_link
boom_link -> arm_link
arm_link -> bucket_link
bucket_link -> bucket_tip
```

Bucket tip pose:

```text
/bucket_tip_pose_base  # geometry_msgs/msg/PoseStamped, frame_id=base_link
/bucket_tip_pose_map   # geometry_msgs/msg/PoseStamped, frame_id=fk_root
/bucket_tip_pose_unity # geometry_msgs/msg/PoseStamped, frame_id=machine_root
/bucket_tip_observation # std_msgs/msg/Float32MultiArray: [x, y, z, pitch_rad]
```

## Configure Geometry

Edit:

```text
config/excavator_geometry.yaml
```

Important dimensions:

```yaml
swing_to_boom_xyz: [0.8, 0.0, 1.0]
boom_to_arm_xyz: [4.5, 0.0, 0.0]
arm_to_bucket_xyz: [3.0, 0.0, 0.0]
bucket_to_tip_xyz: [1.0, 0.0, -0.4]
```

Replace these with CAD or measured values.

Use `angle_offsets` to align encoder zero with model zero:

```yaml
angle_offsets:
  swing: 0.0
  boom: 0.0
  arm: 0.0
  bucket: 0.0
```

Use `joint_signs` to align sensor positive direction with FK positive direction:

```yaml
joint_signs:
  swing: -1.0
  boom: -1.0
  arm: -1.0
  bucket: 1.0
```

The FK angle is:

```text
fk_angle = input_angle * joint_sign + angle_offset
```

## Build

Linux:

```bash
cd /path/to/ROS
colcon build --packages-select excavator_kinematics
source install/setup.bash
```

Windows PowerShell:

```powershell
cd D:\Database\exctvator\ROS
colcon build --packages-select excavator_kinematics
.\install\setup.ps1
```

## Run

```bash
ros2 launch excavator_kinematics excavator_tf.launch.py
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo base_link bucket_tip
```

Check bucket tip position:

```bash
ros2 topic echo /bucket_tip_pose_base
ros2 topic echo /bucket_tip_pose_map
ros2 topic echo /bucket_tip_pose_unity
ros2 topic echo /bucket_tip_observation
```

## Minimal JointState Test

In another terminal:

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'],
  position: [0.0, 0.3, -0.8, 0.5]
}" -r 10
```

Or use the GUI joint sliders:

```bash
ros2 run excavator_kinematics joint_slider_publisher \
  --publish-on-change \
  --initial 0.0 0.0 0.0 0.0
```

Then inspect:

```bash
ros2 topic echo /bucket_tip_pose_base --once
```
