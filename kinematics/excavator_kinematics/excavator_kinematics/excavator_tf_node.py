import math
from typing import Dict, Iterable, List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


Matrix4 = List[List[float]]
Vec3 = Tuple[float, float, float]


class ExcavatorTfNode(Node):
    def __init__(self):
        super().__init__("excavator_tf_node")

        self.frames = {
            "map": self.declare_parameter("frames.map", "map").value,
            "unity": self.declare_parameter("frames.unity", "machine_root").value,
            "base": self.declare_parameter("frames.base", "base_link").value,
            "swing": self.declare_parameter("frames.swing", "swing_link").value,
            "boom": self.declare_parameter("frames.boom", "boom_link").value,
            "arm": self.declare_parameter("frames.arm", "arm_link").value,
            "bucket": self.declare_parameter("frames.bucket", "bucket_link").value,
            "tip": self.declare_parameter("frames.tip", "bucket_tip").value,
        }

        self.joint_names = {
            "swing": self.declare_parameter("joint_names.swing", "swing_joint").value,
            "boom": self.declare_parameter("joint_names.boom", "boom_joint").value,
            "arm": self.declare_parameter("joint_names.arm", "arm_joint").value,
            "bucket": self.declare_parameter("joint_names.bucket", "bucket_joint").value,
        }

        self.map_to_base_xyz = self._vec_param("geometry.map_to_base_xyz", [0.0, 0.0, 0.0])
        self.map_to_base_rpy = self._vec_param("geometry.map_to_base_rpy", [0.0, 0.0, 0.0])
        self.base_to_swing_xyz = self._vec_param("geometry.base_to_swing_xyz", [0.0, 0.0, 0.0])
        self.swing_to_boom_xyz = self._vec_param("geometry.swing_to_boom_xyz", [0.8, 0.0, 1.0])
        self.boom_to_arm_xyz = self._vec_param("geometry.boom_to_arm_xyz", [4.5, 0.0, 0.0])
        self.arm_to_bucket_xyz = self._vec_param("geometry.arm_to_bucket_xyz", [3.0, 0.0, 0.0])
        self.bucket_to_tip_xyz = self._vec_param("geometry.bucket_to_tip_xyz", [1.0, 0.0, -0.4])
        self.bucket_to_tip_rpy = self._vec_param("geometry.bucket_to_tip_rpy", [0.0, 0.0, 0.0])

        self.angle_offsets = {
            "swing": float(self.declare_parameter("angle_offsets.swing", 0.0).value),
            "boom": float(self.declare_parameter("angle_offsets.boom", 0.0).value),
            "arm": float(self.declare_parameter("angle_offsets.arm", 0.0).value),
            "bucket": float(self.declare_parameter("angle_offsets.bucket", 0.0).value),
        }
        self.joint_signs = {
            "swing": float(self.declare_parameter("joint_signs.swing", 1.0).value),
            "boom": float(self.declare_parameter("joint_signs.boom", 1.0).value),
            "arm": float(self.declare_parameter("joint_signs.arm", 1.0).value),
            "bucket": float(self.declare_parameter("joint_signs.bucket", 1.0).value),
        }

        self.publish_map_to_base = bool(
            self.declare_parameter("publish_map_to_base", True).value
        )
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)

        self.joint_positions: Dict[str, float] = {
            self.joint_names["swing"]: 0.0,
            self.joint_names["boom"]: 0.0,
            self.joint_names["arm"]: 0.0,
            self.joint_names["bucket"]: 0.0,
        }

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tip_base_pub = self.create_publisher(PoseStamped, "bucket_tip_pose_base", 10)
        self.tip_map_pub = self.create_publisher(PoseStamped, "bucket_tip_pose_map", 10)
        self.tip_unity_pub = self.create_publisher(PoseStamped, "bucket_tip_pose_unity", 10)
        self.tip_observation_pub = self.create_publisher(Float32MultiArray, "bucket_tip_observation", 10)

        self.create_subscription(JointState, "joint_states", self.on_joint_state, 20)

        if self.publish_map_to_base:
            self.publish_static_map_to_base()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period, self.publish_kinematics)

        self.get_logger().info(
            "Publishing excavator TF tree and bucket tip pose from /joint_states"
        )

    def _vec_param(self, name: str, default: Iterable[float]) -> Vec3:
        value = self.declare_parameter(name, list(default)).value
        if len(value) != 3:
            raise ValueError(f"{name} must contain exactly 3 numbers")
        return float(value[0]), float(value[1]), float(value[2])

    def on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = float(position)

    def publish_static_map_to_base(self):
        transform = transform_msg(
            self.frames["map"],
            self.frames["base"],
            self.map_to_base_xyz,
            quat_from_rpy(*self.map_to_base_rpy),
            self.get_clock().now().to_msg(),
        )
        self.static_tf_broadcaster.sendTransform(transform)

    def publish_kinematics(self):
        now = self.get_clock().now().to_msg()

        q_swing = self._joint("swing")
        q_boom = self._joint("boom")
        q_arm = self._joint("arm")
        q_bucket = self._joint("bucket")

        tf_base_swing = transform_msg(
            self.frames["base"],
            self.frames["swing"],
            self.base_to_swing_xyz,
            quat_from_rpy(0.0, 0.0, q_swing),
            now,
        )
        tf_swing_boom = transform_msg(
            self.frames["swing"],
            self.frames["boom"],
            self.swing_to_boom_xyz,
            quat_from_rpy(0.0, q_boom, 0.0),
            now,
        )
        tf_boom_arm = transform_msg(
            self.frames["boom"],
            self.frames["arm"],
            self.boom_to_arm_xyz,
            quat_from_rpy(0.0, q_arm, 0.0),
            now,
        )
        tf_arm_bucket = transform_msg(
            self.frames["arm"],
            self.frames["bucket"],
            self.arm_to_bucket_xyz,
            quat_from_rpy(0.0, q_bucket, 0.0),
            now,
        )
        tf_bucket_tip = transform_msg(
            self.frames["bucket"],
            self.frames["tip"],
            self.bucket_to_tip_xyz,
            quat_from_rpy(*self.bucket_to_tip_rpy),
            now,
        )

        self.tf_broadcaster.sendTransform(
            [tf_base_swing, tf_swing_boom, tf_boom_arm, tf_arm_bucket, tf_bucket_tip]
        )

        t_base_tip = self.forward_kinematics_base(q_swing, q_boom, q_arm, q_bucket)
        t_map_base = transform_matrix(self.map_to_base_xyz, quat_from_rpy(*self.map_to_base_rpy))
        t_map_tip = matmul(t_map_base, t_base_tip)

        self.tip_base_pub.publish(pose_from_matrix(now, self.frames["base"], t_base_tip))
        self.tip_map_pub.publish(pose_from_matrix(now, self.frames["map"], t_map_tip))
        # 关键：额外发布Unity/MachineRoot左手坐标下的位置，供真机runtime和RViz直接查看。
        unity_position = fk_root_position_to_unity(matrix_translation(t_map_tip))
        pitch_rad = bucket_tip_pitch_rad_from_matrix(t_map_tip)
        self.tip_unity_pub.publish(pose_from_position(now, self.frames["unity"], unity_position))
        self.tip_observation_pub.publish(bucket_tip_observation_message(unity_position, pitch_rad))

    def _joint(self, key: str) -> float:
        name = self.joint_names[key]
        return apply_joint_sign_and_offset(
            self.joint_positions.get(name, 0.0),
            self.joint_signs[key],
            self.angle_offsets[key],
        )

    def forward_kinematics_base(
        self, swing: float, boom: float, arm: float, bucket: float
    ) -> Matrix4:
        t = identity()
        t = matmul(t, transform_matrix(self.base_to_swing_xyz, quat_from_rpy(0.0, 0.0, swing)))
        t = matmul(t, transform_matrix(self.swing_to_boom_xyz, quat_from_rpy(0.0, boom, 0.0)))
        t = matmul(t, transform_matrix(self.boom_to_arm_xyz, quat_from_rpy(0.0, arm, 0.0)))
        t = matmul(t, transform_matrix(self.arm_to_bucket_xyz, quat_from_rpy(0.0, bucket, 0.0)))
        t = matmul(t, transform_matrix(self.bucket_to_tip_xyz, quat_from_rpy(*self.bucket_to_tip_rpy)))
        return t


def transform_msg(parent: str, child: str, xyz: Vec3, quat: Tuple[float, float, float, float], stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    msg.transform.rotation.x = quat[0]
    msg.transform.rotation.y = quat[1]
    msg.transform.rotation.z = quat[2]
    msg.transform.rotation.w = quat[3]
    return msg


def pose_from_matrix(stamp, frame_id: str, matrix: Matrix4) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = matrix[0][3]
    msg.pose.position.y = matrix[1][3]
    msg.pose.position.z = matrix[2][3]
    quat = quat_from_matrix(matrix)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return msg


def pose_from_position(stamp, frame_id: str, xyz: Vec3) -> PoseStamped:
    """从位置构造PoseStamped；Unity左手系姿态暂不作为权威输出。"""
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def matrix_translation(matrix: Matrix4) -> Vec3:
    """取齐次变换矩阵的平移列。"""
    return float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3])


def fk_root_position_to_unity(position: Vec3) -> Vec3:
    """把fk_root/ROS位置转换成Unity/MachineRoot左手坐标位置。"""
    x_forward, y_left, z_up = position
    # 轴约定：ROS +X前 -> Unity +Z，ROS +Y左 -> Unity -X，ROS +Z上 -> Unity +Y。
    return -float(y_left), float(z_up), float(x_forward)


def apply_joint_sign_and_offset(raw_angle_rad: float, joint_sign: float, angle_offset_rad: float) -> float:
    """把传感器/Orin关节角转换成FK内部关节角。"""
    return float(raw_angle_rad) * float(joint_sign) + float(angle_offset_rad)


def bucket_tip_pitch_rad_from_matrix(matrix: Matrix4) -> float:
    """计算训练使用的bucket pitch：bucket_tip局部+Z与fk_root +Z的夹角。"""
    # 关键：齐次矩阵第三列是bucket_tip局部+Z轴在fk_root中的方向，和fk_root +Z点乘即r22。
    z_dot = max(-1.0, min(1.0, float(matrix[2][2])))
    return math.acos(z_dot)


def bucket_tip_observation_values(unity_position: Vec3, pitch_rad: float) -> list[float]:
    """构造observation需要的bucket tip状态片段：[x, y, z, pitch_rad]。"""
    return [
        float(unity_position[0]),
        float(unity_position[1]),
        float(unity_position[2]),
        float(pitch_rad),
    ]


def bucket_tip_observation_message(unity_position: Vec3, pitch_rad: float) -> Float32MultiArray:
    """发布无自定义msg版本的bucket tip observation输入。"""
    message = Float32MultiArray()
    message.data = bucket_tip_observation_values(unity_position, pitch_rad)
    return message


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_from_matrix(m: Matrix4) -> Tuple[float, float, float, float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        )
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        )
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return (
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        )
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return (
        (m[0][2] + m[2][0]) / s,
        (m[1][2] + m[2][1]) / s,
        0.25 * s,
        (m[1][0] - m[0][1]) / s,
    )


def transform_matrix(xyz: Vec3, quat: Tuple[float, float, float, float]) -> Matrix4:
    x, y, z, w = quat
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), xyz[0]],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), xyz[1]],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def identity() -> Matrix4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    out = [[0.0 for _ in range(4)] for _ in range(4)]
    for r in range(4):
        for c in range(4):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(4))
    return out


def main():
    rclpy.init()
    node = ExcavatorTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
