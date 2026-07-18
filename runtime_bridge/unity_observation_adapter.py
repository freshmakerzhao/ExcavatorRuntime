"""把 ROS 右手空间量适配为冻结的 Unity 左手 observation 契约。"""

from __future__ import annotations

import math
from typing import Sequence

from runtime_bridge.observation import BucketTipObservation


class UnityObservationAdapter:
    """ONNX 边界的唯一 ROS→Unity 空间适配器。

    ROS `machine_root_ros` 使用 +X forward, +Y left, +Z up；Unity 训练契约
    使用 +X right, +Y up, +Z forward。该映射是坐标手性转换，不得发布为 ROS TF。
    """

    @staticmethod
    def position_to_unity(position_m: Sequence[float]) -> tuple[float, float, float]:
        """转换一个位置或自由向量；两者的轴映射相同。"""
        if len(position_m) != 3:
            raise ValueError("ROS 空间量必须恰好包含3个分量")
        x_forward, y_left, z_up = (float(value) for value in position_m)
        return -y_left, z_up, x_forward

    def bucket_tip_to_unity(self, bucket_tip: BucketTipObservation) -> BucketTipObservation:
        """转换 tip 位置，保留已冻结的 bucket pitch 与源时间。"""
        return BucketTipObservation(
            position_m=self.position_to_unity(bucket_tip.position_m),
            pitch_rad=float(bucket_tip.pitch_rad),
            stamp_ms=bucket_tip.stamp_ms,
        )

    def ros_pose_to_unity_bucket_tip(
        self,
        *,
        position_m: Sequence[float],
        orientation_xyzw: Sequence[float],
        stamp_ms: int,
        swing_joint_rad: float = 0.0,
    ) -> BucketTipObservation:
        """适配带源时间的 ROS PoseStamped 内容为 Unity training tip 片段。

        Unity 训练契约用 bucket reference 局部 forward 相对 swing frame
        forward 的有符号 pitch。ROS URDF 中 Unity forward 对应 bucket_tip
        局部 +X，swing_joint 绕 -Z 旋转；因此先消除 swing，再在
        swing XZ 平面内绕 +Y 求有符号角。
        """
        if len(orientation_xyzw) != 4:
            raise ValueError("ROS quaternion 必须恰好包含4个分量")
        quaternion = tuple(float(value) for value in orientation_xyzw)
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-9:
            raise ValueError("ROS quaternion 不能为零")
        qx, qy, qz, qw = (value / norm for value in quaternion)
        # First rotation-matrix column: bucket_tip local +X in machine_root_ros.
        bucket_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        bucket_y = 2.0 * (qx * qy + qw * qz)
        bucket_z = 2.0 * (qx * qz - qw * qy)
        swing = float(swing_joint_rad)
        if not math.isfinite(swing):
            raise ValueError("swing_joint_rad must be finite")
        # URDF swing axis is -Z, so machine<-swing is Rz(-swing).
        cosine, sine = math.cos(swing), math.sin(swing)
        bucket_x_in_swing = cosine * bucket_x - sine * bucket_y
        pitch_rad = math.atan2(-bucket_z, bucket_x_in_swing)
        return BucketTipObservation(
            position_m=self.position_to_unity(position_m),
            pitch_rad=pitch_rad,
            stamp_ms=int(stamp_ms),
        )

    def waypoint_values_to_unity(self, waypoint_values: Sequence[float]) -> list[float]:
        """转换 idx 15..26 的三个相对 waypoint 向量，保留末尾标量。"""
        if len(waypoint_values) != 12:
            raise ValueError("waypoint_values 必须是长度12数组，对应 observation idx 15..26")
        unity_values: list[float] = []
        for offset in range(0, 9, 3):
            unity_values.extend(self.position_to_unity(waypoint_values[offset : offset + 3]))
        unity_values.extend(float(value) for value in waypoint_values[9:])
        return unity_values
