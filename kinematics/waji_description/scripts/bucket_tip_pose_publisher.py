#!/usr/bin/python3.12

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class BucketTipPosePublisher(Node):
    def __init__(self):
        super().__init__("bucket_tip_pose_publisher")

        self.root_frame = str(
            self.declare_parameter("root_frame", "machine_root_ros").value
        )
        self.tip_frame = str(
            self.declare_parameter("tip_frame", "bucket_tip").value
        )
        self.pose_topic = str(
            self.declare_parameter(
                "pose_topic", "/bucket_tip_pose_machine_root_ros"
            ).value
        )
        publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 30.0).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        self.waiting_for_tf = False
        period = 1.0 / max(publish_rate_hz, 1.0)
        self.create_timer(period, self.publish_tip_pose)

        self.get_logger().info(
            f"Publishing {self.tip_frame} pose in {self.root_frame} on "
            f"{self.pose_topic} (position and orientation quaternion)"
        )

    def publish_tip_pose(self):
        try:
            root_transform = self.tf_buffer.lookup_transform(
                self.root_frame,
                self.tip_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            if not self.waiting_for_tf:
                self.get_logger().warn(
                    f"Waiting for TF to {self.tip_frame}: {error}"
                )
                self.waiting_for_tf = True
            return

        if self.waiting_for_tf:
            self.get_logger().info(f"TF to {self.tip_frame} is available")
            self.waiting_for_tf = False

        # The TF header retains the source /joint_states time supplied by
        # robot_state_publisher.  Never overwrite it with timer wall time.
        self.pose_publisher.publish(self.pose_from_transform(root_transform))

    @staticmethod
    def pose_from_transform(transform):
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose


def main():
    rclpy.init()
    node = BucketTipPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
