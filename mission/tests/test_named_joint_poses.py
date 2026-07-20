import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from mission.home import ReturnHomeSession, load_named_joint_pose_set


AIRY_ROOT = Path(__file__).resolve().parents[2]
POSES = AIRY_ROOT / "kinematics/waji_description/config/named_joint_poses.json"
URDF = AIRY_ROOT / "kinematics/waji_description/urdf/waji.urdf"


class NamedJointPoseTests(unittest.TestCase):
    def test_placeholder_home_pose_is_bound_to_the_authoritative_urdf(self):
        pose_set = load_named_joint_pose_set(POSES, urdf_path=URDF)

        self.assertEqual(pose_set.urdf_sha256, hashlib.sha256(URDF.read_bytes()).hexdigest())
        self.assertEqual(
            pose_set.joint_order,
            ("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
        )
        self.assertEqual(pose_set.poses["transport_home"].status, "placeholder")

    def test_fresh_joint_samples_complete_after_dwell(self):
        pose = load_named_joint_pose_set(POSES, urdf_path=URDF).poses["transport_home"]
        session = ReturnHomeSession.start(pose, accepted_at_s=1.0)

        session, far = session.observe(
            {"swing_joint": 0.2, "boom_joint": 0.1, "arm_joint": 0.0, "bucket_joint": 0.0},
            sample_stamp_s=1.1,
            now_s=1.1,
        )
        session, entered = session.observe(
            {name: 0.0 for name in pose.joint_order}, sample_stamp_s=1.2, now_s=1.2
        )
        _, completed = session.observe(
            {name: 0.0 for name in pose.joint_order}, sample_stamp_s=1.5, now_s=1.5
        )

        self.assertFalse(far.within_tolerance)
        self.assertTrue(entered.within_tolerance)
        self.assertTrue(completed.completed)
        self.assertFalse(completed.timed_out)

    def test_missing_joint_and_timeout_fail_closed(self):
        pose = load_named_joint_pose_set(POSES, urdf_path=URDF).poses["transport_home"]
        session = ReturnHomeSession.start(pose, accepted_at_s=1.0)
        with self.assertRaisesRegex(ValueError, "joint names"):
            session.observe({"swing_joint": 0.0}, sample_stamp_s=1.1, now_s=1.1)

        _, update = session.observe(
            {name: 1.0 for name in pose.joint_order},
            sample_stamp_s=12.0,
            now_s=12.0,
        )
        self.assertTrue(update.timed_out)
        self.assertFalse(update.completed)

    def test_pose_status_and_urdf_joint_limits_are_enforced(self):
        document = json.loads(POSES.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / "poses.json"
            document["poses"]["transport_home"]["status"] = "unreviewed"
            candidate.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "status"):
                load_named_joint_pose_set(candidate, urdf_path=URDF)

            document["poses"]["transport_home"]["status"] = "placeholder"
            document["poses"]["transport_home"]["position_rad"][0] = 2.0
            candidate.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "outside URDF joint limits"):
                load_named_joint_pose_set(candidate, urdf_path=URDF)

    def test_current_joint_state_outside_urdf_limits_fails_closed(self):
        pose = load_named_joint_pose_set(POSES, urdf_path=URDF).poses["transport_home"]
        session = ReturnHomeSession.start(pose, accepted_at_s=1.0)

        with self.assertRaisesRegex(ValueError, "current swing_joint"):
            session.observe(
                {
                    "swing_joint": 2.0,
                    "boom_joint": 0.0,
                    "arm_joint": 0.0,
                    "bucket_joint": 0.0,
                },
                sample_stamp_s=1.1,
                now_s=1.1,
            )


if __name__ == "__main__":
    unittest.main()
