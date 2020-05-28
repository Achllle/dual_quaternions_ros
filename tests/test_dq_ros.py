from unittest import TestCase
from dual_quaternions import DualQuaternion
from dual_quaternions_ros import from_ros, to_ros_pose, to_ros_transform
import numpy as np

from geometry_msgs.msg import Transform, Pose, Quaternion, Vector3, Point


class TestDualQuaternion(TestCase):

    def setUp(self):
        self.identity_dq = DualQuaternion.identity()
        self.identity_pose = Pose(Point(0., 0., 0.), Quaternion(0., 0., 0., 1.))
        self.identity_transform = Transform(Vector3(0., 0., 0.), Quaternion(0., 0., 0., 1.))

    def test_from_ros_pose(self):
        """Convert ROS pose to dual quaternions"""
        try:
            pose_msg = Pose(Vector3(1, 0.5, -1), Quaternion(0.707, 0, 0.707, 0.707))
            dq = from_ros(pose_msg)
        except Exception as e:
            self.fail(e)

    def test_from_ros_transform(self):
        """Convert ROS transform to dual quaternions"""
        try:
            transform_msg = Transform(Vector3(1, 0.5, -1), Quaternion(0.707, 0, 0.707, 0.707))
            dq = from_ros(transform_msg)
        except Exception as e:
            self.fail(e)

    def test_to_ros_pose(self):
        """Convert dual quaternion to ros pose"""
        self.assertEqual(to_ros_pose(self.identity_dq), self.identity_pose)

    def test_to_ros_transform(self):
        """Convert dual quaternion to ros transform"""
        self.assertEqual(to_ros_transform(self.identity_dq), self.identity_transform)
