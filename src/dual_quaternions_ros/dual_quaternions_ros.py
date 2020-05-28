"""
ROS specific dual quaternion converters

Author: Achille Verheye
License: MIT
"""

from dual_quaternions import DualQuaternion

import geometry_msgs.msg

def from_ros(msg):
    """
    Create a DualQuaternion instance from a ROS Pose or Transform message

    :param msg: geometry_msgs.msg.Pose or geometry_msgs.msg.Transform
    """
    try:
        tra = msg.position
        rot = msg.orientation
    except AttributeError:
        tra = msg.translation
        rot = msg.rotation

    return DualQuaternion.from_quat_pose_array([rot.w, rot.x, rot.y, rot.z, tra.x, tra.y, tra.z])


def to_ros_pose(dual_quat):
    """ROS geometry_msgs.msg.Pose instance"""
    pose_msg = geometry_msgs.msg.Pose()
    quat_pose_arr = dual_quat.quat_pose_array()
    rot = quat_pose_arr[:4]
    tra = quat_pose_arr[4:]
    pose_msg.position = geometry_msgs.msg.Point(*tra)
    # ROS uses [x y z w] format
    pose_msg.orientation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

    return pose_msg


def to_ros_transform(dual_quat):
    """ROS geometry_msgs.msg.Transform instance"""
    transform_msg = geometry_msgs.msg.Transform()
    quat_pose_arr = dual_quat.quat_pose_array()
    rot = quat_pose_arr[:4]
    tra = quat_pose_arr[4:]
    transform_msg.translation = geometry_msgs.msg.Vector3(*tra)
    # ROS uses [x y z w] format
    transform_msg.rotation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

    return transform_msg
