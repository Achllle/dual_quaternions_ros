ROS Dual Quaternions
====================

Simple conversion methods for going from ROS `geometry_msgs` to dual quaternions and vice versa

NOTE: there is no concept of 'from' and 'to' as frame names aren't tracked or used (e.g. use of Pose iso PoseStamped).
It is up to the user to keep track of those.

Requirements
~~~~~~~~~~~~

* dual_quaternions

Usage
-----

Import using::

    from dual_quaternions_ros import from_ros_pose, from_ros_transform, ros_pose, ros_transform

Publishing and getting transforms from tf
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This package purposefully doesn't have methods to receive and publish transforms to tf. Instead, it supports converting
transforms to various ROS messages so you can use the standard way of interfacing: ::

    br = tf2_ros.TransformBroadcaster()
    T_odom_baselink = DualQuaternion(...)
    msg = geometry_msgs.msg.TransformStamped()
    msg.transform = T_odom_baselink.ros_transform
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    br.sendTransform(msg)
