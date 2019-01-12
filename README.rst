ROS Dual Quaternions
====================

.. image:: https://travis-ci.com/Achllle/dual_quaternions_ros.svg?branch=master
    :target: https://travis-ci.com/Achllle/dual_quaternions_ros

Why use dual quaternions?
-------------------------

* dual quaternions have all the advantages of quaternions including easy SLERP, unambiguous representation, no gimbal lock, compact representation
* for some applications, we need to convert matrices to quaternions because of their mathematical advantages like being able to find the exact tangent (exact derivative of dual numbers)
* we want to use quaternions but they can only handle rotation. Dual quaternions are the correct extension to handle translations as well.
  * decomposing rotation and translation and converting rotation into a quaternion and doing interpolation independently causes dependency on the frame of reference, which doesn’t happen with dual quaternions.
* easy normalization. Homogeneous tranformation matrices are orthogonal and due to floating point errors operations on them often result in matrices that need to be renormalized. This can be done using the Gram-Schmidt method but that is a slow algorithm. Quaternion normalization is very fast.
* mathematically pleasing

Installation & Requirements
---------------------------

In this directory::

    pip install -e .

Requirements
~~~~~~~~~~~~

* numpy-quaternion
* geometry_msgs from ROS

Usage
-----

Import using::

    from dual_quaternions_ros import DualQuaternion

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

To Do
-----

* transforming multiple points without using matrices
