ROS Dual Quaternions
====================

|travis| |tags|

.. |travis| image:: https://travis-ci.com/Achllle/dual_quaternions_ros.svg?branch=master
    :target: https://travis-ci.com/Achllle/dual_quaternions_ros

.. |tags| image:: https://img.shields.io/github/v/tag/achllle/dual_quaternions_ros
    :alt: GitHub tag (latest SemVer)
    :target: https://GitHub.com/Achllle/dual_quaternions_ros/tags/

Why use dual quaternions?
-------------------------

* dual quaternions have all the advantages of quaternions including easy ScLERP, unambiguous representation, no gimbal lock, compact representation
* for some applications, we need to convert matrices to quaternions because of their mathematical advantages like being able to find the exact tangent (exact derivative of dual numbers)
* we want to use quaternions but they can only handle rotation. Dual quaternions are the correct extension to handle translations as well.
* easy normalization. Homogeneous tranformation matrices are orthogonal and due to floating point errors operations on them often result in matrices that need to be renormalized. This can be done using the Gram-Schmidt method but that is a slow algorithm. Quaternion normalization is very fast.
* mathematically pleasing

Functionality:

* conversion from and to common data types and ROS messages
* basic dual quaternion operations (add, mul, div, log, exp, conj, inv, pow)
* interpolation: ScLERP, DLB
* transformations

NOTE: there is no concept of 'from' and 'to' as frame names aren't tracked or used (e.g. use of Pose iso PoseStamped).
It is up to the user to keep track of those.

Installation & Requirements
---------------------------

In this directory::

    pip install .

Requirements
~~~~~~~~~~~~

* numpy-quaternion
* scipy
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

References
~~~~~~~~~~

* \K. Daniilidis, E. Bayro-Corrochano, "The dual quaternion approach to hand-eye calibration", IEEE International Conference on Pattern Recognition, 1996
* Kavan, Ladislav & Collins, Steven & Zara, Jiri & O'Sullivan, Carol. (2007). Skinning with dual quaternions. I3D. 39-46. 10.1145/1230100.1230107.
* Kenwright, B. (2012). A Beginners Guide to Dual-Quaternions What They Are, How They Work, and How to Use Them for 3D Character Hierarchies.
* Furrer, Fadri & Fehr, Marius & Novkovic, Tonci & Sommer, Hannes & Gilitschenski, Igor & Siegwart, Roland. (2018). Evaluation of Combined Time-Offset Estimation and Hand-Eye Calibration on Robotic Datasets. 145-159. 10.1007/978-3-319-67361-5_10.
