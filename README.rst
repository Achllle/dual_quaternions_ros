Dual Quaternions ROS
====================

|travis| |tags|

.. |travis| image:: https://travis-ci.com/Achllle/dual_quaternions_ros.svg?branch=master
    :target: https://travis-ci.com/Achllle/dual_quaternions_ros

.. |tags| image:: https://img.shields.io/github/v/tag/achllle/dual_quaternions_ros
    :alt: GitHub tag (latest SemVer)
    :target: https://GitHub.com/Achllle/dual_quaternions_ros/tags/

Simple conversion methods for going from ROS `geometry_msgs` to dual quaternions and vice versa.
For the dual_quaternions repo, see `dual_quaternions <https://github.com/Achllle/dual_quaternions>`__.

Installation
------------

pip
~~~

.. code-block:: bash

  pip install dual_quaternions

ROS1 package
~~~~~~~~~~~

.. code-block:: bash

  apt install ros-$ROS_DISTRO-dual-quaternions-ros

Requirements
~~~~~~~~~~~~

* dual_quaternions
* geometry_msgs

Usage
-----

Import using::

    from dual_quaternions_ros import from_ros, to_ros_pose, to_ros_transform

NOTE: there is no concept of 'from' and 'to' as frame names aren't tracked or used (e.g. use of Pose iso PoseStamped).
It is up to the user to keep track of those to avoid picking a convention (active vs. passive)

Publishing and getting transforms from tf
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This package purposefully doesn't have methods to receive and publish transforms to tf. Instead, it supports converting
transforms to various ROS messages so you can use the standard way of interfacing: ::

    br = tf2_ros.TransformBroadcaster()
    T_odom_baselink = DualQuaternion(...)
    msg = geometry_msgs.msg.TransformStamped()
    msg.transform = T_odom_baselink.to_ros_transform
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    br.sendTransform(msg)
