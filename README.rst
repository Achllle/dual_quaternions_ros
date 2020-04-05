Dual Quaternions ROS
====================

|travis| |tags|

.. |travis| image:: https://travis-ci.com/Achllle/dual_quaternions_ros.svg?branch=master
    :target: https://travis-ci.com/Achllle/dual_quaternions_ros

.. |tags| image:: https://img.shields.io/github/v/tag/achllle/dual_quaternions_ros
    :alt: GitHub tag (latest SemVer)
    :target: https://GitHub.com/Achllle/dual_quaternions_ros/tags/

Simple conversion methods for going from ROS `geometry_msgs` to dual quaternions and vice versa.
For the dual_quaternions pythonic repo, see dual_quaternions_.

-- _dual_quaternions: https://github.com/Achllle/dual_quaternions

Installation
------------

pip
~~~

.. code-block:: bash

  pip install dual_quaternions

ROS package
~~~~~~~~~~~

Release into apt is on its way. Until then you'll have to build the catkin package from source.

.. code-block:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/Achllle/dual_quaternions_ros
  cd ..
  rosdep install --from-paths src/dual_quaternions_ros --ignore-src
  catkin_make

Requirements
~~~~~~~~~~~~

* dual_quaternions
* geometry_msgs


Usage
-----

Import using::

    from dual_quaternions_ros import from_ros_pose, from_ros_transform, ros_pose, ros_transform

NOTE: there is no concept of 'from' and 'to' as frame names aren't tracked or used (e.g. use of Pose iso PoseStamped).
It is up to the user to keep track of those to avoid picking a convention (active vs. passive)

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
