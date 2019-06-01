"""
DualQuaternions for ROS

Author: Achille Verheye
License: MIT
"""
import numpy as np
import quaternion
import warnings
import json

import geometry_msgs.msg


class DualQuaternion(object):
    """
    dual number representation of quaternions to represent rigid transforms

    A quaternion q can be represented as q_r + q_d * eps with eps^2 = 0 and eps != 0

    Several ways to instantiate:
    $ dq = DualQuaternion(q_rot, q_dual) with both arguments instance of numpy-quaternion
    $ dq = DualQuaternion.from_dq_array(np.array([q_rw, q_rx, q_ry, q_rz, q_tx, q_ty, q_tz])
    $ dq = DualQuaternion.from_homogeneous_matrix([[r11, r12, r13, tx],
                                                   [r21, r22, r23, ty],
                                                   [r31, r32, r33, tz],
                                                   [  0,   0,   0,  1])
    $ dq = DualQuaternion.from_ros_pose(geometry_msgs.msg.Pose)
    $ dq = DualQuaternion.from_ros_transform(geometry_msgs.msg.Transform)
    $ dq = DualQuaternion.from_quat_pose_array([q_w, q_x, q_y, q_z, x, y, z])
    $ dq = DualQuaternion.from_translation_vector([x y z])
    $ dq = DualQuaternion.identity() --> zero translation, unit rotation

    The underlying representation for a single quaternion uses the format [w x y z]

    Inspired by 'A Beginners Guide to Dual-Quaternions' by Ben Kenwright and the dual quaternion
    implementation of the hand_eye_calibration package from ethz-asl
    """

    def __init__(self, q_r, q_d):
        if not isinstance(q_r, np.quaternion) or not isinstance(q_d, np.quaternion):
            raise ValueError("q_r and q_d must be of type np.quaternion. Instead received: {} and {}".format(
                type(q_r), type(q_d)))
        self.q_r = q_r
        self.q_d = q_d

    def __str__(self):
        return "rotation: {}, translation: {}, \n".format(repr(self.q_r), repr(self.q_d)) + \
               "translation vector: {}".format(repr(self.translation))

    def __repr__(self):
        return "<DualQuaternion: {0} + {1}e>".format(repr(self.q_r), repr(self.q_d))

    def __mul__(self, other):
        """
        Dual quaternion multiplication

        :param other: right hand side of the multiplication: DualQuaternion instance
        :return product: DualQuaternion object. Math:
                      dq1 * dq2 = dq1_r * dq2_r + (dq1_r * dq2_d + dq1_d * dq2_r) * eps
        """
        q_r_prod = self.q_r * other.q_r
        q_d_prod = self.q_r * other.q_d + self.q_d * other.q_r
        product = DualQuaternion(q_r_prod, q_d_prod)

        return product

    def __imul__(self, other):
        """
        Dual quaternion multiplication with self-assignment: dq1 *= dq2
        See __mul__
        """
        return self.__mul__(other)

    def __div__(self, other):
        """
        Dual quaternion division. See __truediv__

        :param other: DualQuaternion instance
        :return: DualQuaternion instance
        """
        return self.__truediv__(other)

    def __truediv__(self, other):
        """
        Dual quaternion division.

        :param other: DualQuaternion instance
        :return: DualQuaternion instance
        """
        other_r_sq = other.q_r * other.q_r
        prod_r = self.q_r * other.q_r / other_r_sq
        prod_d = (other.q_r * self.q_d - self.q_r * other.q_d) / other_r_sq

        return DualQuaternion(prod_r, prod_d)

    def __add__(self, other):
        """
        Dual Quaternion addition.
        :param other: pure translation dual quaternion
        :return: DualQuaternion(quaternion.one, self.q_d + other.q_d)
        """
        if self.q_r != quaternion.one or other.q_r != quaternion.one:
            warnings.warn('One or both of quaternions passed is not a pure translation!')
        return DualQuaternion(quaternion.one, self.q_d + other.q_d)

    def __eq__(self, other):
        return (self.q_r == other.q_r or self.q_r == -other.q_r) \
               and (self.q_d == other.q_d or self.q_d == -other.q_d)

    def __ne__(self, other):
        return not self == other

    def transform_point(self, point_xyz):
        """
        Apply the transformation to a given vector.

        Does not check frames - make sure you do this yourself.
        :param point_xyz: list or np.array in order: [x y z]
        :return: vector of length 3
        """
        # dq_point = DualQuaternion.identity()
        # dq_point.q_d = np.quaternion(0., point_xyz[0], point_xyz[1], point_xyz[2])
        # transformed_dq = (self * dq_point) * (self.conjugate())
        # return [transformed_dq.q_d.x, transformed_dq.q_d.y, transformed_dq.q_d.z]

        # this works, but not mathematically elegant (?)
        dq_point = DualQuaternion.from_translation_vector(point_xyz)
        transformed_dq = self * dq_point

        return transformed_dq.translation

    def transform_pose(self, pose):
        """
        Apply the transformation to a give pose.

        Example:
        >>> pose_in_b = geometry_msgs.msg.Pose(...)
        >>> T_a_b = DualQuaternion(...)
        >>> pose_in_a = T_a_b.transform_pose(pose_in_b)

        :param pose: geometry_msgs.msg.Pose
        :return: geometry_msgs.msg.Pose
        """
        dq_pose = DualQuaternion.from_ros_pose(pose)
        transformed_dq = self * dq_pose

        return transformed_dq.ros_pose

    @classmethod
    def from_dq_array(cls, r_wxyz_t_wxyz):
        """
        Create a DualQuaternion instance from two quaternions in list format

        :param r_wxyz_t_wxyz: np.array or python list: np.array([q_rw, q_rx, q_ry, q_rz, q_tx, q_ty, q_tz]
        """
        return cls(np.quaternion(*r_wxyz_t_wxyz[:4]), np.quaternion(*r_wxyz_t_wxyz[4:]))

    @classmethod
    def from_homogeneous_matrix(cls, arr):
        """
        Create a DualQuaternion instance from a 4 by 4 homogeneous transformation matrix

        :param arr: 4 by 4 list or np.array
        """
        q_r = quaternion.from_rotation_matrix(arr[:3, :3])
        quat_pose_array = np.zeros(7)
        quat_pose_array[:4] = np.array([q_r.w, q_r.x, q_r.y, q_r.z])

        quat_pose_array[4:] = arr[:3, 3]

        return cls.from_quat_pose_array(quat_pose_array)

    @classmethod
    def from_ros_pose(cls, pose_msg):
        """
        Create a DualQuaternion instance from a ROS Pose msg

        :param pose_msg: geometry_msgs.msg.Pose()
        """
        tra = pose_msg.position
        rot = pose_msg.orientation

        return cls.from_quat_pose_array([rot.w, rot.x, rot.y, rot.z, tra.x, tra.y, tra.z])

    @classmethod
    def from_ros_transform(cls, transform_msg):
        """
        Create a DualQuaternion instance from a ROS Transform msg

        :param transform_msg: geometry_msgs.msg.Transform()
        """
        tra = transform_msg.translation
        rot = transform_msg.rotation

        return cls.from_quat_pose_array([rot.w, rot.x, rot.y, rot.z, tra.x, tra.y, tra.z])

    @classmethod
    def from_quat_pose_array(cls, r_wxyz_t_xyz):
        """
        Create a DualQuaternion object from an array

        :param r_wxyz_t_xyz: list or np.array in order: [q_rw, q_rx, q_ry, q_rz, tx, ty, tz]
        """
        q_r = np.quaternion(*r_wxyz_t_xyz[:4])
        q_d = 0.5 * np.quaternion(0., *r_wxyz_t_xyz[4:]) * q_r

        return cls(q_r, q_d)

    @classmethod
    def from_translation_vector(cls, t_xyz):
        """
        Create a DualQuaternion object from a cartesian point
        :param t_xyz: list or np.array in order: [x y z]
        """
        return cls.from_quat_pose_array(np.append(np.array([1., 0., 0., 0.]), np.array(t_xyz)))

    @classmethod
    def identity(cls):
        return cls(quaternion.one, np.quaternion(0., 0., 0., 0.))

    def conjugate(self):
        """
        Return the individual quaternion conjugates (qr, qd)* = (qr*, qd*)

        This is equivalent to inverse of a homogeneous matrix. See also DualQuaternion.dual_conjugate().
        """
        return DualQuaternion(self.q_r.conjugate(), self.q_d.conjugate())

    def dual_conjugate(self):
        """
        Return the dual quaternion conjugate ( qr + eps*qd )* = ( qr - eps*qd )

        This is the mathematical conjugate. See also DualQuaternion.conjugate()
        """
        return DualQuaternion(self.q_r, -self.q_d)

    def inverse(self):
        """
        Return the dual quaternion inverse

        For unit dual quaternions dq.inverse() = dq.conjugate()
        """
        q_r_inv = self.q_r.inverse()
        return DualQuaternion(q_r_inv, -q_r_inv * self.q_d * q_r_inv)

    def normalize(self):
        """
        Normalize this dual quaternion

        Modifies in place, so this will not preserve self
        """
        normalized = self.normalized
        self.q_r = normalized.q_r
        self.q_d = normalized.q_d

    def is_unit(self):
        """True if the norm of the rotation part of the dual quaternion is 1"""
        return np.allclose(self.q_r.norm(), 1.0)

    def slerp(self, other, t):
        """Spherical Linear Interpolation"""
        raise NotImplementedError()

    def nlerp(self, other, t):
        raise NotImplementedError()

    def save(self, path):
        """Save the transformation to file

        :param path: absolute folder path and filename + extension
        :raises IOError: when the path does not exist
        """
        with open(path, 'w') as outfile:
            json.dump(self.as_dict, outfile)

    @classmethod
    def from_file(cls, path):
        """Load a DualQuaternion from file"""
        with open(path) as json_file:
            qdict = json.load(json_file)

        return cls.from_dq_array([qdict['r_w'], qdict['r_x'], qdict['r_y'], qdict['r_z'],
                                  qdict['d_w'], qdict['d_x'], qdict['d_y'], qdict['d_z']])

    @property
    def homogeneous_matrix(self):
        """Homogeneous 4x4 transformation matrix from the dual quaternion

        :return 4 by 4 np.array
        """
        homogeneous_mat = np.zeros([4, 4])
        rot_mat = quaternion.as_rotation_matrix(self.q_r)
        homogeneous_mat[:3, :3] = rot_mat
        homogeneous_mat[:3, 3] = np.array(self.translation)
        homogeneous_mat[3, 3] = 1.

        return homogeneous_mat

    @property
    def ros_pose(self):
        """ROS geometry_msgs.msg.Pose instance"""
        pose_msg = geometry_msgs.msg.Pose()
        quat_pose_arr = self.quat_pose_array
        rot = quat_pose_arr[:4]
        tra = quat_pose_arr[4:]
        pose_msg.position = geometry_msgs.msg.Point(*tra)
        # ROS uses [x y z w] format
        pose_msg.orientation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

        return pose_msg

    @property
    def ros_transform(self):
        """ROS geometry_msgs.msg.Transform instance"""
        transform_msg = geometry_msgs.msg.Transform()
        quat_pose_arr = self.quat_pose_array
        rot = quat_pose_arr[:4]
        tra = quat_pose_arr[4:]
        transform_msg.translation = geometry_msgs.msg.Vector3(*tra)
        # ROS uses [x y z w] format
        transform_msg.rotation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

        return transform_msg

    @property
    def quat_pose_array(self):
        """
        Get the list version of the dual quaternion as a quaternion followed by the translation vector

        :return: list [q_w, q_x, q_y, q_z, x, y, z]
        """
        return [self.q_r.w, self.q_r.x, self.q_r.y, self.q_r.z] + self.translation

    @property
    def dq_array(self):
        """
        Get the list version of the dual quaternion as the rotation quaternion followed by the translation quaternion

        :return: list [q_rw, q_rx, q_ry, q_rz, q_tx, q_ty, q_tz]
        """
        return [self.q_r.w, self.q_r.x, self.q_r.y, self.q_r.z,
                self.q_d.w, self.q_d.x, self.q_d.y, self.q_d.z]

    @property
    def translation(self):
        """Get the translation component of the dual quaternion in vector form

        :return: list [x y z]
        """
        mult = (2.0 * self.q_d) * self.q_r.conjugate()

        return [mult.x, mult.y, mult.z]

    @property
    def normalized(self):
        """Return a copy of the normalized quaternion rotation"""
        return DualQuaternion(self.q_r.normalized(), self.q_d)

    @property
    def as_dict(self):
        """dictionary containing the dual quaternion"""
        return {'r_w': self.q_r.w, 'r_x': self.q_r.x, 'r_y': self.q_r.y, 'r_z': self.q_r.z,
                'd_w': self.q_d.w, 'd_x': self.q_d.x, 'd_y': self.q_d.y, 'd_z': self.q_d.z}

    @property
    def skew(self):
        """
        TODO

        :return:
        """
        raise NotImplementedError()
