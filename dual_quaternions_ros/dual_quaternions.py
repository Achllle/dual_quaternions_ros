"""
DualQuaternions for ROS

Author: Achille Verheye
License: MIT
"""
import numpy as np
import quaternion
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
    $ dq = DualQuaternion.from_screw([lx, ly, lz], [mx, my, mz], theta, d)

    The underlying representation for a single quaternion uses the format [w x y z]
    The rotation part (non-dual) will always be normalized.
    """

    def __init__(self, q_r, q_d, normalize=False):
        if not isinstance(q_r, np.quaternion) or not isinstance(q_d, np.quaternion):
            raise ValueError("q_r and q_d must be of type np.quaternion. Instead received: {} and {}".format(
                type(q_r), type(q_d)))
        if normalize:
            self.q_d = q_d / q_r.norm()
            self.q_r = q_r.normalize()
        else:
            self.q_r = q_r
            self.q_d = q_d

    def __str__(self):
        return "rotation: {}, translation: {}, \n".format(repr(self.q_r), repr(self.q_d)) + \
               "translation vector: {}".format(repr(self.translation()))

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

    def __rmul__(self, other):
        """Multiplication with a scalar

        :param other: scalar
        """
        return DualQuaternion(self.q_r * other, self.q_d * other)

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
        :param other: dual quaternion
        :return: DualQuaternion(self.q_r + other.q_r, self.q_d + other.q_d)
        """
        return DualQuaternion(self.q_r + other.q_r, self.q_d + other.q_d)

    def __eq__(self, other):
        return (np.isclose(self.q_r, other.q_r) or np.isclose(self.q_r, -other.q_r)) \
               and (np.isclose(self.q_d, other.q_d) or np.isclose(self.q_d, -other.q_d))

    def __ne__(self, other):
        return not self == other

    def transform_point(self, point_xyz):
        """
        Convenience function to apply the transformation to a given vector.
        dual quaternion way of applying a rotation and translation using matrices Rv + t or H[v; 1]
        This works out to be: sigma @ (1 + ev) @ sigma.combined_conjugate()

        If we write self = p + eq, this can be expanded to 1 + eps(rvr* + t)
        with r = p and t = 2qp* which should remind you of Rv + t and the quaternion
        transform_point() equivalent (rvr*)

        Does not check frames - make sure you do this yourself.
        :param point_xyz: list or np.array in order: [x y z]
        :return: vector of length 3
        """
        dq_point = DualQuaternion.from_dq_array([1, 0, 0, 0,
                                                 0, point_xyz[0], point_xyz[1], point_xyz[2]])
        res_dq = self * dq_point * self.combined_conjugate()
        
        return res_dq.dq_array()[5:]

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
        Create a DualQuaternion object from an array of a quaternion r and translation t
        sigma = r + eps/2 * t * r

        :param r_wxyz_t_xyz: list or np.array in order: [q_rw, q_rx, q_ry, q_rz, tx, ty, tz]
        """
        q_r = np.quaternion(*r_wxyz_t_xyz[:4]).normalized()
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

    def quaternion_conjugate(self):
        """
        Return the individual quaternion conjugates (qr, qd)* = (qr*, qd*)

        This is equivalent to inverse of a homogeneous matrix. It is used in applying
        a transformation to a line expressed in Plucker coordinates.
        See also DualQuaternion.dual_conjugate() and DualQuaternion.combined_conjugate().
        """
        return DualQuaternion(self.q_r.conjugate(), self.q_d.conjugate())

    def dual_number_conjugate(self):
        """
        Return the dual number conjugate (qr, qd)* = (qr, -qd)

        This form of conjugate is seldom used.
        See also DualQuaternion.quaternion_conjugate() and DualQuaternion.combined_conjugate().
        """
        return DualQuaternion(self.q_r, -self.q_d)

    def combined_conjugate(self):
        """
        Return the combination of the quaternion conjugate and dual number conjugate
        (qr, qd)* = (qr*, -qd*)

        This form is commonly used to transform a point
        See also DualQuaternion.dual_number_conjugate() and DualQuaternion.quaternion_conjugate().
        """
        return DualQuaternion(self.q_r.conjugate(), -self.q_d.conjugate())

    def inverse(self):
        """
        Return the dual quaternion inverse

        For unit dual quaternions dq.inverse() = dq.quaternion_conjugate()
        """
        q_r_inv = self.q_r.inverse()
        return DualQuaternion(q_r_inv, -q_r_inv * self.q_d * q_r_inv)

    def is_normalized(self):
        """Check if the dual quaternion is normalized"""
        if np.isclose(self.q_r.norm(), 0):
            return True
        rot_normalized = np.isclose(self.q_r.norm(), 1)
        trans_normalized = np.isclose(self.q_d / self.q_r.norm(), self.q_d)
        return rot_normalized and trans_normalized

    def normalize(self):
        """
        Normalize this dual quaternion

        Modifies in place, so this will not preserve self
        """
        normalized = self.normalized()
        self.q_r = normalized.q_r
        self.q_d = normalized.q_d

    def pow(self, exponent):
        """self^exponent"""
        return self.exp(exponent * self.log(self))

    @classmethod
    def exp(cls, dq):
        q_r = dq.q_r.exp()
        q_d = q_r * dq.q_d
        return cls(q_r, q_d)

    @classmethod
    def log(cls, dq):
        q_r = dq.q_r.log()
        q_d = dq.q_r.inverse() * dq.q_d
        return cls(q_r, q_d, normalize=False)

    @classmethod
    def sclerp(cls, start, stop, t):
        """Screw Linear Interpolation"""
        return (stop * start.quaternion_conjugate()).pow(t) * start

    def nlerp(self, other, t):
        raise NotImplementedError()

    def save(self, path):
        """Save the transformation to file

        :param path: absolute folder path and filename + extension
        :raises IOError: when the path does not exist
        """
        with open(path, 'w') as outfile:
            json.dump(self.as_dict(), outfile)

    @classmethod
    def from_file(cls, path):
        """Load a DualQuaternion from file"""
        with open(path) as json_file:
            qdict = json.load(json_file)

        return cls.from_dq_array([qdict['r_w'], qdict['r_x'], qdict['r_y'], qdict['r_z'],
                                  qdict['d_w'], qdict['d_x'], qdict['d_y'], qdict['d_z']])

    def homogeneous_matrix(self):
        """Homogeneous 4x4 transformation matrix from the dual quaternion

        :return 4 by 4 np.array
        """
        homogeneous_mat = np.zeros([4, 4])
        rot_mat = quaternion.as_rotation_matrix(self.q_r)
        homogeneous_mat[:3, :3] = rot_mat
        homogeneous_mat[:3, 3] = np.array(self.translation())
        homogeneous_mat[3, 3] = 1.

        return homogeneous_mat

    def ros_pose(self):
        """ROS geometry_msgs.msg.Pose instance"""
        pose_msg = geometry_msgs.msg.Pose()
        quat_pose_arr = self.quat_pose_array()
        rot = quat_pose_arr[:4]
        tra = quat_pose_arr[4:]
        pose_msg.position = geometry_msgs.msg.Point(*tra)
        # ROS uses [x y z w] format
        pose_msg.orientation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

        return pose_msg

    def ros_transform(self):
        """ROS geometry_msgs.msg.Transform instance"""
        transform_msg = geometry_msgs.msg.Transform()
        quat_pose_arr = self.quat_pose_array()
        rot = quat_pose_arr[:4]
        tra = quat_pose_arr[4:]
        transform_msg.translation = geometry_msgs.msg.Vector3(*tra)
        # ROS uses [x y z w] format
        transform_msg.rotation = geometry_msgs.msg.Quaternion(rot[1], rot[2], rot[3], rot[0])

        return transform_msg

    def quat_pose_array(self):
        """
        Get the list version of the dual quaternion as a quaternion followed by the translation vector
        given a dual quaternion p + eq, the rotation in quaternion form is p and the translation in
        quaternion form is 2qp*

        :return: list [q_w, q_x, q_y, q_z, x, y, z]
        """
        return [self.q_r.w, self.q_r.x, self.q_r.y, self.q_r.z] + self.translation()

    def dq_array(self):
        """
        Get the list version of the dual quaternion as the rotation quaternion followed by the translation quaternion

        :return: list [q_rw, q_rx, q_ry, q_rz, q_tx, q_ty, q_tz]
        """
        return [self.q_r.w, self.q_r.x, self.q_r.y, self.q_r.z,
                self.q_d.w, self.q_d.x, self.q_d.y, self.q_d.z]

    def translation(self):
        """Get the translation component of the dual quaternion in vector form

        :return: list [x y z]
        """
        mult = (2.0 * self.q_d) * self.q_r.conjugate()

        return [mult.x, mult.y, mult.z]

    def normalized(self):
        """Return a copy of the normalized dual quaternion"""
        norm_qr = self.q_r.norm()
        return DualQuaternion(self.q_r/norm_qr, self.q_d/norm_qr)

    def as_dict(self):
        """dictionary containing the dual quaternion"""
        return {'r_w': self.q_r.w, 'r_x': self.q_r.x, 'r_y': self.q_r.y, 'r_z': self.q_r.z,
                'd_w': self.q_d.w, 'd_x': self.q_d.x, 'd_y': self.q_d.y, 'd_z': self.q_d.z}

    def screw(self):
        """
        Get the screw parameters for this dual quaternion.
        Chasles' theorem (Mozzi, screw theorem) states that any rigid displacement is equivalent to a rotation about
        some line and a translation in the direction of the line. This line does not go through the origin!
        This function returns the Plucker coordinates for the screw axis (l, m) as well as the amount of rotation
        and translation, theta and d.
        If the dual quaternion represents a pure translation, theta will be zero and the screw moment m will be at
        infinity.

        :return: l (unit length), m, theta, d
        :rtype np.array(3), np.array(3), float, float
        """
        # start by extracting theta and l directly from the real part of the dual quaternion
        theta = self.q_r.angle()
        theta_close_to_zero = np.isclose(theta, 0)
        t = np.array(self.translation())

        if not theta_close_to_zero:
            l = self.q_r.vec / np.sin(theta/2)  # since q_r is normalized, l should be normalized too

            # displacement d along the line is the projection of the translation onto the line l
            d = np.dot(t, l)

            # m is a bit more complicated. Derivation see K. Daniliidis, Hand-eye calibration using Dual Quaternions
            m = 0.5 * (np.cross(t, l) + np.cross(l, np.cross(t, l) / np.tan(theta / 2)))
        else:
            # l points along the translation axis
            d = np.linalg.norm(t)
            if not np.isclose(d, 0):  # unit transformation
                l = t / d
            else:
                l = (0, 0, 0)
            m = np.array([np.inf, np.inf, np.inf])

        return l, m, theta, d

    @classmethod
    def from_screw(cls, l, m, theta, d):
        """
        Create a DualQuaternion from screw parameters

        :param l: unit vector defining screw axis direction
        :param m: screw axis moment, perpendicular to l and through the origin
        :param theta: screw angle; rotation around the screw axis
        :param d: displacement along the screw axis
        """
        l = np.array(l)
        m = np.array(m)
        if not np.isclose(np.linalg.norm(l), 1):
            raise AttributeError("Expected l to be a unit vector, received {} with norm {} instead"
                                 .format(l, np.linalg.norm(l)))
        theta = float(theta)
        d = float(d)
        q_r = np.quaternion(np.cos(theta/2), 0, 0, 0)
        q_r.vec = np.sin(theta/2) * l
        q_d = np.quaternion(-d/2 * np.sin(theta/2), 0, 0, 0)
        q_d.vec = np.sin(theta/2) * m + d/2 * np.cos(theta/2) * l

        return cls(q_r, q_d)

    def skew(self):
        """
        TODO

        :return:
        """
        raise NotImplementedError()
