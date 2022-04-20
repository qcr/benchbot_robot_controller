# This is our handwritten spatial helpers due to being stuck in
# Python2 and only having ROS's TF mess to work with...
#
# Ideally we'd get to move on from Python3 and use the spatial math
# toolbox

import numpy as np
import transforms3d as t3


def quat_from_SE3(pose):
    return t3.quaternions.mat2quat(pose[0:3, 0:3])[[1, 2, 3, 0]]


def quat_msg_to_SE3(quat_msg):
    return xyzwXYZ_to_SE3(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w, 0, 0,
                          0)


def pose_msg_to_SE3(pose_msg):
    t = pose_msg.pose.position
    r = pose_msg.pose.orientation
    return t3.affines.compose([t.x, t.y, t.z],
                              t3.quaternions.quat2mat([r.w, r.x, r.y, r.z]),
                              [1] * 3)


def rpy_from_SE3(pose):
    return t3.euler.mat2euler(pose[0:3, 0:3])


def SE2_to_xyt(pose):
    return np.array([pose[0, 2], pose[1, 2], yaw_from_SE2(pose)])


def SE3_from_translation(x=0, y=0, z=0):
    p = np.eye(4)
    p[0:3, 3] = np.array([x, y, z])
    return p


def SE3_from_yaw(yaw):
    x = np.eye(4)
    x[0:3, 0:3] = t3.euler.euler2mat(0, 0, yaw)
    return x


def SE3_to_SE2(pose):
    y = t3.euler.mat2euler(pose[0:3, 0:3])[2]
    return np.array([[np.cos(y), -np.sin(y), pose[0, 3]],
                     [np.sin(y), np.cos(y), pose[1, 3]], [0, 0, 1]])


def tf_msg_to_SE3(tf_msg):
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    return t3.affines.compose([t.x, t.y, t.z],
                              t3.quaternions.quat2mat([r.w, r.x, r.y, r.z]),
                              [1] * 3)


def wxyzXYZ_to_SE3(w, x, y, z, X, Y, Z):
    return xyzwXYZ_to_SE3(x, y, z, w, X, Y, Z)


def xyzwXYZ_to_SE3(x, y, z, w, X, Y, Z):
    return t3.affines.compose([X, Y, Z], t3.quaternions.quat2mat([w, x, y, z]),
                              [1] * 3)


def yaw_from_SE2(pose):
    return np.arctan2(pose[1, 0], pose[0, 0])


def pi_wrap(angle):
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi
