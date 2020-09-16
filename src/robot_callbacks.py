import base64
import cv2
import jsonpickle
import jsonpickle.ext.numpy as jet
import numpy as np
import pprint
import ros_numpy
import rospy
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import Twist, Vector3

jet.register_handlers()

_MOVE_HZ = 20

_MOVE_TOL_DIST = 0.01
_MOVE_TOL_YAW = np.deg2rad(1)

_MOVE_ANGLE_K = 3

_MOVE_POSE_K_RHO = 1.5
_MOVE_POSE_K_ALPHA = 7.5
_MOVE_POSE_K_BETA = -3

# TODO move this into robot controller
_MOVE_SPEED_FACTOR = 1


def __ang_to_b(matrix_a, matrix_b):
    # Computes the angle to matrix b, from the point specified in matrix a
    return np.arctan2(matrix_b[1, 3] - matrix_a[1, 3],
                      matrix_b[0, 3] - matrix_a[0, 3])


def __ang_to_b_wrt_a(matrix_a, matrix_b):
    # Computes the 2D angle to the location of homogenous transformation matrix
    # b, wrt a
    diff = __tr_b_wrt_a(matrix_a, matrix_b)
    return np.arctan2(diff[1, 3], diff[0, 3])


def __dist_from_a_to_b(matrix_a, matrix_b):
    # Computes 2D distance from homogenous transformation matrix a to b
    return np.linalg.norm(__tr_b_wrt_a(matrix_a, matrix_b)[0:2, 3])


def __pi_wrap(angle):
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi


def __pose_vector_to_tf_matrix(pv):
    # Expected format is [x,y,z,w,X,Y,Z]
    return np.vstack((np.hstack((Rot.from_quat(pv[0:4]).as_dcm(),
                                 np.array(pv[4:]).reshape(3,
                                                          1))), [0, 0, 0, 1]))


def __safe_dict_get(d, key, default):
    return d[key] if type(d) is dict and key in d else default


def __tf_ros_stamped_to_tf_matrix(tfs):
    # ROS... how do you still not have a method for this in 2020...
    return __pose_vector_to_tf_matrix([
        tfs.transform.rotation.x, tfs.transform.rotation.y,
        tfs.transform.rotation.z, tfs.transform.rotation.w,
        tfs.transform.translation.x, tfs.transform.translation.y,
        tfs.transform.translation.z
    ])


def __tr_b_wrt_a(matrix_a, matrix_b):
    # Computes matrix_b - matrix_a (transform b wrt to transform a)
    return np.matmul(np.linalg.inv(matrix_a), matrix_b)


def __transrpy_to_tf_matrix(trans, rpy):
    # Takes a translation vector & roll pitch yaw vector
    return __pose_vector_to_tf_matrix(
        np.hstack((Rot.from_euler('XYZ', rpy).as_quat(), trans)))


def __yaw(matrix):
    # Extracts the yaw value from a matrix
    return Rot.from_dcm(matrix[0:3, 0:3]).as_euler('XYZ')[2]


def __yaw_b_wrt_a(matrix_a, matrix_b):
    # Computes the yaw diff of homogenous transformation matrix b w.r.t. a
    return Rot.from_dcm(__tr_b_wrt_a(matrix_a,
                                     matrix_b)[0:3, 0:3]).as_euler('XYZ')[2]


def _debug_move(data, publisher, controller):
    # Accepts:
    # - rot_yaw: forms a tf matrix using yaw alone
    # - rot_xyzw: forms a tf matrix using xyzq quat
    # - trans_xyz: forms a tf matrix using xyz translation
    # Priority:
    # - Uses 'rot_yaw' if both rotation options are provided
    # Default:
    # - Uses 0 rotation & translation if values are missing
    def print_pose(pose):
        rpy = Rot.from_dcm(pose[0:3, 0:3]).as_euler('XYZ', degrees=True)
        return ("rpy: %f, %f, %f  xyz: %f, %f, %f" %
                tuple(np.hstack((rpy, pose[0:3, 3].transpose()))))

    pose_a = _current_pose(controller)
    print("STARTING @ POSE: %s" % print_pose(pose_a))
    relative_pose = (__transrpy_to_tf_matrix(
        __safe_dict_get(data, 'trans_xyz', [0, 0, 0]),
        [0, 0, np.deg2rad(__safe_dict_get(data, 'rot_yaw', 0))])
                     if 'rot_yaw' in data else __pose_vector_to_tf_matrix(
                         __safe_dict_get(data, 'rot_xyzw', [0, 0, 0, 1]) +
                         __safe_dict_get(data, 'trans_xyz', [0, 0, 0])))
    print("GOAL POSE (RELATIVE): %s" % print_pose(relative_pose))
    print("GOAL POSE (ABSOLUTE): %s" %
          print_pose(np.matmul(pose_a, relative_pose)))
    _move_to_pose(np.matmul(pose_a, relative_pose), publisher, controller)
    pose_b = _current_pose(controller)
    print("FINAL POSE (RELATIVE): %s" %
          print_pose(__tr_b_wrt_a(pose_a, pose_b)))
    print("FINAL POSE (ABSOLUTE): %s" % print_pose(pose_b))


def _current_pose(controller):
    return __tf_ros_stamped_to_tf_matrix(
        controller.tf_buffer.lookup_transform(
            controller.config['robot']['global_frame'],
            controller.config['robot']['robot_frame'], rospy.Time()))


def _move_to_angle(goal, publisher, controller):
    # Servo until orientation matches that of the requested goal
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not controller._robot('is_collided')['is_collided']:
        # Get latest orientation error
        orientation_error = __yaw_b_wrt_a(_current_pose(controller), goal)

        # Bail if exit conditions are met
        if np.abs(orientation_error) < _MOVE_TOL_YAW:
            break

        # Construct & send velocity msg
        vel_msg.angular.z = (_MOVE_SPEED_FACTOR * _MOVE_ANGLE_K *
                             orientation_error)
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())


def _move_to_pose(goal, publisher, controller):
    # Servo to desired pose using control described in Robotics, Vision, &
    # Control 2nd Ed (Corke, p. 108)
    # NOTE we also had to handle adjusting alpha correctly for reversing
    # rho = distance from current to goal
    # alpha = angle of goal vector in vehicle frame
    # beta = angle between current yaw & desired yaw
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not controller._robot('is_collided')['is_collided']:
        # Get latest position error
        current = _current_pose(controller)
        rho = __dist_from_a_to_b(current, goal)
        alpha = __pi_wrap(__ang_to_b(current, goal) - __yaw(current))
        beta = __pi_wrap(__yaw(goal) - __ang_to_b(current, goal))

        # Identify if the goal is behind us, & appropriately transform the
        # angles to reflect that we will reverse to the goal
        backwards = np.abs(__ang_to_b_wrt_a(current, goal)) > np.pi / 2
        if backwards:
            alpha = __pi_wrap(alpha + np.pi)
            beta = __pi_wrap(beta + np.pi)

        # If within distance tolerance, correct angle & quit (the controller
        # aims to drive the robot in at the correct angle, if it is already
        # "in" but the angle is wrong, it will get stuck!)
        if rho < _MOVE_TOL_DIST:
            _move_to_angle(goal, publisher, controller)
            break

        # Construct & send velocity msg
        vel_msg.linear.x = (_MOVE_SPEED_FACTOR * (-1 if backwards else 1) *
                            _MOVE_POSE_K_RHO * rho)
        vel_msg.angular.z = _MOVE_SPEED_FACTOR * (_MOVE_POSE_K_ALPHA * alpha +
                                                  _MOVE_POSE_K_BETA * beta)
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())


def create_pose_list(data, controller):
    # TODO REMOVE HACK FOR FIXING CAMERA NAME!!!
    tfs = {
        p: __tf_ros_stamped_to_tf_matrix(
            controller.tf_buffer.lookup_transform(
                controller.config['robot']['global_frame'], p, rospy.Time()))
        for p in controller.config['robot']['poses']
    }
    return jsonpickle.encode({
        'camera' if 'left_camera' in k else k: {
            'parent_frame': controller.config['robot']['global_frame'],
            'translation_xyz': v[:-1, -1],
            'rotation_rpy': Rot.from_dcm(v[:-1, :-1]).as_euler('XYZ'),
            'rotation_xyzw': Rot.from_dcm(v[:-1, :-1]).as_quat()
        } for k, v in tfs.items()
    })


def encode_camera_info(data, controller):
    return jsonpickle.encode({
        'frame_id': data.header.frame_id,
        'height': data.height,
        'width': data.width,
        'matrix_intrinsics': np.reshape(data.K, (3, 3)),
        'matrix_projection': np.reshape(data.P, (3, 4))
    })


def encode_color_image(data, controller):
    return {
        'encoding':
            data.encoding,
        'data':
            base64.b64encode(cv2.imencode('.png', ros_numpy.numpify(data))[1])
    }


def encode_depth_image(data, controller):
    return jsonpickle.encode(ros_numpy.numpify(data))


def encode_laserscan(data, controller):
    return jsonpickle.encode({
        'scans':
            np.array([[
                data.ranges[i],
                __pi_wrap(data.angle_min + i * data.angle_increment)
            ] for i in range(0, len(data.ranges))]),
        'range_min':
            data.range_min,
        'range_max':
            data.range_max
    })


def move_angle(data, publisher, controller):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(
        np.matmul(
            _current_pose(controller),
            __transrpy_to_tf_matrix([0, 0, 0], [
                0, 0,
                __pi_wrap(np.deg2rad(__safe_dict_get(data, 'angle', 0)))
            ])), publisher, controller)


def move_distance(data, publisher, controller):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(
        np.matmul(
            _current_pose(controller),
            __transrpy_to_tf_matrix(
                [__safe_dict_get(data, 'distance', 0), 0, 0], [0, 0, 0])),
        publisher, controller)


def move_next(data, publisher, controller):
    # Configure if this is our first step
    if controller.environment_name is None:
        controller.environment_name = (controller.config['environment_names'][
            controller._robot('map_selection_number')['map_selection_number']])
    if ('trajectory_pose_next' not in controller.environment_data[
            controller.environment_name]):
        controller.environment_data[
            controller.environment_name]['trajectory_pose_next'] = 0

    # Servo to the goal pose
    _move_to_pose(
        __pose_vector_to_tf_matrix(
            np.take(
                np.fromstring(
                    controller.environment_data[
                        controller.environment_name]['trajectory_poses']
                    [controller.environment_data[controller.environment_name]
                     ['trajectory_pose_next']].strip()[1:-1],
                    sep=", "), [1, 2, 3, 0, 4, 5, 6])), publisher, controller)

    # Register that we completed this goal
    controller.environment_data[
        controller.environment_name]['trajectory_pose_next'] += 1
    if (controller.environment_data[controller.environment_name]
        ['trajectory_pose_next'] >= len(controller.environment_data[
            controller.environment_name]['trajectory_poses'])):
        rospy.logerr("You have run out of trajectory poses!")
