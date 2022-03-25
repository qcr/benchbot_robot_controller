import numpy as np
import ros_numpy
import rospy
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import Twist

from spatialmath import SE3, UnitQuaternion
from spatialmath_ros import SE3_to_SE2, tf_msg_to_SE3

_DEFAULT_SPEED_FACTOR = 1

_MOVE_HZ = 20

_MOVE_TOL_DIST = 0.01
_MOVE_TOL_YAW = np.deg2rad(1)

_MOVE_ANGLE_K = 3

_MOVE_POSE_K_RHO = 0.7
_MOVE_POSE_K_ALPHA = 7.5
_MOVE_POSE_K_BETA = -3

_MOVE_POSE_LINEAR_LIMITS = [-0.2, 0.5]
_MOVE_POSE_ANGULAR_LIMIT = 0.3


def __pi_wrap(angle):
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi


def __safe_dict_get(d, key, default):
    return d[key] if type(d) is dict and key in d else default


def _define_initial_pose(controller):
    # Check if we need to define initial pose (clean state and not already initialised)
    if not controller.instance.is_dirty(
    ) and 'initial_pose' not in controller.state.keys():
        controller.state['initial_pose'] = tf_msg_to_SE3(
            controller.tf_buffer.lookup_transform(
                controller.config['robot']['global_frame'],
                controller.config['robot']['robot_frame'], rospy.Time()))


def _get_noisy_pose(controller, child_frame):
    # Assumes initial pose of the robot has already been set
    # Get the initial pose of the robot within the world
    world_t_init_pose = controller.state['initial_pose']

    # Get the pose of child_frame w.r.t odom
    # TODO check if we should change odom from fixed name to definable
    odom_t_child = tf_msg_to_SE3(
        controller.tf_buffer.lookup_transform('odom', child_frame,
                                              rospy.Time()))
    # Noisy child should be init pose + odom_t_child
    return world_t_init_pose * odom_t_child


def _current_pose(controller):
    # Ensure initial pose has been defined if it hasn't already
    _define_initial_pose(controller)

    # Find the current robot pose (depending on pose mode)
    if 'ground_truth' in controller.config['task']['name']:
        # TF tree by default provides poses such that robot pose is GT
        # Just return the transform in the tree
        return tf_msg_to_SE3(
            controller.tf_buffer.lookup_transform(
                controller.config['robot']['global_frame'],
                controller.config['robot']['robot_frame'], rospy.Time()))
    else:
        # Convert pose to noisy mode by adding odom->robot transform to initial pose
        return _get_noisy_pose(controller,
                               controller.config['robot']['robot_frame'])


def _move_speed_factor(controller):
    return (controller.config['robot']['speed_factor'] if 'speed_factor'
            in controller.config['robot'] else _DEFAULT_SPEED_FACTOR)


def _move_to_angle(goal, publisher, controller):
    # Servo until orientation matches that of the requested goal
    g = SE3_to_SE2(goal)
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not controller.instance.is_collided():
        # Get latest orientation error
        orientation_error = (SE3_to_SE2(_current_pose).inv() * g).rpy()[2]

        # Bail if exit conditions are met
        if np.abs(orientation_error) < _MOVE_TOL_YAW:
            break

        # Construct & send velocity msg
        vel_msg.angular.z = (_move_speed_factor(controller) * _MOVE_ANGLE_K *
                             orientation_error)
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())


def _move_to_pose(goal, publisher, controller):
    # Servo to desired pose using control described in Robotics, Vision, &
    # Control 2nd Ed (Corke, p. 108)
    # (same variables used, but we add gamma to denote error in robot's
    # heading)
    g = SE3_to_SE2(goal)

    rho = None
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not controller.instance.is_collided() and (rho is None or
                                                     rho > _MOVE_TOL_DIST):
        # Get latest position error
        current = SE3_to_SE2(_current_pose(controller))
        error = SE3_to_SE2(current.inv()) * g
        backwards = np.abs(np.arctan2(error.t[1], error.t[0])) > np.pi / 2

        # Calculate values used in the controller
        rho = np.linalg.norm(error.t)
        alpha = np.arctan2(*np.flip(error.t))
        beta = __pi_wrap(-current.theta() - alpha + g.theta())

        # Construct velocity message
        if backwards:
            vel_msg.linear.x = -1 * _MOVE_POSE_K_RHO * rho
            vel_msg.angular.z = (
                _MOVE_POSE_K_ALPHA * __pi_wrap(alpha + np.pi) +
                _MOVE_POSE_K_BETA * __pi_wrap(beta + np.pi))
        else:
            vel_msg.linear.x = _MOVE_POSE_K_RHO * rho
            vel_msg.angular.z = (_MOVE_POSE_K_ALPHA * alpha +
                                 _MOVE_POSE_K_BETA * beta)

        # Apply per-robot speed factors, and speed limits
        vel_msg.linear.x = _move_speed_factor(controller) * vel_msg.linear.x
        vel_msg.angular.z = _move_speed_factor(controller) * vel_msg.angular.z

        vel_msg.linear.x = (_MOVE_POSE_LINEAR_LIMITS[1] if
                            vel_msg.linear.x > _MOVE_POSE_LINEAR_LIMITS[1] else
                            _MOVE_POSE_LINEAR_LIMITS[0] if vel_msg.linear.x <
                            _MOVE_POSE_LINEAR_LIMITS[0] else vel_msg.linear.x)
        vel_msg.angular.z = (_MOVE_POSE_K_ALPHA
                             if vel_msg.angular.z > _MOVE_POSE_ANGULAR_LIMIT
                             else -_MOVE_POSE_K_ALPHA if vel_msg.angular.z <
                             -_MOVE_POSE_ANGULAR_LIMIT else vel_msg.angular.z)

        # Publish velocity
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())


def create_pose_list(data, controller):
    # Ensure the initial pose has been defined if not already
    _define_initial_pose(controller)

    # Check what mode we are in for poses (ground_truth or noisy)
    gt_mode = controller.config['task']['localisation'] != 'noisy'
    tfs = {
        p: tf_msg_to_SE3(
            controller.tf_buffer.lookup_transform(
                controller.config['robot']['global_frame'], p, rospy.Time()))
        if gt_mode else _get_noisy_pose(controller, p)
        # If we are in noisy mode, poses become initial pose plus odom->target
        for p in controller.config['robot']['poses']
        if p != 'initial_pose'
    }

    # Add the initial pose if desired (not in tf tree)
    if 'initial_pose' in controller.config['robot']['poses']:
        tfs['initial_pose'] = controller.state['initial_pose']

    # TODO REMOVE HACK FOR FIXING CAMERA NAME!!!
    return {
        'camera' if 'left_camera' in k else k: {
            'parent_frame': controller.config['robot']['global_frame'],
            'translation_xyz': v.t,
            'rotation_rpy': v.rpy(),
            'rotation_xyzw': UnitQuaternion(v.rpy()).vec_xyzs
        } for k, v in tfs.items()
    }


def encode_camera_info(data, controller):
    return {
        'frame_id': data.header.frame_id,
        'height': data.height,
        'width': data.width,
        'matrix_intrinsics': np.reshape(data.K, (3, 3)),
        'matrix_projection': np.reshape(data.P, (3, 4))
    }


def encode_color_image(data, controller):
    return {'encoding': data.encoding, 'data': ros_numpy.numpify(data)}


def encode_depth_image(data, controller):
    return ros_numpy.numpify(data)


def encode_segment_image(data, controller):
    return {
        'class_segment_img': ros_numpy.numpify(data.class_segment_img),
        'instance_segment_img': ros_numpy.numpify(data.instance_segment_img),
        'class_ids': {
            class_name: class_id
            for (class_name, class_id) in zip(data.class_names, data.class_ids)
        }
    }


def encode_laserscan(data, controller):
    return {
        'scans':
            np.array([[
                data.ranges[i],
                __pi_wrap(data.angle_min + i * data.angle_increment)
            ] for i in range(0, len(data.ranges))]),
        'range_min':
            data.range_min,
        'range_max':
            data.range_max
    }


def move_angle(data, publisher, controller):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(
        _current_pose(controller) *
        SE3.Rz(__safe_dict_get(data, 'angle', 0), unit='deg'), publisher,
        controller)


def move_distance(data, publisher, controller):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(_current_pose * SE3.Tx(__safe_dict_get(data, 'distance', 0)),
                  publisher, controller)


def move_next(data, publisher, controller):
    # Configure if this is our first step
    if ('trajectory_pose_next' not in controller.state):
        controller.state['trajectory_pose_next'] = 0
        controller.state['trajectory_poses'] = controller.config[
            'environments'][
                controller.state['selected_environment']]['trajectory_poses']

    # Servo to the goal pose
    t = controller.state['trajectory_poses'][
        controller.state['trajectory_pose_next']]
    _move_to_pose(
        SE3(t[4::]) * UnitQuaternion(t[0], t[1:4]), publisher, controller)

    # Register that we completed this goal
    controller.state['trajectory_pose_next'] += 1
    if (controller.state['trajectory_pose_next'] >= len(
            controller.state['trajectory_poses'])):
        rospy.logerr("You have run out of trajectory poses!")
