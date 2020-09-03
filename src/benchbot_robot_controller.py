from __future__ import print_function

import argparse
import flask
from gevent import event, pywsgi, signal
import os
import re
import sys

# What does the controller need to do:

# ACTIONS
# - check for collisions (this is also the source of that route... should do directly)
# - send cmd velocities on the right topic
# - receive pose updates via the right topic

# STATE MANAGEMENT
# - Receive & report collision / dirty states
# - Run commands to "bring up system" (simulator: pose cmd, sim cmd, unreal cmd, real: robot specific launch file)
# - Manage the lifecycle of these running processes
# - Detect when topics have died & error gracefully

# RECEIVE FOLLOWING DATA FROM RUN SCRIPT
# - Map details (simulator: map path, real: costmap data?)
# - Possibly a list of map paths for scene change detection mode
# - Corresponding start poses

# PROVIDE ROUTES FOR
# - "hello" ping
# - Checking if collision has occurred
# - Checking if state is dirty
# - Connectivity check to required topics
# - Query currently running map number
# - Request move to next map
# - Restarting current map
# - Restarting from first map

# MISC
# - Write logs so they can be replayed on demand...
# - Configurable port & auto_start

# Proposed solution:

# 1. The controller is started by the run script for simulation (as it's the
#    same machine, & manually started prior on the real robot)
# 2. As the controller is now not guaranteed to be started by the script, data
#    shouldn't be passed through the constructor... it instead needs to be
#    passed via a 'config' route or something
# 3. Autostart now will not start until it receives a config
# 4. The 'config' will be a JSON dict of everything the controller needs to know
#    (could be annoying to create in bash...)
# 5. The controller is a flask server for handling requests, but also advertises
#    actuation services on ROS (i.e. when a supervisor has to resolve
#    'move_distance, 0.5', it would send a service request to the controller
#    asking to move that distance)
# 6. All the process handling logic has to be generalised... (note the command
#    to run depends on the robot selection, so should be part of the 'config')

DEFAULT_CONFIG = {
    'file_dirty_state': '/tmp/benchbot_dirty',
    'file_collisions': '/tmp/benchbot_collision',
    'logs_dir': '/tmp/benchbot_logs',
    'map_data': [],
    'start_cmds': [],
    'start_poses': [],
}


class Controller(object):

    def __init__(self, port=10000, auto_start=True):
        self.robot_address = 'http://0.0.0.0:' + str(port)
        self._auto_start = auto_start
        self._config = None
        self._config_valid = False
        self._instance = None

    def next(self):
        raise NotImplementedError(
            "Support for moving to the next scene in an "
            "experiment on a real robot is not yet implemented")

    def restart(self):
        # Restarts the robot by returning it to the starting point
        pass

    def run(self):
        # Setup all of the robot management functions
        # TODO some of these should probably be POST methods...
        robot_flask = flask.Flask(__name__)

        @robot_flask.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify(
                "Hello, I am the BenchBot real robot controller")

        @robot_flask.route('/configure', methods=['POST'])
        def __configure(new_config):
            self.setConfig(new_config)
            if self._auto_start and self._config_valid:
                self.start()
            return flask.jsonify({'configuration_valid': self._config_valid})

        @robot_flask.route('/is_collided', methods=['GET'])
        def __is_collided():
            return flask.jsonify({'is_collided': False})
            return flask.jsonify({'is_collided': self._instance.is_collided()})

        @robot_flask.route('/is_dirty', methods=['GET'])
        def __is_dirty():
            return flask.jsonify({'is_dirty': False})
            # return flask.jsonify({'is_dirty': self._instance.is_dirty()})

        @robot_flask.route('/is_running', methods=['GET'])
        def __is_running():
            return flask.jsonify({'is_running': True})
            # return flask.jsonify({'is_running': self._instance.is_running()})

        @robot_flask.route('/map_selection_number', methods=['GET'])
        def __map_selection_number():
            return flask.jsonify({'map_selection_number': 0})
            # return flask.jsonify({'map_selection_number': self._map_selection})

        @robot_flask.route('/next', methods=['GET'])
        def __next():
            raise NotImplementedError(
                "Support for moving to the next scene in an "
                "experiment on a real robot is not yet implemented")

        @robot_flask.route('/reset', methods=['GET'])
        def __reset():
            # Resets the simulator in the current scene
            try:
                self.restart()
                success = self._instance.is_running()
            except Exception as e:
                rospy.logerr(e)
                success = False
            return flask.jsonify({'reset_success': success})

        @robot_flask.route('/restart', methods=['GET'])
        def __restart():
            return __reset()

        # Configure our server
        robot_server = pywsgi.WSGIServer(
            re.split('http[s]?://', self.robot_address)[-1], robot_flask)
        evt = event.Event()
        signal.signal(signal.SIGINT, evt.set)
        signal.signal(signal.SIGQUIT, evt.set)
        signal.signal(signal.SIGTERM, evt.set)

        # Run the server & start the real robot controller
        robot_server.start()
        print("\nReal robot controller is now available @ '%s' ..." %
              self.robot_address)
        if self.auto_start:
            print("Starting the requested real robot ROS stack ... ", end="")
            sys.stdout.flush()
            self.start()
            print("Done")
        evt.wait()
        print("\nShutting down the real robot ROS stack & exiting ...")
        robot_server.stop()
        self.stop()
        print("Stopped")

    def setConfig(self, config):
        # Copy in the merged dicts
        self._config = DEFAULT_CONFIG.copy()
        self._config.update(config)

        # Update verdict on whether config is valid (things like auto_start may
        # be waiting for a valid config)
        # TODO proper checks...
        self._config_valid = True

    def start(self):
        pass

    def stop(self):
        pass


def __valid_file(value):
    path = os.path.abspath(value)
    if not os.path.exists(os.path.dirname(path)):
        raise argparse.ArgumentTypeError(
            "Output file path '%s' is in a non-existent directory" % value)
    return path


def __valid_envs_path(value):
    return True


def __valid_map_paths(value):
    return value.split(':')


def __valid_map_path(map_path, envs_path, metadata_location):
    valid_paths = [
        yaml.safe_load(open(f))['map_path'] for f in glob.glob(
            os.path.join(envs_path, metadata_location, '*.yaml'))
    ]
    if map_path not in valid_paths:
        raise argparse.ArgumentTypeError(
            "Map path '%s' was not found in the environment metadata files at:"
            " %s" % (map_path, os.path.join(envs_path, metadata_location)))
    return map_path


def __valid_metadata_location(envs_path, metadata_location):
    if not os.path.exists(os.path.join(envs_path, metadata_location)):
        raise argparse.ArgumentTypeError(
            "Environment metadata location does not exist: %s" %
            os.path.join(envs_path, metadata_location))
    return metadata_location


def __valid_poses(value):
    # TODO perform a much more robust check for pose validity...
    values = value.split(':')
    for p in values:
        if not re.search(r'\[[0-9\.\-e, ^\]]*\]', value):
            raise argparse.ArgumentTypeError(
                "Start pose is not in expected 7-field vector format: %s" %
                value)
    return values


if __name__ == "__main__":
    # Parse the input arguments sanely
    parser = argparse.ArgumentParser(
        description="Controller for BenchBot real robot")
    parser.add_argument('map_paths', type=__valid_map_paths)
    parser.add_argument('start_poses', type=__valid_poses)
    parser.add_argument('--file-collisions',
                        default='benchbot_collision',
                        type=__valid_file)
    parser.add_argument('--file-dirty-state',
                        default='benchbot_dirty',
                        type=__valid_file)
    parser.add_argument('--path-envs',
                        default=os.path.abspath('.'),
                        type=__valid_envs_path)
    parser.add_argument('--ros-command', default=None)
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()
    # __valid_metadata_location(args.path_envs, args.metadata_location)
    # for m in args.map_paths:
    #     __valid_map_path(m, args.path_envs, args.metadata_location)

    # Use the controller to run & manage the real robot
    rrc = RealRobotController(args.port)
    rrc.run()
