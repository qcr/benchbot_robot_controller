from __future__ import print_function

import argparse
import flask
from gevent import event, pywsgi, signal
import os
import re
import subprocess
import sys
import traceback

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

DEFAULT_CONFIG_ROBOT = {
    'file_dirty_state': '/tmp/benchbot_dirty',
    'file_collisions': '/tmp/benchbot_collision',
    'logs_dir': '/tmp/benchbot_logs',
    'start_cmds': [],
}

VARIABLES = {
    'ENVS_PATH': "os.path.dirname(self.config_env['path'])",
    'ISAAC_PATH': "os.environ['ISAAC_SDK_PATH']",
    'MAP_PATH': "self.config_env['map_path']",
    'SIM_PATH': "os.environ['BENCHBOT_SIMULATOR_PATH']",
    'START_POSE': "self.config_env['start_pose_local']"
}

_CMD_DELETE_FILE = 'rm -f $FILENAME'
_CMD_FILE_EXISTS = '[ -f $FILENAME ]'


class ControllerInstance(object):

    def __init__(self, config_robot, config_env):
        self.config_robot = config_robot
        self.config_env = config_env

        self._cmds = None
        self._processes = None
        self._log_files = None

    def _replace_variables(self, text):
        for k, v in VARIABLES.items():
            text = text.replace("$%s" % k, str(eval(v)))
        return text

    def is_collided(self):
        return (subprocess.Popen(_CMD_FILE_EXISTS.replace(
            '$FILENAME', self.config_robot['file_collisions']),
                                 shell=True,
                                 executable='/bin/bash').wait() == 0)

    def is_dirty(self):
        return (subprocess.Popen(_CMD_FILE_EXISTS.replace(
            '$FILENAME', self.config_robot['file_dirty_state']),
                                 shell=True,
                                 executable='/bin/bash').wait() == 0)

    def is_running(self):
        # TODO do this properly with a list of topics sent as part of the
        # config
        return self._processes != None

    def start(self):
        if self.is_running():
            print("\nController Instance already appears to be running. "
                  "Please stop the existing instance before starting again.")
            return False

        # Get a set of commands by replacing variables with the config values
        self._cmds = [
            self._replace_variables(c) for c in self.config_robot['start_cmds']
        ]
        print("Running command:")
        print(self._cmds[0])

        # Start the set of commands, holding onto the process so we can manage
        # the lifecycle
        self.start_logging()
        self._processes = [
            subprocess.Popen(c,
                             shell=True,
                             executable='/bin/bash',
                             stdout=l,
                             stderr=l,
                             preexec_fn=os.setsid)
            for c, l in zip(self._cmds, self._log_files)
        ]

        # Wait until we move into a running state
        # TODO
        return True

    def start_logging(self):
        if not os.path.exists(self.config_robot['logs_dir']):
            os.makedirs(self.config_robot['logs_dir'])
        self._log_files = [
            open(os.path.join(self.config_robot['logs_dir'], str(i)), 'w+')
            for i in range(0, len(self._cmds))
        ]

    def stop(self):
        if not self.is_running():
            print("Controller Instance is not running. Skipping stop.")
            return False

        # Stop all of the open processes & logging
        for p in self._processes:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        for p in self._processes:
            p.wait()
        self._processes = None
        self.stop_logging()

        # Clear all temporary files
        for f in [
                self.config_robot['file_collisions'],
                self.config_robot['file_dirty_state']
        ]:
            subprocess.Popen(_CMD_DELETE_FILE.replace('$FILENAME', f),
                             shell=True,
                             executable='/bin/bash').wait()

    def stop_logging(self):
        # TODO
        pass


class RobotController(object):

    def __init__(self, port=10000, auto_start=True):
        self.robot_address = 'http://0.0.0.0:' + str(port)

        self._auto_start = auto_start
        self._config = None
        self._config_valid = False
        self._instance = None
        self._map_selection = None

    def next(self):
        raise NotImplementedError(
            "Support for moving to the next scene in an "
            "experiment on a real robot is not yet implemented")

    def restart(self):
        # Restarts the robot by returning it to the starting point
        self.stop()
        self.start()

    def run(self):
        # Setup all of the robot management functions
        # TODO some of these should probably be POST methods...
        robot_flask = flask.Flask(__name__)

        @robot_flask.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify(
                "Hello, I am the BenchBot real robot controller")

        @robot_flask.route('/configure', methods=['POST'])
        def __configure():
            try:
                self.setConfig(flask.request.json)
            except Exception as e:
                print(traceback.format_exc())
                raise (e)
            return flask.jsonify({'configuration_valid': self._config_valid})

        @robot_flask.route('/is_collided', methods=['GET'])
        def __is_collided():
            return flask.jsonify({'is_collided': False})
            # return flask.jsonify({'is_collided': self._instance.is_collided()})

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
                print(e)
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
        print("\nRobot controller is now available @ '%s' ..." %
              self.robot_address)
        print("Waiting to receive valid config data...")
        while not self._config_valid:
            if evt.wait(0.1):
                break

            if self._auto_start and self._config_valid:
                print("Starting the requested real robot ROS stack ... ",
                      end="")
                sys.stdout.flush()
                self.start()
                print("Done")

        # Wait until we get an exit signal, then shut down gracefully
        evt.wait()
        print("\nShutting down the real robot ROS stack & exiting ...")
        robot_server.stop()
        self.stop()
        print("Stopped")

    def setConfig(self, config):
        # Copy in the merged dicts
        self._config = {
            'robot': DEFAULT_CONFIG_ROBOT.copy(),
            'environments': {}
        }
        for k in self._config:
            self._config[k].update(config[k])

        # Set map selection as lowest order by default
        self._map_selection = min(self._config['environments'].items(),
                                  key=lambda e: e[1]['order'])[0]

        # Update verdict on whether config is valid (things like auto_start may
        # be waiting for a valid config)
        # TODO proper checks...
        self._config_valid = True

    def start(self):
        self._instance = ControllerInstance(
            self._config['robot'],
            self._config['environments'][self._map_selection])
        self._instance.start()

    def stop(self):
        if self._instance is not None:
            self._instance.stop()


if __name__ == "__main__":
    # Parse the input arguments sanely
    parser = argparse.ArgumentParser(
        description="Controller for a BenchBot robot")
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()

    # Use the controller to run & manage the real robot
    rrc = RobotController(args.port)
    rrc.run()
