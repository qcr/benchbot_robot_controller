from __future__ import print_function

import argparse
import copy
import flask
from gevent import event, pywsgi, signal
import importlib
import jsonpickle
import jsonpickle.ext.numpy as jet
import os
import re
import rospy
import subprocess
import sys
import time
import tf2_ros
import threading
import traceback
import json

jet.register_handlers()

DEFAULT_CONFIG_ROBOT = {
    'file_dirty_state': '/tmp/benchbot_dirty',
    'file_collisions': '/tmp/benchbot_collision',
    'logs_dir': '/tmp/benchbot_logs',
    'start_cmds': [],
}

DEFAULT_CONFIG_ENV = {"object_labels": []}

DEFAULT_STATE = {"selected_environment": 0}

CONN_API_TO_ROS = 'api_to_ros'
CONN_ROS_TO_API = 'ros_to_api'
CONN_ROSCACHE_TO_API = 'roscache_to_api'
CONNS = [CONN_API_TO_ROS, CONN_ROS_TO_API, CONN_ROSCACHE_TO_API]

TIMEOUT_ROS_PING = 5
TIMEOUT_STARTUP = 9000000000000

VARIABLES = {
    'ENVS_PATH':
        "os.path.dirname(self.config_env['_file_path'])",
    'ISAAC_PATH':
        "os.environ.get('ISAAC_SDK_PATH', '')",
    'MAP_PATH':
        "self.config_env['map_path']",
    'ROBOT_PATH':
        "os.path.dirname(self.config_robot['_file_path'])",
    'SIM_PATH':
        "os.environ.get('BENCHBOT_SIMULATOR_PATH', '')",
    'START_POSE':
        "self.config_env['start_pose']",
    'OBJECT_LABELS':
        'str(json.dumps([{"name": lbl.encode("ascii")} for lbl in self.config_env["object_labels"]]))'
}

_CMD_DELETE_FILE = 'rm -f $FILENAME'
_CMD_FILE_EXISTS = '[ -f $FILENAME ]'


def _to_simple_dict(data):
    out = {}
    if hasattr(data, '__slots__'):
        for k in data.__slots__:
            if hasattr(getattr(data, k), '__slots__'):
                out[k] = _to_simple_dict(getattr(data, k))
            else:
                out[k] = getattr(data, k)
    else:
        out = data
    return out


class ControllerInstance(object):

    def __init__(self, config_robot, config_env, ros_subs):
        self.config_robot = config_robot
        self.config_env = config_env

        self.ros_subs = ros_subs

        self._cmds = None
        self._processes = None
        self._log_files = None

    def _replace_variables(self, text):
        for k, v in VARIABLES.items():
            text = text.replace("$%s" % k, str(eval(v)))
        return text

    def health_check(self, check_running=True):
        # Checks the health of the currently running instance, printing errors
        # if they're encountered
        fails = [p.poll() is not None for p in self._processes]
        if any(fails):
            i = fails.index(True)
            print("\nTHE PROCESS STARTED BY THE FOLLOWING "
                  "COMMAND HAS EXITED EARLY:")
            print("\t%s" % self._cmds[i])
            print("\nDUMPING LOGGED OUTPUT:")
            with open(os.path.join(self.config_robot['logs_dir'], str(i)),
                      'r') as f:
                print(f.read())
            return False
        elif check_running and not self.is_running():
            return False
        return True

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
        if self.ros_subs is None:
            return self._processes != None
        else:
            try:
                for s in self.ros_subs:
                    rospy.wait_for_message(s.resolved_name,
                                           s.data_class,
                                           timeout=TIMEOUT_ROS_PING)
            except Exception as e:
                rospy.loginfo("FAILURE: %s" % e)
                return False
            return True

    def start(self):
        if self.is_running():
            print("\nController Instance already appears to be running. "
                  "Please stop the existing instance before starting again.")
            return False

        # Get a set of commands by replacing variables with the config values
        self._cmds = [
            self._replace_variables(c) for c in self.config_robot['start_cmds']
        ]

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
        start_time = time.time()
        while not self.is_running():
            time.sleep(0.25)
            if not self.health_check(check_running=False):
                return False
            elif (time.time() - start_time > TIMEOUT_STARTUP and
                  not self.health_check()):
                print("\nROBOT WAS NOT DETECTED TO BE RUNNING AFTER %ss (no "
                      "data on ROS TOPICS). DUMPING LOGS FOR ALL COMMANDS..." %
                      TIMEOUT_STARTUP)
                for i, _ in enumerate(self._cmds):
                    print("COMMAND:")
                    print("\t%s" % self._cmds[i])
                    print("OUTPUT:")
                    with open(
                            os.path.join(self.config_robot['logs_dir'],
                                         str(i)), 'r') as f:
                        print(f.read())
                return False
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

        # Wait until controller instance is detected as not running
        while self.is_running():
            time.sleep(0.25)

    def stop_logging(self):
        for f in self._log_files:
            f.close()


class RobotController(object):

    def __init__(self, port=10000, auto_start=True):
        self.robot_address = 'http://0.0.0.0:' + str(port)

        self._auto_start = auto_start

        self.config = None
        self.config_valid = False

        self.state = None
        self.connections = {}
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.instance = None

        self.wipe()

    @staticmethod
    def _attempt_connection_imports(connection_data):
        topic_class = None
        if 'ros_type' in connection_data:
            x = connection_data['ros_type'].split('/')
            topic_class = getattr(importlib.import_module('%s.msg' % x[0]),
                                  x[1])

        callback_robot_fn = None
        if 'callback_robot' in connection_data:
            callback_robot_fn = RobotController._dynamic_callback_import(
                connection_data['callback_robot'])

        callback_caching_fn = None
        if 'callback_caching' in connection_data:
            callback_caching_fn = RobotController._dynamic_callback_import(
                connection_data['callback_caching'])

        return (topic_class, callback_robot_fn, callback_caching_fn)

    @staticmethod
    def _dynamic_callback_import(callback_string):
        x = callback_string.rsplit('.', 1)
        return getattr(importlib.import_module(x[0]), x[1])

    def _call_connection(self, connection_name, data=None):
        if (self.connections[connection_name]['type']
                in [CONN_ROS_TO_API, CONN_ROSCACHE_TO_API]):
            # Overwrite the data because it is an observation (data should be
            # none anyway with an observation as we do not 'parameterise' an
            # observation)
            self.connections[connection_name]['condition'].acquire()
            data = copy.deepcopy(self.connections[connection_name]['data'])
            self.connections[connection_name]['condition'].release()

            return (data if
                    self.connections[connection_name]['callback_robot'] is None
                    else self.connections[connection_name]['callback_robot'](
                        data, self))
        elif self.connections[connection_name]['type'] == CONN_API_TO_ROS:
            if self.connections[connection_name]['callback_robot'] is None:
                self.connections[connection_name]['ros'].publish(data)
            else:
                self.connections[connection_name]['callback_robot'](
                    data, self.connections[connection_name]['ros'], self)
        else:
            print("UNIMPLEMENTED CONNECTION CALL: %s" %
                  self.connections[connection_name]['type'])

    def _env_next(self):
        return (self.state['selected_environment'] +
                1 if self.state['selected_environment'] +
                1 < len(self.config['environments']) else 0)

    def _generate_subscriber_callback(self, connection_name):

        def __cb(data):
            if (self.connections[connection_name]['type'] ==
                    CONN_ROSCACHE_TO_API):
                data = self.connections[connection_name]['callback_caching'](
                    data, self.connections[connection_name]['data'])
            self.connections[connection_name]['condition'].acquire()
            self.connections[connection_name]['data'] = data
            self.connections[connection_name]['condition'].notify()
            self.connections[connection_name]['condition'].release()

        return __cb

    def _register_connection(self, connection_name, connection_data):
        # Pull out imported components from the connection data
        topic_class, callback_robot_fn, callback_caching_fn = (
            RobotController._attempt_connection_imports(connection_data))

        # Register the connection with the robot
        self.connections[connection_name] = {
            'type': connection_data['connection'],
            'callback_robot': callback_robot_fn,
            'callback_caching': callback_caching_fn,
            'ros': None,
            'data': None,
            'condition': threading.Condition()
        }

        # Construct connections if possible
        if topic_class != None:
            if connection_data['connection'] in [
                    CONN_ROS_TO_API, CONN_ROSCACHE_TO_API
            ]:
                self.connections[connection_name]['ros'] = rospy.Subscriber(
                    connection_data['ros_topic'], topic_class,
                    self._generate_subscriber_callback(connection_name))
            elif connection_data['connection'] == CONN_API_TO_ROS:
                self.connections[connection_name]['ros'] = rospy.Publisher(
                    connection_data['ros_topic'], topic_class, queue_size=1)
            else:
                print("UNIMPLEMENTED POST CONNECTION: %s" %
                      connection_data['connection'])

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
            return flask.jsonify("Hello, I am the BenchBot robot controller")

        @robot_flask.route('/config/', methods=['GET'])
        def __config_full():
            return flask.jsonify(self.config)

        @robot_flask.route('/config/<config>', methods=['GET'])
        def __config(config):
            if config in self.config:
                return flask.jsonify(self.config[config])
            else:
                rospy.logerr("Requested non-existent config: %s" % config)
                flask.abort(404)

        @robot_flask.route('/configure', methods=['POST'])
        def __configure():
            try:
                self.set_config(flask.request.json)
            except Exception as e:
                print(traceback.format_exc())
                raise (e)
            return flask.jsonify({'configuration_valid': self.config_valid})

        @robot_flask.route('/connections/<connection>',
                           methods=['GET', 'POST'])
        def __connection(connection):
            # Handle all connection calls (typically sent via the supervisor)
            if connection not in self.config['robot']['connections']:
                rospy.logerr("Requested undefined connection: %s" % connection)
                flask.abort(404)
            try:
                return flask.jsonify(
                    jsonpickle.encode(
                        self._call_connection(connection,
                                              data=flask.request.get_json())))
            except Exception as e:
                rospy.logerr(
                    "Robot Controller failed on processing connection "
                    "'%s' with error:\n%s" % (connection, repr(e)))
                flask.abort(500)

        @robot_flask.route('/is_collided', methods=['GET'])
        def __is_collided():
            return flask.jsonify({'is_collided': self.instance.is_collided()})

        @robot_flask.route('/is_dirty', methods=['GET'])
        def __is_dirty():
            return flask.jsonify({'is_dirty': self.instance.is_dirty()})

        @robot_flask.route('/is_finished', methods=['GET'])
        def __is_finished():
            return flask.jsonify({
                'is_finished':
                    (False if 'trajectory_pose_next' not in self.state else
                     self.state['trajectory_pose_next'] >= len(
                         self.state['trajectory_poses']))
            })

        @robot_flask.route('/is_running', methods=['GET'])
        def __is_running():
            try:
                return flask.jsonify(
                    {'is_running': self.instance.is_running()})
            except Exception as e:
                rospy.logerr(e)

        @robot_flask.route('/next', methods=['GET'])
        def __next():
            try:
                if self._env_next() == 0:
                    raise ValueError(
                        "There is no next map; at the end of the list")
                self.stop()
                self.state['selected_environment'] = self._env_next()
                self.start()
                success = True
            except Exception as e:
                rospy.logerr(e)
                success = False
            return flask.jsonify({'next_success': success})

        @robot_flask.route('/reset', methods=['GET'])
        def __reset():
            # Resets the robot in the current scene
            try:
                self.restart()
                success = self.instance.is_running()
            except Exception as e:
                rospy.logerr(e)
                success = False
            return flask.jsonify({'reset_success': success})

        @robot_flask.route('/restart', methods=['GET'])
        def __restart():
            # Restarts the robot in the FIRST scene
            self.wipe()
            resp = __reset()
            resp.data = resp.data.replace('reset', 'restart')
            return resp

        @robot_flask.route('/selected_environment', methods=['GET'])
        def __selected_env():
            try:
                return flask.jsonify({
                    'name':
                        self.config['environments']
                        [self.state['selected_environment']]['name'],
                    'variant':
                        self.config['environments']
                        [self.state['selected_environment']]['variant'],
                    'number':
                        self.state['selected_environment']
                })
            except Exception as e:
                rospy.logerr(e)

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
        while not self.config_valid:
            if evt.wait(0.1):
                break

            if self._auto_start and self.config_valid:
                print("Starting the requested real robot ROS stack ... ",
                      end="")
                sys.stdout.flush()
                self.start()
                print("Done")

        # Wait until we get an exit signal or crash, then shut down gracefully
        while self.instance.health_check():
            if evt.wait(0.1):
                break
        print("\nShutting down the real robot ROS stack & exiting ...")
        robot_server.stop()
        self.stop()
        print("Stopped")

    def set_config(self, config):
        # Stop any running instances (we are bailing on any notion of dynamic
        # reconfiguration...)
        self.stop()

        # Copy in the merged dicts
        self.config = {
            'environments': [
                DEFAULT_CONFIG_ENV.copy() for e in config['environments']
            ],
            'robot': DEFAULT_CONFIG_ROBOT.copy(),
            'task': {}
        }
        for k in self.config.keys():
            if isinstance(self.config[k], dict):
                self.config[k].update(config[k])
            elif isinstance(self.config[k], list):
                for i, _ in enumerate(self.config[k]):
                    self.config[k][i].update(config[k][i])

        # Register all of the required connections
        for k, v in self.config['robot']['connections'].items():
            if 'connection' in v and v['connection'] in CONNS:
                self._register_connection(k, v)
            else:
                raise ValueError(
                    "Robot connection definition '%s' has "
                    "unsupported connection type: %s" %
                    (k, v['connection'] if 'connection' in v else None))

        # Update verdict on whether config is valid (things like auto_start may
        # be waiting for a valid config)
        # TODO proper checks...
        self.config_valid = True

    def start(self):
        self.state = {
            k: v for k, v in self.state.items() if k in DEFAULT_STATE
        }
        self.instance = ControllerInstance(
            self.config['robot'],
            self.config['environments'][self.state['selected_environment']], [
                c['ros']
                for c in self.connections.values()
                if c['type'] == CONN_ROS_TO_API and c['ros'] != None
            ])
        self.instance.start()

    def stop(self):
        if self.instance is not None:
            self.instance.stop()

    def wipe(self):
        self.state = copy.deepcopy(DEFAULT_STATE)
