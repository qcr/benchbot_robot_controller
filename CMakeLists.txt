cmake_minimum_required(VERSION 3.0.2)
project(benchbot_robot_controller)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES benchbot_robot_controller
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )
