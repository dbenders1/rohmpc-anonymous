cmake_minimum_required(VERSION 3.0.2)
project(rosbag2json)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/rosbag2json.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
