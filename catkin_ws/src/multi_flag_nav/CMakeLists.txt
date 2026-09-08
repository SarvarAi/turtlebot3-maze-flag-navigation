cmake_minimum_required(VERSION 3.0.2)
project(multi_flag_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  move_base_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    actionlib_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    move_base_msgs
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/multi_flag_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
