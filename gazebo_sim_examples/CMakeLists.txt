cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_examples)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ensure_px4_params.py
  scripts/offboard_velocity_follow.py
  scripts/ugv_circle_tracking_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
