cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_examples)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roslaunch
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ensure_px4_params.py
  scripts/nmpc_tracking_regression_monitor.py
  scripts/offboard_velocity_follow.py
  scripts/ugv_circle_tracking_controller.py
  scripts/ugv_state_replay_validation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/fs150_uav1_nmpc_tracking.launch)
  roslaunch_add_file_check(launch/scout_ugv1_nmpc_tracking.launch)
endif()
