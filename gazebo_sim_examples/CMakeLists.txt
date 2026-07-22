cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_examples)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ensure_px4_params.py
  scripts/nmpc_tracking_regression_monitor.py
  scripts/offboard_velocity_follow.py
  scripts/uav_auto_takeoff_track.py
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
