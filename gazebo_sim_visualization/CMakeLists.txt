cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  roslaunch
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs tf2_ros visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/fs150_vrpn_tracking_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/fs150_vrpn_tracking_rviz.launch)
endif()
