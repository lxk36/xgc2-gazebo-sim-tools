cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  roslaunch
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_auto_visualizer_node
  src/gazebo_auto_visualizer_node.cpp
)
target_link_libraries(gazebo_auto_visualizer_node
  ${catkin_LIBRARIES}
)

install(TARGETS gazebo_auto_visualizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/gazebo_auto_visualization_rviz.launch)
endif()
