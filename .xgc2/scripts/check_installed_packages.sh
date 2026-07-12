#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-manager >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-examples >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-visualization >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-worlds >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s ros-noetic-xgc2-multirotor-controller >/dev/null
dpkg -s ros-noetic-xgc2-ugv-controller >/dev/null
dpkg -s ros-noetic-xgc2-estimator-hover-thrust >/dev/null
dpkg -s ros-noetic-xgc2-estimator-rigid-state >/dev/null
dpkg -s ros-noetic-xgc2-estimator-rigid-state-msgs >/dev/null
dpkg -s ros-noetic-xgc2-px4-multirotor-controller-msgs >/dev/null
dpkg -s ros-noetic-std-msgs >/dev/null
test "$(rospack find gazebo_session_manager)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_session_manager"
test "$(rospack find gazebo_sim_examples)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_examples"
test "$(rospack find gazebo_sim_visualization)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization"
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"
test "$(rospack find px4_multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/px4_multirotor_controller"
test "$(rospack find multirotor_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller"
test "$(rospack find unicycle_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory"
test "$(rospack find hover_thrust_estimator)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator"
test "$(rospack find estimator_vrpn_px4_rotor_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/empty.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/square_box/square_box.world"
test -f /usr/include/xgc2_math/filter/butterworth_filter.hpp
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_examples/uav_auto_takeoff_track.py"
rosrun gazebo_sim_examples uav_auto_takeoff_track.py --help >/tmp/xgc2-uav-auto-takeoff-track-help.txt

roslaunch --files gazebo_session_manager session_manager.launch world_name:=/tmp/xgc2-empty.world >/tmp/xgc2-gazebo-session-manager-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true port:=3883 publish_rate:=120.0 >/tmp/xgc2-vrpn-server-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] >/tmp/xgc2-vrpn-client-files.txt
roslaunch --files gazebo_sim_visualization gazebo_auto_visualization_rviz.launch rviz:=false >/tmp/xgc2-gazebo-auto-visualization-files.txt
roslaunch --files gazebo_sim_examples fs150_uav1_nmpc_tracking.launch >/tmp/xgc2-fs150-uav1-nmpc-tracking-files.txt
if grep -F "vrpn_router" /tmp/xgc2-fs150-uav1-nmpc-tracking-files.txt >/dev/null; then
  echo "fs150_uav1_nmpc_tracking.launch unexpectedly uses vrpn_router" >&2
  exit 1
fi
roslaunch --files gazebo_sim_examples scout_ugv1_nmpc_tracking.launch >/tmp/xgc2-scout-ugv1-nmpc-tracking-files.txt
roslaunch --dump-params gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] \
  | grep -F "/vrpn_client_node/broadcast_tf: false" >/dev/null

echo "Installed package check passed"
