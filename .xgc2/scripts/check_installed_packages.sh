#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-manager >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-examples >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-worlds >/dev/null
dpkg -s ros-noetic-xgc2-controller >/dev/null
dpkg -s ros-noetic-xgc2-estimator-hover-thrust >/dev/null
dpkg -s ros-noetic-xgc2-estimator-rigid-state >/dev/null
test "$(rospack find gazebo_session_manager)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_session_manager"
test "$(rospack find gazebo_sim_examples)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_examples"
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"
test "$(rospack find multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_controller"
test "$(rospack find hover_thrust_estimator)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator"
test "$(rospack find estimator_rigid_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/empty.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/weston_robot_empty/weston_robot_empty.world"

roslaunch --files gazebo_session_manager session_manager.launch world_name:=/tmp/xgc2-empty.world >/tmp/xgc2-gazebo-session-manager-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true port:=3883 publish_rate:=120.0 >/tmp/xgc2-vrpn-server-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] >/tmp/xgc2-vrpn-client-files.txt
roslaunch --files gazebo_sim_examples fs150_uav1_nmpc_tracking.launch >/tmp/xgc2-fs150-uav1-nmpc-tracking-files.txt
if grep -F "vrpn_router" /tmp/xgc2-fs150-uav1-nmpc-tracking-files.txt >/dev/null; then
  echo "fs150_uav1_nmpc_tracking.launch unexpectedly uses vrpn_router" >&2
  exit 1
fi
roslaunch --dump-params gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] \
  | grep -F "/vrpn_client_node/broadcast_tf: false" >/dev/null

echo "Installed package check passed"
