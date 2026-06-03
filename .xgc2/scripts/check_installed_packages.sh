#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-manager >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
test "$(rospack find gazebo_session_manager)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_session_manager"
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"

roslaunch --files gazebo_session_manager session_manager.launch world_name:=/tmp/xgc2-empty.world >/tmp/xgc2-gazebo-session-manager-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true port:=3883 publish_rate:=120.0 >/tmp/xgc2-vrpn-server-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] >/tmp/xgc2-vrpn-client-files.txt

echo "Installed package check passed"
