#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-examples >/dev/null
dpkg -s ros-noetic-xgc2-gazebo-scene >/dev/null

test "$(rospack find gazebo_sim_examples)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_examples"
test "$(rospack find xgc2_gazebo_scene)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_examples/launch/fs150_ugv_vrpn.launch"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_examples/uav_auto_takeoff_track.py"
test ! -e "/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-tools.json"

test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_motion.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_gazebo_scene/ObstacleDefinition.h"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/srv/ConfigureMotions.srv"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py"

if ldd "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so" | grep -q 'not found'; then
  echo "Installed Gazebo Scene SystemPlugin has unresolved shared libraries" >&2
  exit 1
fi
ldd "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so" \
  | grep -E "libxgc2_gazebo_scene_motion\\.so => /opt/ros/${ROS_DISTRO}/lib/" >/dev/null
if readelf -d \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_motion.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so" \
    | grep -Eq '(RPATH|RUNPATH)'; then
  echo "Installed Gazebo Scene libraries contain RPATH/RUNPATH" >&2
  exit 1
fi
nm -D --defined-only "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so" \
  | grep -E '[[:space:]]RegisterPlugin$' >/dev/null
python3 -c 'from xgc2_gazebo_scene.msg import ObstacleDefinition, ObstacleStateArray'
python3 -c 'from xgc2_gazebo_scene.srv import ConfigureMotions, StopMotions'

echo "Installed package check passed"
