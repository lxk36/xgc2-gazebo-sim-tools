# xgc2-gazebo-sim-tools

Gazebo Classic tools for XGC2 ROS Noetic simulation.

This repository contains:

- `gazebo_session_manager`
- `gazebo_sim_examples`
- `gazebo_sim_vrpn_bridge`

It publishes:

- `ros-noetic-xgc2-gazebo-sim-manager`
- `ros-noetic-xgc2-gazebo-sim-examples`
- `ros-noetic-xgc2-gazebo-sim-vrpn-bridge`

The full verified Gazebo simulation suite is published by `xgc2-gazebo-sim` through:

- `ros-noetic-xgc2-gazebo-sim-all`
- `ros-noetic-xgc2-gazebo-sim-all-latest`

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-gazebo-sim-manager ros-noetic-xgc2-gazebo-sim-examples ros-noetic-xgc2-gazebo-sim-vrpn-bridge ros-noetic-xgc2-gazebo-sim-worlds
```

## Smoke Test

```bash
roslaunch --files gazebo_session_manager session_manager.launch world_name:=/tmp/xgc2-empty.world
roslaunch --files gazebo_sim_examples fs150_ugv_vrpn.launch
roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1]
```
