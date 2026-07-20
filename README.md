# xgc2-gazebo-sim-tools

Legacy Gazebo Classic example launch orchestration for XGC2 ROS Noetic.

This repository temporarily retains `gazebo_sim_examples` while equivalent
workflows migrate to the XGC2 ground station.

It publishes:

- `ros-noetic-xgc2-gazebo-sim-examples`

`gazebo_sim_visualization` and `gazebo_sim_vrpn_bridge` are split child
repositories and are assembled with the full suite in `xgc2-gazebo-sim`.

The full verified Gazebo simulation suite is published by `xgc2-gazebo-sim` through:

- `ros-noetic-xgc2-gazebo-sim-all`
- `ros-noetic-xgc2-gazebo-sim-all-latest`

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-gazebo-sim-examples ros-noetic-xgc2-gazebo-sim-visualization ros-noetic-xgc2-gazebo-sim-vrpn-bridge ros-noetic-xgc2-gazebo-sim-worlds
```

## Smoke Test

```bash
roslaunch --files gazebo_sim_examples fs150_ugv_vrpn.launch
roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1]
```
