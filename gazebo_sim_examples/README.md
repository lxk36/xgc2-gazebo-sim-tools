# Gazebo Simulation Examples

This package contains example launch orchestration for XGC2 Gazebo Classic
simulation stacks.

## FS150 UGV Simulation Quickstart

This package starts one FS150 PX4 1.12 SITL vehicle, one Scout UGV, Gazebo-backed
VRPN tracking, and optional offboard follow tests.  It is an orchestration
package: FS150 airframe and estimator startup parameters are owned by
`gazebo_sim_fs150_sitl`.

### Launch

```bash
source /opt/ros/noetic/setup.bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn.launch
```

The FS150 vehicle id exposed to users is the MAVLink system id.  The launch file
maps it to the PX4 instance internally:

```text
PX4 instance = fs150_id - 1
MAV_SYS_ID   = PX4 instance + 1
```

Default `fs150_id:=4` therefore starts PX4 instance `3` and exposes MAVLink
system id `4`.  If no id is passed, the ROS namespace and model name remain
`uav1`.

### FS150 Startup Parameters

The launch file explicitly passes
`$(find gazebo_sim_fs150_sitl)/config/generated/fs150-sitl.params` via
`fs150_param_file` and keeps `fs150_reset_params=true`, so every restart uses
the FS150 simulator's base parameters instead of stale PX4 work-directory state.
Estimator fusion policy, including motion-capture height selection and disabled
range/terrain aiding, is documented in the `gazebo_sim_fs150_sitl` README.

After launch, verify the active PX4 parameters with:

```bash
rosrun mavros mavparam -n /uav1/mavros get EKF2_AID_MASK
rosrun mavros mavparam -n /uav1/mavros get EKF2_HGT_MODE
rosrun mavros mavparam -n /uav1/mavros get EKF2_RNG_AID
rosrun mavros mavparam -n /uav1/mavros get EKF2_TERR_MASK
rosrun mavros mavparam -n /uav1/mavros get MPC_USE_HTE
```

The expected values are `24`, `3`, `0`, `0`, and `1`.

### UAV Offboard Follow Quickstart

```bash
roslaunch gazebo_sim_examples fs150_uav1_offboard_follow.launch
```

This launch starts the UAV-side runtime pieces used by the follow test:

- `vrpn_client_ros` subscribes to the `uav1` and `ugv1` trackers from the VRPN
  server.
- `vrpn_router` forwards only the `uav1` tracker into
  `/uav1/mavros/vision_pose/pose`; it does not publish TF by default.
- `fs150_ensure_px4_params` confirms `MPC_USE_HTE=1` through MAVROS so PX4 hover
  thrust estimation is enabled explicitly.
- `offboard_velocity_follow.py` arms in OFFBOARD, holds the takeoff point for 15
  seconds, then ramps into UGV VRPN pose/twist tracking.

If the VRPN client/router is already running, pass `start_vrpn_router:=false`.
If PX4 parameters are being managed manually, pass `ensure_px4_params:=false`.
