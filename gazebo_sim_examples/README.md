# Gazebo Simulation Examples

This package contains example launch orchestration for XGC2 Gazebo Classic
simulation stacks.

## FS150 UGV Simulation Quickstart

This package starts one FS150 PX4 1.12 SITL vehicle, one Scout UGV, Gazebo-backed
VRPN tracking, and optional offboard follow tests.  It is an orchestration
package: FS150 airframe and estimator startup parameters are owned by
`gazebo_sim_fs150_sitl`.

### Launch

Start only the Gazebo simulation and the Gazebo-backed VRPN tracker server:

```bash
source /opt/ros/noetic/setup.bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn.launch
```

Start the `uav1` and `ugv1` VRPN client/router separately:

```bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn_router.launch
```

Or start the simulation and VRPN router together:

```bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn_stack.launch
```

The offboard follow algorithm remains a separate launch:

```bash
roslaunch gazebo_sim_examples fs150_uav1_offboard_follow.launch
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

### UAV Offboard Follow Algorithm

```bash
roslaunch gazebo_sim_examples fs150_uav1_offboard_follow.launch
```

This launch starts the follow algorithm and optional PX4 parameter guard:

- `fs150_ensure_px4_params` confirms `MPC_USE_HTE=1` through MAVROS so PX4 hover
  thrust estimation is enabled explicitly.
- `offboard_velocity_follow.py` arms in OFFBOARD, holds the takeoff point for 15
  seconds, then ramps into UGV VRPN pose/twist tracking.

The algorithm expects the router launch to already provide
`/vrpn_client_node/uav1/pose`, `/vrpn_client_node/ugv1/pose`, and
`/vrpn_client_node/ugv1/twist`. For legacy one-shot tests it can still include
the router with `start_vrpn_router:=true`.
If PX4 parameters are being managed manually, pass `ensure_px4_params:=false`.
