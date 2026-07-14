# Gazebo Simulation Examples

This package contains example launch orchestration for XGC2 Gazebo Classic
simulation stacks.

## FS150 UGV Simulation Quickstart

This package starts one FS150 PX4 1.12 SITL vehicle, one Scout UGV, Gazebo-backed
VRPN tracking, and optional offboard follow tests.  It is an orchestration
package: FS150 airframe and estimator startup parameters are owned by
`gazebo_sim_fs150_sitl`, Gazebo-backed VRPN server parameters are owned by
`gazebo_sim_vrpn_bridge`, shared world assets are owned by
`gazebo_sim_worlds`, and VRPN state estimation is owned by
`estimator_rigid_state`.

### Launch

Start only the Gazebo simulation and the Gazebo-backed VRPN tracker server:

```bash
source /opt/ros/noetic/setup.bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn.launch
```

Start the simulation, VRPN client, and rigid-state estimators together:

```bash
roslaunch gazebo_sim_examples fs150_ugv_vrpn_stack.launch
```

The FS150/Scout stack sets `ugv_wheel_contact_mu2:=0.20`,
`ugv_wheel_contact_slip2:=0.5`, and `ugv_angular_command_gain:=3.15` by
default. This is a Scout Gazebo skid-steer response calibration measured in the
FS150 example world at `v=0.5 m/s`, `cmd_vel.angular.z=0.25 rad/s`. The contact
settings reduce lateral stick-slip during small-radius turns, and the yaw gain
scales only the differential wheel term inside the UGV base controller. The
physical wheel separation and linear velocity mapping remain unchanged.
Residual deviation from an ideal unicycle model is expected: the simulated UGV is
a four-wheel skid-steer chassis with wheel-ground friction, lateral slip, and
wheel velocity controllers between `cmd_vel` and the realized body velocity.

The offboard follow algorithm remains a separate launch:

```bash
roslaunch gazebo_sim_examples fs150_uav1_offboard_follow.launch
```

The single-machine FS150 NMPC tracking stack is:

```bash
roslaunch gazebo_sim_examples fs150_uav1_nmpc_tracking.launch
```

It starts one FS150 PX4 1.12 SITL vehicle, the Gazebo-backed VRPN server,
`vrpn_client_ros`, `estimator_vrpn_px4_rotor_state`, `hover_thrust_estimator`,
`px4_multirotor_controller`, and the UAV reference trajectory state-machine node. The rigid-state estimator subscribes to raw VRPN pose and
keeps its corrected pose on `alg/state_estimator/corrected_vision_pose` for
diagnostics. When the XGC NMPC Automation runs through an Experiment, the
Adapter derived from the pinned Swarm Asset is the sole
publisher to `mavros/vision_pose/pose`; it relays the configured rigid body's
raw pose without an XYZ offset. The default
takeoff height is 3 m. The default Custom1 request is an analytic circle-entry
reference with 3 m radius, 3 m/s horizontal speed, 3 m altitude, and a 1 m
sinusoidal height offset. After the launch is up, drive the controller through
its normal state-machine command topic.

To wait for MAVROS and the rigid-state estimator, take off to 3 m, and then
enter the `custom1` trajectory-tracking state, run:

```bash
rosrun gazebo_sim_examples uav_auto_takeoff_track.py --ns uav1 --height 3.0
```

The command exits with `0` only after the controller reports `Custom1` and two
new, increasing NMPC debug samples report successful solver status `0`. It
exits with `2` if MAVROS, the state estimate, or the controller command
subscriber is not ready before the deadline, `3` if the requested takeoff
altitude is not reached in time, and `4` if NMPC tracking is not confirmed
before `--tracking-timeout`.

The single-Scout UGV NMPC tracking stack is:

```bash
roslaunch gazebo_sim_examples scout_ugv1_nmpc_tracking.launch
```

It starts one Scout, Gazebo-backed VRPN server/client, `estimator_vrpn_ugv_state`,
`unicycle_reference_trajectory`, and `unicycle_ugv_controller`. The default
reference is a 3 m radius circle-entry trajectory at 1 m/s. After the launch is
up, start tracking with:

```bash
rostopic pub -1 /command std_msgs/String "data: 'track'"
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
- `ugv_circle_tracking_controller.py` replaces the old open-loop UGV command by
  default. It subscribes to `/vrpn_client_node/ugv1/pose`, initializes a circle
  from the UGV's initial pose, and publishes closed-loop `/ugv1/cmd_vel`. The
  default circle is 2 m radius at 0.5 m/s.

The algorithm expects the VRPN client to already provide
`/vrpn_client_node/uav1/pose`, `/vrpn_client_node/ugv1/pose`, and
`/vrpn_client_node/ugv1/twist`.
If PX4 parameters are being managed manually, pass `ensure_px4_params:=false`.
To restore the previous open-loop UGV command, pass `ugv_drive_mode:=open_loop`.
