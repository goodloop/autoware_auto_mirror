Starting and testing the behavior planner {#behavior-planner-howto}
=======================================

@tableofcontents

# How to start the stack

Start simulation as described in @ref lgsvl.
Additionally, to configure LGSVL for this demonstration:

1. Maps: use this [map link](https://assets.dev.lgsvlsimulator.com/d5b8bb0b7f49875a8a4bbf83c50b3a4fe53779c7/environment_AutonomouStuff)
2. Vehicles: Select `ROS2 native` bridge type and paste content of `AutowareAuto/src/tools/autoware_demos/config/svl/avp-sensors.json` into the `Sensors` text box
3. Simulations: In `General` tab, `Select Cluster = Local Machine` and untick any boxes.
In `Map & Vehicles` tab, ensure to untick `Run simulation in interactive mode`.
In `Traffic` tab, untick all selection.
The `Weather` tab is irrelevant

*terminal 1*

```{bash}
# start sim according to instructions above but don't drive away yet to make sure we can localize ourselves
# then start RViz2 for visualization
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py
```

*terminal 2*
```{bash}
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ stdbuf -o L ros2 launch autoware_demos avp_sim.launch.py
```

The `stdbuf` command above is needed because the default in ROS is to only output lines from `stdout` when the buffer is full.
This command changes that setting to use a "line buffer" which outputs every line, providing more debugging information.

*terminal 3*
```{bash}
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic echo /planning/trajectory
```

Move the vehicle on LGSVL to the position shown in the image.
This is the "pick-up/drop-off zone" on the parking lot roadway in front of the three parking spots directly outside of the front door of the AutonomouStuff building.
![StartPose](images/avp-demo-start-pose.png)

Before selecting a goal, you will need to initialize localization.
To do this switch to the `rviz` window, click the `2D Pose Estimate` button at the top, and then click at the approximate location where the vehicle currently is in the map and drag in the direction of the vehicle's heading.
You can verify that the vehicle has been localized by the vehicle model jumping to the new location and the real-time lider scans matching up with the static lidar map.

Next, to select a parking spot graphically, click the `2D Nav Goal` button in `rviz`, click in the goal location, and drag in the direction of the goal heading.

Optionally, to send a goal position/heading programmatically:

*terminal 4*
```{bash}
> ade enter
ade$ ros2 topic pub /planning/goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: -96.46856384277344, y: 58.39532775878906}, orientation: {z: 0.42554035782814026, w: 0.9049394130706787}}'} --once
```

if you want to park in the backward direction, send:
```{bash}
> ade enter
ade$ ros2 topic pub /planning/goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: -98.56259155273438, y: 60.99168395996094}, orientation: {z: -0.42844402469653825, w: 0.9035683248663778}}'} --once
```

## Verify that Behavior Planner receives routes from Global Path
On Terminal 2, you should see following message output:

```{bash}
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: Received route
```

## Verify that Lane Planner and Parking Planner are called by Behavior Planner
On Terminal 2, you should see the following message output:

```{bash}
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: sent parking trajectory action goal
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: Received trajectory from planner
```

## Verify that Behavior Planner outputs a Trajectory for MPC to follow
On Terminal 3, you should see a trajectory message coming out from the behavior planner.

See the example below

```{bash}
ade$ ros2 topic echo /planning/trajectory
1617078146.280549 [0]       ros2: using network interface enp3s0 (udp/192.168.0.113) selected arbitrarily from: enp3s0, docker0
---
header:
  stamp:
    sec: 1617077481
    nanosec: 563900928
  frame_id: map
points:
- time_from_start:
    sec: 0
    nanosec: 0
  x: -60.46516418457031
  y: 82.08631896972656
  heading:
    real: -0.338379830121994
    imag: 0.9410096406936646
  longitudinal_velocity_mps: 2.777777910232544
  lateral_velocity_mps: 0.0
  acceleration_mps2: 0.0
  heading_rate_rps: 0.0019613588228821754
  front_wheel_angle_rad: -0.009999978356063366
  rear_wheel_angle_rad: 0.0
- time_from_start:
    sec: 0
    nanosec: 228084512
  x: -60.946189880371094
  y: 81.67398071289062
  heading:
    real: 0.3368988037109375
    imag: -0.9415408372879028
  longitudinal_velocity_mps: 2.777777910232544
  lateral_velocity_mps: 0.0
  acceleration_mps2: 0.0
  heading_rate_rps: 0.0
  front_wheel_angle_rad: 9.720058005768806e-05
  rear_wheel_angle_rad: 0.0
```
