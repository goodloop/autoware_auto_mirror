Record/Replay Planner {#recordreplay-planner-howto}
=====================

[TOC]

# Record/Replay a Trajectory in SVL

Autoware.Auto is capable of recording a path of waypoints to disk and then loading and attempting to follow that path.
This demonstration combines several subsystems including @ref ndt-initialization "NDT Localization" and the @ref perception-stack-howto.
To test this functionality, do the following:

## Prerequisites

The following instructions assume you are running the demo inside of ADE.
For instructions on setting up an ADE environment, see @ref installation-ade.
For instructions on setting up Autoware.Auto without ADE, see @ref installation-no-ade.
If running outside of ADE, replace `source /opt/AutowareAuto/setup.bash` with `source /<path_to_your_autoware_folder>/install/setup.bash`.

## Instructions

1. [Launch the SVL simulator](@ref lgsvl) with the Lexus RX 450h and the AutonomouStuff Parking Lot map. Do not start the simulation at this point.

2. In a new terminal:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos recordreplay_planner_demo.launch.py
```

3. In `rviz`, locate the spawn point of the vehicle in the AutonomouStuff parking lot map using the point cloud as a reference.
@image html images/recordreplay_spawn_point.png "SVL Spawn Point" width=800px
Use the "2D Pose Estimate" tool in `rviz` to provide an initial pose estimate for localization.
@image html images/recordreplay_pose_estimate.jpeg "Initial Pose Estimate" width=800px

4. Using the SVL web interface, start the simulation.
Once localization has begun estimating the vehicle's position, the view in `rviz` will jump away from the vehicle. Re-center the view on the vehicle.

5. In a new terminal:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

6. In SVL, drive the vehicle around to record the path.

7. When finished recording, go to the terminal in which you ran the `ros2 action send_goal` command and hit `CTRL+C` to stop recording.

8. In SVL, hit `F12` to re-center the vehicle at the default spawn point.

9. Stop the Autoware Stack by pressing `CTRL+C` in terminal where `ros2 launch autoware_demos recordreplay_planner_demo.launch.py` is running.

10. Repeat steps 2. and 3.

11. To replay the recorded path:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

12. When the vehicle reaches to the goal of the replayed trajectory, the planner stops automatically and outputs `status::SUCCEED` to the terminal in which you ran the `ros2 action send_goal` command.

You can modify end conditions by tuning parameters in [recordreplay_planner.param.yaml](src/launch/autoware_demos/param/recordreplay_planner.param.yaml). The planner terminates planning when both of the following conditions are satisfied:
* `goal_distance_threshold_m`: threshold for the distance between `nav_base` frame and the last point in the replayed trajectory
* `goal_angle_threshold_rad`: threshold for the heading angle between `nav_base` frame and the last point in the replayed trajectory

### Controller option

By default the above launch command will run the planner with the MPC controller. You can run the pure pursuit controller instead of the MPC controller by running the following command,
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos recordreplay_planner_demo.launch.py run_pure_pursuit:=True
```

Note that the pure pursuit controller is more primitive and does not stop for objects on the path that could cause collision

### Looping

To enable trajectory looping, modify [recordreplay_planner.param.yaml](src/launch/autoware_demos/param/recordreplay_planner.param.yaml):

* Set `loop_trajectory` to `True`
* Set `loop_max_gap_m` to be the maximum gap (meter) allowed between the start and end point to complete the loop, such as `10`.

Note that it is better to stop the recording command while the vehicle is still driving. Otherwise the vehicle may come to a stop at the end of the trajectory and not move across to the start point. It is even better to edit the saved path file and optimize the speed and pose at the joint.
