Record/Replay Planner {#recordreplay-planner-howto}
=====================

[TOC]

# Record/Replay a Trajectory in LGSVL

Autoware.Auto is capable of recording a path of waypoints to disk and then loading and attempting to follow that path.
This demonstration combines several subsystems including @ref ndt-howto "NDT Localization" and the @ref perception-stack-howto.
To test this functionality, do the following:

@note The following instructions assume you are running the demo inside of ADE.
If running outside of ADE, first [Build Autoware](@ref building) and then replace `source /opt/AutowareAuto/setup.bash` with `source /<path_to_your_autoware_folder>/install/setup.bash`.

- [Launch the LGSVL simulator](@ref lgsvl) with the Lexus RX 450h and the AutonomouStuff Parking Lot map. Do not start the simulation at this point.

- In a new terminal:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_auto_avp_demo recordreplay_planner_demo.launch.py
```

- In `rviz`, locate the spawn point of the vehicle in the AutonomouStuff parking lot map using the point cloud as a reference.

@image html images/recordreplay_spawn_point.png "LGSVL Spawn Point" width=800px

- Use the "2D Pose Estimate" tool in `rviz` to provide an initial pose estimate for localization.

@image html images/recordreplay_pose_estimate.jpeg "Initial Pose Estimate" width=800px

- Using the LGSVL web interface, start the simulation.
- Once localization has begun estimating the vehicle's position, the view in `rviz` will jump away from the vehicle. Re-center the view on the vehicle.
- In a new terminal:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/recordtrajectory recordreplay_planner_actions/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```
- In LGSVL, drive the vehicle around to record the path.
- When finished recording, go to the terminal in which you ran the `ros2 action send_goal` command and hit `CTRL+C` to stop recording.
- In LGSVL, drive the vehicle back to the spawn point/beginning of the recorded path, facing approximately the same direction as when you started recording.
- To replay the recorded path:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/replaytrajectory replayreplay_planner_actions/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

The vehicle should automatically stop for any detected obstacles on the path.

@note In LGSVL, you can use the `n` key to enable and disable dynamic traffic.
