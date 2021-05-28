simple_planning_simulator {#simple_planning_simulator-package-design}
===========


# Purpose / Use cases

This node simulates the vehicle motion for a vehicle command in 2D using a simple vehicle model.



# Design


The purpose of this simulator is for the integration test of planning and control modules. This does not simulate sensing or perception, but is implemented in pure c++ only and works without GPU.

## Assumptions / Known limits

 - It simulates only in 2D motion.
 - It does not perform physical operations such as collision and sensing, but only calculates the integral results of vehicle dynamics.


## Inputs / Outputs / API


**input**
 - /vehicle/vehicle_command [`autoware_auto_msgs/msg/VehicleControlCommand`] : target command to drive a vehicle.
 - /vehicle/state_command [`autoware_auto_msgs/msg/VehicleStateCommand`] : target state command (e.g. gear).
 - /localization/initialpose [`geometry_msgs/msg/PoseWithCovarianceStamped`] : for initial pose

**output**
 - /tf [`tf2_msgs/msg/TFMessage`] : simulated vehicle pose (base_link)
 - /vehicle/vehicle_kinematic_state [`autoware_auto_msgs/msg/VehicleKinematicState`] : simulated kinematic state (defined in CoM)
 - /vehicle/state_report [`autoware_auto_msgs/msg/VehicleStateReport`] : current vehicle state (e.g. gear, mode, etc.)


<!-- ## Inner-workings / Algorithms -->


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
This is originally developed in the Autoware.AI. See the link below.

https://github.com/Autoware-AI/simulation/tree/master/wf_simulator

# Future extensions / Unimplemented parts

 - Improving the accuracy of vehicle models (e.g., adding steering dead zones and slip behavior)
 - Cooperation with modules that output pseudo pointcloud or pseudo perception results


# Related issues

 - #1142: Follow-up to #570 - Integrate simple_planning_simulator Into CI
