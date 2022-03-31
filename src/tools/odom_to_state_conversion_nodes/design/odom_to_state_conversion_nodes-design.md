odom_to_state_conversion_nodes {#odom_to_state_conversion_nodes-package-design}
===========

This is the design document for the `odom_to_state_conversion_nodes` package.


# Purpose / Use cases

This node converts `autoware_auto_vehicle_msgs/msg/VehicleOdometry` and `nav_msgs/msg/Odometry` to
`autoware_auto_vehicle_msgs/msg/VehicleKinematicState`, which is widely used in Autoware planning and control packages.

Previously, the job of this node was combined in `lgsvl_interface` and `ssc_interface`, which may not be friendly for other platforms such as the F1TENTH.

# Design

This node combines `VehicleOdometry` and `Odometry` to
make `VehicleKinematicState`.

Only when both `VehicleOdometry` and `Odometry` are received will the node start outputting `VehicleKinematicState`.

For populating velocity, the user has the freedom to choose between both sources.


## Assumptions / Known limits

`VehicleKinematicState` is currently only published on receiving `nav_msgs/msg/Odometry`, but not `autoware_auto_vehicle_msgs/msg/VehicleOdometry`. No check is in place regarding how the two messages are in sync.

`lgsvl_interface` and `ssc_interface` has more rigorous checks on the condition of outputing `VehicleKinematicState`.

`acceleration_mps2` is not populated in the output message.

## Inputs / Outputs / API

### Subscribers

- `autoware_auto_vehicle_msgs/msg/VehicleOdometry`
- `nav_msgs/msg/Odometry`

### Publishers

- `autoware_auto_vehicle_msgs/msg/VehicleKinematicState`

### Parameters

- `state_topic`: topic of the VehicleKinematicState to publish
- `vehicle_odom_topic`: topic of the VehicleOdometry to subscribe to
- `odom_topic`: topic of the nav_msgs/msg/Odometry to subscribe to
- `use_vehicle_odom_for_velocity`: switch between velocity sources (VehicleOdometry or Odometry)

## Inner-workings / Algorithms
<!-- If applicable -->


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
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->
