Emergency Handler {#emergency_handler}
===========

This is the design document for the `emergency_handler` package.

# Purpose / Use cases

The purpose of the `emergency_handler` is to detect the emergency state based on the input
information and to generate emergency control and state commands.

# Design

The components checks multiple sources and generates emergency
velocities/state commands with the diagnostic info.
The emergency commands are sent as long as the vehicle is stopped
or the emergency state is not present.
Also it contains the heartbeat checker (watchdog) that generates emergency state
if the `DrivingCapability` message is not received with the specified frequency.
The emergency state can be also cleared by the specific service.

## Inputs / Outputs / API

Inputs
* `autoware_auto_msgs/msg/DrivingCapability` is used to determine a vehicle capabilities
* `autoware_auto_msgs/msg/AutowareState` is used to determine the Autoware system state.
* `autoware_auto_msgs/msg/VehicleControlCommand` is used to determine current wheels angle.
* `autoware_auto_msgs/msg/VehicleStateReport` is used to determine if the vehicle
  is in manual or autonomous mode.
* `autoware_auto_msgs/msg/VehicleOdometry` is used to calculate a current vehicle velocity.

Outputs

* `autoware_auto_msgs/msg/VehicleControlCommand` contains emergency control commands.
* `autoware_auto_msgs/msg/VehicleStateCommand` contains the vehicle state commands.
* `autoware_auto_msgs/msg/EmergencyMode` is used to propagate information about emergency mode.
* `autoware_auto_msgs/msg/HazardStatusStamped` contains details about the hazard state.
* `autoware_auto_msgs/msg/DiagnosticArray` contains diagnostic information about the current state.

Services

* `std_srvs/srv/Trigger` (/system/clear_emergency) is used to request clear of
  the emergency state.

# Related issues

- #1245 Autoware monitoring system: Implement Emergency Handler
- #821 - Detect when nodes' incoming messages are skipped