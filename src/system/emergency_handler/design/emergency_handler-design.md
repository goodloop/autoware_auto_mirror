Emergency Handler {#emergency_handler}
===========

This is the design document for the `emergency_handler` package.

# Purpose / Use cases

The purpose of the `emergency_handler` is to ...

# Design

- Features:
  - checks multiple sources (autoware state, velocity commands, driving capabilities,
    velocities) and generates emergency state/emergency commands with the diagnostic info
  - also it contains the heartbeat checker (watchdog)
  - the emergency state can be cleared by the specific service
- Service Servers:
  - /system/clear_emergency: `std_srvs/srv/Trigger`

## Inputs / Outputs / API

Inputs
* `autoware_auto_msgs/msg/VehicleOdometry` is used to calculate a current vehicle velocity.
* `autoware_auto_msgs/msg/VehicleStateReport` is used to determine if the vehicle
  is in manual or autonomous mode.

Outputs

Services

  # Related issues