# Emergency Handler

- Features:
  - checks multiple sources (autoware state, velocity commands, driving capabilities,
    velocities) and generates emergency state/emergency commands with the diagnostic info
  - also it contains the heartbeat checker (watchdog)
  - the emergency state can be cleared by the specific service
- Service Servers:
  - /system/clear_emergency: `std_srvs/srv/Trigger`