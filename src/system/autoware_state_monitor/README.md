# Autoware State Monitor

- Features:
  - provides *State Machine* which generates the system state (`/vehicle/state`)
    based on the components states
  - possible states are: 
    - *InitializingVehicle*
    - *WaitingForRoute*
    - *Planning*
    - *WaitingForEngage*
    - *Driving*
    - *Emergency*
    - *ArrivedGoal*
    - *Finalizing*
  - monitors the states (timeout, frequency) of specified topics
    and transformations (for example `base_link`-`map`)
  - published topics/transformations stated to the `/diagnostics` topics
  - states are based on for example:
    - checking if the vehicle is near to the goal (it compares goal
      from `route` with a current pose from `tf`)
    - checking if the vehicle is stopped (based on twist/odometry)
  - Service Servers:
    - /autoware/reset_route: `std_srvs/srv/Trigger`
    - /autoware/shutdown: `std_srvs/srv/Trigger`