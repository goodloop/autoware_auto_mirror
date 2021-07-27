# Autoware State Monitor

// TODO: move to design

- Features:
  - provides *State Machine* which generates the system state (`/vehicle/state`)
    based on the components states
  - possible states are: 
    - *InitializingVehicle*
    - *WaitingForRoute*
    - *Planning*
    - *WaitingForEngage*
    - *Driving*
    - *ArrivedGoal*
    - *Finalizing*
  - states are based on for example:
    - checking if the vehicle is near to the goal (it compares goal
      from `route` with a current pose from `tf`)
    - checking if the vehicle is stopped (based on twist/odometry)
  - Service Servers:
    - /autoware/shutdown: `std_srvs/srv/Trigger`