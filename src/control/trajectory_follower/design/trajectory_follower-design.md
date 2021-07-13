Trajectory Follower {#trajectory_follower-package-design}
===========

This is the design document for the `trajectory_follower` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
This package provides the library code used by the nodes of the `trajectory_follower_nodes` package.
Mainly, it implements two algorithms:
- Model-Predictive Control (MPC) for the computation of lateral steering commands.
- PID control for the computation of velocity and acceleration commands.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->


## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

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
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1057
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1058
