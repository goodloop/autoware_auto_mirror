autoware_auto_launch {#autoware-auto-launch-package-design}
===========

This is the design document for the `autoware_auto_launch` package.


# Purpose / Use cases
<!-- Required -->
Parts of the AutowareAuto stack could be split into different pipelines allowing for modularity and component reuse between different ODDs and use-cases. 


# Design
<!-- Required -->
The launch files will be split into four different files:
 * localization
 * mapping
 * perception
 * planning
 * sensors
 * simulator
 * vehicle
 * visualization


## Assumptions / Known limits
<!-- Required -->


## Inputs / Outputs / API
<!-- Required -->


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
