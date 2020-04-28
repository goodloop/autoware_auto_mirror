Architecture Overview
======================

# Introduction

Autoware.Auto started as a rewrite of Autoware.AI for ROS2, and architecture design is also renewed.
One of the lessons learned from Autoware.Auto is that it became difficult to improve Autoware.AI features because of:
- No concrete architecture designed
- A lot of technical debt
	- Tight coupling between modules
	- Unclear responsibility of modules

The purpose of the new design is to:
- Defining concrete and simple interface between modules
- Define a layered modules with less interdependency
- Clarify the role of each module

By defining simplified interface between modules:
- Internal processing in Autoware becomes more transparent
- Joint developement of devlopers becomes easier due to less interdependency between modules
- User's can easilty replace a module with their own software component(e.g. localization) just by "wrapping" their software to adjust to Autoware inteface

Note that the initial focus of this architecture design was solely on function of driving capability, and the following features are left as future work:
* Real-time processing
* HMI
* Fail safe
* Redundant system
* State monitoring system

# Use Case
When we designed the architecture, we have set the use case of Autoware to be last-one-mile travel. 

An example would be the following:

**Description:** Travelling from to grocery store in the same city  
**Actors:** User, Vehicle with Autoware installed (Autoware)  
**Assumption:**  
The environment is assumed to be 
- urban or suburban area that is less than 1 km^2.
- fine weather
- Accurate HD map for the environment is available

**Basic Flow:**  
1. **User:** starts a browser and access Autoware page from phone. Press "Summon", and the app sends user’s GPS location to Autoware
2. **Autoware:** plans the route to the user’s location, and show it on the user’s phone
3. **User:** confirms the route and press “Engage”
4. **Autoware:** starts driving autonomously to the requested location and pulls over to the side of the road
5. **User:** rides on to the vehicle and press "Go Home"
6. **Autoware:** Plans the route to the user’s location
7. **User:** confirms the route and press “Engage”
8. **Autoware:** Drives autonomously to user's home

# Requirements
To achieve the above use case, we set the functional requirement of the Autoware as following:
- Autoware can plan the route to the specified goal in the specified environment.
- Autoware can drive along the planned route without violation of traffic rules.
- (Nice to have) Autoware drives smooth driving for a comfortable ride with a limited jerk and acceleration.

The above requirements are broken down into detailed requirements, which are explained in each stack page.

Since Autoware is open source and is meant to be used/developed by anyone around the world, we also set some non-functional requirements for the architecture:
- Architecture is extensible for new algorithms without changing the interface
- Architecture is extensible to adapt to new traffic rules for different countries
- The role and interface of a module must be clearly defined

# High-level Architecture Design
Here is the high-level overview of Autoware architecture.

![Overview](/docs/images/architecture-overview.svg)

This architecture consists of the following stacks:
- [Sensing](sensing/Sensing.md) provides sensor data as ROS messages.
- [Map](map/Map.md) provides geometric and semantic information about an environment.
- [Localization](localization/Localization.md) provides vehicle's position.
- [Perception](perception/Perception.md) extracts meaningful information from sensor data.
- [Planning](planning/Planning.md) plans trajectory that vehicle must follow.
- [Control](control/Control.md) calculates control command for the vehicle to follow given trajectory.
- [Vehicle](vehicle/Vehicle.md) acts as interface for vehicle hardware.

Each stack is decomposed into smaller components. The details are explained in each page. 
