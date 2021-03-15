# Autoware.Auto

[Autoware](https://www.autoware.org/) is the world's first "all-in-one" open-source software for self-driving vehicles hosted under the Autoware Foundation.

The [Autoware.Auto project](https://www.autoware.auto/), based on [ROS 2](https://docs.ros.org/en/foxy/), is the next generation successor of the [Autoware.AI project](https://www.autoware.ai/), based on [ROS 1](http://wiki.ros.org/Documentation).

## Installation
Before installing Autoware.Auto, make sure your machine satisfies the [system dependencies and target environments](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/target-environments.html).
The recommended method for installing Autoware.Auto is through the use of ADE (a Docker based tool) which comes with a [pre-built version](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation.html) to ensure that all developers in the project have a common and consistent development environment.
There is also the option to [build Autoware.Auto from source](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/building.html) directly which will require you to compile the source code.
If you wish to run a vehicle in simulation, install the LGSVL simulation using the instructions provided [here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html).

## Getting Started
Once Autoware.Auto is installed on your machine, begin by sourceing your installation to allow the software to find all the neccessary dependencies. If you installed via ADE then run `source /opt/AutowareAuto/setup.bash`, otherwise run `source ~/AutowareAuto/install/setup.bash` to source your installation. 

Many launch configurations rely on a point cloud map, which is managed via git lfs. To download it, run 
`git lfs pull --exclude="" --include="*"`

A number of [Operational Design Domain (ODD) tutorials](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/usage.html) are available to help you to get started quickly. These demos work on real cars (subject to having the necessary hardware requirments) or within the LGSVL simulation environment. Let's begin by running the Autoware.Auto 3D perception stack to visualise the Velodyne pointcloud, classify groundplane poinds and cluster non-ground points for object detection as shown in the image below.


## Contributing 
Autoware.Auto welcomes code and documentation contributions from the community. To get started with making contributions, first read the [contributor's guide](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributors-guide.html).

## Getting Help
Please post any Autoware.Auto related questions on [ROS Answers](https://answers.ros.org/questions/), using the `autoware` tag. If you want to discuss a topic with the general Autoware community or ask a question not related to a problem with Autoware then use [the Autoware category on ROS Discourse](https://discourse.ros.org/c/autoware/46). If you need any further help, follow the appropriate support channel outlined in the [support guidelines](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/support-guidelines.html).

## Official Documentation
Please see [the documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/) for all information, including how to build, run, and contribute to Autoware.Auto.
