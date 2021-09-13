#!/bin/bash

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`

if [ -z $ROS_DISTRO ]; then
    echo "ROS_DISTRO environment variable not set. Source ROS2 distro or set it manually."
    exit 1
fi

vcs import $SCRIPTPATH < $SCRIPTPATH/scenario.simulator.v2.repos
vcs import $SCRIPTPATH/src/external/scenario_simulator/external/ < $SCRIPTPATH/src/external/scenario_simulator/dependency_$ROS_DISTRO.repos
