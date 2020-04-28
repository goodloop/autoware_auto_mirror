#!/usr/bin/env python3

# Copyright 2020 StreetScooter GmbH, Aachen, Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from autoware_auto_msgs.msg import Trajectory
#from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand

import motion_model_testing_simulator.example_simulation as example

class ControllerTestingNode(Node):
    def __init__(self):
        super().__init__('controller_testing_node')
        self.declare_parameter('state_frame')
        self.declare_parameter('trajectory_frame')

        # Publisher
        self._publisher_state = self.create_publisher(
            VehicleKinematicState, "vehicle_state", 0)
        self._publisher_trajectory = self.create_publisher(
            Trajectory, "planned_trajectory", 0)

        # Subscriber
        self._subscriber_controls = self.create_subscription(
            VehicleControlCommand, "control_command",
            self.control_callback, 0)

        # Simulator
        #self._simulator = SimFactory('ackermann')

        # Parameters
        self.param_state_frame = self.get_parameter('state_frame')._value
        self.param_trajectory_frame = \
            self.get_parameter('trajectory_frame')._value
        self._current_state = None

        # Trigger testing
        #self._start_test()
        example.main()

    def control_callback(self, msg):
        # Log incoming controls for analysis / visualization

        # Check if we want to continue with the simulation
        # TODO: What should the break conditions be?
        # Real time passed, sim time passed, number control iterations?

        # Convert current state and control command for sim

        # Trigger one simulation step and update current state
        #self._current_state =

        # Check if we have to send another trajectory or just the current state?
        # Ignore the next callback if trajectory triggered?
        # Or keep it but somehow flagged?

        # Send mpc trigger again
        pass

    def _start_test(self):
        # Initial trajectory
        # TODO: Create trajectory or use spoofer module?
        init_trajectory_msg = Trajectory()
        # TODO: Use realtime clock? Does mpc work with a simulated clock?
        init_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        #init_trajectory_msg.header.frame_id = self.param_trajectory_frame_id

        # Initial system state
        init_state_msg = VehicleKinematicState()
        # TODO: Use realtime clock? Does mpc work with a simulated clock?
        init_state_msg.header.stamp = self.get_clock().now().to_msg()
        #init_state_msg.header.frame_id = self.param_state_frame_id
        self._current_state = init_state_msg

        # Send first data to mpc
        # Both publishes trigger a control calculation but for the first,
        # there is missing information, so nothing happens
        self._publisher_trajectory.publish(init_trajectory_msg)
        self._publisher_state.publish(self._current_state)
