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
from builtin_interfaces.msg import Duration

from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand

import math
import time

from scipy.spatial.transform import Rotation as R

import motion_model_testing_simulator.minisim as minisim
import motion_model_testing_simulator.ackermann_model as ackermannModel

class ControllerTestingNode(Node):
    def __init__(self):
        super().__init__('controller_testing_node')
        self.declare_parameter('state_frame')
        self.declare_parameter('trajectory_frame')
        self.declare_parameter('sim_time_step_s')

        # Publisher
        self._publisher_state = self.create_publisher(
            VehicleKinematicState, "vehicle_state", 0)
        self._publisher_trajectory = self.create_publisher(
            Trajectory, "planned_trajectory", 0)

        # Subscriber
        self._subscriber_controls = self.create_subscription(
            VehicleControlCommand, "control_command",
            self.control_callback, 0)

        # Timer to send initial trigger as soon as spinning
        self._timer_initial_trigger = self.create_timer(0.1, self._start_test)

        # Parameters
        self.param_state_frame = self.get_parameter('state_frame')._value
        self.param_trajectory_frame = \
            self.get_parameter('trajectory_frame')._value
        self.param_sim_time_step = self.get_parameter('sim_time_step_s')._value
        self._current_state = None

        # Init simulator
        vehicle_dynamics = ackermannModel.AckermannDynamics()
        memory_recorder = minisim.SimulationRecorderToMemory()
        self._simulator = minisim.ExternalControllerSim(
            vehicle_dynamics,
            self.param_sim_time_step,
            listeners={"recorder": memory_recorder},
        )

    def control_callback(self, current_command_msg):
        # Log incoming controls for analysis / visualization

        # Check if we want to continue with the simulation
        # TODO: What should the break conditions be?
        # Real time passed, sim time passed, number control iterations?

        # Convert to AckermannState/ AckermannCommand
        current_state = self.convertToAckermannState(self._current_state)
        current_command = self.convertToAckermannCommand(current_command_msg)

        # Trigger one simulation step and update current state
        forward_simulated_state = self._simulator.simulate_iteration(
            current_state, current_command)
        self._current_state = \
            self.convertToVehicleKinematicState(forward_simulated_state)

        # Check if we have to send another trajectory or just the current state?
        # Ignore the next callback if trajectory triggered?
        # Or keep it but somehow flagged?

        # Send mpc trigger again
        self._publisher_state.publish(self._current_state)

    def convertToVehicleKinematicState(self, state: ackermannModel.AckermannState):
        # TODO: Write conversion from model state to autoware msg
        state_msg = VehicleKinematicState()
        # TODO: Which time are we using? Sim, real?
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = self.param_state_frame
        return state_msg

    def convertToAckermannState(self, state_msg: VehicleKinematicState):
        # TODO: Write conversion from autoware msg to model state
        return ackermannModel.AckermannState(

        )

    def convertToAckermannCommand(self, command_msg: VehicleControlCommand):
        return ackermannModel.AckermannCommand(
            command_msg.long_accel_mps2,
            command_msg.front_wheel_angle_rad
        )

    def _start_test(self):
        # Only call this once - so destroy the calling timer
        self.destroy_timer(self._timer_initial_trigger)
        self._timer_initial_trigger = None

        # Initial system state
        init_state_msg = VehicleKinematicState()
        # TODO: Use realtime clock? Does mpc work with a simulated clock?
        init_state_msg.header.stamp = self.get_clock().now().to_msg()
        init_state_msg.header.frame_id = self.param_state_frame
        self._current_state = init_state_msg

        # Initial trajectory
        # TODO: Create trajectory or use spoofer module?
        init_trajectory_msg = self.createStraightLineTrajectory(self._current_state,
            20, 1.0, 0.1)
        # TODO: Use realtime clock? Does mpc work with a simulated clock?
        init_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        init_trajectory_msg.header.frame_id = self.param_trajectory_frame

        # Send first data to mpc
        # Both publishes trigger a control calculation but for the first,
        # there is missing information, so nothing happens
        self._publisher_trajectory.publish(init_trajectory_msg)
        # Make sure trajectory is send first
        time.sleep(0.1)
        self._publisher_state.publish(self._current_state)

    def secondsToRosDuration(self, seconds_in):
        secs = int(seconds_in)
        nsecs = int((seconds_in - secs) * math.pow(10,9))
        return Duration(sec=secs, nanosec=nsecs)

    def createStraightLineTrajectory(self, init_state, length, speed,
        discretization_m):
        num_points = int(length / discretization_m)
        num_points_max = 100 # max. length in Trajectory.msg
        if (num_points > num_points_max):
            num_points = num_points_max
            self.get_logger().warn('Only 100 points available - discretization set to %s' %
                float(length/num_points_max))
        discretization_distance_m = float(length / num_points)
        discretization_time_s = float(discretization_distance_m/speed)

        trajectory_msg = Trajectory()

        # start at base_link
        init_point = init_state.state
        trajectory_msg.points.append(init_point)

        # x, y, z, w
        r = R.from_quat([[0.0, 0.0,
            init_point.heading.imag,
            init_point.heading.real]])

        for i in range(1,num_points-1):
            trajectory_point = TrajectoryPoint()
            trajectory_point.time_from_start = \
                self.secondsToRosDuration(discretization_time_s * i)

            traj_point = [discretization_distance_m * i, 0.0, 0.0]
            traj_point_trans = r.apply(traj_point)
            trajectory_point.x = traj_point_trans[0][0]
            trajectory_point.y = traj_point_trans[0][1]
            trajectory_point.heading = init_point.heading
            trajectory_point.longitudinal_velocity_mps = float(speed)

            trajectory_msg.points.append(trajectory_point)

        return trajectory_msg
