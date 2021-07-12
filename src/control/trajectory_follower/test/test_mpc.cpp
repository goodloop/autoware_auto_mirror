// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include <string>
#include <vector>

#include "trajectory_follower/mpc.hpp"
#include "trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "common/types.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gtest/gtest.h"

namespace
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
typedef autoware_auto_msgs::msg::Trajectory Trajectory;
typedef autoware_auto_msgs::msg::TrajectoryPoint TrajectoryPoint;
typedef autoware_auto_msgs::msg::VehicleKinematicState VehicleKinematicState;
typedef geometry_msgs::msg::Pose Pose;
typedef autoware_auto_msgs::msg::AckermannLateralCommand AckermannLateralCommand;

class MPCTest : public ::testing::Test
{
protected:
  trajectory_follower::MPCParam param;
  // Test inputs
  Trajectory dummy_straight_trajectory;
  Trajectory dummy_right_turn_trajectory;
  VehicleKinematicState neutral_steer;
  Pose pose_zero;
  float64_t default_velocity = 1.0;
  rclcpp::Logger logger = rclcpp::get_logger("mpc_test_logger");
  // Vehicle model parameters
  float64_t wheelbase = 2.7;
  float64_t steer_limit = 1.0;
  float64_t steer_tau = 0.1;
  float64_t mass_fl = 600.0;
  float64_t mass_fr = 600.0;
  float64_t mass_rl = 600.0;
  float64_t mass_rr = 600.0;
  float64_t cf = 155494.663;
  float64_t cr = 155494.663;
  // Filters parameter
  float64_t steering_lpf_cutoff_hz = 3.0;
  float64_t error_deriv_lpf_cutoff_hz = 5.0;
  // Test Parameters
  float64_t admissible_position_error = 5.0;
  float64_t admissible_yaw_error_rad = M_PI_2;
  float64_t steer_lim = 0.610865;  // 35 degrees
  float64_t steer_rate_lim = 2.61799;  // 150 degrees
  float64_t ctrl_period = 0.03;
  float64_t traj_resample_dist = 0.1;
  int path_filter_moving_ave_num = 35;
  int curvature_smoothing_num = 35;
  bool8_t enable_path_smoothing = true;
  bool8_t use_steer_prediction = true;
  bool8_t enable_yaw_recalculation = true;

  void SetUp() override
  {
    param.prediction_horizon = 50;
    param.prediction_dt = 0.1;
    param.zero_ff_steer_deg = 0.5;
    param.input_delay = 0.0;
    param.acceleration_limit = 2.0;
    param.velocity_time_constant = 0.3;
    param.steer_tau = 0.1;
    param.weight_lat_error = 1.0;
    param.weight_heading_error = 1.0;
    param.weight_heading_error_squared_vel = 1.0;
    param.weight_terminal_lat_error = 1.0;
    param.weight_terminal_heading_error = 0.1;
    param.low_curvature_weight_lat_error = 0.1;
    param.low_curvature_weight_heading_error = 0.0;
    param.low_curvature_weight_heading_error_squared_vel = 0.3;
    param.weight_steering_input = 1.0;
    param.weight_steering_input_squared_vel = 0.25;
    param.weight_lat_jerk = 0.0;
    param.weight_steer_rate = 0.0;
    param.weight_steer_acc = 0.000001;
    param.low_curvature_weight_steering_input = 1.0;
    param.low_curvature_weight_steering_input_squared_vel = 0.25;
    param.low_curvature_weight_lat_jerk = 0.0;
    param.low_curvature_weight_steer_rate = 0.0;
    param.low_curvature_weight_steer_acc = 0.000001;
    param.low_curvature_thresh_curvature = 0.0;

    TrajectoryPoint p;
    p.x = 0.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_straight_trajectory.points.push_back(p);
    p.x = 1.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_straight_trajectory.points.push_back(p);
    p.x = 2.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_straight_trajectory.points.push_back(p);
    p.x = 3.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_straight_trajectory.points.push_back(p);
    p.x = 4.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_straight_trajectory.points.push_back(p);

    p.x = -1.0f;
    p.y = -1.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_right_turn_trajectory.points.push_back(p);
    p.x = 0.0f;
    p.y = 0.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_right_turn_trajectory.points.push_back(p);
    p.x = 1.0f;
    p.y = -1.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_right_turn_trajectory.points.push_back(p);
    p.x = 2.0f;
    p.y = -2.0f;
    p.longitudinal_velocity_mps = 1.0f;
    dummy_right_turn_trajectory.points.push_back(p);
  }

  void initializeMPC(trajectory_follower::MPC & mpc)
  {
    mpc.m_param = param;
    mpc.m_admissible_position_error = admissible_position_error;
    mpc.m_admissible_yaw_error_rad = admissible_yaw_error_rad;
    mpc.m_steer_lim = steer_lim;
    mpc.m_steer_rate_lim = steer_rate_lim;
    mpc.m_ctrl_period = ctrl_period;
    mpc.m_use_steer_prediction = use_steer_prediction;
    // Init filters
    mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
    // Init trajectory
    mpc.setReferenceTrajectory(
      dummy_straight_trajectory, traj_resample_dist, enable_path_smoothing,
      path_filter_moving_ave_num, enable_yaw_recalculation,
      curvature_smoothing_num);
    neutral_steer.state.front_wheel_angle_rad = 0.0;
    pose_zero.position.x = 0.0;
    pose_zero.position.y = 0.0;
  }
};  // class MPCTest

/* cppcheck-suppress syntaxError */
TEST_F(MPCTest, initialize_and_calculate) {
  trajectory_follower::MPC mpc;
  EXPECT_FALSE(mpc.hasVehicleModel());
  EXPECT_FALSE(mpc.hasQPSolver());

  const std::string vehicle_model_type = "kinematics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModel>(
    wheelbase, steer_limit,
    steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(mpc);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, initialize_and_calculate_right_turn) {
  trajectory_follower::MPC mpc;
  EXPECT_FALSE(mpc.hasVehicleModel());
  EXPECT_FALSE(mpc.hasQPSolver());

  const std::string vehicle_model_type = "kinematics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModel>(
    wheelbase, steer_limit,
    steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(mpc);
  mpc.setReferenceTrajectory(
    dummy_right_turn_trajectory, traj_resample_dist, enable_path_smoothing,
    path_filter_moving_ave_num, enable_yaw_recalculation,
    curvature_smoothing_num);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, osqp_calculate) {
  trajectory_follower::MPC mpc;
  initializeMPC(mpc);

  const std::string vehicle_model_type = "kinematics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModel>(
    wheelbase, steer_limit,
    steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverOSQP>(logger);
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  // TODO(Maxime CLEMENT): with OSQP this function returns false despite finding correct solutions
  mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd);
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, osqp_calculate_right_turn) {
  trajectory_follower::MPC mpc;
  initializeMPC(mpc);
  mpc.setReferenceTrajectory(
    dummy_right_turn_trajectory, traj_resample_dist, enable_path_smoothing,
    path_filter_moving_ave_num, enable_yaw_recalculation,
    curvature_smoothing_num);

  const std::string vehicle_model_type = "kinematics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModel>(
    wheelbase, steer_limit,
    steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverOSQP>(logger);
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  // TODO(Maxime CLEMENT): with OSQP this function returns false despite finding correct solutions
  mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd);
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, kinematics_no_delay_calculate) {
  trajectory_follower::MPC mpc;
  initializeMPC(mpc);

  const std::string vehicle_model_type = "kinematics_no_delay";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init filters
  mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  // Init trajectory
  mpc.setReferenceTrajectory(
    dummy_straight_trajectory, traj_resample_dist, enable_path_smoothing,
    path_filter_moving_ave_num, enable_yaw_recalculation,
    curvature_smoothing_num);
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, kinematics_no_delay_calculate_right_turn) {
  trajectory_follower::MPC mpc;
  initializeMPC(mpc);
  mpc.setReferenceTrajectory(
    dummy_right_turn_trajectory, traj_resample_dist, enable_path_smoothing,
    path_filter_moving_ave_num, enable_yaw_recalculation,
    curvature_smoothing_num);

  const std::string vehicle_model_type = "kinematics_no_delay";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init filters
  mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, dynamic_calculate) {
  trajectory_follower::MPC mpc;
  initializeMPC(mpc);

  const std::string vehicle_model_type = "dynamics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::DynamicsBicycleModel>(
    wheelbase, mass_fl, mass_fr,
    mass_rl, mass_rr, cf, cr);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, multi_solve_with_buffer) {
  trajectory_follower::MPC mpc;
  const std::string vehicle_model_type = "kinematics";
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<trajectory_follower::KinematicsBicycleModel>(
    wheelbase, steer_limit,
    steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr =
    std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(mpc);

  mpc.m_input_buffer = {0.0, 0.0, 0.0};
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, default_velocity, pose_zero, ctrl_cmd));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
}

TEST(test_mpc, calcStopDistance) {
  using autoware::motion::control::trajectory_follower::MPC;
  using autoware_auto_msgs::msg::Trajectory;
  using autoware_auto_msgs::msg::TrajectoryPoint;

  MPC mpc;

  Trajectory trajectory_msg;
  TrajectoryPoint p;
  // Point 0
  p.x = 0.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 1
  p.x = 1.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 2 - STOP
  p.x = 2.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 3
  p.x = 3.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 4
  p.x = 4.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 5
  p.x = 5.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 6 - STOP
  p.x = 6.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 7 - STOP
  p.x = 7.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);

  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(mpc.calcStopDistance(trajectory_msg, 7), -1.0);
}
}  // namespace
