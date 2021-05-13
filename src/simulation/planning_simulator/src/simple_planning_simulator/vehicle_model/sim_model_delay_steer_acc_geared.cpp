// Copyright 2021 The Autoware Foundation.
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

#include <algorithm>

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_command.hpp"

SimModelDelaySteerAccGeared::SimModelDelaySteerAccGeared(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant)
: SimModelInterface(6 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT))
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGeared::getX() {return state_(IDX::X);}
double SimModelDelaySteerAccGeared::getY() {return state_(IDX::Y);}
double SimModelDelaySteerAccGeared::getYaw() {return state_(IDX::YAW);}
double SimModelDelaySteerAccGeared::getVx() {return state_(IDX::VX);}
double SimModelDelaySteerAccGeared::getVy() {return 0.0;}
double SimModelDelaySteerAccGeared::getAx() {return state_(IDX::ACCX);}
double SimModelDelaySteerAccGeared::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelDelaySteerAccGeared::getSteer() {return state_(IDX::STEER);}
void SimModelDelaySteerAccGeared::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  const auto prev_vx = state_(IDX::VX);

  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  state_(IDX::VX) = calcVelocityWithGear(state_, gear_);

  // calc acc directly after gear considerataion
  state_(IDX::ACCX) = (state_(IDX::VX) - prev_vx) / std::max(dt, 1.0e-5);
}

void SimModelDelaySteerAccGeared::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    acc_input_queue_.push_back(0.0);
  }
  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelDelaySteerAccGeared::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) {return std::max(std::min(val, u), l);};

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double acc_des = sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_);
  const double steer_des = sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_);
  double steer_rate = -(steer - steer_des) / steer_time_constant_;
  steer_rate = sat(steer_rate, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}


double SimModelDelaySteerAccGeared::calcVelocityWithGear(
  const Eigen::VectorXd & state, const uint8_t gear) const
{
  using autoware_auto_msgs::msg::VehicleStateCommand;
  if (gear == VehicleStateCommand::GEAR_DRIVE ||
    gear == VehicleStateCommand::GEAR_LOW ||
    gear == VehicleStateCommand::GEAR_NEUTRAL)
  {
    if (state_(IDX::VX) < 0.0) {
      return 0.0;
    }
  } else if (gear_ == VehicleStateCommand::GEAR_REVERSE) {
    if (state_(IDX::VX) > 0.0) {
      return 0.0;
    }
  } else if (gear_ == VehicleStateCommand::GEAR_PARK) {
    return 0.0;
  }

  return state_(IDX::VX);
}
