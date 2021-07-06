// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include <iostream>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
KinematicsBicycleModel::KinematicsBicycleModel(
  const double & wheelbase, const double & steer_lim, const double & steer_tau)
: VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2)
{
  m_wheelbase = wheelbase;
  m_steer_lim = steer_lim;
  m_steer_tau = steer_tau;
}

void KinematicsBicycleModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Cd, Eigen::MatrixXd & Wd,
  const double & dt)
{
  auto sign = [](double x) {return (x > 0.0) - (x < 0.0);};

  /* Linearize delta around delta_r (reference delta) */
  double delta_r = atan(m_wheelbase * m_curvature);
  if (abs(delta_r) >= m_steer_lim) {delta_r = m_steer_lim * static_cast<double>(sign(delta_r));}
  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));
  double velocity = m_velocity;
  if (abs(m_velocity) < 1e-04) {velocity = 1e-04 * (m_velocity >= 0 ? 1 : -1);}

  Ad << 0.0, velocity, 0.0, 0.0, 0.0, velocity / m_wheelbase * cos_delta_r_squared_inv, 0.0, 0.0,
    -1.0 / m_steer_tau;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  Ad = (I - dt * 0.5 * Ad).inverse() * (I + dt * 0.5 * Ad);  // bilinear discretization

  Bd << 0.0, 0.0, 1.0 / m_steer_tau;
  Bd *= dt;

  Cd << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  Wd << 0.0,
    -velocity * m_curvature +
    velocity / m_wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
    0.0;
  Wd *= dt;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd & Uref)
{
  Uref(0, 0) = std::atan(m_wheelbase * m_curvature);
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
