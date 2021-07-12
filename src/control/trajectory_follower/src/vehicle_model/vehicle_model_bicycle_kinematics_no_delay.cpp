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

#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
KinematicsBicycleModelNoDelay::KinematicsBicycleModelNoDelay(
  const float64_t & wheelbase, const float64_t & steer_lim)
: VehicleModelInterface(/* dim_x */ 2, /* dim_u */ 1, /* dim_y */ 2)
{
  m_wheelbase = wheelbase;
  m_steer_lim = steer_lim;
}

void KinematicsBicycleModelNoDelay::calculateDiscreteMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Cd, Eigen::MatrixXd & Wd,
  const float64_t & dt)
{
  auto sign = [](float64_t x) {return (x > 0.0) - (x < 0.0);};

  /* Linearize delta around delta_r (reference delta) */
  float64_t delta_r = atan(m_wheelbase * m_curvature);
  if (abs(delta_r) >= m_steer_lim) {delta_r = m_steer_lim * static_cast<float64_t>(sign(delta_r));}
  float64_t cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

  Ad << 0.0, m_velocity, 0.0, 0.0;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  Ad = I + Ad * dt;

  Bd << 0.0, m_velocity / m_wheelbase * cos_delta_r_squared_inv;
  Bd *= dt;

  Cd << 1.0, 0.0, 0.0, 1.0;

  Wd << 0.0, -m_velocity / m_wheelbase * delta_r * cos_delta_r_squared_inv;
  Wd *= dt;
}

void KinematicsBicycleModelNoDelay::calculateReferenceInput(Eigen::MatrixXd & Uref)
{
  Uref(0, 0) = std::atan(m_wheelbase * m_curvature);
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
