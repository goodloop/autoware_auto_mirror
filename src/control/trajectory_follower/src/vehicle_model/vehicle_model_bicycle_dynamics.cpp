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

#include <algorithm>

#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
DynamicsBicycleModel::DynamicsBicycleModel(
  const float64_t & wheelbase, const float64_t & mass_fl,
  const float64_t & mass_fr, const float64_t & mass_rl,
  const float64_t & mass_rr, const float64_t & cf, const float64_t & cr)
: VehicleModelInterface(/* dim_x */ 4, /* dim_u */ 1, /* dim_y */ 2)
{
  m_wheelbase = wheelbase;

  const float64_t mass_front = mass_fl + mass_fr;
  const float64_t mass_rear = mass_rl + mass_rr;

  m_mass = mass_front + mass_rear;
  m_lf = m_wheelbase * (1.0 - mass_front / m_mass);
  m_lr = m_wheelbase * (1.0 - mass_rear / m_mass);
  m_iz = m_lf * m_lf * mass_front + m_lr * m_lr * mass_rear;
  m_cf = cf;
  m_cr = cr;
}

void DynamicsBicycleModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Cd, Eigen::MatrixXd & Wd,
  const float64_t & dt)
{
  /*
   * x[k+1] = Ad*x[k] + Bd*u + Wd
   */

  const float64_t vel = std::max(m_velocity, 0.01);

  Ad = Eigen::MatrixXd::Zero(m_dim_x, m_dim_x);
  Ad(0, 1) = 1.0;
  Ad(1, 1) = -(m_cf + m_cr) / (m_mass * vel);
  Ad(1, 2) = (m_cf + m_cr) / m_mass;
  Ad(1, 3) = (m_lr * m_cr - m_lf * m_cf) / (m_mass * vel);
  Ad(2, 3) = 1.0;
  Ad(3, 1) = (m_lr * m_cr - m_lf * m_cf) / (m_iz * vel);
  Ad(3, 2) = (m_lf * m_cf - m_lr * m_cr) / m_iz;
  Ad(3, 3) = -(m_lf * m_lf * m_cf + m_lr * m_lr * m_cr) / (m_iz * vel);

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  Eigen::MatrixXd Ad_inverse = (I - dt * 0.5 * Ad).inverse();

  Ad = Ad_inverse * (I + dt * 0.5 * Ad);  // bilinear discretization

  Bd = Eigen::MatrixXd::Zero(m_dim_x, m_dim_u);
  Bd(0, 0) = 0.0;
  Bd(1, 0) = m_cf / m_mass;
  Bd(2, 0) = 0.0;
  Bd(3, 0) = m_lf * m_cf / m_iz;

  Wd = Eigen::MatrixXd::Zero(m_dim_x, 1);
  Wd(0, 0) = 0.0;
  Wd(1, 0) = (m_lr * m_cr - m_lf * m_cf) / (m_mass * vel) - vel;
  Wd(2, 0) = 0.0;
  Wd(3, 0) = -(m_lf * m_lf * m_cf + m_lr * m_lr * m_cr) / (m_iz * vel);

  Bd = (Ad_inverse * dt) * Bd;
  Wd = (Ad_inverse * dt * m_curvature * vel) * Wd;

  Cd = Eigen::MatrixXd::Zero(m_dim_y, m_dim_x);
  Cd(0, 0) = 1.0;
  Cd(1, 2) = 1.0;
}

void DynamicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd & Uref)
{
  const float64_t vel = std::max(m_velocity, 0.01);
  const float64_t Kv = m_lr * m_mass / (2 * m_cf * m_wheelbase) - m_lf * m_mass /
    (2 * m_cr * m_wheelbase);
  Uref(0, 0) = m_wheelbase * m_curvature + Kv * vel * vel * m_curvature;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
