// Copyright 2021 The Autoware Foundation
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

/*
 *    Representation
 * e      : lateral error
 * de     : derivative of lateral error
 * th     : heading angle error
 * dth    : derivative of heading angle error
 * steer  : steering angle (input)
 * v      : velocity
 * m      : mass
 * Iz     : inertia
 * lf     : length from center to front tire
 * lr     : length from center to rear tire
 * cf     : front tire cornering power
 * cr     : rear tire cornering power
 * k      : curvature on reference trajectory point
 *
 *    State & Input
 * x = [e, de, th, dth]^T
 * u = steer
 *
 *    Linearized model around reference point (v=vr)
 *          [0,                   1,                0,                        0]       [       0]       [ 0] dx/dt = [0,
 * -(cf+cr)/m/vr,        (cf+cr)/m,       (lr*cr-lf*cf)/m/vr] * x + [    cf/m] * u + [(lr*cr-lf*cf)/m/vr*k - vr*k] [0,
 * 0,                0,                        1]       [       0]       [                          0] [0,
 * (lr*cr-lf*cf)/Iz/vr, (lf*cf-lr*cr)/Iz, -(lf^2*cf+lr^2*cr)/Iz/vr]       [lf*cf/Iz]       [   -(lf^2*cf+lr^2*cr)/Iz/vr]
 *
 * Reference : Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics
 * Institute, Carnegie Mellon University, February 2009.
 */

#ifndef TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_
#define TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_

#include "trajectory_follower/vehicle_model/vehicle_model_interface.hpp"

#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::float64_t;
/**
 * Vehicle model class of bicycle dynamics
 * @brief calculate model-related values
 */
class TRAJECTORY_FOLLOWER_PUBLIC DynamicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] mass_fl mass applied to front left tire [kg]
   * @param [in] mass_fr mass applied to front right tire [kg]
   * @param [in] mass_rl mass applied to rear left tire [kg]
   * @param [in] mass_rr mass applied to rear right tire [kg]
   * @param [in] cf front cornering power
   * @param [in] cr rear cornering power
   */
  DynamicsBicycleModel(
    const float64_t & wheelbase, const float64_t & mass_fl, const float64_t & mass_fr,
    const float64_t & mass_rl, const float64_t & mass_rr,
    const float64_t & cf, const float64_t & cr);

  /**
   * @brief destructor
   */
  ~DynamicsBicycleModel() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk
   * @param [in] Ad coefficient matrix
   * @param [in] Bd coefficient matrix
   * @param [in] Cd coefficient matrix
   * @param [in] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, Eigen::MatrixXd & Cd,
    const float64_t & dt) override;

  /**
   * @brief calculate reference input
   * @param [out] Uref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & Uref) override;

private:
  float64_t m_wheelbase;  //!< @brief wheelbase length [m]
  float64_t m_lf;         //!< @brief length from center of mass to front wheel [m]
  float64_t m_lr;         //!< @brief length from center of mass to rear wheel [m]
  float64_t m_mass;       //!< @brief total mass of vehicle [kg]
  float64_t m_iz;         //!< @brief moment of inertia [kg * m2]
  float64_t m_cf;         //!< @brief front cornering power [N/rad]
  float64_t m_cr;         //!< @brief rear cornering power [N/rad]
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_
