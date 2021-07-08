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

/*
 *    Representation
 * e      : lateral error
 * th     : heading angle error
 * steer  : steering angle (input)
 * v      : velocity
 * W      : wheelbase length
 * tau    : time constant for steering dynamics
 *
 *    State & Input
 * x = [e, th]^T
 * u = steer
 *
 *    Nonlinear model
 * dx1/dt = v * sin(x2)
 * dx2/dt = v * tan(u) / W
 *
 *    Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
 *  dx/dt = [0, vr] * x + [                  0] * u + [                           0]
 *          [0,  0]       [vr/W/cos(steer_r)^2]       [-vr*steer_r/W/cos(steer_r)^2]
 *
 */

#ifndef TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_NO_DELAY_HPP_  // NOLINT
#define TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_NO_DELAY_HPP_  // NOLINT

#include "trajectory_follower/vehicle_model/vehicle_model_interface.hpp"
#include "trajectory_follower/visibility_control.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
/**
 * Vehicle model class of bicycle kinematics without steering delay
 * @brief calculate model-related values
 */
class TRAJECTORY_FOLLOWER_PUBLIC KinematicsBicycleModelNoDelay : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   */
  KinematicsBicycleModelNoDelay(const double & wheelbase, const double & steer_lim);

  /**
   * @brief destructor
   */
  ~KinematicsBicycleModelNoDelay() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Cd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Cd, Eigen::MatrixXd & Wd,
    const double & dt) override;

  /**
   * @brief calculate reference input
   * @param [out] Uref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & Uref) override;

private:
  double m_wheelbase;  //!< @brief wheelbase length [m]
  double m_steer_lim;  //!< @brief steering angle limit [rad]
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_NO_DELAY_HPP_