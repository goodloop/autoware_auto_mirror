// Copyright 2018-2021 The Autoware Foundation
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

#ifndef TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
#define TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_

#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
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
using autoware::common::types::bool8_t;
class TRAJECTORY_FOLLOWER_PUBLIC QPSolverInterface
{
public:
  /**
   * @brief destructor
   */
  virtual ~QPSolverInterface() = default;

  /**
   * @brief solve QP problem : minimize J = U' * Hmat * U + fvec' * U without constraint
   * @param [in] Hmat parameter matrix in object function
   * @param [in] fvec parameter matrix in object function
   * @param [in] A parameter matrix for constraint lbA < A*U < ubA
   * @param [in] lb parameter matrix for constraint lb < U < ub
   * @param [in] ub parameter matrix for constraint lb < U < ub
   * @param [in] lbA parameter matrix for constraint lbA < A*U < ubA
   * @param [in] ubA parameter matrix for constraint lbA < A*U < ubA
   * @param [out] U optimal variable vector
   * @return ture if the problem was solved
   */
  virtual bool8_t solve(
    const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
    const Eigen::VectorXd & ubA, Eigen::VectorXd & U) = 0;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
