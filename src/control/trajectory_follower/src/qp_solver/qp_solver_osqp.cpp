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

#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"

#include <string>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
QPSolverOSQP::QPSolverOSQP(const rclcpp::Logger & logger)
: logger_{logger} {}
bool8_t QPSolverOSQP::solve(
  const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
  const Eigen::VectorXd & ubA, Eigen::VectorXd & U)
{
  const Eigen::Index raw_a = A.rows();
  const Eigen::Index col_a = A.cols();
  const Eigen::Index DIM_U = ub.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(DIM_U, DIM_U);

  // convert matrix to vector for osqpsolver
  std::vector<float64_t> f(&fvec(0), fvec.data() + fvec.cols() * fvec.rows());

  std::vector<float64_t> lower_bound;
  std::vector<float64_t> upper_bound;

  for (int64_t i = 0; i < DIM_U; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int64_t i = 0; i < col_a; ++i) {
    lower_bound.push_back(lbA(i));
    upper_bound.push_back(ubA(i));
  }

  Eigen::MatrixXd osqpA = Eigen::MatrixXd(DIM_U + col_a, raw_a);
  osqpA << I, A;

  /* execute optimization */
  auto result = osqpsolver_.optimize(Hmat, osqpA, f, lower_bound, upper_bound);

  std::vector<float64_t> U_osqp = std::get<0>(result);
  U =
    Eigen::Map<Eigen::Matrix<float64_t, Eigen::Dynamic, 1>>(
    &U_osqp[0],
    static_cast<Eigen::Index>(U_osqp.size()), 1);

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int64_t status_polish = std::get<2>(result);
  if (status_polish == -1) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unsuccessful)", status_polish);
    return false;
  }
  if (status_polish == 0) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unperformed)", status_polish);
    return true;
  }
  return true;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
