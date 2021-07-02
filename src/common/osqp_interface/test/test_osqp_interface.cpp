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

#include <tuple>
#include <vector>

#include "eigen3/Eigen/Core"
#include "gtest/gtest.h"
#include "osqp_interface/osqp_interface.hpp"


/// Problem taken from https://github.com/osqp/osqp/blob/master/tests/basic_qp/generate_problem.py
TEST(test_osqp_interface, basic_qp) {
  auto check_result =
    [](const std::tuple<std::vector<double>, std::vector<double>, int, int> & result) {
      EXPECT_EQ(std::get<2>(result), 1);
      EXPECT_EQ(std::get<3>(result), 1);
      ASSERT_EQ(std::get<0>(result).size(), size_t(2));
      ASSERT_EQ(std::get<1>(result).size(), size_t(2));
      EXPECT_DOUBLE_EQ(std::get<0>(result)[0], 0.3);
      EXPECT_DOUBLE_EQ(std::get<0>(result)[1], 0.7);
      EXPECT_DOUBLE_EQ(std::get<1>(result)[0], -2.9);
      EXPECT_NEAR(std::get<1>(result)[1], 0.2, 1e-6);
    };

  {
    // Define problem after initialization
    common::osqp::OSQPInterface osqp;
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(2, 4);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<double> q = {1.0, 1.0};
    std::vector<double> l = {1.0, 0.0, 0.0, -common::osqp::INF};
    std::vector<double> u = {1.0, 0.7, 0.7, common::osqp::INF};
    std::tuple<std::vector<double>, std::vector<double>, int, int> result = osqp.optimize(
      P, A, q, l, u);
    check_result(result);
  }
  {
    // Define problem during initialization
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(2, 4);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<double> q = {1.0, 1.0};
    std::vector<double> l = {1.0, 0.0, 0.0, -common::osqp::INF};
    std::vector<double> u = {1.0, 0.7, 0.7, common::osqp::INF};
    common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    std::tuple<std::vector<double>, std::vector<double>, int, int> result = osqp.optimize();
    check_result(result);
  }
  {
    std::tuple<std::vector<double>, std::vector<double>, int, int> result;
    // Dummy initial problem
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 4);
    std::vector<double> q(2);
    std::vector<double> l(4);
    std::vector<double> u(4);
    common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    osqp.optimize();
    // Redefine problem
    Eigen::MatrixXd P_new(2, 2);
    P_new << 4, 1, 1, 2;
    Eigen::MatrixXd A_new(2, 4);
    A_new << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<double> q_new = {1.0, 1.0};
    std::vector<double> l_new = {1.0, 0.0, 0.0, -common::osqp::INF};
    std::vector<double> u_new = {1.0, 0.7, 0.7, common::osqp::INF};
    osqp.updateP(P_new);
    osqp.updateA(A_new);
    osqp.updateQ(q_new);
    osqp.updateBounds(l_new, u_new);
    result = osqp.optimize();
    // TODO(Maxime CLEMENT): wrong result after individually updating the parameters
    // check_result(result);
    osqp.initializeProblem(P_new, A_new, q_new, l_new, u_new);
    result = osqp.optimize();
    check_result(result);
  }
}

TEST(test_osqp_interface_update, smoke_test)
{
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 4);
  std::vector<double> q(2);
  std::vector<double> l(4);
  std::vector<double> u(4);
  common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
  osqp.updateL({-2.0, -2.0, -2.0, -2.0});
  osqp.updateU({2.0, 2.0, 2.0, 2.0});
  // lower bound higher than the upper bound generates an error message
  osqp.updateL({3.0, 3.0, 3.0, 3.0});
  osqp.updateEpsAbs(0.5);
  osqp.updateEpsRel(0.1);
  osqp.updateMaxIter(100);
  osqp.updateRhoInterval(1);
  osqp.updateVerbose(true);
  osqp.updateVerbose(false);
}
