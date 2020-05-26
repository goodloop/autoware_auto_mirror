// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef OPTIMIZATION__OPTIMIZER_HPP_
#define OPTIMIZATION__OPTIMIZER_HPP_

#include <optimization/optimization_problem.hpp>
#include <optimization/optimizer_options.hpp>
#include <optimization/line_search.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/SVD>
#include <limits>
#include <memory>
#include <cmath>

namespace autoware
{
namespace common
{
namespace optimization
{
// Optimization solver base class(CRTP) for a given optimization problem.
template<typename Derived>
class OPTIMIZATION_PUBLIC Optimizer : public common::helper_functions::crtp<Derived>
{
public:
  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \tparam OptimizationProblemT Optimization problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \tparam DomainValueT Type of the parameter
  /// \tparam EigenSolverT Type of eigen solver to be used internallt for solving the
  /// necessary linear equations. By default set to `Eigen::LDLT`.
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT, typename OptionsT,
    typename EigenSolverT = Eigen::LDLT<typename OptimizationProblemT::Hessian>>
  OptimizationSummary solve(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const OptionsT & options)
  {
    return this->impl().template solve_<OptimizationProblemT, DomainValueT, EigenSolverT>(
      optimization_problem, x0, x_out, options);
  }
};


/// Optimizer using the Newton method with line search
template<typename LineSearchT>
class OPTIMIZATION_PUBLIC NewtonsMethod : public Optimizer<NewtonsMethod<LineSearchT>>
{
public:
  using StepT = float_t;

  /// Constructor to initialize the line search method
  explicit NewtonsMethod(const LineSearchT & line_searcher)
  : m_line_searcher(line_searcher) {}

  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \tparam OptimizationProblemT Optimization problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \tparam DomainValueT Type of the parameter
  /// \tparam EigenSolverT Type of eigen solver to be used internallt for solving the
  /// necessary linear equations. By default set to `Eigen::LDLT`.
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT, typename EigenSolverT>
  OptimizationSummary solve_(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const NewtonOptimizationOptions & options)
  {
    // Get types from the method's templated parameter
    using Value = typename OptimizationProblemT::Value;
    using Jacobian = typename OptimizationProblemT::Jacobian;
    using Hessian = typename OptimizationProblemT::Hessian;
    TerminationType termination_type{TerminationType::NO_CONVERGENCE};
    Value score_previous{0.0};
    Jacobian jacobian{Jacobian{}.setZero()};
    Hessian hessian{Hessian{}.setZero()};
    DomainValueT opt_direction{DomainValueT{}.setZero()};

    if (!x0.allFinite()) {   // Early exit for invalid input.
      return OptimizationSummary{0.0, TerminationType::FAILURE, 0UL};
    }

    // Initialize
    x_out = x0;

    // Get score, Jacobian and Hessian (pre-computed using evaluate)
    optimization_problem.evaluate(x_out, ComputeMode{}.set_score().set_jacobian().set_hessian());
    score_previous = optimization_problem(x_out);
    optimization_problem.jacobian(x_out, jacobian);
    optimization_problem.hessian(x_out, hessian);

    // Early exit if the initial solution is good enough.
    if (jacobian.template lpNorm<Eigen::Infinity>() <= options.gradient_tolerance()) {
      // As there's no newton solution yet, jacobian can be a good substitute.
      return OptimizationSummary{jacobian.norm(), TerminationType::CONVERGENCE, 0UL};
    }

    // Iterate until convergence, error, or maximum number of iterations
    auto nr_iterations = 0UL;
    for (; nr_iterations < options.max_num_iterations(); ++nr_iterations) {
      if (!x_out.allFinite() || !jacobian.allFinite() || !hessian.allFinite()) {
        termination_type = TerminationType::FAILURE;
        break;
      }
      // Find decent direction using Newton's method
      EigenSolverT solver(hessian);
      opt_direction = solver.solve(-jacobian);

      // Check if there was a problem during Eigen's solve()
      if (!opt_direction.allFinite()) {
        termination_type = TerminationType::FAILURE;
        break;
      }
      // Calculate and apply step length
      // TODO(zozen): with guarnteed sufficient decrease as in [More, Thuente 1994]
      // would need partial results passed to optimization_problem before call, as in:
      // computeStepLengthMT (x0, x_delta, x_delta_norm, transformation_epsilon_/2, ...
      // and would pre-compute score/jacobian/hessian as during init in evaluate!
      // also needs the sign to know the direction of optimization?
      const auto step = m_line_searcher.compute_step_length(x_out, opt_direction,
          optimization_problem);
      const auto prev_x_norm = x_out.norm();
      x_out += step;

      // Check change in parameter relative to the parameter value
      // tolerance added to the norm for stability when the norm is close to 0
      // (Inspired from https://github.com/ceres-solver/ceres-solver/blob/4362a2169966e08394252098
      // c80d1f26764becd0/include/ceres/tiny_solver.h#L244)
      const auto parameter_tolerance =
        options.parameter_tolerance() * (prev_x_norm + options.parameter_tolerance());
      if (step.norm() <= parameter_tolerance) {
        termination_type = TerminationType::CONVERGENCE;
        RCLCPP_WARN(rclcpp::get_logger("opt"), "param convergence");
        break;
      }

      // Update value, Jacobian and Hessian (pre-computed using evaluate)
      optimization_problem.evaluate(x_out, ComputeMode{}.set_score().set_jacobian().set_hessian());
      const auto score = optimization_problem(x_out);
      optimization_problem.jacobian(x_out, jacobian);
      optimization_problem.hessian(x_out, hessian);

      // Check if the max-norm of the gradient is small enough.
      if (jacobian.template lpNorm<Eigen::Infinity>() <= options.gradient_tolerance()) {
        termination_type = TerminationType::CONVERGENCE;
        RCLCPP_WARN(rclcpp::get_logger("opt"), "grad convergence");
        break;
      }

      // Check change in cost function.
      if (std::fabs(score - score_previous) <=
        (options.function_tolerance() * std::fabs(score_previous)))
      {
        termination_type = TerminationType::CONVERGENCE;
        RCLCPP_WARN(rclcpp::get_logger("opt"), "score convergence");
        break;
      }
      score_previous = score;
    }

    if(termination_type == TerminationType::NO_CONVERGENCE){
        RCLCPP_WARN(rclcpp::get_logger("opt"), "no convergence");
    }

    // Returning summary consisting of the following three values:
    // estimated_distance_to_optimum, convergence_tolerance_criteria_met, number_of_iterations_made
    return OptimizationSummary{opt_direction.norm(), termination_type, nr_iterations};
  }

private:
  // initialize on construction
  LineSearchT m_line_searcher;
};
}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZER_HPP_
