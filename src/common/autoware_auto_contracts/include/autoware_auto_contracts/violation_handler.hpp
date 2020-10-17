// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the autoware_auto_contracts violation handlers.

#ifndef AUTOWARE_AUTO_CONTRACTS__VIOLATION_HANDLER_HPP_
#define AUTOWARE_AUTO_CONTRACTS__VIOLATION_HANDLER_HPP_

#include <exception>
#include <stdexcept>
#include <string>

#include "contracts_lite/contract_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autoware
{
namespace common
{
namespace contracts
{

/**
 * @brief This function can be specified as the contract violation handler.
 * @note This function does not return. It throws std::runtime_error.
 */
inline void handler_with_continuation(
  const contracts_lite::ContractViolation & violation)
{
  throw std::runtime_error("CONTRACT VIOLATION: " +
          contracts_lite::ContractViolation::string(violation));
}

/**
 * @brief This function can be specified as the contract violation handler.
 * @note This function does not return. It calls std::terminate.
 */
inline void handler_without_continuation(
  const contracts_lite::ContractViolation & violation) noexcept
{
  RCLCPP_FATAL(rclcpp::get_logger(
      "contract_violation_handler"), contracts_lite::ContractViolation::string(violation));
  std::terminate();
}

}  // namespace contracts
}  // namespace common
}  // namespace autoware

/**
 * @brief Define the build-dependent contract violation handler.
 *
 * @implements{CONTRACTS003}
 */
#ifdef CONTRACT_VIOLATION_CONTINUATION_MODE_ON
#define CONTRACT_VIOLATION_HANDLER(violation) \
  ::autoware::common::contracts::handler_with_continuation(violation)
#else
#define CONTRACT_VIOLATION_HANDLER(violation) \
  ::autoware::common::contracts::handler_without_continuation(violation)
#endif

#include "contracts_lite/enforcement.hpp"

#endif  // AUTOWARE_AUTO_CONTRACTS__VIOLATION_HANDLER_HPP_
