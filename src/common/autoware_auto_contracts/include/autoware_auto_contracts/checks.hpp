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
/// \brief This file defines the autoware_auto_contracts real type.

#ifndef AUTOWARE_AUTO_CONTRACTS__CHECKS_HPP_
#define AUTOWARE_AUTO_CONTRACTS__CHECKS_HPP_

#include <contracts_lite/range_checks.hpp>

#include "autoware_auto_contracts/acute_radian.hpp"
#include "autoware_auto_contracts/acute_degree.hpp"
#include "autoware_auto_contracts/nonnegative_real.hpp"
#include "autoware_auto_contracts/real.hpp"
#include "autoware_auto_contracts/size_bound.hpp"
#include "autoware_auto_contracts/strictly_eps_positive_real.hpp"
#include "autoware_auto_contracts/strictly_positive_real.hpp"
#include "autoware_auto_contracts/scalar_flicker.hpp"

namespace autoware
{
namespace common
{
/** @brief This namespace contains type definitions for the contracts library. */
namespace contracts {}  // namespace contracts
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_CONTRACTS__CHECKS_HPP_
