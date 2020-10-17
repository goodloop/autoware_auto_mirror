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
/// \brief This file defines the autoware_auto_contracts strictly positive real
/// type.

#ifndef AUTOWARE_AUTO_CONTRACTS__STRICTLY_EPS_POSITIVE_REAL_HPP_
#define AUTOWARE_AUTO_CONTRACTS__STRICTLY_EPS_POSITIVE_REAL_HPP_

#include <contracts_lite/range_checks.hpp>
#include <limits>

#include "autoware_auto_contracts/violation_handler.hpp"
#include "common/types.hpp"

namespace autoware
{
namespace common
{
using autoware::common::types::FEPS;
namespace contracts
{
/**
 * @brief Container for strictly epsilon positive reals.
 *
 * @note A strictly epsilon positive real is 'valid' if and only if it is in [FEPS, inf), where FEPS > 0
 *
 * @invariant The float value of these objects is guaranteed to be valid upon successful construction.
 *
 * @implements{CONTRACTS002}
 */
template<typename T>
class StrictlyEpsPositiveReal
{
  static_assert(std::numeric_limits<T>::is_iec559,
    "StrictlyPositiveReal numeric type must be IEEE float compliant.");

public:
  StrictlyEpsPositiveReal() = delete;

  /** @brief Allow objects to be directly cast to float types. */
  operator T() const {return r_;}

  /**
   * @brief Converting constructor for valid strictly epsilon positive real objects.
   *
   * @post The class invariant validity condition holds (see invariant in StrictlyPositiveReal).
   */
  StrictlyEpsPositiveReal(T r)
  : r_(r)
  {
    static_assert(FEPS > 0.0f, "FEPS must be strictly positive");
    DEFAULT_ENFORCE(contracts_lite::range_checks::in_range_closed_open(
        r_, static_cast<T>(FEPS), std::numeric_limits<T>::infinity()));
  }

private:
  T r_;
};

typedef StrictlyEpsPositiveReal<common::types::float32_t> StrictlyEpsPositiveRealf;
typedef StrictlyEpsPositiveReal<common::types::float64_t> StrictlyEpsPositiveReald;

}  // namespace contracts
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_CONTRACTS__STRICTLY_EPS_POSITIVE_REAL_HPP_
