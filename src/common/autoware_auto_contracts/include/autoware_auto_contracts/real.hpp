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

#ifndef AUTOWARE_AUTO_CONTRACTS__REAL_HPP_
#define AUTOWARE_AUTO_CONTRACTS__REAL_HPP_

#include <contracts_lite/range_checks.hpp>
#include <limits>

#include "autoware_auto_contracts/violation_handler.hpp"
#include "common/types.hpp"

namespace autoware
{
namespace common
{
namespace contracts
{
/**
 * @brief Container for reals.
 *
 * @note A real is 'valid' if and only if it is in (-inf, inf).
 *
 * @invariant The float value of these objects is guaranteed to be valid upon
 * successful construction.
 */
template<typename T>
class Real
{
  static_assert(std::numeric_limits<T>::is_iec559,
    "Real numeric type must be IEEE float compliant.");

public:
  Real() = delete;

  /** @brief Allow objects to be directly cast to float types. */
  operator T() const {return r_;}

  /**
   * @brief Converting constructor for valid real objects.
   *
   * @post The class invariant validity condition holds (see invariant in Real).
   */
  Real(T r)
  : r_(r)
  {
    DEFAULT_ENFORCE(contracts_lite::range_checks::in_range_open_open(
        r_, -std::numeric_limits<T>::infinity(),
        std::numeric_limits<T>::infinity()));
  }

private:
  T r_;
};

typedef Real<common::types::float32_t> Realf;
typedef Real<common::types::float64_t> Reald;

}  // namespace contracts
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_CONTRACTS__REAL_HPP_
