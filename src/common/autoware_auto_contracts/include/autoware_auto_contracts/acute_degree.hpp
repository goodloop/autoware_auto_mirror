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
/// \brief This file defines the autoware_auto_contracts acute degree type.

#ifndef AUTOWARE_AUTO_CONTRACTS__ACUTE_DEGREE_HPP_
#define AUTOWARE_AUTO_CONTRACTS__ACUTE_DEGREE_HPP_

#include <contracts_lite/range_checks.hpp>
#include <limits>
#include <utility>

#include "autoware_auto_contracts/violation_handler.hpp"
#include "common/types.hpp"

namespace autoware
{
namespace common
{
namespace contracts
{
/** @brief Forward declaration for conversion functionality. */
template<typename T>
class AcuteRadian;

/**
 * @brief Container for acute degrees.
 *
 * @note An acute degree is 'valid' if and only if it is in [0, 90).
 *
 * @invariant The float value of these objects is guaranteed to be valid upon
 * successful construction.
 *
 * @implements{CONTRACTS002}
 */
template<typename T>
class AcuteDegree
{
  static_assert(std::numeric_limits<T>::is_iec559,
    "AcuteDegree numeric type must be IEEE float compliant.");

public:
  AcuteDegree() = delete;

  /** @brief Allow objects to be directly cast to float types. */
  operator T() const;

  /**
   * @brief Conversion assignment from radians.
   *
   * @post See AcuteDegree(T r)
   */
  AcuteDegree & operator=(AcuteRadian<T> r);

  /** @brief Make this explicit to distinguish from above. */
  AcuteDegree & operator=(T r);

  /**
   * @brief Conversion constructor from radians.
   *
   * @post See AcuteDegree(T r)
   */
  AcuteDegree(AcuteRadian<T> r);

  /**
   * @brief Converting constructor for valid acute degree objects.
   *
   * @post The class invariant validity condition holds (see invariant in
   * AcuteDegree).
   */
  AcuteDegree(T r);

private:
  T r_;

  /** @brief Conversion function. */
  static T radian_to_degree(T r);
};

typedef AcuteDegree<::autoware::common::types::float32_t> AcuteDegreef;
typedef AcuteDegree<::autoware::common::types::float64_t> AcuteDegreed;

}  // namespace contracts
}  // namespace common
}  // namespace autoware

#include "autoware_auto_contracts/acute_radian.hpp"

namespace autoware
{
namespace common
{
namespace contracts
{

//------------------------------------------------------------------------------

template<typename T>
T AcuteDegree<T>::radian_to_degree(T r)
{
  constexpr auto ratio = static_cast<T>(180) / static_cast<T>(::autoware::common::types::PI);
  return r * ratio;
}

//------------------------------------------------------------------------------

template<typename T>
AcuteDegree<T>::operator T() const {return r_;}

//------------------------------------------------------------------------------

template<typename T>
AcuteDegree<T>::AcuteDegree(AcuteRadian<T> r)
: AcuteDegree<T>(AcuteDegree<T>::radian_to_degree(r)) {}

//------------------------------------------------------------------------------

template<typename T>
AcuteDegree<T> & AcuteDegree<T>::operator=(AcuteRadian<T> r)
{
  *this = AcuteDegree<T>{AcuteDegree<T>::radian_to_degree(r)};
  return *this;
}

//------------------------------------------------------------------------------

template<typename T>
AcuteDegree<T> & AcuteDegree<T>::operator=(T r)
{
  *this = AcuteDegree<T>{std::move(r)};
  return *this;
}

//------------------------------------------------------------------------------

template<typename T>
AcuteDegree<T>::AcuteDegree(T r)
: r_(r)
{
  DEFAULT_ENFORCE(contracts_lite::range_checks::in_range_closed_open(
      r_, static_cast<T>(0), static_cast<T>(90)));
}

//------------------------------------------------------------------------------

}  // namespace contracts
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_CONTRACTS__ACUTE_DEGREE_HPP_
