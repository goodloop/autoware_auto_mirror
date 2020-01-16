/// \copyright Copyright 2017-2019 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file covers simple constants and misc functions

#ifndef HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_

#include <cmath>

#include "common/types.hpp"

namespace autoware
{
namespace common
{
/// \brief Common constants and functions that may be used
///        throughout the codebase.
namespace helper_functions
{
/// \brief th_deg - phi_deg, normalized to +/- 180 deg
/// \param[in] th_deg the reference angle
/// \param[in] phi_deg the test angle
/// \return angle from reference angle to test angle
inline float32_t angle_distance_deg(const float32_t th_deg, const float32_t phi_deg)
{
  return fmodf((th_deg - phi_deg) + 540.0F, 360.0F) - 180.0F;
}

/// \brief converts a radian value to a degree value
/// \param[in] rad_val the radian value to convert
/// \return the radian value in degrees
inline float32_t rad2deg(const float32_t rad_val)
{
  return rad_val * 57.2958F;
}
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware


#endif  // HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_
