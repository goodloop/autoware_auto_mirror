// Copyright 2020 Mapless AI, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef HELPER_FUNCTIONS__ABSOLUTE_FLOAT_COMPARISONS_HPP_
#define HELPER_FUNCTIONS__ABSOLUTE_FLOAT_COMPARISONS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace comparisons
{

/**
 * @brief Check for approximate equality in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' and 'b' are within 'eps' of each other.
 */
template<typename T>
bool abs_eq(const T & a, const T & b, const T & eps)
{
  static_assert(std::is_floating_point<T>::value,
    "Float comparisons only support floating point types.");

  return std::abs(a - b) <= eps;
}

/**
 * @brief Check for approximate less than in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is less than 'b' minus 'eps'.
 */
template<typename T>
bool abs_lt(const T & a, const T & b, const T & eps)
{
  return !abs_eq(a, b, eps) && (a < b);
}

/**
 * @brief Check for approximate less than or equal in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is less than or equal to 'b' plus 'eps'.
 */
template<typename T>
bool abs_lte(const T & a, const T & b, const T & eps)
{
  return abs_eq(a, b, eps) || (a < b);
}

/**
 * @brief Check for approximate greater than or equal in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is greater than or equal to 'b' minus 'eps'.
 */
template<typename T>
bool abs_gte(const T & a, const T & b, const T & eps)
{
  return !abs_lt(a, b, eps);
}

/**
 * @brief Check for approximate greater than in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is greater than 'b' minus 'eps'.
 */
template<typename T>
bool abs_gt(const T & a, const T & b, const T & eps)
{
  return !abs_lte(a, b, eps);
}

/**
 * @brief Check whether a value is within epsilon of zero.
 * @pre eps >= 0
 * @return True iff 'a' is within 'eps' of zero.
 */
template<typename T>
bool abs_eq_zero(const T & a, const T & eps)
{
  return abs_eq(a, static_cast<T>(0), eps);
}

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__ABSOLUTE_FLOAT_COMPARISONS_HPP_
