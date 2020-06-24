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

#ifndef HELPER_FUNCTIONS__RELATIVE_FLOAT_COMPARISONS_HPP_
#define HELPER_FUNCTIONS__RELATIVE_FLOAT_COMPARISONS_HPP_

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
 * @brief
 * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * @pre rel_eps >= 0
 * @return True iff 'a' and 'b' are within relative 'rel_eps' of each other.
 */
template<typename T>
bool rel_eq(const T & a, const T & b, const T & rel_eps)
{
  static_assert(std::is_floating_point<T>::value,
    "Float comparisons only support floating point types.");

  const auto delta = std::abs(a - b);
  const auto larger = std::max(std::abs(a), std::abs(b));
  const auto max_rel_delta = (larger * rel_eps);
  return delta <= max_rel_delta;
}

// TODO(jeff): As needed, add relative variants of <, <=, >, >=

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__RELATIVE_FLOAT_COMPARISONS_HPP_
