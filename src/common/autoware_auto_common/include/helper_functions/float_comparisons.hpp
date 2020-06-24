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

#ifndef HELPER_FUNCTIONS__FLOAT_COMPARISONS_HPP_
#define HELPER_FUNCTIONS__FLOAT_COMPARISONS_HPP_

#include "helper_functions/absolute_float_comparisons.hpp"
#include "helper_functions/relative_float_comparisons.hpp"

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace comparisons
{

/**
 * @brief Check for approximate equality in absolute and relative terms.
 *
 * @note This method should be used only if an explicit relative or absolute
 * comparison is not appropriate for the particular use case.
 *
 * @pre abs_eps >= 0
 * @pre rel_eps >= 0
 * @return True iff 'a' and 'b' are within 'eps' or 'rel_eps' of each other
 */
template<typename T>
bool approx_eq(const T & a, const T & b, const T & abs_eps, const T & rel_eps)
{
  const auto are_absolute_eq = abs_eq(a, b, abs_eps);
  const auto are_relative_eq = rel_eq(a, b, rel_eps);
  return are_absolute_eq || are_relative_eq;
}

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__FLOAT_COMPARISONS_HPP_
