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

#include <gtest/gtest.h>
#include <string>

#include "boost/optional.hpp"

#include "helper_functions/comparisons.hpp"

// cppcheck does not like gtest macros inside of namespaces:
// https://sourceforge.net/p/cppcheck/discussion/general/thread/e68df47b/
// use a namespace alias instead of putting macros into the namespace
namespace geom = autoware::common::helper_functions;

namespace
{
const auto a = 1.317;
const auto b = 2.0;
const auto c = -5.2747;
const auto d = 0.0;
const auto e = -5.2747177;
const auto f = -5.2749;
const auto epsilon = 0.0001;
}  // namespace

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, exclusive_or) {
  EXPECT_TRUE(geom::exclusive_or(0, 1));
  EXPECT_TRUE(geom::exclusive_or(1, 0));
  EXPECT_FALSE(geom::exclusive_or(0, 0));
  EXPECT_FALSE(geom::exclusive_or(1, 1));

  EXPECT_TRUE(geom::exclusive_or(false, true));
  EXPECT_TRUE(geom::exclusive_or(true, false));
  EXPECT_FALSE(geom::exclusive_or(false, false));
  EXPECT_FALSE(geom::exclusive_or(true, true));

  const boost::optional<std::string> a_true = std::string("Hello, world!");
  const boost::optional<std::string> a_false = boost::none;

  EXPECT_TRUE(geom::exclusive_or(a_false, a_true));
  EXPECT_TRUE(geom::exclusive_or(a_true, a_false));
  EXPECT_FALSE(geom::exclusive_or(a_false, a_false));
  EXPECT_FALSE(geom::exclusive_or(a_true, a_true));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_zero) {
  EXPECT_TRUE(geom::approx_zero(d, epsilon));
  EXPECT_TRUE(geom::approx_zero(d + epsilon * epsilon, epsilon));
  EXPECT_FALSE(geom::approx_zero(d + 2.0 * epsilon, epsilon));
  EXPECT_FALSE(geom::approx_zero(1.0, epsilon));
  EXPECT_TRUE(geom::approx_zero(0.0, 0.0));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_rel_eq) {
  EXPECT_TRUE(geom::approx_rel_eq(c, e, epsilon, 0.0));
  EXPECT_TRUE(geom::approx_rel_eq(e, c, epsilon, 0.0));
  EXPECT_TRUE(geom::approx_rel_eq(a, a, epsilon, 0.0));
  EXPECT_TRUE(geom::approx_rel_eq(a, a, 0.0, 0.0));

  EXPECT_FALSE(geom::approx_rel_eq(c, e, 0.0, 0.0));
  EXPECT_FALSE(geom::approx_rel_eq(e, c, 0.0, 0.0));
  EXPECT_FALSE(geom::approx_rel_eq(a, b, epsilon, 0.0));
  EXPECT_FALSE(geom::approx_rel_eq(b, a, epsilon, 0.0));

  EXPECT_TRUE(geom::approx_rel_eq(c, e, 0.0, 1.0));
  EXPECT_TRUE(geom::approx_rel_eq(e, c, 0.0, 1.0));
  EXPECT_TRUE(geom::approx_rel_eq(a, b, epsilon, 1.0));
  EXPECT_TRUE(geom::approx_rel_eq(b, a, epsilon, 1.0));

  EXPECT_TRUE(geom::approx_rel_eq(1.0, 1.1, 0.2, 0.01));
  EXPECT_TRUE(geom::approx_rel_eq(10000.0, 10010.0, 0.2, 0.01));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_eq) {
  EXPECT_TRUE(geom::approx_eq(c, e, epsilon));
  EXPECT_TRUE(geom::approx_eq(e, c, epsilon));
  EXPECT_FALSE(geom::approx_eq(c, e, 0.0));
  EXPECT_FALSE(geom::approx_eq(e, c, 0.0));
  EXPECT_FALSE(geom::approx_eq(a, b, epsilon));
  EXPECT_FALSE(geom::approx_eq(b, a, epsilon));
  EXPECT_TRUE(geom::approx_eq(a, a, epsilon));
  EXPECT_TRUE(geom::approx_eq(a, a, 0.0));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_ne) {
  EXPECT_FALSE(geom::approx_ne(c, e, epsilon));
  EXPECT_FALSE(geom::approx_ne(e, c, epsilon));
  EXPECT_TRUE(geom::approx_ne(c, e, 0.0));
  EXPECT_TRUE(geom::approx_ne(e, c, 0.0));
  EXPECT_TRUE(geom::approx_ne(a, b, epsilon));
  EXPECT_TRUE(geom::approx_ne(b, a, epsilon));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_lt) {
  EXPECT_TRUE(geom::approx_lt(f, c, 0.0));
  EXPECT_TRUE(geom::approx_lt(f, c, epsilon));
  EXPECT_FALSE(geom::approx_lt(c, f, epsilon));
  EXPECT_FALSE(geom::approx_lt(d, d, epsilon));
  EXPECT_FALSE(geom::approx_lt(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_le) {
  EXPECT_TRUE(geom::approx_le(c, e, epsilon));
  EXPECT_TRUE(geom::approx_le(e, c, epsilon));
  EXPECT_FALSE(geom::approx_le(c, e, 0.0));
  EXPECT_TRUE(geom::approx_le(e, c, 0.0));
  EXPECT_TRUE(geom::approx_le(c, e, epsilon));
  EXPECT_TRUE(geom::approx_le(e, c, epsilon));
  EXPECT_TRUE(geom::approx_le(a, b, epsilon));
  EXPECT_FALSE(geom::approx_le(b, a, epsilon));
  EXPECT_TRUE(geom::approx_le(d, d, epsilon));
  EXPECT_TRUE(geom::approx_le(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_gt) {
  EXPECT_TRUE(geom::approx_gt(c, e, 0.0));
  EXPECT_FALSE(geom::approx_gt(c, e, epsilon));
  EXPECT_FALSE(geom::approx_gt(f, c, epsilon));
  EXPECT_TRUE(geom::approx_gt(c, f, epsilon));
  EXPECT_FALSE(geom::approx_gt(d, d, epsilon));
  EXPECT_FALSE(geom::approx_gt(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(HelperFunctions_Comparisons, approx_ge) {
  EXPECT_TRUE(geom::approx_ge(c, e, 0.0));
  EXPECT_FALSE(geom::approx_ge(e, c, 0.0));
  EXPECT_TRUE(geom::approx_ge(c, e, epsilon));
  EXPECT_TRUE(geom::approx_ge(e, c, epsilon));
  EXPECT_FALSE(geom::approx_ge(f, c, epsilon));
  EXPECT_TRUE(geom::approx_ge(c, f, epsilon));
  EXPECT_TRUE(geom::approx_ge(d, d, epsilon));
  EXPECT_TRUE(geom::approx_ge(d, d, 0.0));
}

//------------------------------------------------------------------------------
