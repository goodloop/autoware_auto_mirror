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

#include "helper_functions/float_comparisons.hpp"

// cppcheck does not like gtest macros inside of namespaces:
// https://sourceforge.net/p/cppcheck/discussion/general/thread/e68df47b/
// use a namespace alias instead of putting macros into the namespace
namespace comp = autoware::common::helper_functions::comparisons;

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

TEST(HelperFunctions_Comparisons, approx_eq) {
  EXPECT_TRUE(comp::approx_eq(c, e, epsilon, 0.0));
  EXPECT_TRUE(comp::approx_eq(e, c, epsilon, 0.0));
  EXPECT_TRUE(comp::approx_eq(a, a, epsilon, 0.0));
  EXPECT_TRUE(comp::approx_eq(a, a, 0.0, 0.0));

  EXPECT_FALSE(comp::approx_eq(c, e, 0.0, 0.0));
  EXPECT_FALSE(comp::approx_eq(e, c, 0.0, 0.0));
  EXPECT_FALSE(comp::approx_eq(a, b, epsilon, 0.0));
  EXPECT_FALSE(comp::approx_eq(b, a, epsilon, 0.0));

  EXPECT_TRUE(comp::approx_eq(c, e, 0.0, 1.0));
  EXPECT_TRUE(comp::approx_eq(e, c, 0.0, 1.0));
  EXPECT_TRUE(comp::approx_eq(a, b, epsilon, 1.0));
  EXPECT_TRUE(comp::approx_eq(b, a, epsilon, 1.0));

  EXPECT_TRUE(comp::approx_eq(1.0, 1.1, 0.2, 0.01));
  EXPECT_TRUE(comp::approx_eq(10000.0, 10010.0, 0.2, 0.01));
}

//------------------------------------------------------------------------------
