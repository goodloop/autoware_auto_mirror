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

#include <limits>

#include "autoware_auto_contracts/real.hpp"
#include "gtest/gtest.h"

namespace c = autoware::common::contracts;

static constexpr auto NaNf = std::numeric_limits<float>::quiet_NaN();
static constexpr auto INFf = std::numeric_limits<float>::infinity();

//------------------------------------------------------------------------------

TEST(Contract, Realf) {
  EXPECT_THROW({c::Realf{NaNf};}, std::runtime_error);
  EXPECT_THROW({c::Realf{INFf};}, std::runtime_error);
  EXPECT_THROW({c::Realf{-NaNf};}, std::runtime_error);
  EXPECT_THROW({c::Realf{-INFf};}, std::runtime_error);
  EXPECT_THROW(
  {
    c::Realf r{0.0f};
    r = NaNf;
  }, std::runtime_error);
  EXPECT_THROW(
  {
    c::Realf r{0.0f};
    r = INFf;
  }, std::runtime_error);
  EXPECT_THROW(
  {
    c::Realf r{0.0f};
    r = r + INFf;
  }, std::runtime_error);
  EXPECT_NO_THROW(
  {
    {c::Realf{0.0f};}
    {
      c::Realf r{0.0f};
      r = 1.0f;
      float tmp = r;
      r = tmp + 1.0f;
    }
  });
}

//------------------------------------------------------------------------------
