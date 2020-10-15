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

#define _USE_MATH_DEFINES
#include <cmath>

#include "autoware_auto_contracts/acute_degree.hpp"
#include "autoware_auto_contracts/acute_radian.hpp"
#include "gtest/gtest.h"

namespace c = autoware::common::contracts;
using autoware::common::types::FEPS;
static constexpr auto quarter_turn_rad = static_cast<float>(M_PI) / 4.0f;

//------------------------------------------------------------------------------

TEST(DimensionalAnalysis, AcuteDegreef) {
    const c::AcuteDegreef d{45.0f};
    EXPECT_NEAR(c::AcuteRadianf{d}, quarter_turn_rad, FEPS);

    const c::AcuteRadianf r = d;
    EXPECT_NEAR(r, quarter_turn_rad, FEPS);
}

//------------------------------------------------------------------------------

TEST(DimensionalAnalysis, AcuteRadianf) {
    const c::AcuteRadianf r{quarter_turn_rad};
    ASSERT_EQ(r, quarter_turn_rad);
    EXPECT_NEAR(c::AcuteDegreef{r}, 45.0f, FEPS);

    const c::AcuteDegreef d = r;
    EXPECT_NEAR(d, 45.0f, FEPS);
}

//------------------------------------------------------------------------------
