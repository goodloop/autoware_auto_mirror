// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <common/types.hpp>
#include <fake_test_node/fake_test_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;
using FakeNodeFixtureParametrized = autoware::tools::testing::FakeTestNodeParametrized<bool8_t>;
using std_msgs::msg::Bool;
using std_msgs::msg::Int32;

/// @test Test that we can use a non-parametrized test.
TEST_F(FakeNodeFixture, TestDuplicate) {
}

INSTANTIATE_TEST_CASE_P(
  FakeNodeFixtureTests,
  FakeNodeFixtureParametrized,
  // cppcheck-suppress syntaxError  // cppcheck doesn't like the trailing comma.
  ::testing::Values(-5, 0, 42), /*This comment needed to avoid a warning*/);

/// @test Test that we can use a parametrized test.
TEST_P(FakeNodeFixtureParametrized, TestDuplicate) {
}
