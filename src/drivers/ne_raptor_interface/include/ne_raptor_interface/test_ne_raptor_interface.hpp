// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_

#include <gtest/gtest.h>

#include <ne_raptor_interface/ne_raptor_interface.hpp>
#include <rclcpp/clock.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>

using autoware::ne_raptor_interface::NERaptorInterface;
using autoware::drivers::vehicle_interface::DbwStateMachine;
using autoware::drivers::vehicle_interface::DbwState;

class NERaptorInterface_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_test_node", "/gtest");
    ne_raptor_interface_ = std::make_unique<autoware::ne_raptor_interface::NERaptorInterface>(
      *node_, 0xABCD, 1.0F, 1.0F, 2.0F, 3.0F, 3.0F, 9.0F, 9.0F);
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<autoware::ne_raptor_interface::NERaptorInterface> ne_raptor_interface_;
};

template<typename T>
void wait_for_subscriber(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for subscriber");
    }
  }
}


template<typename T>
void wait_for_publisher(
  const T & sub_ptr,
  const uint32_t num_expected_pubs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (sub_ptr->get_publisher_count() < num_expected_pubs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for publisher");
    }
  }
}
#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_HPP_
