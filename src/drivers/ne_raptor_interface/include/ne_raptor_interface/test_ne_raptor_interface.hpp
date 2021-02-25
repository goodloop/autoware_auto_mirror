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
#include <ne_raptor_interface/test_ne_raptor_interface_listener.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>

using autoware::ne_raptor_interface::NERaptorInterface;
using autoware::ne_raptor_interface::NERaptorInterfaceListener;
using autoware::drivers::vehicle_interface::DbwStateMachine;
using autoware::drivers::vehicle_interface::DbwState;

/* Node init values */
const uint16_t c_ecu_build_num = 0xABCD;
const float32_t c_front_axle_to_cog = 1.5F;
const float32_t c_rear_axle_to_cog = 0.5F;
const float32_t c_steer_to_tire_ratio = 2.0F;
const float32_t c_accel_limit = 3.0F;
const float32_t c_decel_limit = 3.0F;
const float32_t c_pos_jerk_limit = 9.0F;
const float32_t c_neg_jerk_limit = 9.0F;

class NERaptorInterface_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    i_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_test_node", "/gtest");
    l_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_listener_node", "/gtest");
    ne_raptor_interface_ = std::make_unique<NERaptorInterface>(
      *i_node_,
      c_ecu_build_num,
      c_front_axle_to_cog,
      c_rear_axle_to_cog,
      c_steer_to_tire_ratio,
      c_accel_limit,
      c_decel_limit,
      c_pos_jerk_limit,
      c_neg_jerk_limit
    );
    test_listener_ = std::make_unique<NERaptorInterfaceListener>(
      *l_node_
    );
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr i_node_, l_node_;
  std::unique_ptr<NERaptorInterface> ne_raptor_interface_;
  std::unique_ptr<NERaptorInterfaceListener> test_listener_;
  rclcpp::Clock test_clock{RCL_SYSTEM_TIME};
};  // class NERaptorInterface_test

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
