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
#include <ne_raptor_interface/test_ne_raptor_interface_talker.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>

using autoware::ne_raptor_interface::NERaptorInterface;
using autoware::ne_raptor_interface::NERaptorInterfaceListener;
using autoware::ne_raptor_interface::NERaptorInterfaceTalker;
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

/* Other useful constants */
#define kNumTests_VSC  (15)
#define kNumTests_HLCC (2)
#define kNumTests_VCC  (2)
using namespace std::literals::chrono_literals; //NOLINT
const std::chrono::nanoseconds C_TIMEOUT_NANO = 1000000000ns;
const uint8_t C_TIMEOUT_ITERATIONS = 10;

class NERaptorInterface_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    i_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_test_node", "/gtest");
    l_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_listener_node", "/gtest");
    t_node_ = std::make_shared<rclcpp::Node>("ne_raptor_interface_talker_node", "/gtest");
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
    test_talker_ = std::make_unique<NERaptorInterfaceTalker>(
      *t_node_
    );
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr i_node_, l_node_, t_node_;
  std::unique_ptr<NERaptorInterface> ne_raptor_interface_;
  std::unique_ptr<NERaptorInterfaceListener> test_listener_;
  std::unique_ptr<NERaptorInterfaceTalker> test_talker_;
  rclcpp::Clock test_clock{RCL_SYSTEM_TIME};

  // Struct types for test sets
  struct test_vsc
  {
    VehicleStateCommand in_vsc;  // Input: vehicle state command
    GearCmd exp_gc;              // Expected output: gear command
    GlobalEnableCmd exp_enable;  // Expected output: global enable command
    MiscCmd exp_mc;              // Expected output: misc command
    bool8_t exp_success;         // Expected output: send_state_command
  };
  struct test_hlcc
  {
    HighLevelControlCommand in_hlcc;  // Input: high level control command
    VehicleStateCommand in_vsc;       // Input: vehicle state command (set current gear)
    AcceleratorPedalCmd exp_apc;      // Expected output: accelerator pedal command
    BrakeCmd exp_bc;                  // Expected output: brake command
    SteeringCmd exp_sc;               // Expected output: steering command
  };
  struct test_vcc
  {
    VehicleControlCommand in_vcc;  // Input: vehicle control command
    VehicleStateCommand in_vsc;    // Input: vehicle state command (set current gear)
    AcceleratorPedalCmd exp_apc;   // Expected output: accelerator pedal command
    BrakeCmd exp_bc;               // Expected output: brake command
    SteeringCmd exp_sc;            // Expected output: steering command
  };
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
