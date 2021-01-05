// Copyright 2020 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <experimental/optional>
#include <gtest/gtest.h>
#include <joystick_vehicle_interface/joystick_vehicle_interface_node.hpp>
#include <common/types.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using joystick_vehicle_interface::Axes;
using joystick_vehicle_interface::Buttons;
using joystick_vehicle_interface::JoystickVehicleInterfaceNode;
using autoware::common::types::bool8_t;

enum class PubType
{
  Raw,
  Basic,
  HighLevel
};

struct JoyMapping
{
  PubType pub_type;
  JoystickVehicleInterfaceNode::AxisMap axis_map;
  JoystickVehicleInterfaceNode::AxisScaleMap axis_scale_map;
  JoystickVehicleInterfaceNode::AxisScaleMap axis_offset_map;
  JoystickVehicleInterfaceNode::ButtonMap button_map;
};  // struct JoyMapping

class joy_vi_test : public ::testing::TestWithParam<JoyMapping>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }
};  // class joy_vi_test

template<typename T>
struct SubAndMsg
{
  typename rclcpp::Subscription<T>::SharedPtr sub_{nullptr};
  std::experimental::optional<T> msg_{};
  SubAndMsg(rclcpp::Node & nd, const std::string & topic)
  {
    if ("null" != topic) {
      sub_ = nd.create_subscription<T>(
        topic, rclcpp::QoS{10U}.reliable().durability_volatile(),
        [this](std::shared_ptr<T> msg) {msg_ = *msg;});
    }
  }
};

TEST_P(joy_vi_test, basic_mapping)
{
  const auto param = GetParam();
  const std::string control_command =
    (PubType::HighLevel == param.pub_type) ? "high_level" :
    (PubType::Raw == param.pub_type) ? "raw" :
    (PubType::Basic == param.pub_type) ? "basic" : "null";
  constexpr auto state_command_topic = "test_state_command_topic";
  constexpr auto joy_topic = "test_joy_topic";
  constexpr auto recordreplay_command_topic = "recordreplay_cmd";
  const bool recordreplay_command_enabled = true;

  const auto test_nd = std::make_shared<rclcpp::Node>("test_joystick_vehicle_interface_talker");
  const auto qos = rclcpp::SensorDataQoS{};
  const auto joy_pub = test_nd->create_publisher<sensor_msgs::msg::Joy>(joy_topic, qos);
  SubAndMsg<autoware_auto_msgs::msg::RawControlCommand>
  raw{*test_nd, (control_command == "raw") ? "raw_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::HighLevelControlCommand>
  high_level{*test_nd, (control_command == "high_level") ? "high_level_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::VehicleControlCommand>
  basic{*test_nd, (control_command == "basic") ? "basic_command" : "null"};
  SubAndMsg<autoware_auto_msgs::msg::VehicleStateCommand> state{*test_nd, state_command_topic};
  SubAndMsg<std_msgs::msg::UInt8> recordreplay{*test_nd, recordreplay_command_topic};

  ASSERT_NE(state.sub_, nullptr);
  switch (param.pub_type) {
    case PubType::Raw:
      ASSERT_NE(raw.sub_, nullptr);
      break;
    case PubType::Basic:
      ASSERT_NE(basic.sub_, nullptr);
      break;
    case PubType::HighLevel:
      ASSERT_NE(high_level.sub_, nullptr);
      break;
  }

  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("control_command", control_command);
  params.emplace_back("state_command_topic", state_command_topic);
  params.emplace_back("joy_topic", joy_topic);
  params.emplace_back("recordreplay_command_enabled", recordreplay_command_enabled);

  params.emplace_back("axes.throttle", static_cast<uint8_t>(param.axis_map.at(Axes::THROTTLE)));
  params.emplace_back("axes.brake", static_cast<uint8_t>(param.axis_map.at(Axes::BRAKE)));
  params.emplace_back("axes.front_steer",
    static_cast<uint8_t>(param.axis_map.at(Axes::FRONT_STEER)));
  params.emplace_back("axes.rear_steer", static_cast<uint8_t>(param.axis_map.at(Axes::REAR_STEER)));
  params.emplace_back("axes.curvature", static_cast<uint8_t>(param.axis_map.at(Axes::CURVATURE)));
  params.emplace_back("axes.acceleration",
    static_cast<uint8_t>(param.axis_map.at(Axes::ACCELERATION)));

  params.emplace_back("axis_scale.throttle", param.axis_scale_map.at(Axes::THROTTLE));
  params.emplace_back("axis_scale.brake", param.axis_scale_map.at(Axes::BRAKE));
  params.emplace_back("axis_scale.front_steer", param.axis_scale_map.at(Axes::FRONT_STEER));
  params.emplace_back("axis_scale.rear_steer", param.axis_scale_map.at(Axes::REAR_STEER));
  params.emplace_back("axis_scale.curvature", param.axis_scale_map.at(Axes::CURVATURE));
  params.emplace_back("axis_scale.acceleration", param.axis_scale_map.at(Axes::ACCELERATION));

  params.emplace_back("axis_offset.throttle", param.axis_offset_map.at(Axes::THROTTLE));
  params.emplace_back("axis_offset.brake", param.axis_offset_map.at(Axes::BRAKE));
  params.emplace_back("axis_offset.front_steer", param.axis_offset_map.at(Axes::FRONT_STEER));
  params.emplace_back("axis_offset.rear_steer", param.axis_offset_map.at(Axes::REAR_STEER));
  params.emplace_back("axis_offset.curvature", param.axis_offset_map.at(Axes::CURVATURE));
  params.emplace_back("axis_offset.acceleration", param.axis_offset_map.at(Axes::ACCELERATION));

  params.emplace_back("buttons.autonomous",
    static_cast<uint8_t>(param.button_map.at(Buttons::AUTONOMOUS_TOGGLE)));
  params.emplace_back("buttons.headlights",
    static_cast<uint8_t>(param.button_map.at(Buttons::HEADLIGHTS_TOGGLE)));
  params.emplace_back("buttons.wiper",
    static_cast<uint8_t>(param.button_map.at(Buttons::WIPER_TOGGLE)));
  params.emplace_back("buttons.gear_drive",
    static_cast<uint8_t>(param.button_map.at(Buttons::GEAR_DRIVE)));
  params.emplace_back("buttons.gear_reverse",
    static_cast<uint8_t>(param.button_map.at(Buttons::GEAR_REVERSE)));
  params.emplace_back("buttons.gear_park",
    static_cast<uint8_t>(param.button_map.at(Buttons::GEAR_PARK)));
  params.emplace_back("buttons.gear_neutral",
    static_cast<uint8_t>(param.button_map.at(Buttons::GEAR_NEUTRAL)));
  params.emplace_back("buttons.gear_low",
    static_cast<uint8_t>(param.button_map.at(Buttons::GEAR_LOW)));
  params.emplace_back("buttons.blinker_left",
    static_cast<uint8_t>(param.button_map.at(Buttons::BLINKER_LEFT)));
  params.emplace_back("buttons.blinker_right",
    static_cast<uint8_t>(param.button_map.at(Buttons::BLINKER_RIGHT)));
  params.emplace_back("buttons.blinker_hazard",
    static_cast<uint8_t>(param.button_map.at(Buttons::BLINKER_HAZARD)));
  params.emplace_back("buttons.velocity_up",
    static_cast<uint8_t>(param.button_map.at(Buttons::VELOCITY_UP)));
  params.emplace_back("buttons.velocity_down",
    static_cast<uint8_t>(param.button_map.at(Buttons::VELOCITY_DOWN)));
  params.emplace_back("buttons.recordreplay_start_record",
    static_cast<uint8_t>(param.button_map.at(Buttons::RECORDREPLAY_START_RECORD)));
  params.emplace_back("buttons.recordreplay_start_replay",
    static_cast<uint8_t>(param.button_map.at(Buttons::RECORDREPLAY_START_REPLAY)));
  params.emplace_back("buttons.recordreplay_stop",
    static_cast<uint8_t>(param.button_map.at(Buttons::RECORDREPLAY_STOP)));

  node_options.parameter_overrides(params);

  const auto nd = std::make_shared<JoystickVehicleInterfaceNode>(
    node_options);
  // "test_joystick_vehicle_interface",
  // "",
  // control_command,
  // state_command_topic,
  // joy_topic,
  // recordreplay_command_enabled,
  // param.axis_map,
  // param.axis_scale_map,
  // param.axis_offset_map,
  // param.button_map);

  // make joy
  sensor_msgs::msg::Joy joy_msg{};
  joy_msg.axes = {-0.1F, -0.2F, -0.3F, -0.4F, -0.5F};
  joy_msg.buttons = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
  const auto timer = test_nd->create_wall_timer(
    std::chrono::milliseconds{1LL},
    [&joy_msg, &joy_pub]() {joy_pub->publish(joy_msg);});
  // Execute
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(test_nd);
  exec.add_node(nd);
  while ((!raw.msg_) && (!basic.msg_) && (!high_level.msg_)) {
    exec.spin_some(std::chrono::milliseconds{1LL});
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }
  // check that you got stuff
  {
    if (raw.sub_) {
      EXPECT_TRUE(raw.msg_);
    }
    if (basic.sub_) {
      EXPECT_TRUE(basic.msg_);
    }
    if (high_level.sub_) {
      EXPECT_TRUE(high_level.msg_);
    }
    if (HasFailure()) {FAIL();}
  }
  // Helper
  const auto axis_check_fn = [ = ](Axes axis, auto value) -> bool8_t {
      const auto it = param.axis_map.find(axis);
      if ((param.axis_map.end() != it) && (it->second < joy_msg.axes.size())) {
        const auto scale_it = param.axis_scale_map.find(axis);
        const auto scale = param.axis_scale_map.end() == scale_it ?
          JoystickVehicleInterfaceNode::DEFAULT_SCALE : scale_it->second;
        const auto offset_it = param.axis_offset_map.find(axis);
        const auto offset = param.axis_offset_map.end() == offset_it ?
          JoystickVehicleInterfaceNode::DEFAULT_OFFSET : offset_it->second;
        using ValT = decltype(value);
        const auto expect_val =
          static_cast<ValT>((scale * joy_msg.axes[it->second]) + offset);
        if (std::is_floating_point<ValT>::value) {
          return std::fabs(value - expect_val) < std::numeric_limits<ValT>::epsilon();
        } else {
          return value == expect_val;
        }
      }
      return true;
    };
  // Check raw message
  if (raw.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, raw.msg_->stamp);
    constexpr auto BIG_NUM = 999999;  // Mostly checking for unsigned underflow
    // Throttle
    EXPECT_TRUE(axis_check_fn(Axes::THROTTLE, raw.msg_->throttle));
    EXPECT_LT(raw.msg_->throttle, static_cast<decltype(raw.msg_->throttle)>(BIG_NUM));
    // Brake
    EXPECT_TRUE(axis_check_fn(Axes::BRAKE, raw.msg_->brake));
    EXPECT_LT(raw.msg_->brake, static_cast<decltype(raw.msg_->brake)>(BIG_NUM));
    // Front steer
    EXPECT_TRUE(axis_check_fn(Axes::FRONT_STEER, raw.msg_->front_steer));
    // Rear steer
    EXPECT_TRUE(axis_check_fn(Axes::REAR_STEER, raw.msg_->rear_steer));
  }
  // Check basic message
  if (basic.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, basic.msg_->stamp);
    // Front steer
    EXPECT_TRUE(axis_check_fn(Axes::FRONT_STEER, basic.msg_->front_wheel_angle_rad));
    // Rear steer
    EXPECT_TRUE(axis_check_fn(Axes::REAR_STEER, basic.msg_->rear_wheel_angle_rad));
    // Acceleration
    EXPECT_TRUE(axis_check_fn(Axes::ACCELERATION, basic.msg_->long_accel_mps2));
  }
  // Button helper
  const auto button_check_fn = [ = ](Buttons button) -> auto {
      const auto it = param.button_map.find(button);
      if ((param.button_map.end() != it) && (it->second < joy_msg.buttons.size())) {
        return 1 == joy_msg.buttons[it->second];
      }
      return false;
    };
  // check high level message
  if (high_level.msg_) {
    // Exactly one of the high level buttons should be on.. otherwise there's no point in this test
    ASSERT_TRUE(button_check_fn(Buttons::VELOCITY_UP) ^ (button_check_fn(Buttons::VELOCITY_DOWN)));
    // Check sign
    const auto velocity = high_level.msg_->velocity_mps;
    if (button_check_fn(Buttons::VELOCITY_UP)) {
      EXPECT_GT(velocity, 0.0F);
    } else {
      EXPECT_LT(velocity, 0.0F);
    }
    // Must be modulo the increment
    EXPECT_LT(
      std::fabs(std::fmod(velocity, JoystickVehicleInterfaceNode::VELOCITY_INCREMENT)),
      std::numeric_limits<decltype(velocity)>::epsilon());
    // Curvature
    EXPECT_TRUE(axis_check_fn(Axes::CURVATURE, high_level.msg_->curvature));
  }
  // Check state message
  if (state.msg_) {
    EXPECT_EQ(joy_msg.header.stamp, state.msg_->stamp);
    // It's all buttons here
    using VSC = autoware_auto_msgs::msg::VehicleStateCommand;
    // Toggle buttons depend on state
    // Since joy message is always the same, these buttons can be in one of two states
    const auto toggle_case1 =
      (!button_check_fn(Buttons::HORN_TOGGLE) || state.msg_->horn) &&
      (!button_check_fn(Buttons::HAND_BRAKE_TOGGLE) || state.msg_->hand_brake) &&
      (button_check_fn(Buttons::AUTONOMOUS_TOGGLE) == (state.msg_->mode == VSC::MODE_AUTONOMOUS)) &&
      (button_check_fn(Buttons::HEADLIGHTS_TOGGLE) ==
      (state.msg_->headlight == VSC::HEADLIGHT_ON)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == VSC::WIPER_LOW));
    const auto toggle_case2 =
      (!button_check_fn(Buttons::HORN_TOGGLE) || !state.msg_->horn) &&
      (!button_check_fn(Buttons::HAND_BRAKE_TOGGLE) || !state.msg_->hand_brake) &&
      (button_check_fn(Buttons::AUTONOMOUS_TOGGLE) == (state.msg_->mode == VSC::MODE_MANUAL)) &&
      (button_check_fn(Buttons::HEADLIGHTS_TOGGLE) ==
      (state.msg_->headlight == VSC::HEADLIGHT_OFF)) &&
      (button_check_fn(Buttons::WIPER_TOGGLE) == (state.msg_->wiper == VSC::WIPER_OFF));
    if (toggle_case1 || toggle_case2) {
      EXPECT_TRUE(true);  // Pushed logic into conditional for printing purposes
    } else {
      std::cerr << "Fail with toggle case:\n";
      const auto err_print =
        [ = ](auto name, auto val, auto expected_val1, auto expected_val2, auto button) {
          if (button_check_fn(button)) {
            std::cerr << name << " = " << static_cast<int32_t>(val) << ", expected " <<
              static_cast<int32_t>(expected_val1) << ", or " <<
              static_cast<int32_t>(expected_val2) << "\n";
          }
        };
      err_print("Horn", state.msg_->horn, true, false, Buttons::HORN_TOGGLE);
      err_print("hand_brake", state.msg_->hand_brake, true, false, Buttons::HAND_BRAKE_TOGGLE);
      err_print(
        "mode", state.msg_->mode, VSC::MODE_AUTONOMOUS, VSC::MODE_MANUAL,
        Buttons::AUTONOMOUS_TOGGLE);
      err_print(
        "headlight", state.msg_->headlight, VSC::HEADLIGHT_ON,
        VSC::HEADLIGHT_OFF, Buttons::HEADLIGHTS_TOGGLE);
      err_print("wiper", state.msg_->wiper, VSC::WIPER_LOW, VSC::WIPER_OFF, Buttons::WIPER_TOGGLE);
      EXPECT_TRUE(false);
    }
    // Easy checks w/o state
    EXPECT_EQ(button_check_fn(Buttons::GEAR_DRIVE), state.msg_->gear == VSC::GEAR_DRIVE);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_REVERSE), state.msg_->gear == VSC::GEAR_REVERSE);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_NEUTRAL), state.msg_->gear == VSC::GEAR_NEUTRAL);
    EXPECT_EQ(button_check_fn(Buttons::GEAR_PARK), state.msg_->gear == VSC::GEAR_PARK);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_LEFT), state.msg_->blinker == VSC::BLINKER_LEFT);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_RIGHT), state.msg_->blinker == VSC::BLINKER_RIGHT);
    EXPECT_EQ(button_check_fn(Buttons::BLINKER_HAZARD), state.msg_->blinker == VSC::BLINKER_HAZARD);
  }

  // Record Replay
  if (recordreplay.msg_) {
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_START_RECORD), recordreplay.msg_->data == 1u);
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_START_REPLAY), recordreplay.msg_->data == 2u);
    EXPECT_EQ(button_check_fn(Buttons::RECORDREPLAY_STOP), recordreplay.msg_->data == 3u);
    EXPECT_EQ(
      !button_check_fn(Buttons::RECORDREPLAY_START_RECORD) &&
      !button_check_fn(Buttons::RECORDREPLAY_START_REPLAY) &&
      !button_check_fn(Buttons::RECORDREPLAY_STOP), recordreplay.msg_->data == 0u);
  }
}

INSTANTIATE_TEST_CASE_P(
  test,
  joy_vi_test,
  ::testing::Values(
    // Raw control command
    JoyMapping{
  PubType::Raw,
  {{Axes::THROTTLE, 0U}, {Axes::BRAKE, 1U}, {Axes::FRONT_STEER, 2U}},
  {{Axes::THROTTLE, -1.0F}, {Axes::BRAKE, 2.0F}, {Axes::FRONT_STEER, -3.0F}},
  {{Axes::THROTTLE, 2.0F}, {Axes::BRAKE, 5.0F}, {Axes::FRONT_STEER, -10.0F}},
  {}
},
    // Basic control command
    JoyMapping{
  PubType::Basic,
  {{Axes::ACCELERATION, 3U}, {Axes::FRONT_STEER, 0U}, {Axes::REAR_STEER, 4U}},
  {{Axes::ACCELERATION, 12.0F}, {Axes::FRONT_STEER, -3.55F}, {Axes::REAR_STEER, 5.0F}},
  {{Axes::ACCELERATION, -1.1F}, {Axes::FRONT_STEER, 5.2F}, {Axes::REAR_STEER, -2.3F}},
  {}
},
    // State
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::AUTONOMOUS_TOGGLE, 0U}, {Buttons::HEADLIGHTS_TOGGLE, 1U}, {Buttons::WIPER_TOGGLE, 2U},
    {Buttons::GEAR_DRIVE, 3U}, {Buttons::BLINKER_LEFT, 4U}}
},
    // High level
    JoyMapping{
  PubType::HighLevel,
  {{Axes::CURVATURE, 0U}},
  {{Axes::CURVATURE, 2.5F}},
  {{Axes::CURVATURE, 0.4F}},
  {{Buttons::VELOCITY_UP, 0U}, {Buttons::VELOCITY_DOWN, 1U}}
},
    // RecordReplay
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_START_RECORD, 1U}}
},
    JoyMapping{
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_START_REPLAY, 1U}}
},
    JoyMapping {
  PubType::Basic,
  {},
  {},
  {},
  {{Buttons::RECORDREPLAY_STOP, 1U}}
}
));
