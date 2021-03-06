// Copyright 2018 the Autoware Foundation
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

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "gtest/gtest.h"
#include "xsens_driver/xsens_imu_translator.hpp"
#include "xsens_driver/test_xsens_common.hpp"

using autoware::drivers::xsens_driver::XsensImuTranslator;

using XsensDriver = xsens_driver_common<XsensImuTranslator, sensor_msgs::msg::Imu>;

TEST_F(XsensDriver, Basic)
{
  std::vector<uint8_t> data = {
    0x70, 0x20, 0x78, 0x1C, 0x10, 0x5C, 0x4A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x01, 0x19, 0x5C, 0x00,
    0x05, 0x00, 0x11, 0x00, 0x07, 0x1E, 0x5E, 0x00, 0x08, 0x0D, 0x14, 0x00, 0x0B, 0x21, 0x5F, 0x00,
    0x0D, 0x2B, 0x5F, 0x00, 0x0F, 0x1A, 0x54, 0x00, 0x11, 0x24, 0x5F, 0x00, 0x12, 0x0B, 0x5C, 0x00,
    0x13, 0x1C, 0x5F, 0x00, 0x1C, 0x21, 0x5F, 0x00, 0x1E, 0x24, 0x5F, 0x01, 0x83, 0x25, 0x16, 0x01,
    0x85, 0x23, 0x16, 0x01, 0x8A, 0x1E, 0x16, 0x05, 0x01, 0x18, 0x1C, 0x05, 0x04, 0x00, 0x21, 0x05,
    0x05, 0x00, 0x21, 0x06, 0x05, 0x1A, 0x1D, 0x06, 0x06, 0x16, 0x14, 0x06, 0x0E, 0x14, 0x1C, 0x06,
    0x0F, 0x1E, 0x1F, 0x06, 0x10, 0x00, 0x10, 0x06, 0x11, 0x14, 0x14, 0x06, 0x12, 0x00, 0x10, 0x06,
    0x17, 0x0A, 0x14, 0x06, 0x18, 0x16, 0x1C, 0x06, 0x1E, 0x00, 0x10, 0x10, 0x60, 0x04, 0x22, 0xD5,
    0x58, 0x97, 0x24
  };

  xsens_driver_common_test(data);
}

int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
