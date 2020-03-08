// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef TEST_DRIVER_H_
#define TEST_DRIVER_H_

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

#include "common/types.hpp"
#include "velodyne_driver/vlp16_translator.hpp"
#include "gtest/gtest.h"
/// test uint16_t conversion using example from documentation
TEST(helpers, uint32)
{
  const uint8_t x = 0x71;
  const uint8_t y = 0x33;
  EXPECT_EQ(autoware::drivers::velodyne_driver::to_uint32(x, y), 28979U);
}

using autoware::common::types::float32_t;
using autoware::drivers::velodyne_driver::Vlp16Translator;
using autoware::drivers::velodyne_driver::make_point;

// This is a real packet captured. This was used originally in the deprecated velodyne_spoof package
// The points are about 6-25 m away, at an angle of about 1.4-1.5 rad
static const uint8_t static_packet[sizeof(Vlp16Translator::Packet)] =
{
  0xff, 0xee, 0x29, 0x6a, 0xc4, 0x0c, 0x1d, 0x00, 0x00, 0x01, 0x9c, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x98, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x71, 0x0c, 0x15, 0x00, 0x00, 0x03, 0xc2, 0x0d, 0x03, 0x00,
  0x00, 0x01, 0xe2, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02, 0xc1, 0x0c, 0x1b, 0x00, 0x00, 0x01, 0xa1, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x94, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x7b, 0x0c, 0x15, 0x00, 0x00, 0x03, 0xd6, 0x1b, 0x04, 0x00,
  0x00, 0x01, 0xb8, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02,
  0xff, 0xee, 0x51, 0x6a, 0xc0, 0x0c, 0x1b, 0x00, 0x00, 0x01, 0xa9, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x90, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x7b, 0x0c, 0x15, 0x00, 0x00, 0x03, 0xda, 0x0d, 0x03, 0x00,
  0x00, 0x01, 0xdf, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x6b, 0x0f, 0x02, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02, 0xc1, 0x0c, 0x1b, 0x00, 0x00, 0x01, 0xa9, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x97, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x7d, 0x0c, 0x13, 0x00, 0x00, 0x03, 0xf0, 0x1b, 0x04, 0x00,
  0x00, 0x01, 0xd5, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x56, 0x0f, 0x02, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02,
  0xff, 0xee, 0x7b, 0x6a, 0xc5, 0x0c, 0x1b, 0x00, 0x00, 0x01, 0xa8, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x9a, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x7b, 0x0c, 0x15, 0x00, 0x00, 0x03, 0xc3, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0xc9, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x81, 0x0f, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02, 0xcf, 0x0c, 0x18, 0x00, 0x00, 0x01, 0xa9, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x9a, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x7f, 0x0c, 0x13, 0x00, 0x00, 0x03, 0xaf, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xbc, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x5e, 0x0f, 0x08, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02,
  0xff, 0xee, 0xa1, 0x6a, 0xcf, 0x0c, 0x18, 0x00, 0x00, 0x01, 0xb5, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0x98, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x81, 0x0c, 0x13, 0x00, 0x00, 0x03, 0xa3, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xc8, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x51, 0x0f, 0x12, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02, 0xcd, 0x0c, 0x18, 0x00, 0x00, 0x01, 0xb9, 0x0c, 0x17, 0x00, 0x00, 0x02,
  0xa0, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x87, 0x0c, 0x11, 0x00, 0x00, 0x03, 0xaa, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xb2, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x26, 0x0f, 0x21, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x02,
  0xff, 0xee, 0xc9, 0x6a, 0xcd, 0x0c, 0x1b, 0x00, 0x00, 0x01, 0xb9, 0x0c, 0x1a, 0x00, 0x00, 0x02,
  0xa1, 0x0c, 0x10, 0x00, 0x00, 0x02, 0x86, 0x0c, 0x11, 0x00, 0x00, 0x03, 0xab, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xa0, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x33, 0x0f, 0x1e, 0x00, 0x00,
  0x01, 0xb5, 0x10, 0x03, 0xd3, 0x0c, 0x16, 0x00, 0x00, 0x01, 0xb4, 0x0c, 0x15, 0x00, 0x00, 0x02,
  0xac, 0x0c, 0x15, 0x00, 0x00, 0x02, 0x8c, 0x0c, 0x0e, 0x00, 0x00, 0x03, 0xab, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xd4, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x76, 0x0f, 0x12, 0x00, 0x00,
  0x01, 0x55, 0x0e, 0x03,
  0xff, 0xee, 0xf1, 0x6a, 0xf9, 0x0c, 0x0d, 0x00, 0x00, 0x01, 0xba, 0x0c, 0x05, 0x00, 0x00, 0x02,
  0xbd, 0x11, 0x09, 0x00, 0x00, 0x02, 0x92, 0x15, 0x0a, 0x00, 0x00, 0x03, 0xa5, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0xca, 0x2e, 0x13, 0x00, 0x00, 0x02, 0xda, 0x51, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
  0x01, 0x42, 0x0e, 0x01, 0x15, 0x0d, 0x0d, 0x15, 0xd9, 0x2d, 0xfb, 0x0e, 0x08, 0x00, 0x00, 0x02,
  0xba, 0x11, 0x09, 0x00, 0x00, 0x02, 0x93, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xab, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0xb7, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x5b, 0x10, 0x0f, 0x00, 0x00,
  0x01, 0xa4, 0x10, 0x11,
  0xff, 0xee, 0x19, 0x6b, 0x10, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0x05, 0x0f, 0x0a, 0x00, 0x00, 0x02,
  0xb8, 0x11, 0x09, 0x00, 0x00, 0x02, 0x92, 0x15, 0x0c, 0x00, 0x00, 0x03, 0xa7, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0xa5, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x48, 0x10, 0x13, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x64, 0x16, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0xfc, 0x0e, 0x0a, 0x00, 0x00, 0x02,
  0xba, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x8d, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xad, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0x99, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x37, 0x10, 0x1b, 0x00, 0x00,
  0x01, 0x14, 0x0e, 0x18,
  0xff, 0xee, 0x41, 0x6b, 0x14, 0x0d, 0x0b, 0x09, 0xd5, 0x2d, 0xfe, 0x0e, 0x0a, 0x00, 0x00, 0x02,
  0xc6, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x8d, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb6, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x97, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x72, 0x60, 0x00, 0x29, 0x10, 0x13, 0x00, 0x00,
  0x01, 0xe5, 0x0d, 0x0c, 0x1b, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0xf9, 0x0e, 0x0d, 0xeb, 0xd7, 0x2c,
  0xb8, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x8f, 0x15, 0x0c, 0x00, 0x00, 0x03, 0xb5, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0x9c, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x33, 0x0f, 0x0d, 0x00, 0x00,
  0x01, 0xd9, 0x0d, 0x12,
  0xff, 0xee, 0x69, 0x6b, 0x1b, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0xfb, 0x0e, 0x0a, 0xeb, 0xd7, 0x2c,
  0xc5, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x93, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb9, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x8c, 0x2e, 0x13, 0x00, 0x00, 0x02, 0x96, 0x60, 0x00, 0x32, 0x0f, 0x21, 0x00, 0x00,
  0x01, 0xd1, 0x0d, 0x11, 0x14, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0x01, 0x0f, 0x0c, 0x00, 0x00, 0x02,
  0xbd, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x93, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xca, 0x1b, 0x12, 0x00,
  0x00, 0x01, 0x96, 0x2e, 0x13, 0xa2, 0x0f, 0x0e, 0x00, 0x00, 0x01, 0x29, 0x0f, 0x26, 0x61, 0xce,
  0x30, 0xd6, 0x0d, 0x16,
  0xff, 0xee, 0x91, 0x6b, 0x17, 0x0d, 0x0b, 0xb7, 0xb6, 0x39, 0x03, 0x0f, 0x0a, 0x00, 0x00, 0x02,
  0xbc, 0x11, 0x13, 0x00, 0x00, 0x02, 0x8d, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xbf, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x96, 0x2e, 0x0e, 0x9b, 0x0f, 0x11, 0x00, 0x00, 0x01, 0x1b, 0x0f, 0x16, 0x00, 0x00,
  0x01, 0xb2, 0x0d, 0x09, 0x1b, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0xfd, 0x0e, 0x0a, 0x00, 0x00, 0x02,
  0xbe, 0x11, 0x13, 0x00, 0x00, 0x02, 0x8d, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb1, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x9d, 0x2e, 0x13, 0x83, 0x0f, 0x16, 0x98, 0x60, 0x00, 0x06, 0x0f, 0x10, 0x00, 0x00,
  0x01, 0xed, 0x0d, 0x0e,
  0xff, 0xee, 0xb9, 0x6b, 0x19, 0x0d, 0x0d, 0x87, 0xb9, 0x3b, 0xf9, 0x0e, 0x0b, 0xe5, 0xb8, 0x3a,
  0xc0, 0x11, 0x13, 0x00, 0x00, 0x02, 0x8d, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb1, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x9a, 0x2e, 0x13, 0x8e, 0x0f, 0x19, 0xb2, 0x60, 0x00, 0x90, 0x0e, 0x0d, 0x7b, 0xb7,
  0x3a, 0xeb, 0x0d, 0x10, 0x19, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0xff, 0x0e, 0x0a, 0x79, 0xb6, 0x39,
  0xb4, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x91, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xa4, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x8f, 0x2e, 0x13, 0x87, 0x0f, 0x14, 0x00, 0x00, 0x01, 0x53, 0x0e, 0x19, 0x4f, 0xb3,
  0x39, 0xb5, 0x0d, 0x12,
  0xff, 0xee, 0xe1, 0x6b, 0x15, 0x0d, 0x0b, 0x0f, 0xb4, 0x38, 0x02, 0x0f, 0x0a, 0x00, 0x00, 0x02,
  0xba, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x95, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb5, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x8a, 0x2e, 0x13, 0x5e, 0x0f, 0x12, 0x00, 0x00, 0x01, 0x6f, 0x0e, 0x19, 0x00, 0x00,
  0x01, 0xfc, 0x0d, 0x12, 0x16, 0x0d, 0x0b, 0x00, 0x00, 0x01, 0x00, 0x0f, 0x0a, 0x00, 0x00, 0x02,
  0xb8, 0x11, 0x0e, 0x00, 0x00, 0x02, 0x97, 0x15, 0x0b, 0x00, 0x00, 0x03, 0xb5, 0x1b, 0x0d, 0x00,
  0x00, 0x01, 0x8e, 0x2e, 0x13, 0x57, 0x0f, 0x15, 0x08, 0x61, 0x00, 0xcd, 0x0e, 0x12, 0x00, 0x00,
  0x01, 0xc4, 0x0d, 0x0c,
  0xac, 0x1a, 0x32, 0x20, 0x37, 0x22
};


class velodyne_driver : public ::testing::Test
{
public:
  velodyne_driver()
  {
    (void)memcpy(&pkt, static_packet, sizeof(pkt));
    out.reserve(Vlp16Translator::POINT_BLOCK_CAPACITY);
  }

protected:
  Vlp16Translator::Packet pkt;
  std::vector<autoware::common::types::PointXYZIF> out;
};  // class velodyne_driver

/// make sure instantiating the driver doesn't result in the process shitting the bed
TEST_F(velodyne_driver, basic)
{
  const geometry_msgs::msg::Point32 offset_m;
  const geometry_msgs::msg::Point32 rotation_rad;
  const Vlp16Translator::Config cfg{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(0.0F, 0.0F, 0.0F),
    0.0F,
    100.0F,
    0.0F,
    360.0F};
  Vlp16Translator driver(cfg);
  driver.convert(pkt, out);
  EXPECT_LE(out.size(),
    Vlp16Translator::NUM_POINTS_PER_BLOCK * Vlp16Translator::NUM_BLOCKS_PER_PACKET);
  // Mostly just a sanity check: All points should fall in a pie slice
  float32_t min_r = std::numeric_limits<float32_t>::max();
  float32_t max_r = 0.0F;
  float32_t min_th = std::numeric_limits<float32_t>::max();
  float32_t max_th = -std::numeric_limits<float32_t>::max();
  float32_t min_phi = std::numeric_limits<float32_t>::max();
  float32_t max_phi = -std::numeric_limits<float32_t>::max();
  uint32_t last_id = 0U;
  for (uint32_t idx = 0U; idx < out.size(); ++idx) {
    autoware::common::types::PointXYZIF & pt = out[idx];
    if (0U == idx) {
      last_id = pt.id;
    }
    const float32_t th = atan2f(pt.y, pt.x);
    const float32_t r_xy = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    const float32_t phi = atan2f(pt.z, r_xy);
    const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
   // Update min/max
    min_r = std::min(min_r, r);
    max_r = std::max(max_r, r);
    if (fabsf(th) > 0.00000001F) {
      // missing returns are (0, 0)
      min_th = std::min(min_th, th);
      max_th = std::max(max_th, th);
    }
    min_phi = std::min(min_phi, phi);
    max_phi = std::max(max_phi, phi);
    // Sanity check on intensity
    EXPECT_LE(pt.intensity, 255.0F);
    EXPECT_GE(pt.intensity, 0.0F);
    // IDs are contiguous
    EXPECT_TRUE((pt.id == last_id) || (pt.id == (last_id + 1U)));
    last_id = pt.id;
  }
  EXPECT_GE(min_r, 0.0F);
  EXPECT_LE(max_r, 130.0F) << max_r;  // max range from spec sheet
  // compute angle differences
  const float32_t dth = max_th - min_th;
  const float32_t th_diff = fabsf(atan2f(sinf(dth), cosf(dth)));
  // @ 755 packets/sec, and 5-20 Hz revs, one packet can cover no more than
  // and no less than
  EXPECT_LE(th_diff, 3.14159F / (755.0F / 20.0F)) << max_th << ", " << min_th;
  EXPECT_GE(th_diff, 3.14159F / 151.0F);
  // max difference in elevation angles should be no more than 20 degrees
  const float32_t dphi = max_phi - min_phi;
  const float32_t phi_diff = fabsf(atan2f(sinf(dphi), cosf(dphi)));
  EXPECT_LE(phi_diff, (20.0F * 3.14159F / 180.0F) + 0.001F);
}

/// Force errors
TEST_F(velodyne_driver, bad_cases)
{
  // r_max < r_min case
  EXPECT_THROW(
      Vlp16Translator::Config cfg(
      300.0F,
      make_point(0.0F, 0.0F, 0.0F),
      make_point(0.0F, 0.0F, 0.0F),
      90.0F,
      10.0F,
      0.0F,
      360.0F),
  std::runtime_error);
  // overlapping min/max azimuth case (approximately equal -> indices are equal)
  Vlp16Translator::Config cfg2{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(0.0F, 0.0F, 0.0F),
    10.0F,
    90.0F,
    299.99999F,
    300.0F};
  EXPECT_THROW(Vlp16Translator driver(cfg2), std::runtime_error);
}

// Make sure configurations do what they say they do
TEST_F(velodyne_driver, config_radius)
{
  const float32_t min_r = 7.0F;
  const float32_t max_r = 25.0F;
  ASSERT_LT(min_r, max_r);
  const Vlp16Translator::Config cfg{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(0.0F, 0.0F, 0.0F),
    min_r,
    max_r,
    0.0F,
    360.0F};
  Vlp16Translator driver(cfg);
  driver.convert(pkt, out);
  EXPECT_LE(out.size(),
    Vlp16Translator::NUM_POINTS_PER_BLOCK * Vlp16Translator::NUM_BLOCKS_PER_PACKET);
  uint32_t last_id = 0U;
  for (uint32_t idx = 0U; idx < out.size(); ++idx) {
    autoware::common::types::PointXYZIF & pt = out[idx];
    if (0U == idx) {
      last_id = pt.id;
    }
    const float32_t th = atan2f(pt.y, pt.x);
    const float32_t r_xy = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    const float32_t phi = atan2f(pt.z, r_xy);
    const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
    EXPECT_LE(r, max_r);
    EXPECT_GE(r, min_r);
    // Sanity check on intensity
    EXPECT_LE(pt.intensity, 255.0F);
    EXPECT_GE(pt.intensity, 0.0F);
    // IDs are contiguous
    EXPECT_TRUE((pt.id == last_id) || (pt.id == (last_id + 1U)));
    last_id = pt.id;
  }
}

// z axis rotation
TEST_F(velodyne_driver, config_rotation1)
{
  const Vlp16Translator::Config cfg{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(0.0F, 0.0F, -1.0F),
    0.1F,
    150.0F,
    0.0F,
    360.0F};
  Vlp16Translator driver(cfg);
  driver.convert(pkt, out);
  EXPECT_LE(out.size(),
    Vlp16Translator::NUM_POINTS_PER_BLOCK * Vlp16Translator::NUM_BLOCKS_PER_PACKET);
  uint32_t last_id = 0U;
  for (uint32_t idx = 0U; idx < out.size(); ++idx) {
    autoware::common::types::PointXYZIF & pt = out[idx];
    if (0U == idx) {
      last_id = pt.id;
    }
    const float32_t th = atan2f(pt.y, pt.x);
    const float32_t r_xy = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    const float32_t phi = atan2f(pt.z, r_xy);
    const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
    // We know that th is normally around 1.4-1.5, so now it should be around 0.4
    EXPECT_LT(fabsf(th - 0.45F), 0.1F) << th;
    // Sanity check on intensity
    EXPECT_LE(pt.intensity, 255.0F);
    EXPECT_GE(pt.intensity, 0.0F);
    // IDs are contiguous
    EXPECT_TRUE((pt.id == last_id) || (pt.id == (last_id + 1U)));
    last_id = pt.id;
  }
}
// do a roll / x axis rotation
TEST_F(velodyne_driver, config_rotation2)
{
  const Vlp16Translator::Config cfg{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(3.14159F / 2.0F, 0.0F, 0.0F),
    0.1F,
    150.0F,
    0.0F,
    360.0F};
  Vlp16Translator driver(cfg);
  driver.convert(pkt, out);
  EXPECT_LE(out.size(),
    Vlp16Translator::NUM_POINTS_PER_BLOCK * Vlp16Translator::NUM_BLOCKS_PER_PACKET);
  uint32_t last_id = 0U;
  for (uint32_t idx = 0U; idx < out.size(); ++idx) {
    autoware::common::types::PointXYZIF & pt = out[idx];
    if (0U == idx) {
      last_id = pt.id;
    }
    const float32_t th = atan2f(pt.y, pt.x);
    const float32_t r_xy = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    const float32_t phi = atan2f(pt.z, r_xy);
    const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
    // points should vaguely be pointing up
    EXPECT_LT(fabsf(phi - 3.14159F / 2.0F), 0.3F);
    // Sanity check on intensity
    EXPECT_LE(pt.intensity, 255.0F);
    EXPECT_GE(pt.intensity, 0.0F);
    // IDs are contiguous
    EXPECT_TRUE((pt.id == last_id) || (pt.id == (last_id + 1U)));
    last_id = pt.id;
  }
}

// figure out what the runtime of convert() is, locally
TEST_F(velodyne_driver, benchmark)
{
  const Vlp16Translator::Config cfg{
    300.0F,
    make_point(0.0F, 0.0F, 0.0F),
    make_point(0.0F, 0.0F, 0.0F),
    0.0F,
    100.0F,
    0.0F,
    360.0F};
  autoware::drivers::velodyne_driver::Vlp16Translator driver(cfg);
  const uint32_t num_runs = 200U;
  auto time_begin = std::chrono::steady_clock::now();
  for (uint32_t idx = 0U; idx < num_runs; ++idx) {
    driver.convert(pkt, out);
  }
  auto time_end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin);
  std::cerr << "convert() average runtime: " << duration.count() / num_runs << " µs\n";
}

#endif  // TEST_DRIVER_H_
