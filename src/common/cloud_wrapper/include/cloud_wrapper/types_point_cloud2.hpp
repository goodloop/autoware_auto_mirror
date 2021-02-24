// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the types_point_cloud2 namespace.

#ifndef BUILD_SRC_COMMON_CLOUD_WRAPPER_INCLUDE_CLOUD_WRAPPER_TYPES_POINT_CLOUD2_HPP_
#define BUILD_SRC_COMMON_CLOUD_WRAPPER_INCLUDE_CLOUD_WRAPPER_TYPES_POINT_CLOUD2_HPP_

#include "common/types.hpp"
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware
{
namespace common
{
namespace types_point_cloud2
{
using types::float32_t;
using types::float64_t;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField make_point_field(
  const PointField::_name_type & name,
  const PointField::_datatype_type & datatype,
  const PointField::_offset_type & offset,
  const PointField::_count_type & count)
{
  PointField field;
  field.name = name;
  field.datatype = datatype;
  field.offset = offset;
  field.count = count;
  return field;
}

/// PointXYZ
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointXYZ
{
  PointXYZ() = default;
  PointXYZ(float32_t x, float32_t y, float32_t z)
  : x(x), y(y), z(z) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointXYZ{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1)
};

/// PointXYZId
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointXYZId
{
  PointXYZId() = default;
  PointXYZId(float32_t x, float32_t y, float32_t z, uint16_t id)
  : x(x), y(y), z(z), id(id) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  uint16_t id{0};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointXYZId{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("id", PointField::UINT16, 12, 1)
};

/// PointXYZI
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointXYZI
{
  PointXYZI() = default;
  PointXYZI(float32_t x, float32_t y, float32_t z, float32_t intensity)
  : x(x), y(y), z(z), intensity(intensity) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  float32_t intensity{0.0f};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointXYZI{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("intensity", PointField::FLOAT32, 12, 1)
};

/// PointXYZIId
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointXYZIId
{
  PointXYZIId() = default;
  PointXYZIId(float32_t x, float32_t y, float32_t z, float32_t intensity, uint16_t id)
  : x(x), y(y), z(z), intensity(intensity), id(id) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  float32_t intensity{0.0f};
  uint16_t id{0};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointXYZIId{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("intensity", PointField::FLOAT32, 12, 1),
  make_point_field("id", PointField::UINT16, 16, 1)
};

/// PointLgsvl
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointLgsvl
{
  PointLgsvl() = default;
  PointLgsvl(float32_t x, float32_t y, float32_t z, uint8_t intensity, float64_t timestamp)
  : x(x), y(y), z(z), intensity(intensity), timestamp(timestamp) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  uint32_t _padding_4_01{0};
  uint8_t intensity{0};
  uint8_t _padding_1_01{0};
  uint16_t _padding_2_01{0};
  uint32_t _padding_4_02{0};
  float64_t timestamp{0.0};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointLgsvl{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("intensity", PointField::UINT8, 16, 1),
  make_point_field("timestamp", PointField::FLOAT64, 24, 1)
};

}  // namespace types_point_cloud2
}  // namespace common
}  // namespace autoware


#endif //BUILD_SRC_COMMON_CLOUD_WRAPPER_INCLUDE_CLOUD_WRAPPER_TYPES_POINT_CLOUD2_HPP_
