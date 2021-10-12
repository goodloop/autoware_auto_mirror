// Copyright 2020-2021 Arm Ltd., TierIV
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

#ifndef APOLLO_LIDAR_SEGMENTATION__FEATURE_MAP_HPP_
#define APOLLO_LIDAR_SEGMENTATION__FEATURE_MAP_HPP_

#include <common/types.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace apollo_lidar_segmentation
{
using autoware::common::types::float32_t;

/// \brief Abstract interface for FeatureMap.
struct FeatureMapInterface
{
public:
  const int32_t channels;
  const int32_t width;
  const int32_t height;
  const int32_t range;
  float32_t * max_height_data;      // channnel 0
  float32_t * mean_height_data;     // channnel 1
  float32_t * count_data;           // channnel 2
  float32_t * direction_data;       // channnel 3
  float32_t * top_intensity_data;   // channnel 4
  float32_t * mean_intensity_data;  // channnel 5
  float32_t * distance_data;        // channnel 6
  float32_t * nonempty_data;        // channnel 7
  std::vector<float32_t> map_data;
  virtual void initializeMap(std::vector<float32_t> & map) = 0;
  virtual void resetMap(std::vector<float32_t> & map) = 0;
  explicit FeatureMapInterface(int32_t _channels, int32_t _width, int32_t _height, int32_t _range);
};

/// \brief FeatureMap with no extra feature channels.
struct FeatureMap : public FeatureMapInterface
{
  explicit FeatureMap(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with an intensity feature channel.
struct FeatureMapWithIntensity : public FeatureMapInterface
{
  explicit FeatureMapWithIntensity(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with a constant feature channel.
struct FeatureMapWithConstant : public FeatureMapInterface
{
  explicit FeatureMapWithConstant(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with constant and intensity feature channels.
struct FeatureMapWithConstantAndIntensity : public FeatureMapInterface
{
  explicit FeatureMapWithConstantAndIntensity(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};
}  // namespace apollo_lidar_segmentation
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
#endif  // APOLLO_LIDAR_SEGMENTATION__FEATURE_MAP_HPP_
