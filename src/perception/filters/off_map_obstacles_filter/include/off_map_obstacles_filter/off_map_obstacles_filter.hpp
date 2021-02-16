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
/// \brief This file defines the off_map_obstacles_filter class.

#ifndef OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_
#define OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_

#include <memory>

#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "common/types.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "off_map_obstacles_filter/visibility_control.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware
{
/// \brief A namespace for the temporary off-map obstacles filter.
namespace off_map_obstacles_filter
{

using float64_t = autoware::common::types::float64_t;

class OFF_MAP_OBSTACLES_FILTER_PUBLIC OffMapObstaclesFilter
{
public:
  OffMapObstaclesFilter(std::shared_ptr<lanelet::LaneletMap> map, float64_t overlap_threshold);

  visualization_msgs::msg::MarkerArray bboxes_in_map_frame_viz(
    const geometry_msgs::msg::TransformStamped & map_from_base_link,
    const autoware_auto_msgs::msg::BoundingBoxArray & msg) const;
  /// \param map_from_base_link The transform that transforms things from base_link to map.
  /// \param bbox The bounding box with coordinates in the base_link frame.
  /// \return True if the bbox overlaps any map element.
  /// This function assumes that bboxes are 2.5d, i.e. only have yaw but no roll or pitch
  void remove_off_map_bboxes(
    const geometry_msgs::msg::TransformStamped & map_from_base_link,
    autoware_auto_msgs::msg::BoundingBoxArray & msg) const;

private:
  const std::shared_ptr<lanelet::LaneletMap> m_map;
  const float64_t m_overlap_threshold = 1.0;
};

}  // namespace off_map_obstacles_filter
}  // namespace autoware

#endif  // OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_
