// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lanelet2_map_provider class.

#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <common/types.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_map_provider/visibility_control.hpp>

#include <iostream>
#include <string>
#include <memory>

#include "autoware_auto_msgs/msg/had_map_bin.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
/// \brief TODO(simon.thompson): Document namespaces!
namespace lanelet2_map_provider
{

/// \class Lanelet2MapProvider
/// \brief Provides functoins to load and access a lanelet2 OSM map.
class LANELET2_MAP_PROVIDER_PUBLIC Lanelet2MapProvider
{
public:
  /// \brief Constructor from a transform between earth and map frames
  /// \param map_filename The lanelet map filename
  /// \param stf The earth to map transform for projection of map data
  Lanelet2MapProvider(
    const std::string & map_filename,
    const geometry_msgs::msg::TransformStamped & stf);
  /// \brief Constructor from latitude, longitude, altitude
  /// \param map_filename The lanelet map filename
  /// \param origin_lat Latitude of the map origin
  /// \param origin_lon Longitude of the map origin
  /// \param origin_ele Elevation of the map origin
  Lanelet2MapProvider(
    const std::string & map_filename, const float64_t origin_lat,
    const float64_t origin_lon, const float64_t origin_ele);
  /// The map itself. After the constructor logic has been done,
  /// this is guaranteed to be initialized.
  std::shared_ptr<lanelet::LaneletMap> m_map;

private:
  /// \brief Internal function used by the constructor
  /// \param map_filename The lanelet map filename
  /// \param origin_lat Latitude of the map origin
  /// \param origin_lon Longitude of the map origin
  /// \param origin_ele Elevation of the map origin
  void load_map(
    const std::string & map_filename, const float64_t origin_lat,
    const float64_t origin_lon, const float64_t origin_ele);
};

}  // namespace lanelet2_map_provider

}  // namespace autoware

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
