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
/// \brief This file defines the lanelet2_map_provider_node class.

#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_map_provider/lanelet2_map_provider.hpp>
#include <memory>

#include "autoware_auto_mapping_msgs/srv/had_map_service.hpp"
#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"

namespace autoware
{
namespace lanelet2_map_provider
{

/// \class Lanelet2MapProviderNode
/// \brief ROS 2 Node for semantic map provider using lanelet2 map format.
/// Node loads a lanelet2 OSM format map using the class Lanelet2MapProvider
/// and then runs a service for supplying other
/// nodes map information according to their requests. Requests are defined by
/// a sequences of requested primitives as well as geometric bounds on requested map
/// data

class LANELET2_MAP_PROVIDER_PUBLIC Lanelet2MapProviderNode : public rclcpp::Node
{
public:
  /// \brief default constructor, loads the map and transforms it from earth frame
  /// into map frame.
  /// *breaks docs test - no node_name generated by automatic package generator
  /// -- param[in] node_name name of the node for rclcpp internals
  /// \throw runtime error if failed to start threads or configure driver
  explicit Lanelet2MapProviderNode(const rclcpp::NodeOptions & options);

  /// \brief Handles the node service requests
  /// \param request Service request message for map data specifying map content and geom. bounds
  /// \param response Service repsone to request, containing a sub-set of map data
  /// but nethertheless containing a complete and valid lanelet2 map
  void handle_request(
    std::shared_ptr<autoware_auto_mapping_msgs::srv::HADMapService_Request> request,
    std::shared_ptr<autoware_auto_mapping_msgs::srv::HADMapService_Response> response);

private:
  /// If the origin is not defined by parameters, get the transform describing
  /// the map origin (earth->map transform, set by the pcd
  /// map provider). Geocentric lanelet2 coordinates can then be obtained from the map frame.
  /// \return The map origin in ECEF ENU transform
  geometry_msgs::msg::TransformStamped get_map_origin();

  std::unique_ptr<Lanelet2MapProvider> m_map_provider;
  rclcpp::Service<autoware_auto_mapping_msgs::srv::HADMapService>::SharedPtr m_map_service;
};

}  // namespace lanelet2_map_provider

}  // namespace autoware

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_
