// Copyright 2020 Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__POLICIES_HPP_
#define POINT_CLOUD_MAPPING__POLICIES_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <memory>
#include <string>
#include <utility>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
/// CRTP base class of file-writing trigger policy for the mapper.
/// \tparam Derived Implementation class.
template<typename Derived>
class TriggerPolicyBase : public common::helper_functions::crtp<Derived>
{
public:
  /// Check the map and report if the map is ready for writing. Behavior is implementation
  /// defined.
  /// \tparam MapMsgT Increment type of the map representation. (To be deduced.)
  /// \return True if the map should be written to a file.
  template<typename MapMsgT>
  bool ready(const std::unique_ptr<MapRepresentationBase<MapMsgT>> & map_ptr)
  {
    return this->impl().ready_(map_ptr);
  }
};

/// Trigger map writing when map reaches its capacity.
class CapacityTrigger : public TriggerPolicyBase<CapacityTrigger>
{
public:
  template<typename MapMsgT>
  bool ready_(const std::unique_ptr<MapRepresentationBase<MapMsgT>> & map_ptr)
  {
    return map_ptr->size() >= map_ptr->capacity();
  }
};

/// CRTP base class of file name prefix generator for the mapper.
/// \tparam Derived Implementation class.
template<typename Derived>
class PrefixGeneratorBase : public common::helper_functions::crtp<Derived>
{
public:
  // TODO(yunus.caliskan): return const ref instead?

  /// Get the final file name prefix for a given base prefix.
  /// \param base_prefix Base prefix that will be used to generate the final prefix.
  /// \return Final file name prefix.
  std::string get(const std::string & base_prefix)
  {
    return this->impl().get_(base_prefix);
  }
};

/// Prefix generator that adds the current time stamp to the end of the base prefix.
class TimeStampPrefixGenerator : public PrefixGeneratorBase<TimeStampPrefixGenerator>
{
public:
  std::string get_(const std::string & base_prefix);
};
}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POLICIES_HPP_
