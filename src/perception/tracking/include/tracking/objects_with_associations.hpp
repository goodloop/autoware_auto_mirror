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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the multi_object_tracking class.


#ifndef TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_
#define TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_


#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <tracking/visibility_control.hpp>

#include <utility>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

namespace detail
{
template<class MsgT>
std::size_t get_size(const MsgT & msg);

template<>
inline std::size_t get_size(const autoware_auto_msgs::msg::DetectedObjects & msg)
{
  return msg.objects.size();
}
}  // namespace detail

enum class Matched
{
  kNothing,
  kExistingTrack,
  kNewTrack
};

struct Association
{
  Matched matched;
  std::size_t match_index;
};

using Associations = std::vector<Association>;

template<class MsgT>
class TRACKING_PUBLIC Associated
{
public:
  explicit Associated(const MsgT & objects)
  : m_objects{objects}, m_associations(detail::get_size(objects), {Matched::kNothing, 0UL}) {}

  explicit Associated(MsgT && objects)
  : m_objects{std::move(objects)},
    m_associations(detail::get_size(objects), {Matched::kNothing, 0UL}) {}

  const MsgT & objects() const noexcept {return m_objects;}

  const std::vector<Association> & associations() const noexcept
  {
    return m_associations;
  }
  std::vector<Association> & associations() noexcept{return m_associations;}

private:
  MsgT m_objects;
  Associations m_associations;
};

using ObjectsWithAssociations = Associated<autoware_auto_msgs::msg::DetectedObjects>;
using RoisWithAssociations = Associated<autoware_auto_msgs::msg::ClassifiedRoiArray>;

}  // namespace tracking
}  // namespace perception
}  // namespace autoware


#endif  // TRACKING__OBJECTS_WITH_ASSOCIATIONS_HPP_
