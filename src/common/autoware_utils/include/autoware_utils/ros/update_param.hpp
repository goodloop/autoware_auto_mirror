// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__ROS__UPDATE_PARAM_HPP_
#define AUTOWARE_UTILS__ROS__UPDATE_PARAM_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace autoware_utils
{
/// \brief Get value of of the param from given parameter name
/// \tparam T Type value of the parameter
/// \param params Vector of parameters
/// \param name Name of the parameter value
/// \param value Parameter value to be updated
/// \return If the vector of parameters contains name return true, else false.
template<class T>
bool updateParam(const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr =
    std::find_if(
    params.cbegin(), params.cend(), [&name](const rclcpp::Parameter & p) {
      return p.get_name() == name;
    });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__UPDATE_PARAM_HPP_
