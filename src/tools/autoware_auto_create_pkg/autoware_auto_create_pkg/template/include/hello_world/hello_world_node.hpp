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
/// \brief This file defines the hello_world_node class.

#ifndef HELLO_WORLD__HELLO_WORLD_NODE_HPP_
#define HELLO_WORLD__HELLO_WORLD_NODE_HPP_

#include <hello_world/hello_world.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware
{
namespace hello_world
{

/// \class hello_world_c
/// \brief ROS 2 Node for hello world.
class HELLO_WORLD_PUBLIC hello_world_c : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp internals
  /// \throw runtime error if failed to start threads or configure driver
  explicit hello_world_c(const std::string & node_name);

  /// \brief default destructor
  virtual ~hello_world_c();

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.
};
}  // namespace hello_world
}  // namespace autoware

#endif  // HELLO_WORLD__HELLO_WORLD_NODE_HPP_
