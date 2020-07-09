// Copyright 2020 Tier IV, Inc.
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

#ifndef TEST_EUCLIDEAN_CLUSTER_NODES_HPP_
#define TEST_EUCLIDEAN_CLUSTER_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>

using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;

TEST(euclidean_cluster_nodes, instantiate)
{
  // Basic test to ensure that EuclideanClusterNode can be instantiated
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;

  node_options.parameter_overrides(params);
//   ASSERT_THROW(EuclideanClusterNode{node_options}, rclcpp::ParameterTypeException);
  ASSERT_NO_THROW(EuclideanClusterNode{node_options});
}

#endif  // TEST_EUCLIDEAN_CLUSTER_NODES_HPP_
