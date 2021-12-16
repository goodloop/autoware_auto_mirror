// Copyright 2018-2020 the Autoware Foundation, Arm Limited
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

#include <lslidar_nodes/lslidar_cloud_node.hpp>
#include <rcutils/cmdline_parser.h>

#include <string>
#include <memory>
#include <vector>
#include <cstdio>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

// this file is simply a main file to create a ros1 style standalone node
int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;

  try {
    rclcpp::init(argc, argv);

    const auto run = [](const auto & nd_ptr) {
        while (rclcpp::ok()) {
          rclcpp::spin(nd_ptr);
        }
        rclcpp::shutdown();
      };

    const auto * arg = rcutils_cli_get_option(argv, &argv[argc], "--model");
    if (arg != nullptr) {
      auto model = std::string(arg);
      std::transform(
        model.begin(), model.end(), model.begin(), [](const auto & c) {
          return std::tolower(c);
        });
      if (model == "ls16") {
        run(
          std::make_shared<
            autoware::drivers::lslidar_nodes::LS16DriverNode>(
            "ls16_driver_node",
            rclcpp::NodeOptions{}));
      } else if (model == "ls32") {
        run(
          std::make_shared<
            autoware::drivers::lslidar_nodes::LS32DriverNode>(
            "ls32_driver_node",
            rclcpp::NodeOptions{}));
      } else if (model == "ch128") {
        run(
          std::make_shared<
            autoware::drivers::lslidar_nodes::CH128DriverNode>(
            "ch128_driver_node",
            rclcpp::NodeOptions{}));
      }else if (model == "ch64w") {
        run(
          std::make_shared<
            autoware::drivers::lslidar_nodes::CH64wDriverNode>(
            "ch64w_driver_node",
            rclcpp::NodeOptions{}));
      } else {
        throw std::runtime_error("Model " + model + " is not supperted.");
      }
    } else {
      throw std::runtime_error("Please specify a lslidar model using --model argument");
    }
  } catch (const std::exception & err) {
    // RCLCPP logging macros are not used in error handling because they would depend on vptr's
    // logger. This dependency would result in a crash when vptr is a nullptr
    std::cerr << err.what() << std::endl;
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown error encountered, exiting..." << std::endl;
    ret = -1;
  }
  rclcpp::shutdown();
  return ret;
}
