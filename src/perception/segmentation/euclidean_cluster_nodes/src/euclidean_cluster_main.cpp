// Copyright 2019 Apex.AI, Inc.
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

#if __cplusplus >= 201703L
#include <filesystem>
namespace std_fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace std_fs = std::experimental::filesystem;
#endif

#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>
#include <iostream>
#include <memory>
#include <rcutils/cmdline_parser.h>

static constexpr const char * PROGRAM_NAME = "euclidean_cluster_exe";

void print_usage()
{
  printf("Usage for %s:\n", PROGRAM_NAME);
  printf("%s [--config_file path] [--node_name name] [--node_namespace] [-h]\n", PROGRAM_NAME);
}

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);
  std_fs::path default_config_path =
    std_fs::absolute(argv[0])
      .parent_path()
      .parent_path()
      .parent_path() /
    "share" / "euclidean_cluster_nodes" / "vlp16_lexus_cluster.param.yaml";

  int32_t ret = 0;
  try {
    if(rcutils_cli_option_exist(argv, &argv[argc], "-h") ||
       rcutils_cli_option_exist(argv, &argv[argc], "--help"))
    {
      print_usage();
      return 0;
    }
    const char * config_file = default_config_path.c_str();
    const char * arg = rcutils_cli_get_option(argv, &argv[argc], "--config_file");
    if (nullptr != arg) {
      config_file = arg;
    }
    const char * node_name = "euclidean_cluster_cloud_node";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_name");
    if (nullptr != arg) {
      node_name = arg;
    }
    const char * node_namespace = "";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_namespace");
    if (nullptr != arg) {
      node_namespace = arg;
    }


    using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;
    const auto nd_ptr = std::make_shared<EuclideanClusterNode>(
      node_name,
      node_namespace,
      config_file);

    rclcpp::spin(nd_ptr);

    rclcpp::shutdown();
  } catch (const std::exception & e) {
    // RCLCPP logging macros are not used in error handling because they would depend on nd_ptr's
    // logger. This dependency would result in a crash when nd_ptr is a nullptr
    std::cerr << (e.what());
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown exception caught. Exiting...";
    ret = -1;
  }
  return ret;
}
