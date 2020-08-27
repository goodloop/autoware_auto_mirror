// Copyright 2020 TierIV, Inc.
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

#include "rosbag2_naming_tool/rosbag2_naming_tool.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cctype>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/writer.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/converter_interfaces/serialization_format_converter.hpp"

rosbag2_naming_tool::NamingToolNode::NamingToolNode(
  const rclcpp::NodeOptions & options)
: Node("rosbag2_naming_tool", options)
{
  rosbag2::SequentialReader reader;
  rosbag2::StorageOptions input_storage_options{};
  rosbag2::Writer writer;
  rosbag2::StorageOptions output_storage_options{};

  input_storage_options.uri = declare_parameter("input_dataset").get<std::string>();
  input_storage_options.storage_id = "sqlite3";

  output_storage_options.uri = declare_parameter("output_dataset").get<std::string>();
  output_storage_options.storage_id = "sqlite3";

  rosbag2::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(input_storage_options, converter_options);

  writer.open(output_storage_options, converter_options);

  auto topics = reader.get_all_topics_and_types();

  for (auto topic : topics) {
    auto topic_new = topic;
    if ("/nmea_sentence" == topic_new.name) {
      topic_new.name = "/gnss/nmea_sentence_raw";
    } else if ("/gnss/fix" == topic_new.name) {
      topic_new.name = "/gnss/nav_sat_fix_raw";
    }
    writer.create_topic(topic_new);
  }

  RCLCPP_INFO(get_logger(), "Processing rosbag2 file: " + input_storage_options.uri);
  while (reader.has_next()) {
    auto serialized_message = reader.read_next();
    if ("/nmea_sentence" == serialized_message->topic_name) {
      serialized_message->topic_name = "/gnss/nmea_sentence_raw";
    } else if ("/gnss/fix" == serialized_message->topic_name) {
      serialized_message->topic_name = "/gnss/nav_sat_fix_raw";
    }
    writer.write(serialized_message);
  }
  RCLCPP_INFO(get_logger(), "File written: " + output_storage_options.uri);
  rclcpp::shutdown();
}

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_naming_tool::NamingToolNode)
