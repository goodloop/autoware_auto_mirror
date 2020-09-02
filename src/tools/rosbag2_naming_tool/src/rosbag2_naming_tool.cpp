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

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nmea_msgs/msg/sentence.hpp>

#include <map>
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

  auto topic_renamings = declare_parameter("topics").get<std::vector<std::string>>();

  std::map<std::string, std::string> topic_renamings_map;

  for (auto it = topic_renamings.begin(); it < topic_renamings.end(); ++it) {
    auto && old_topic = *it;
    auto && new_topic = *(++it);
    topic_renamings_map[old_topic] = new_topic;
  }

  auto frame_renamings = declare_parameter("frames").get<std::vector<std::string>>();

  std::map<std::string, std::string> frame_renamings_map;

  for (auto it = frame_renamings.begin(); it < frame_renamings.end(); ++it) {
    auto && old_frame = *it;
    auto && new_frame = *(++it);
    frame_renamings_map[old_frame] = new_frame;
  }

  for (auto topic : topics) {
    auto topic_new = topic;
    try {
      topic_new.name = topic_renamings_map.at(topic.name);
    } catch (const std::out_of_range &) {
      topic_renamings_map[topic.name] = topic.name;
    }
    writer.create_topic(topic_new);
  }

  RCLCPP_INFO(get_logger(), "Processing rosbag2 file: " + input_storage_options.uri);

  while (reader.has_next()) {
    auto serialized_message = reader.read_next();
    serialized_message->topic_name = topic_renamings_map[serialized_message->topic_name];
    if ("/gnss/nav_sat_fix_raw" == serialized_message->topic_name) {
      apply_frame_transformation<sensor_msgs::msg::NavSatFix>(serialized_message,
        "sensor_msgs/msg/NavSatFix", frame_renamings_map);
    } else if ("/lidar_front/points_raw" == serialized_message->topic_name) {
      apply_frame_transformation<sensor_msgs::msg::PointCloud2>(serialized_message,
        "sensor_msgs/msg/PointCloud2", frame_renamings_map);
    } else if ("/gnss/nmea_sentence_raw" == serialized_message->topic_name) {
      apply_frame_transformation<nmea_msgs::msg::Sentence>(serialized_message,
        "nmea_msgs/msg/Sentence", frame_renamings_map);
    }
    writer.write(serialized_message);
  }
  RCLCPP_INFO(get_logger(), "File written: " + output_storage_options.uri);
  rclcpp::shutdown();
}

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_naming_tool::NamingToolNode)
