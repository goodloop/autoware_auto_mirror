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

#ifndef ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_
#define ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/writer.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/converter_interfaces/serialization_format_converter.hpp"

namespace rosbag2_naming_tool
{
class NamingToolNode : public rclcpp::Node
{
public:
  explicit NamingToolNode(const rclcpp::NodeOptions & options);

private:
  template<typename MessageT>
  void apply_frame_transformation(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message,
    const std::string & message_type,
    const std::map<std::string, std::string> & frame_renamings_map)
  {
    MessageT msg;
    auto ros_message = std::make_shared<rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();
    ros_message->message = &msg;
    auto type_support = rosbag2::get_typesupport(message_type, "rosidl_typesupport_cpp");

    rosbag2::SerializationFormatConverterFactory factory;
    std::unique_ptr<
      rosbag2::converter_interfaces::SerializationFormatDeserializer
    > cdr_deserializer;
    cdr_deserializer = factory.load_deserializer("cdr");

    cdr_deserializer->deserialize(serialized_message, type_support, ros_message);

    msg.header.frame_id = frame_renamings_map.at(msg.header.frame_id);

    std::unique_ptr<rosbag2::converter_interfaces::SerializationFormatSerializer> cdr_serializer;
    cdr_serializer = factory.load_serializer("cdr");
    cdr_serializer->serialize(ros_message, type_support, serialized_message);
  }
};

}  // namespace rosbag2_naming_tool

#endif  // ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_
