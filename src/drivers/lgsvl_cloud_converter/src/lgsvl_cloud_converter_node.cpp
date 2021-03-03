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

#include "lgsvl_cloud_converter/lgsvl_cloud_converter_node.hpp"
#include <cloud_wrapper/cloud_ptr_wrapper.hpp>
#include <algorithm>
#include <exception>

namespace autoware
{
namespace drivers
{
namespace lgsvl_cloud_converter
{

LgsvlCloudConverterNode::LgsvlCloudConverterNode(const rclcpp::NodeOptions & options)
:  Node("lgsvl_cloud_converter", options),
  pub_ptr_cloud_output_{this->create_publisher<sensor_msgs::msg::PointCloud2>(
      static_cast<std::string>(declare_parameter(
        "name_topic_cloud_out").get<std::string>()),
      10)}
  ,
  sub_ptr_cloud_input_(this->create_subscription<PointCloud2>(
      static_cast<std::string>(declare_parameter(
        "name_topic_cloud_in").get<std::string>()),
      rclcpp::QoS(10),
      std::bind(&LgsvlCloudConverterNode::callback_cloud_input,
      this,
      std::placeholders::_1)))
{
}

void LgsvlCloudConverterNode::callback_cloud_input(const PointCloud2::SharedPtr msg)
{
  try {
    using autoware::common::cloud_wrapper::CloudPtrWrapper;
    using autoware::common::types_point_cloud2::PointXYZI;
    using autoware::common::types_point_cloud2::PointLgsvl;
    CloudPtrWrapper<PointLgsvl> cloud_lgsvl(msg);
    CloudPtrWrapper<PointXYZI> cloud_xyzi(
      autoware::common::types_point_cloud2::fields_PointXYZI);
    cloud_xyzi.reserve(cloud_lgsvl.size());
    cloud_xyzi.get_msg_ptr()->header = cloud_lgsvl.get_msg_ptr()->header;

    auto point_lgsvl_to_xyzi = [](const PointLgsvl & point_in) -> PointXYZI {
        return {
        point_in.x,
        point_in.y,
        point_in.z,
        static_cast<float>(point_in.intensity)};
      };

    std::transform(
      cloud_lgsvl.begin(),
      cloud_lgsvl.end(),
      std::back_inserter(cloud_xyzi),
      point_lgsvl_to_xyzi);
    pub_ptr_cloud_output_->publish(*cloud_xyzi.get_msg_ptr());
  } catch (std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Exception occured: " + std::string(ex.what()));
  }

}

}  // namespace lgsvl_cloud_converter
}  // namespace drivers
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::drivers::lgsvl_cloud_converter::LgsvlCloudConverterNode)
