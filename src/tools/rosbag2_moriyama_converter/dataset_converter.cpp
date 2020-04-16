// Copyright 2020 Apex.AI, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using std::placeholders::_1;

class DatasetConverter : public rclcpp::Node
{
public:
  DatasetConverter()
  : Node("dataset_converter")
  {
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_raw", 10, std::bind(&DatasetConverter::point_cloud_callback, this, _1));
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_front/points_raw", 10);
  }

private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    msg->header.frame_id = "lidar_front";
    point_cloud_publisher_->publish(*msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatasetConverter>());
  rclcpp::shutdown();
  return 0;
}
