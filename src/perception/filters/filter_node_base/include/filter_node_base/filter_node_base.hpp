// Copyright 2021 Tier IV, Inc.
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

/// \copyright Copyright 2021 Tier IV, Inc.
/// \file
/// \brief This file defines the FilterNodeBase class.

#ifndef FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
#define FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_


#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "filter_node_base/visibility_control.hpp"
#include "common/types.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Include TF
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
// PCL includes
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"


using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace filter_node_base
{

/** \brief Searches for a parameter defined by name and type T in the provided rclcpp::Parameter vector.
 * \param p Vector of parameters for this node
 * \param name String name of the parameter
 * \param_t value Return value of the parameter
 * \return bool8_t Return true if the parameter is found, else false
 */
template<typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(
    p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == name;
    });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/// \class FilterNodeBase
/// \brief The abstract class used for filter applications
class FILTER_NODE_BASE_PUBLIC FilterNodeBase : public rclcpp::Node
{
public:
  // using PointCloud2 = sensor_msgs::msg::PointCloud2;
  // using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  /** \brief The default constructor for the FilterNodeBase class
   * \param filter_name The name of the node to pass on to rclcpp::Node
   * \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
   */
  explicit FilterNodeBase(
    const std::string & filter_name = "filter_node_base",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Callback handler for parameter services */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_filter_;

  /** \brief The input PointCloud2 subscriber */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;

  /** \brief The desired user filter field name */
  std::string filter_field_name_;

  /** \brief Mutex used to access to the parameters */
  std::mutex mutex_;

  /** \brief The maximum queue size. */
  size_t max_queue_size_;

  /** \brief Virtual abstract filter method called by the computePublish method at the arrival of each pointcloud message.
   * \param input The input point cloud dataset.
   * \param indices A pointer to the vector of point indices to use.
   * \param output The resultant filtered PointCloud2
   */
  FILTER_NODE_BASE_LOCAL virtual void filter(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input,
    sensor_msgs::msg::PointCloud2 & output) = 0;

  /** \brief Validate a sensor_msgs::msg::PointCloud2 message
   *
   * Method ensure thats the size of the pointcloud defined by the width,
   * height and point_step correspond to the data size.
   *
   * \param cloud Input sensor_msgs::msg::PointCloud2 to be validated
   * \return bool8_t True if the pointcloud is valid, false if not
   */
  FILTER_NODE_BASE_LOCAL inline bool8_t is_valid(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
        "and frame %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
      return false;
    }
    return true;
  }

private:
  /** \brief Parameter service callback
   * \param p Vector of rclcpp::Parameters belonging to the node
   * \return rcl_interfaces::msg::SetParametersResult Return type for the parameter service call
   */
  FILTER_NODE_BASE_LOCAL rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & p);

  /** \brief Callback used to receive pointcloud data.
   *
   * After checking the validity of the received pointcloud message, call the filter method and
   * publish the filtered pointcloud on a new topic.
   *
   * \param msg Input pointcloud message to be processed by the filter
   */
  FILTER_NODE_BASE_LOCAL void pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
};
}  // namespace filter_node_base
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
