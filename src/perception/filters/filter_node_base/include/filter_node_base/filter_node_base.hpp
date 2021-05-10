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

/// \copyright Copyright 2021 The Autoware Foundation
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

/** \brief For parameter service callback */
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
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;
  typedef sensor_msgs::msg::PointCloud2::ConstSharedPtr PointCloud2ConstPtr;

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;

  /** \brief The default constructor for the FilterNodeBase class
   * \param filter_name The name of the node to pass on to rclcpp::Node
   * \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
   */
  explicit FilterNodeBase(
    const std::string & filter_name = "filter_node_base",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_filter_;

  /** \brief The input PointCloud2 subscriber */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief The desired user filter field name */
  std::string filter_field_name_;

  /** \brief Internal mutex */
  std::mutex mutex_;

  /** \brief The maximum queue size. */
  size_t max_queue_size_;

  /** \brief Virtual abstract filter method called by the computePublish method at the arrival of each pointcloud message.
   * \param input The input point cloud dataset.
   * \param indices A pointer to the vector of point indices to use.
   * \param output The resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output) = 0;

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input The input point cloud dataset.
   * \param indices A pointer to the vector of point indices to use.
   */
  void compute_publish(const PointCloud2ConstPtr & msg);

  /** \brief Validate a sensor_msgs::msg::PointCloud2 message
   *
   * Method ensure thats the size of the pointcloud defined by the width,
   * height and point_step correspond to the data size.
   *
   * \param msg Input sensor_msgs::msg::PointCloud2 to be validated
   */
  inline bool is_valid(const PointCloud2ConstPtr & cloud)
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
        "and frame %sreceived!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
      return false;
    }
    return true;
  }

private:
  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & p);

  /** \brief PointCloud2 */
  void pointcloud_callback(const PointCloud2ConstPtr cloud);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace filter_node_base
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
