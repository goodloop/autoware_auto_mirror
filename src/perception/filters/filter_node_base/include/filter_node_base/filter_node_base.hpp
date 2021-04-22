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
#include <string>
#include <vector>

#include "boost/thread/mutex.hpp"
#include "filter_node_base/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "common/types.hpp"

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

namespace sync_policies = message_filters::sync_policies;

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

  typedef pcl_msgs::msg::PointIndices PointIndices;
  typedef PointIndices::SharedPtr PointIndicesPtr;
  typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

  typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
  typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
  typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

  typedef pcl::IndicesPtr IndicesPtr;
  typedef pcl::IndicesConstPtr IndicesConstPtr;

  typedef message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>
    ExactTimeSyncPolicy;
  typedef message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>
    ApproximateTimeSyncPolicy;

  /** \brief The default constructor for the FilterNodeBase class
   * \param filter_name The name of the node to pass on to rclcpp::Node
   * \param options An rclcpp::NodeOptions object to pass on to rclcpp::Node
   */
  explicit FilterNodeBase(
    const std::string & filter_name = "filter_node_base",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief The input PointCloud2 subscriber */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief The message filter subscriber for PointCloud2 */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief The desired user filter field name */
  std::string filter_field_name_;

  /** \brief Internal mutex */
  boost::mutex mutex_;

  /** \brief Virtual abstract filter method called by the computePublish method at the arrival of each pointcloud message
   * \param input The input point cloud dataset.
   * \param indices A pointer to the vector of point indices to use.
   * \param output The resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input The input point cloud dataset.
   * \param indices A pointer to the vector of point indices to use.
   */
  void computePublish(const PointCloud2ConstPtr & input, const IndicesPtr & indices);

  //////////////////////
  // from PCLNodelet //
  //////////////////////
  /** \brief Set to true if point indices are used.
   *
   * When receiving a point cloud, if use_indices_ is false, the entire
   * point cloud is processed for the given operation. If use_indices_ is
   * true, then the ~indices topic is read to get the vector of point
   * indices specifying the subset of the point cloud that will be used for
   * the operation. In the case where use_indices_ is true, the ~input and
   * ~indices topics must be synchronised in time, either exact or within a
   * specified jitter.
   **/
  bool8_t use_indices_;
  /** \brief True if we use an approximate time synchronizer
   * versus an exact one (false by default). */
  bool8_t approximate_sync_;

  /** \brief The maximum queue size. */
  size_t max_queue_size_;

  /** \brief Validate a sensor_msgs::msg::PointCloud2 message
   *
   * Method ensure thats the size of the pointcloud defined by the width,
   * height and point_step correspond to the data size.
   *
   * \param cloud Input sensor_msgs::msg::PointCloud2 to be validated
   */
  inline bool isValid(const PointCloud2ConstPtr & cloud)
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

  /** \brief Validate a pcl_msgs::msg::PointIndices message
   *
   * Not implemented properly - just returns true
   */
  inline bool isValid(
    const PointIndicesConstPtr &, const std::string & = "indices")
  {
    return true;
  }

  /** \brief Validate a pcl_msgs::msg::ModelCoefficients message
   *
   * Not implemented properly - just returns true
   */
  inline bool isValid(
    const ModelCoefficientsConstPtr &, const std::string & = "model")
  {
    return true;
  }

private:
  /** \brief Synchronized input, and indices */
  std::shared_ptr<ExactTimeSyncPolicy> sync_input_indices_e_;
  std::shared_ptr<ApproximateTimeSyncPolicy> sync_input_indices_a_;

  /** \brief PointCloud2 + Indices data callback */
  void input_indices_callback(const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace filter_node_base
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
