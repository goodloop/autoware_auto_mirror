// Copyright 2020-2021 Arm Ltd., TierIV
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

#ifndef APOLLO_LIDAR_SEGMENTATION__APOLLO_LIDAR_SEGMENTATION_HPP_
#define APOLLO_LIDAR_SEGMENTATION__APOLLO_LIDAR_SEGMENTATION_HPP_

#include <tvm_vendor/dlpack/dlpack.h>
#include <tvm_vendor/tvm/runtime/module.h>
#include <tvm_vendor/tvm/runtime/packed_func.h>
#include <tvm_vendor/tvm/runtime/registry.h>

#include <apollo_lidar_segmentation/cluster2d.hpp>
#include <apollo_lidar_segmentation/feature_generator.hpp>
#include <apollo_lidar_segmentation/visibility_control.hpp>
#include <tvm_utility/pipeline.hpp>

#include <autoware_auto_perception_msgs/msg/bounding_box_array.hpp>
#include <common/types.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace apollo_lidar_segmentation
{
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware_auto_perception_msgs::msg::BoundingBoxArray;
using tvm_utility::pipeline::TVMArrayContainer;
using tvm_utility::pipeline::TVMArrayContainerVector;

/// \brief Pre-precessing step of the TVM pipeline.
class APOLLO_LIDAR_SEGMENTATION_LOCAL ApolloLidarSegmentationPreProcessor
  : public tvm_utility::pipeline::PreProcessor<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  /// \param[in] range The range of the 2D grid.
  /// \param[in] use_intensity_feature Enable input channel intensity feature.
  /// \param[in] use_constant_feature Enable input channel constant feature.
  /// \param[in] min_height The minimum height.
  /// \param[in] max_height The maximum height.
  explicit ApolloLidarSegmentationPreProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config, int32_t range,
    bool8_t use_intensity_feature, bool8_t use_constant_feature, float32_t min_height,
    float32_t max_height);

  /// \brief Transfer the input data to a TVM array.
  /// \param[in] pc_ptr Input pointcloud.
  /// \return A TVM array containing the pointcloud data.
  /// \throw std::runtime_error If the features are incorrectly configured.
  TVMArrayContainerVector schedule(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr);

private:
  const int64_t input_channels;
  const int64_t input_width;
  const int64_t input_height;
  const int64_t datatype_bytes;
  const std::shared_ptr<FeatureGenerator> feature_generator;
  TVMArrayContainer output;
};

/// \brief Post-precessing step of the TVM pipeline.
class APOLLO_LIDAR_SEGMENTATION_LOCAL ApolloLidarSegmentationPostProcessor
  : public tvm_utility::pipeline::PostProcessor<std::shared_ptr<BoundingBoxArray>>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  /// \param[in] pc_ptr Pointcloud containing the initial input information to be matched against
  ///                   the result of the inference.
  /// \param[in] range The range of the 2D grid.
  /// \param[in] objectness_thresh The threshold of objectness for filtering out non-object cells.
  /// \param[in] score_threshold The detection confidence score threshold for filtering out the
  ///                            candidate clusters.
  /// \param[in] height_thresh If it is non-negative, the points that are higher than the predicted
  ///                          object height by height_thresh are filtered out.
  /// \param[in] min_pts_num The candidate clusters with less than min_pts_num points are removed.
  explicit ApolloLidarSegmentationPostProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr, int32_t range,
    float32_t objectness_thresh, float32_t score_threshold, float32_t height_thresh,
    int32_t min_pts_num);

  /// \brief Copy the inference result.
  /// \param[in] input The result of the inference engine.
  /// \return The inferred data.
  std::shared_ptr<BoundingBoxArray> schedule(const TVMArrayContainerVector & input);

private:
  const int64_t output_channels;
  const int64_t output_width;
  const int64_t output_height;
  const int64_t datatype_bytes;
  const float32_t objectness_thresh_;
  const float32_t score_threshold_;
  const float32_t height_thresh_;
  const int32_t min_pts_num_;
  const std::shared_ptr<float32_t> inferred_data;
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc_ptr_;
  const std::shared_ptr<Cluster2D> cluster2d_;
};

/// \brief Handle the neural network inference over the input point cloud.
class APOLLO_LIDAR_SEGMENTATION_PUBLIC ApolloLidarSegmentation
{
public:
  /// \brief Constructor
  /// \param[in] range The range of the 2D grid.
  /// \param[in] score_threshold The detection confidence score threshold for filtering out the
  ///                            candidate clusters in the post-processing step.
  /// \param[in] use_intensity_feature Enable input channel intensity feature.
  /// \param[in] use_constant_feature Enable input channel constant feature.
  /// \param[in] z_offset The offset to translate up the input pointcloud before the inference.
  /// \param[in] min_height The minimum height.
  /// \param[in] max_height The maximum height.
  /// \param[in] objectness_thresh The threshold of objectness for filtering out non-object cells in
  ///                              the obstacle clustering step.
  /// \param[in] min_pts_num In the post-processing step, the candidate clusters with less than
  ///                        min_pts_num points are removed.
  /// \param[in] height_thresh If it is non-negative, the points that are higher than the predicted
  ///                          object height by height_thresh are filtered out in the
  ///                          post-processing step.
  explicit ApolloLidarSegmentation(
    int32_t range, float32_t score_threshold,
    bool8_t use_intensity_feature, bool8_t use_constant_feature, float32_t z_offset,
    float32_t min_height, float32_t max_height, float32_t objectness_thresh, int32_t min_pts_num,
    float32_t height_thresh);

  /// \brief Detect obstacles.
  /// \param[in] input Input pointcloud.
  /// \return Detected obstacles.
  /// \throw tf2::TransformException If the pointcloud transformation fails.
  /// \throw std::runtime_error If the features are incorrectly configured.
  std::shared_ptr<const BoundingBoxArray> detectDynamicObjects(
    const sensor_msgs::msg::PointCloud2 & input);

private:
  const int32_t range_;
  const float32_t score_threshold_;
  const float32_t z_offset_;
  const float32_t objectness_thresh_;
  const int32_t min_pts_num_;
  const float32_t height_thresh_;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_ptr_;

  // Pipeline
  using PrePT = ApolloLidarSegmentationPreProcessor;
  using IET = tvm_utility::pipeline::InferenceEngineTVM;
  using PostPT = ApolloLidarSegmentationPostProcessor;

  const std::shared_ptr<PrePT> PreP;
  const std::shared_ptr<IET> IE;
  const std::shared_ptr<PostPT> PostP;

  const std::shared_ptr<tvm_utility::pipeline::Pipeline<PrePT, IET, PostPT>> pipeline;

  /// \brief Move up the input pointcloud by z_offset_ and transform the pointcloud to target_frame_
  ///        if needed.
  /// \param[in] input
  /// \param[out] transformed_cloud
  /// \throw tf2::TransformException If the pointcloud transformation fails.
  void APOLLO_LIDAR_SEGMENTATION_LOCAL transformCloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud);

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);

  const std::string target_frame_ = "base_link";
};
}  // namespace apollo_lidar_segmentation
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
#endif  // APOLLO_LIDAR_SEGMENTATION__APOLLO_LIDAR_SEGMENTATION_HPP_
