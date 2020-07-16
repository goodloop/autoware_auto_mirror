// Copyright 2019 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ndt/ndt_localizer.hpp>
#include <optimization/newtons_method_optimizer.hpp>
#include <optimization/line_search/fixed_line_search.hpp>
#include <common/types.hpp>
#include <limits>
#include <utility>

#include "test_ndt_optimization.hpp"
#include "test_ndt_utils.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::NdtLocalizer;
using autoware::localization::ndt::P2DNDTLocalizerConfig;
using autoware::localization::ndt::P2DNDTOptimizationConfig;
using autoware::localization::ndt::transform_adapters::pose_to_transform;
using autoware::localization::ndt::transform_adapters::transform_to_pose;

using autoware::common::optimization::FixedLineSearch;
using NewtonOptimizer = autoware::common::optimization::NewtonsMethodOptimizer<FixedLineSearch>;
using OptimizerOptions = autoware::common::optimization::OptimizationOptions;
using FixedLineSearch = autoware::common::optimization::FixedLineSearch;
using StaticNDTMap = autoware::localization::ndt::StaticNDTMap;
using P2DNDTOptimizationProblem =
  autoware::localization::ndt::P2DNDTOptimizationProblem<StaticNDTMap>;
using P2DTestLocalizer = NdtLocalizer<NewtonOptimizer, P2DNDTOptimizationProblem>;

using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;

class P2DLocalizerTest : public OptimizationTestContext, public ::testing::Test
{
public:
  P2DLocalizerTest()
  : m_localizer_config{
      P2DNDTOptimizationConfig{0.45},
      OptimizerOptions{5U, 0.0002, 0.0002, 1e-4},
      m_grid_config,
      m_downsampled_cloud.width,
      m_guess_time_tol} {}

protected:
  void SetUp() override
  {
    ASSERT_EQ(m_dynamic_map.size(), m_static_map.size());
    for (const auto & pt_it : m_voxel_centers) {
      const auto static_vx = m_static_map.cell(pt_it.second)[0U];
      const auto dynamic_vx = m_dynamic_map.cell(pt_it.second)[0U];
      ASSERT_TRUE(static_vx.usable());
      ASSERT_TRUE(dynamic_vx.usable());

      ASSERT_TRUE(static_vx.centroid().isApprox(dynamic_vx.centroid(),
        std::numeric_limits<Real>::epsilon()));
      ASSERT_TRUE(static_vx.inverse_covariance().isApprox(
          dynamic_vx.inverse_covariance(),
          std::numeric_limits<Real>::epsilon() * 1e2));
    }
  }

  const std::chrono::milliseconds m_guess_time_tol{10};
  P2DNDTLocalizerConfig<OptimizerOptions> m_localizer_config;
  float32_t m_step_size{0.12};
};

class P2DLocalizerParameterTest : public P2DLocalizerTest,
  public ::testing::WithParamInterface<PoseParams> {};


TEST_P(P2DLocalizerParameterTest, sanity_test) {
  /////////////// Initialization
  const auto map_time = std::chrono::system_clock::now();
  const auto scan_time = std::chrono::system_clock::now() + std::chrono::seconds(10);

  constexpr auto translation_tol = 1e-2;
  constexpr auto rotation_tol = 1e-2;
  const auto & param = GetParam();
  EigenPose<Real> diff = param.pose;
  TransformStamped diff_tf2;

  EigenPose<Real> pose_out;

  TransformStamped transform_initial;

  // Set scan and guess time stamp to be relatively later as in a realistic use case.
  m_downsampled_cloud.header.stamp = ::time_utils::to_message(scan_time);
  transform_initial.header.stamp = ::time_utils::to_message(scan_time);
  transform_initial.transform.rotation.w = 1.0;

  // Convert eigen pose to ros transform
  pose_to_transform(diff, diff_tf2.transform);
  diff_tf2.header.frame_id = "custom";

  // Ensure the scan won't be translated out of its voxel
  ASSERT_LT(diff(0), m_voxel_size.x);
  ASSERT_LT(diff(1), m_voxel_size.y);
  ASSERT_LT(diff(2), m_voxel_size.z);

  // Prepare the scan measurement by translating the correct scan by a diff.
  auto translated_cloud = m_downsampled_cloud;
  tf2::doTransform(m_downsampled_cloud, translated_cloud, diff_tf2);
  translated_cloud.header.stamp = ::time_utils::to_message(scan_time);

  // Prepare the map cloud.
  auto map_cloud = dynamic_map_to_cloud(m_dynamic_map);
  map_cloud.header.stamp = ::time_utils::to_message(map_time);

  StaticNDTMap map{m_localizer_config.map_config()};
  map.insert(map_cloud);

  P2DTestLocalizer localizer{
    NewtonOptimizer{FixedLineSearch{m_step_size}, m_localizer_config.optimizer_options()},
    m_localizer_config.optimization_config().outlier_ratio(),
    m_localizer_config.guess_time_tolerance()};

  PoseWithCovarianceStamped ros_pose_out{};
  localizer.register_measurement(translated_cloud, map, transform_initial, ros_pose_out);

  transform_to_pose(ros_pose_out.pose.pose, pose_out);

  EigenPose<Real> neg_diff = -diff;
  is_pose_approx(pose_out, neg_diff, translation_tol, rotation_tol);

  std::cout << "The scan is transformed away from the map by: \n" << diff << std::endl;
  std::cout << "The estimated pose difference after \n" << pose_out << std::endl;
}

INSTANTIATE_TEST_CASE_P(sanity_test, P2DLocalizerParameterTest,
  ::testing::Values(
    PoseParams{0.0, 0.65, 0.0, 0.0, 0.0, 0.0},
    PoseParams{0.7, 0.0, 0.7, 0.0, 0.0, 0.0},
    PoseParams{0.0, 0.1, 0.1, 0.0, M_PI / 72.0, 0.0},
    PoseParams{0.0, -0.2, 0.0, 0.0, M_PI / 72.0, M_PI / 72.0}
  ), );


TEST_F(P2DLocalizerParameterTest, delayed_scan) {
  TransformStamped transform_initial;

  const auto now = std::chrono::system_clock::now();
  const auto dt = std::chrono::milliseconds(1);

  // Set scan time.
  m_downsampled_cloud.header.stamp = ::time_utils::to_message(now - dt);
  transform_initial.header.stamp = ::time_utils::to_message(now - dt);
  // set map time:
  auto map_cloud = dynamic_map_to_cloud(m_dynamic_map);
  map_cloud.header.stamp = ::time_utils::to_message(now + dt);

  StaticNDTMap map{m_localizer_config.map_config()};
  map.insert(map_cloud);

  P2DTestLocalizer localizer{
    NewtonOptimizer{FixedLineSearch{m_step_size}, m_localizer_config.optimizer_options()},
    m_localizer_config.optimization_config().outlier_ratio(),
    m_localizer_config.guess_time_tolerance()};

  PoseWithCovarianceStamped dummy_pose;
  EXPECT_THROW(
    localizer.register_measurement(m_downsampled_cloud, map, transform_initial, dummy_pose),
    std::logic_error
  );
}

TEST_F(P2DLocalizerParameterTest, async_initial_guess) {
  TransformStamped transform_initial{};
  PoseWithCovarianceStamped ros_pose_out{};


  const auto now = std::chrono::system_clock::now();
  constexpr auto dt = std::chrono::milliseconds(1);

  // The following assumption will be used for the test to work.
  ASSERT_LT(dt, m_guess_time_tol);

  const auto map_time = now;
  const auto scan_time = now + dt;
  const auto guess_time_tolerated1 = scan_time + m_guess_time_tol - dt;
  const auto guess_time_tolerated2 = scan_time - m_guess_time_tol + dt;
  const auto guess_time_tolerated3 = scan_time + m_guess_time_tol;
  const auto guess_time_tolerated4 = scan_time - m_guess_time_tol;
  const auto guess_time_late = scan_time + m_guess_time_tol + dt;
  const auto guess_time_early = scan_time - m_guess_time_tol - dt;

  // Set scan time.
  m_downsampled_cloud.header.stamp = ::time_utils::to_message(scan_time);
  // set map time:
  auto map_cloud = dynamic_map_to_cloud(m_dynamic_map);
  map_cloud.header.stamp = ::time_utils::to_message(map_time);

  StaticNDTMap map{m_localizer_config.map_config()};
  map.insert(map_cloud);

  P2DTestLocalizer localizer{
    NewtonOptimizer{FixedLineSearch{m_step_size}, m_localizer_config.optimizer_options()},
    m_localizer_config.optimization_config().outlier_ratio(),
    m_localizer_config.guess_time_tolerance()};

  auto set_and_get = [&transform_initial](auto time_point) {
      transform_initial.header.stamp = ::time_utils::to_message(time_point);
      return transform_initial;
    };
  PoseWithCovarianceStamped dummy_pose;
  EXPECT_NO_THROW(
    localizer.register_measurement(m_downsampled_cloud, map, set_and_get(scan_time), dummy_pose)
  );
  EXPECT_NO_THROW(localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_tolerated1), dummy_pose));
  EXPECT_NO_THROW(localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_tolerated2), dummy_pose));
  EXPECT_NO_THROW(localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_tolerated3), dummy_pose));
  EXPECT_NO_THROW(localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_tolerated4), dummy_pose));

  EXPECT_THROW(
    localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_late), dummy_pose),
    std::domain_error);
  EXPECT_THROW(
    localizer.register_measurement(
      m_downsampled_cloud, map, set_and_get(guess_time_early), dummy_pose),
    std::domain_error);
}
