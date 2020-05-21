// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_mapping/policies.hpp>
#include <memory>
#include <string>
#include <utility>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
template<typename MapIncrementT>
using RegistrationResult = std::pair<std::reference_wrapper<const MapIncrementT>,
    geometry_msgs::msg::PoseWithCovarianceStamped>;

/// Virtual base class of the mapper. The mapper is expected to
/// * Receive observations
/// * Register observations to the existing map
/// * Compute and pass the map increment to the used map representation and
/// export the map according to the provided policies.
/// \tparam LocalizerT Localizer to be used for registration.
/// \tparam MapIncrementT Type of increment to extend the map representation.
/// \tparam ObservationMsgT Type of observation used to build the map.
/// \tparam WriteTriggerPolicyT Policy specifying in what condition to write to a file.
/// \tparam PrefixGeneratorT Functor that generates filename prefixes for a given base prefix.
template<typename LocalizerT, typename MapIncrementT,
  typename ObservationMsgT, class WriteTriggerPolicyT, class PrefixGeneratorT>
class POINT_CLOUD_MAPPING_PUBLIC MapperBase
{
public:
  using LocalizerBase = localization::localization_common::RelativeLocalizerBase<ObservationMsgT,
      MapIncrementT, typename LocalizerT::RegistrationSummary>;
  using PoseWithCovarianceStamped = typename LocalizerBase::PoseWithCovarianceStamped;
  using TransformStamped = typename LocalizerBase::Transform;
  using LocalizerBasePtr = std::unique_ptr<LocalizerBase>;
  using MapBasePtr = std::unique_ptr<MapRepresentationBase<MapIncrementT>>;
  using RegistrationResultT = RegistrationResult<MapIncrementT>;

  /// Constructor.
  /// \param map_filename_prefix Base filename prefix that will be used to
  /// generate the file name.
  /// \param localizer Rvalue reference to the localizer pointer.
  /// \param map Rvalue reference to the map representation pointer.
  MapperBase(
    const std::string & map_filename_prefix, LocalizerBasePtr && localizer,
    MapBasePtr && map)
  : m_base_fn_prefix{map_filename_prefix}, m_localizer{std::forward<LocalizerBasePtr>(localizer)},
    m_map{std::forward<MapBasePtr>(map)} {}

  /// Insert an observation to the mapper. This observation gets registered to the existing map
  /// of the localizer. The increment is computed using the registration result and the observation
  /// and finally the increment is passed to the map representation.
  /// \param observation Observation to be used to grow the map.
  /// \param transform_initial Initial transform estimate of the observation with respect to
  /// the map. This is needed by the relative localizer.
  /// \return A pair containing the computed map increment and the associated pose estimate.
  RegistrationResultT observe(
    const ObservationMsgT & observation,
    const TransformStamped & transform_initial)
  {
    PoseWithCovarianceStamped registered_pose;
    m_localizer->register_measurement(observation, transform_initial, registered_pose);
    const auto & increment = get_map_increment(observation, registered_pose);
    update_map(increment, registered_pose);

    if (m_trigger_policy.ready(m_map)) {
      write_to_file();
    }
    return std::make_pair(std::cref(increment), registered_pose);
  }

  /// Virtual base destructor. Triggers map writing in case there is unwritten data in the map.
  virtual ~MapperBase()
  {
    write_to_file();
  }

protected:
  void write_to_file()
  {
    if (m_map->size() > 0U) {
      m_map->write(m_fn_prefix_generator.get(m_base_fn_prefix));
    }
  }

  void set_write_trigger(WriteTriggerPolicyT && write_trigger)
  {
    m_trigger_policy = std::move(write_trigger);
  }

  void set_prefix_generator(PrefixGeneratorT && prefix_generator)
  {
    m_fn_prefix_generator = std::move(prefix_generator);
  }

private:
  /// Pass the increment to the map. The default behavior is to expect that the map
  /// and the localizer have independent representations and push the increment to both
  /// the underlying map representation and the localizer.
  /// \param increment Increment to pass to the maps.
  /// \param pose Pose to be inserted with the increment.
  virtual void update_map(const MapIncrementT & increment, PoseWithCovarianceStamped pose)
  {
    // By default two distinct maps for the localizer and the mapper are assumed, so they are
    // inserted separately.
    const auto insert_summary = m_map->try_add_observation(increment, pose);
    switch (insert_summary.update_type) {
      case MapUpdateType::NEW:
        m_localizer->set_map(increment);
        break;
      case MapUpdateType::UPDATE:
        m_localizer->insert_to_map(increment);
        break;
      case MapUpdateType::NO_CHANGE:
      default:
        break;
    }
  }

  /// Concert the observation to a map increment ready to be inserted to the map.
  /// \param observation Observation to be used for computing the increment.
  /// \param registered_pose Registered pose of the observation.
  /// \return The map increment that is desired by the map representation.
  virtual const MapIncrementT & get_map_increment(
    const ObservationMsgT & observation,
    const PoseWithCovarianceStamped & registered_pose) = 0;

  std::string m_base_fn_prefix;
  LocalizerBasePtr m_localizer;
  MapBasePtr m_map;
  WriteTriggerPolicyT m_trigger_policy;
  PrefixGeneratorT m_fn_prefix_generator;
};

/// PointCloudMapper creates maps from point cloud observations.
/// \tparam LocalizerT Localizer type used for registration.
/// \tparam WriteTriggerPolicyT Policy specifying in what condition to write to a file.
/// \tparam PrefixGeneratorT Functor that generates filename prefixes for a given base prefix.
template<typename LocalizerT, class WriteTriggerPolicyT, class FileNamePrefixGeneratorT>
class PointCloudMapper : public MapperBase<LocalizerT,
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
    WriteTriggerPolicyT, FileNamePrefixGeneratorT>
{
public:
  using ParentT = MapperBase<LocalizerT,
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
      WriteTriggerPolicyT, FileNamePrefixGeneratorT>;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Cloud = sensor_msgs::msg::PointCloud2;

  /// Constructor.
  /// \param map_filename_prefix Base filename prefix that will be used to
  /// generate the file name.
  /// \param localizer Rvalue reference to the localizer pointer.
  /// \param map Rvalue reference to the map representation pointer.
  /// \param map_frame Map frame id.
  explicit PointCloudMapper(
    const std::string & map_filename_prefix,
    typename ParentT::LocalizerBasePtr && localizer,
    typename ParentT::MapBasePtr && map, const std::string & map_frame)
  : ParentT(map_filename_prefix, std::forward<typename ParentT::LocalizerBasePtr>(localizer),
      std::forward<typename ParentT::MapBasePtr>(map))
  {
    common::lidar_utils::init_pcl_msg(m_cached_increment, map_frame);
  }

private:
  /// Transform the observed point cloud into the map frame using the registered
  /// pose.
  /// \param observation Point cloud observation.
  /// \param registered_pose Registered pose of the observation.
  /// \return The transformed point cloud;
  const Cloud & get_map_increment(
    const Cloud & observation,
    const PoseWithCovarianceStamped & registered_pose) override
  {
    reset_cached_msg(get_msg_size(observation));
    // Convert pose to transform for `doTransform()`
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = registered_pose.header.stamp;
    tf.header.frame_id = registered_pose.header.frame_id;
    tf.child_frame_id = observation.header.frame_id;
    const auto & trans = registered_pose.pose.pose.position;
    const auto & rot = registered_pose.pose.pose.orientation;
    tf.transform.translation.set__x(trans.x).set__y(trans.y).set__z(trans.z);
    tf.transform.rotation.set__x(rot.x).set__y(rot.y).set__z(rot.z).set__w(rot.w);

    tf2::doTransform(observation, m_cached_increment, tf);
    return m_cached_increment;
  }

private:
  void reset_cached_msg(std::size_t size)
  {
    sensor_msgs::PointCloud2Modifier inc_modifier{m_cached_increment};
    inc_modifier.clear();
    inc_modifier.resize(size);
  }
  std::size_t get_msg_size(const Cloud & msg)
  {
    const auto safe_indices = common::lidar_utils::sanitize_point_cloud(msg);
    // Only do the division when necessary.
    return (safe_indices.data_length == msg.data.size()) ?
           msg.width : (safe_indices.data_length / safe_indices.point_step);
  }

  Cloud m_cached_increment;
};

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
