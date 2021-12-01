// Copyright 2020 Tier IV, Inc.
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

#include <cmath>

#include "scene_module/crosswalk/scene_walkway.hpp"
#include "utilization/util.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;
using std::int32_t;

WalkwayModule::WalkwayModule(
  const int64_t module_id, const lanelet::ConstLanelet & walkway,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  walkway_(walkway),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool WalkwayModule::modifyPathVelocity(
  autoware_auto_msgs::msg::PathWithLaneId * path)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_constants_.offset_longitudinal_max;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;

  const auto input = *path;

  if (state_ == State::APPROACH) {
    // create polygon
    lanelet::CompoundPolygon3d lanelet_polygon = walkway_.polygon3d();
    Polygon polygon;
    for (const auto & lanelet_point : lanelet_polygon) {
      polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
    }
    polygon.outer().push_back(polygon.outer().front());
    polygon = isClockWise(polygon) ? polygon : inverseClockWise(polygon);

    lanelet::Optional<lanelet::ConstLineString3d> stop_line_opt =
      getStopLineFromMap(static_cast<int32_t>(module_id_), planner_data_, "crosswalk_id");
    if (!!stop_line_opt) {
      if (!insertTargetVelocityPoint(
          input, stop_line_opt.get(), planner_param_.stop_margin, 0.0, *planner_data_, *path,
          debug_data_, first_stop_path_point_index_))
      {
        return false;
      }
    } else {
      if (!insertTargetVelocityPoint(
          input, polygon, planner_param_.stop_line_distance + planner_param_.stop_margin, 0.0,
          *planner_data_, *path, debug_data_, first_stop_path_point_index_))
      {
        return false;
      }
    }

    // update state
    const Point self_pose = {
      planner_data_->current_pose.pose.position.x, planner_data_->current_pose.pose.position.y};
    const Point stop_pose = {
      debug_data_.first_stop_pose.position.x, debug_data_.first_stop_pose.position.y};
    const double distance = bg::distance(stop_pose, self_pose);
    const double distance_threshold = 1.0;
    debug_data_.stop_judge_range = distance_threshold;
    if (distance < distance_threshold && planner_data_->isVehicleStopped()) {state_ = State::STOP;}
    return true;
  } else if (state_ == State::STOP) {
    if (planner_data_->isVehicleStopped()) {
      state_ = State::SURPASSED;
    }
  } else if (state_ == State::SURPASSED) {
  }
  return true;
}
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware
