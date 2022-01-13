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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include <geometry/common_2d.hpp>
#include <global_velocity_planner/global_velocity_planner.hpp>
#include <had_map_utils/had_map_utils.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
namespace autoware
{
namespace global_velocity_planner
{

using autoware_auto_planning_msgs::msg::HADMapRoute;

float distance_calculate(TrajectoryPoint p1, TrajectoryPoint p2)
{
  return static_cast<float>(sqrt(
           (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x) +
           (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y)));
}

size_t get_closest_lanelet(const lanelet::ConstLanelets & lanelets, const TrajectoryPoint & point)
{
  float64_t closest_distance = std::numeric_limits<float64_t>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto & llt = lanelets.at(i);
    const auto & point2d =
      lanelet::Point2d(lanelet::InvalId, point.pose.position.x, point.pose.position.y)
      .basicPoint2d();
    const float64_t distance = lanelet::geometry::distanceToCenterline2d(llt, point2d);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}
autoware_auto_planning_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(
  const lanelet::ConstPoint3d & pt, const float32_t velocity)
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
  trajectory_point.pose.position.y = pt.y();
  trajectory_point.pose.position.x = pt.x();
  trajectory_point.longitudinal_velocity_mps = velocity;
  return trajectory_point;
}

lanelet::Point3d convertToLaneletPoint(const autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return lanelet::Point3d(lanelet::InvalId, pt.pose.position.x, pt.pose.position.y, 0.0);
}

float find_acceleration(TrajectoryPoint p1, TrajectoryPoint p2)
{
  return (p2.longitudinal_velocity_mps * p2.longitudinal_velocity_mps -
         p1.longitudinal_velocity_mps * p1.longitudinal_velocity_mps) /
         (2 * distance_calculate(p1, p2));
}

float find_velocity(TrajectoryPoint p1, TrajectoryPoint p2, float longitudinal_acceleration)
{
  return sqrt(
    (p1.longitudinal_velocity_mps * p1.longitudinal_velocity_mps) +
    (2 * (longitudinal_acceleration)) * distance_calculate(p1, p2));
}

GlobalVelocityPlanner::GlobalVelocityPlanner(
  const VehicleConfig & vehicle_param, const GlobalVelocityPlannerConfig & planner_config)
: vehicle_param(vehicle_param), velocity_planner_config(planner_config), last_point(0)
{
  map = std::make_shared<lanelet::LaneletMap>();
  way_points = std::make_shared<std::vector<point>>();
  route = std::make_shared<HADMapRoute>();
  last_point = 0;
  is_route_ready = false;
}

void GlobalVelocityPlanner::set_route(
  const HADMapRoute & route, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  *this->route = route;
  this->map = lanelet_map_ptr;
}

bool GlobalVelocityPlanner::is_route_empty()
{
  return this->route->segments.empty();
}

void GlobalVelocityPlanner::clear_route()
{
  this->route->segments.clear();
  this->last_point = 0;
  this->trajectory.points.clear();
  this->way_points->clear();
  this->is_route_ready = false;
}

bool GlobalVelocityPlanner::is_route_over()
{
  return last_point == way_points->size() - 2;  // because mpc dont look closest trajectory index
}

void GlobalVelocityPlanner::calculate_waypoints()
{
  is_route_ready = false;
  lanelet::ConstLanelets lanelets;

  using lanelet::utils::to2D;

  for (const auto & segment : route->segments) {
    const auto & primitive = segment.primitives.front();
    try {
      const auto lane = this->map->laneletLayer.get(primitive.id);


      lanelets.push_back(lane);
    } catch (const lanelet::NoSuchPrimitiveError & ex) {
      // stop adding lanelets if lane cannot be found. e.g. goal is outside of queried submap
      break;
    }
  }
  if (lanelets.empty()) {
    std::cout << "no lanelet" << std::endl;
    return;
  }

  TrajectoryPoint start_point, final_point;
  point start, final;
  start_point.pose = route->start_pose;
  start.point = start_point;

  final_point.pose = route->goal_pose;
  final.point = final_point;
  //    std::cout << start_point.x<< ", " <<  start_point.y << std::endl;


  // fill the vector
  size_t start_index = get_closest_lanelet(lanelets, start_point);
  //    std::cout << start_index<< ", " <<  final_index << std::endl;

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr =
    lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  for (size_t i = start_index; i < lanelets.size(); i++) {
    const auto & lanelet = lanelets.at(i);
    const auto & centerline = autoware::common::had_map_utils::generateFineCenterline(
      lanelet, static_cast<float64_t>(velocity_planner_config.trajectory_resolution));
    const auto speed_limit =
      static_cast<float32_t>(traffic_rules_ptr->speedLimit(lanelet).speedLimit.value());
    if (i == start_index) {
      start.speed_limit = speed_limit;
      way_points->push_back(start);  // push the initial point
    }
    float64_t start_length = 0;
    if (i == start_index) {
      start_length = lanelet::geometry::toArcCoordinates(
        to2D(centerline), to2D(convertToLaneletPoint(start_point)))
        .length;
    }
    auto end_length = static_cast<float64_t>(std::numeric_limits<float32_t>::max());
    if (i == lanelets.size() - 1) {
      const auto goal_point = convertToLaneletPoint(final_point);
      end_length = lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(goal_point)).length;
    }
    float64_t accumulated_length = 0;

    for (size_t j = 1; j < centerline.size(); j++) {
      const auto llt_prev_pt = centerline[j - 1];
      const auto llt_pt = centerline[j];
      accumulated_length += lanelet::geometry::distance2d(to2D(llt_prev_pt), to2D(llt_pt));
      if (accumulated_length < start_length) {
        continue;
      }
      if (accumulated_length > end_length) {
        break;
      }
      point temp;
      temp.point = convertToTrajectoryPoint(llt_pt, 0);

      temp.speed_limit = speed_limit;
      if (temp.point.pose != way_points->at(way_points->size() - 1).point.pose) {
        way_points->push_back(temp);
      }
    }
  }
  GlobalVelocityPlanner::calculate_curvatures();
  GlobalVelocityPlanner::vel_wrt_lateral_acceleration();
  GlobalVelocityPlanner::vel_wrt_longitudinal_acceleration();
  GlobalVelocityPlanner::set_acceleration();
  std::cout << " SIZE IS = " << way_points->size()<< std::endl;
  for(size_t i = 0; i < way_points->size(); i++){
    auto & p = way_points->at(i).point;
    if (!isfinite(p.pose.position.x))
    {
      std::cout << "At index = " << i << " p.pose.position.x not finite" << std::endl;
    }
    if (!isfinite(p.pose.position.y))
    {
      std::cout << "At index = " << i << " p.pose.position.y not finite" << std::endl;
    }
    if (!isfinite(p.pose.orientation.w))
    {
      std::cout << "At index = " << i << " p.pose.orientation.w not finite" << std::endl;
    }
    if (!isfinite(p.pose.orientation.x))
    {
      std::cout << "At index = " << i << " p.pose.orientation.x not finite" << std::endl;
    }
    if (!isfinite(p.pose.orientation.y))
    {
      std::cout << "At index = " << i << " p.pose.orientation.y not finite" << std::endl;
    }
    if (!isfinite(p.pose.orientation.z))
    {
      std::cout << "At index = " << i << " p.pose.orientation.z not finite" << std::endl;
    }
    if (!isfinite(p.longitudinal_velocity_mps))
    {
      std::cout << "At index = " << i << " p.longitudinal_velocity_mps not finite" << std::endl;
    }
    if (!isfinite(p.lateral_velocity_mps))
    {
      std::cout << "At index = " << i << " p.lateral_velocity_mps not finite" << std::endl;
    }
    if (!isfinite(p.heading_rate_rps))
    {
      std::cout << "At index = " << i << " p.heading_rate_rps not finite" << std::endl;
    }
    if (!isfinite(p.front_wheel_angle_rad))
    {
      std::cout << "At index = " << i << " p.front_wheel_angle_rad not finite: " << p.front_wheel_angle_rad << std::endl;
    }
    if (!isfinite(p.rear_wheel_angle_rad))
    {
      std::cout << "At index = " << i << " p.rear_wheel_angle_rad not finite" << std::endl;
    }
  }
  is_route_ready = true;
}

void GlobalVelocityPlanner::set_steering_angle(point & pt)
{
  // TODO(berkay6): Add the steering angle for the first point
  const auto wheel_base =
    vehicle_param.length_cg_front_axel() + vehicle_param.length_cg_rear_axel();
  if (std::isnan(pt.curvature)) {
    pt.point.front_wheel_angle_rad = 0.0;
    return;
  }
  pt.point.front_wheel_angle_rad = std::atan(wheel_base * pt.curvature);
  std::cout << pt.point.front_wheel_angle_rad << std::endl;
}

void GlobalVelocityPlanner::set_orientation(size_t i)
{
  float64_t angle = 0;
  auto & pt = way_points->at(i).point;
  if (i + 1 < way_points->size()) {
    const auto & next_pt = way_points->at(i + 1).point;
    angle = std::atan2(
      next_pt.pose.position.y - pt.pose.position.y, next_pt.pose.position.x - pt.pose.position.x);
  } else if (i != 0) {
    const auto & prev_pt = way_points->at(i - 1).point;
    angle = std::atan2(
      pt.pose.position.y - prev_pt.pose.position.y, pt.pose.position.x - prev_pt.pose.position.x);
  }
  pt.pose.orientation = motion::motion_common::from_angle(angle);
}

void GlobalVelocityPlanner::calculate_curvatures()
{
  for (size_t i = 0; i < way_points->size(); i++) {
    if (i == 0 || i == way_points->size() - 1) {
      way_points->at(i).curvature = NAN;  // first and last points can not have curvature values
    } else {
      // Menger Curvature calculation
      auto area = static_cast<float>(
        abs(
          way_points->at(i - 1).point.pose.position.x *
          (way_points->at(i).point.pose.position.y -
          way_points->at(i + 1).point.pose.position.y) +
          way_points->at(i).point.pose.position.x * (way_points->at(i + 1).point.pose.position.y -
          way_points->at(i - 1).point.pose.position.y) +
          way_points->at(i + 1).point.pose.position.x *
          (way_points->at(i - 1).point.pose.position.y -
          way_points->at(i).point.pose.position.y)) /
        2);
      way_points->at(i).curvature =
        (4 * area /
        (distance_calculate(way_points->at(i + 1).point, way_points->at(i).point) *
        distance_calculate(way_points->at(i - 1).point, way_points->at(i).point) *
        distance_calculate(way_points->at(i + 1).point, way_points->at(i - 1).point)));
    }
    GlobalVelocityPlanner::set_steering_angle(way_points->at(i));
    GlobalVelocityPlanner::set_orientation(i);
  }
}

void GlobalVelocityPlanner::vel_wrt_lateral_acceleration()
{
  for (size_t i = 0; i < way_points->size(); i++) {
    auto & pt = way_points->at(i);
    if (i == 0 || i == way_points->size() - 1) {
      pt.point.longitudinal_velocity_mps = 0;  // initialize first and last points velocity
      continue;
    }
    float tmp = sqrt((velocity_planner_config.lateral_acceleration) / pt.curvature);
    if (tmp < pt.speed_limit) {
      pt.point.longitudinal_velocity_mps = tmp;
    } else {
      pt.point.longitudinal_velocity_mps = pt.speed_limit;
    }
  }
}

void GlobalVelocityPlanner::vel_wrt_longitudinal_acceleration()
{
  int dir = 1;
  int i = 0;
  float acceleration_offset = 0.00001f;  // we need it bcs stuckking somewhere
  while (i + dir < static_cast<int>(way_points->size())) {
    auto & pt1 = way_points->at(static_cast<size_t>(i));
    auto & pt2 = way_points->at(static_cast<size_t>(i + dir));
    if (
      find_acceleration(pt1.point, pt2.point) > velocity_planner_config.longitudinal_acceleration)
    {
      pt2.point.longitudinal_velocity_mps = find_velocity(
        pt1.point,
        pt2.point,
        velocity_planner_config.longitudinal_acceleration - acceleration_offset);
      i = i + dir;
      continue;
    }
    if (
      find_acceleration(pt1.point, pt2.point) <
      -velocity_planner_config.longitudinal_acceleration)
    {
      i = i + dir;
      dir = -dir;
    } else {
      if (dir == -1) {
        dir = -dir;
      }
      i = i + dir;
    }
  }
}

size_t GlobalVelocityPlanner::get_closest_index(const State & pose)
{
  size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  auto x = pose.state.pose.position.x;
  auto y = pose.state.pose.position.y;
  for (size_t i = 0; i < way_points->size(); i++) {
    double distance = sqrt(
      (x - way_points->at(i).point.pose.position.x) *
      (x - way_points->at(i).point.pose.position.x) +
      (y - way_points->at(i).point.pose.position.y) *
      (y - way_points->at(i).point.pose.position.y));
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

void GlobalVelocityPlanner::set_acceleration()
{
  for (size_t i = 0; i < way_points->size(); i++) {
    if (i == way_points->size() - 1) {
      way_points->at(i).point.acceleration_mps2 = 0.0f;
    } else {
      way_points->at(i).point.acceleration_mps2 =
        find_acceleration(way_points->at(i).point, way_points->at(i + 1).point);
    }
  }
}

void GlobalVelocityPlanner::set_time_from_start()
{
  if (trajectory.points.empty()) {
    return;
  }
  float32_t t = 0.0;
  // special operation for first point
  auto & first_point = trajectory.points.at(0);
  first_point.time_from_start.sec = 0;
  first_point.time_from_start.nanosec = 0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    auto v = 0.5f * (trajectory.points[i - 1].longitudinal_velocity_mps +
      trajectory.points[i].longitudinal_velocity_mps);
    t += common::geometry::norm_2d(
      common::geometry::minus_2d(trajectory.points[i - 1], trajectory.points[i])) /
      std::max(std::fabs(v), 0.5f);
    float32_t t_s = 0;
    float32_t t_ns = std::modf(t, &t_s) * 1.0e9f;
    trajectory.points[i].time_from_start.sec = static_cast<int32_t>(t_s);
    trajectory.points[i].time_from_start.nanosec = static_cast<uint32_t>(t_ns);
  }
}

bool GlobalVelocityPlanner::need_trajectory()
{
  if(trajectory.points.empty()){
    return true;
  }
  if(trajectory.points.size() < 50){
    if(way_points->size() - last_point <= trajectory.points.size()){
      return false;
    }
    return true;
  }
  return false;

}

void GlobalVelocityPlanner::calculate_trajectory(const State & pose)
{
//  const size_t front_adder = 0;
//  size_t begin = last_point + trajectory.points.size() + front_adder;
//  size_t closest_index = GlobalVelocityPlanner::get_closest_index(pose);
//  if (closest_index >= last_point) {
//    trajectory.points.erase(
//      trajectory.points.begin(),
//      trajectory.points.begin() + static_cast<uint32_t>(closest_index - last_point));
//    size_t length = std::min(
//      (autoware_auto_planning_msgs::msg::Trajectory::CAPACITY - trajectory.points.size()),
//      way_points->size() - begin);
//    for (size_t j = 0; j < length; j++) {
//      trajectory.points.push_back(way_points->at(begin + j).point);
//    }
//  }
//  else {
//    trajectory.points.resize(trajectory.points.size() - (last_point - closest_index));
//    for (size_t j = 0; j < (last_point - closest_index); j++) {
//      trajectory.points.insert(trajectory.points.begin(), way_points->at(last_point - j).point);
//    }
//  }
//  GlobalVelocityPlanner::set_time_from_start();
//  last_point = closest_index;
    size_t closest_index = GlobalVelocityPlanner::get_closest_index(pose) + 1;
    if(need_trajectory()){
      trajectory.points.clear();
      size_t length = std::min(
        (autoware_auto_planning_msgs::msg::Trajectory::CAPACITY - trajectory.points.size()),
        way_points->size() - closest_index);
      for (size_t j = 0; j < length; j++) {
        trajectory.points.push_back(way_points->at(closest_index + j).point);
      }
      GlobalVelocityPlanner::set_time_from_start();
    }
    else {
      if (closest_index > last_point) {
        std::cout << closest_index - last_point << std::endl;
        trajectory.points.erase(
          trajectory.points.begin(),
          trajectory.points.begin() + static_cast<uint32_t>(closest_index - last_point));
      }
      else{
        if(trajectory.points.size() + last_point - closest_index > trajectory.CAPACITY){
          trajectory.points.resize(trajectory.points.size() - (last_point - closest_index));
        }
        for (size_t j = 0; j < (last_point - closest_index); j++) {
        trajectory.points.insert(trajectory.points.begin(), way_points->at(last_point -1 - j).point);
        }
      }
    }
    last_point = closest_index;
}
}  // namespace global_velocity_planner
}  // namespace autoware