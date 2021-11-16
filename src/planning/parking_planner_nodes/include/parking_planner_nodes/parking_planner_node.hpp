// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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
#ifndef PARKING_PLANNER_NODES__PARKING_PLANNER_NODE_HPP_
#define PARKING_PLANNER_NODES__PARKING_PLANNER_NODE_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <parking_planner_nodes/visibility_control.hpp>
#include <parking_planner/parking_planner.hpp>

#include <trajectory_planner_node_base/trajectory_planner_node_base.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_planning_msgs/action/plan_trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <motion_common/motion_common.hpp>
#include <motion_common/config.hpp>
#include <common/types.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>
#include <vector>
#include <thread>
#include <future>

using autoware::common::types::float64_t;
using motion::motion_common::VehicleConfig;
using motion::motion_common::Real;

namespace autoware
{
namespace motion
{
namespace planning
{
namespace parking_planner_nodes
{
using PlannerPtr = std::unique_ptr<autoware::motion::planning::parking_planner::ParkingPlanner>;
using HADMapService = autoware_auto_mapping_msgs::srv::HADMapService;
using HADMapRoute = autoware_auto_planning_msgs::msg::HADMapRoute;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using ParkerNLPCostWeights = autoware::motion::planning::parking_planner::NLPCostWeights<float64_t>;
using ParkerVehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using ParkerVehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;
using ParkingPolytope = autoware::motion::planning::parking_planner::Polytope2D<float64_t>;
using ParkingPlanner = autoware::motion::planning::parking_planner::ParkingPlanner;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using AutowareTrajectory = autoware_auto_planning_msgs::msg::Trajectory;

class PARKING_PLANNER_NODES_PUBLIC ParkingPlannerNode : public
  autoware::trajectory_planner_node_base::TrajectoryPlannerNodeBase
{
public:
  explicit ParkingPlannerNode(const rclcpp::NodeOptions & options);

protected:
  HADMapService::Request create_map_request(const HADMapRoute & had_map_route);

  AutowareTrajectory plan_trajectory(
    const HADMapRoute & had_map_route,
    const lanelet::LaneletMapPtr & lanelet_map_ptr);

  PlannerPtr m_planner{nullptr};

  // Debug topics
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_debug_obstacles_publisher;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    m_debug_trajectory_publisher;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::BoundingBoxArray>::SharedPtr
    m_debug_start_end_publisher;

private:
  PARKING_PLANNER_NODES_LOCAL void init(
    const VehicleConfig & vehicle_param,
    const ParkerNLPCostWeights & optimization_weights,
    const ParkerVehicleState & lower_state_bounds,
    const ParkerVehicleState & upper_state_bounds,
    const ParkerVehicleCommand & lower_command_bounds,
    const ParkerVehicleCommand & upper_command_bounds
  );

  PARKING_PLANNER_NODES_LOCAL void debug_publish_obstacles(
    const std::vector<ParkingPolytope> & obstacles);

  PARKING_PLANNER_NODES_LOCAL void debug_publish_start_and_end(
    const ParkerVehicleState & start,
    const ParkerVehicleState & end);

  PARKING_PLANNER_NODES_LOCAL void debug_publish_trajectory(
    const AutowareTrajectory & trajectory);
};  // class parkingPlannerNode
}  // namespace parking_planner_nodes
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER_NODES__PARKING_PLANNER_NODE_HPP_
