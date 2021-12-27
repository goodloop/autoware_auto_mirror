// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef ASTAR_SEARCH__BASE_PLANNING_ALGORITHM_HPP_
#define ASTAR_SEARCH__BASE_PLANNING_ALGORITHM_HPP_

#include <astar_search/visibility_control.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <functional>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace autoware
{
namespace planning
{
namespace astar_search
{
double normalizeRadian(
  const double rad, const double min_rad = -M_PI, const double max_rad = M_PI);
int discretizeAngle(const double theta, const size_t theta_size);

enum class NodeStatus : uint8_t { None, Open, Closed, Obstacle };

struct IndexXYT
{
  int x;
  int y;
  int theta;
};

struct IndexXY
{
  int x;
  int y;
};

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const size_t theta_size);
geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const size_t theta_size);
geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global);
geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local);

/// \brief Definition of essential robot dimensions
struct ASTAR_SEARCH_PUBLIC RobotShape
{
  double length;     ///< Robot's length (bound with X axis direction) [m]
  double width;      ///< Robot's length (bound with Y axis direction)  [m]
  double cg2back;    ///< Robot's distance from center of gravity to back [m]
};

/// \brief Parameters defining algorithm configuration
struct ASTAR_SEARCH_PUBLIC AstarParam
{
  // base configs
  /// Indicate if should search for solutions in backward direction
  bool use_back;
  /// Indicate if solutions should be exclusively behind the goal
  bool only_behind_solutions;
  /// Planning time limit [msec]
  double time_limit;

  // robot configs
  /// Definition of robot shape
  RobotShape robot_shape;
  /// Minimum possible turning radius to plan trajectory [m]
  double minimum_turning_radius;
  /// Maximum possible turning radius to plan trajectory [m]
  double maximum_turning_radius;
  /// Number of levels of discretization between minimum and maximum turning radius [-]
  size_t turning_radius_size;

  // search configs
  /// Number of possible headings, discretized between <0, 2pi> [-]
  size_t theta_size;
  /// Cost of changing moving direction [-]
  double reverse_weight;
  /// Distance weight for trajectory cost estimation
  double distance_heuristic_weight;
  /// Lateral tolerance of goal pose [m]
  double goal_lateral_tolerance;
  /// Longitudinal tolerance of goal pose [m]
  double goal_longitudinal_tolerance;
  /// Angular tolerance of goal pose [rad]
  double goal_angular_tolerance;

  // costmap configs
  /// Threshold value of costmap cell to be regarded as an obstacle [-]
  int64_t obstacle_threshold;
};

struct AstarWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  bool is_back = false;
};

/// \brief Trajectory points representation as an algorithms output
struct ASTAR_SEARCH_PUBLIC AstarWaypoints
{
  std_msgs::msg::Header header;           ///< Mostly timestamp and frame information
  std::vector<AstarWaypoint> waypoints;   ///< Vector of trajectory waypoints
};

/// \brief Possible planning results
enum class SearchStatus
{
  SUCCESS,                      ///< Planning successful
  FAILURE_COLLISION_AT_START,   ///< Collision at start position detected
  FAILURE_COLLISION_AT_GOAL,    ///< Collision at goal position detected
  FAILURE_TIMEOUT_EXCEEDED,     ///< Planning timeout exceeded
  FAILURE_NO_PATH_FOUND         ///< Planner didn't manage to find path
};

/// \brief Determines if passed status is a success status
inline bool ASTAR_SEARCH_PUBLIC isSuccess(const SearchStatus & status)
{
  return status == SearchStatus::SUCCESS;
}


class BasePlanningAlgorithm
{
public:
  explicit BasePlanningAlgorithm(const AstarParam & astar_param)
  : astar_param_(astar_param)
  {
  }
  /// \brief Robot dimensions setter
  /// \param[in] robot_shape RobotShape object
  virtual void setRobotShape(const RobotShape & robot_shape) {astar_param_.robot_shape = robot_shape;}

  /// \brief Set occupancy grid for planning
  /// \param[in] costmap nav_msgs::msg::OccupancyGrid type object
  virtual void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap);

  /// \brief Create trajectory plan
  /// \param[in] start_pose Start position
  /// \param[in] goal_pose Goal position
  /// \return SearchStatus flag showing if planning succeeded or not
  virtual SearchStatus makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) = 0;

  /// \brief Check if there will be collision on generated trajectory
  /// \param[in] trajectory Generated trajectory
  /// \return True if detected collision
  bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;

  /// \brief Fetch algorithm solution
  /// \return AstarWaypoints created trajectory
  const AstarWaypoints & getWaypoints() const {return waypoints_;}

  virtual ~BasePlanningAlgorithm() {}

protected:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes);
  bool detectCollision(const IndexXYT & base_index) const;
  inline bool isOutOfRange(const IndexXYT & index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {return true;}
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {return true;}
    return false;
  }

  inline bool isObs(const IndexXYT & index) const
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    // So, basically .at() is not necessary.
    return is_obstacle_table_[static_cast<size_t>(index.y)][static_cast<size_t>(index.x)];
  }

  AstarParam astar_param_;

  // costmap as occupancy grid
  nav_msgs::msg::OccupancyGrid costmap_;

  // collision indexes cache
  std::vector<std::vector<IndexXY>> coll_indexes_table_;

  // is_obstacle's table
  std::vector<std::vector<bool>> is_obstacle_table_;

  // pose in costmap frame
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // result path
  AstarWaypoints waypoints_;

};

}  // namespace astar_search
}  // namespace planning
}  // namespace autoware

#endif  // ASTAR_SEARCH__BASE_PLANNING_ALGORITHM_HPP_
