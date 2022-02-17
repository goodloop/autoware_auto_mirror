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

#ifndef ASTAR_SEARCH__ASTAR_SEARCH_HPP_
#define ASTAR_SEARCH__ASTAR_SEARCH_HPP_

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

#include "astar_search/base_planning_algorithm.hpp"
// #include "astar_search/reeds_shepp.hpp"

namespace autoware
{
namespace planning
{
namespace astar_search
{
struct AstarNode
{
  NodeStatus status = NodeStatus::None;  // node status
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  double gc = 0;                         // actual cost
  double hc = 0;                         // heuristic cost
  bool is_back;                          // true if the current direction of the vehicle is back
  AstarNode * parent = nullptr;          // parent node

  double cost() const {return gc + hc;}
};

struct NodeComparison
{
  bool operator()(const AstarNode * lhs, const AstarNode * rhs)
  {
    return lhs->cost() > rhs->cost();
  }
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double shift_theta;
  double step;
  bool is_back;

  NodeUpdate rotated(const double theta) const
  {
    NodeUpdate result = *this;
    result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
    result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
    return result;
  }

  NodeUpdate flipped() const
  {
    NodeUpdate result = *this;
    result.shift_y = -result.shift_y;
    result.shift_theta = -result.shift_theta;
    return result;
  }

  NodeUpdate reversed() const
  {
    NodeUpdate result = *this;
    result.shift_x = -result.shift_x;
    result.shift_theta = -result.shift_theta;
    result.is_back = !result.is_back;
    return result;
  }
};

/// \class AstarSearch
/// \brief A* Hybrid algorithm implementation using ROS2 typical structures
class ASTAR_SEARCH_PUBLIC AstarSearch : public BasePlanningAlgorithm
{
public:
  using TransitionTable = std::vector<std::vector<NodeUpdate>>;

  /// \brief Default and only constructor for AstarSearch class
  /// \param[in] astar_param Hybrid A* algorithm configuration parameters
  explicit AstarSearch(const AstarParam & astar_param);

  /// \brief Robot dimensions setter
  /// \param[in] robot_shape RobotShape object
  // void setRobotShape(const RobotShape & robot_shape) {astar_param_.robot_shape = robot_shape;}

  /// \brief Set occupancy grid for planning
  /// \param[in] costmap nav_msgs::msg::OccupancyGrid type object
  void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap) override;

  /// \brief Create trajectory plan
  /// \param[in] start_pose Start position
  /// \param[in] goal_pose Goal position
  /// \return SearchStatus flag showing if planning succeeded or not
  SearchStatus makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) override;


private:
  SearchStatus search();
  void setPath(const AstarNode & goal);
  bool setStartNode();
  bool setGoalNode() const;
  double estimateCost(const geometry_msgs::msg::Pose & pose) const;


  bool isOutOfRange(const IndexXYT & index) const override;
  bool isObs(const IndexXYT & index) const override;
  bool isGoal(const AstarNode & node) const;

  AstarNode * getNodeRef(const IndexXYT & index);

  // hybrid astar variables
  TransitionTable transition_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison> openlist_;
};

}  // namespace astar_search
}  // namespace planning
}  // namespace autoware

#endif  // ASTAR_SEARCH__ASTAR_SEARCH_HPP_
