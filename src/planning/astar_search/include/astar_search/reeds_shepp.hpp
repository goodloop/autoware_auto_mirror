// Copyright 2021 Tier IV, Inc. All rights reserved.
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
/*
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Guan-Horng Liu.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Guan-Horng Liu
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef ASTAR_SEARCH__REEDS_SHEPP_HPP_
#define ASTAR_SEARCH__REEDS_SHEPP_HPP_

#include <cassert>
#include <functional>
#include <limits>

namespace autoware
{
namespace planning
{
namespace parking
{

struct ReedsSheppNode
{
  double x;
  double y;
  double phi;

  ReedsSheppNode timeflipped() const
  {
    ReedsSheppNode node = *this;
    node.x = -1.0 * node.x;
    node.phi = -1.0 * node.phi;
    return node;
  }

  ReedsSheppNode reflected() const
  {
    ReedsSheppNode node = *this;
    node.y = -1.0 * node.y;
    node.phi = -1.0 * node.phi;
    return node;
  }
};

// struct ReedsSheppPathParameter
// {
//   double t = std::numeric_limits<double>::max();
//   double u = 0.0;
//   double v = 0.0;
//   double w = 0.0;
//   double x = 0.0;

//   ReedsSheppPathParameter timeflipped() const
//   {
//     ReedsSheppPathParameter parameters = *this;
//   }

//   ReedsSheppPathParameter reflected() const
//   {

//   }
// }

class ReedsSheppStateSpace
{
public:
  /** \brief The Reeds-Shepp path segment types */
  enum ReedsSheppPathSegmentType {NO_OPERATION, COUNTERCLOCKWISE, STRAIGHT, CLOCKWISE};

  ///? TODO change to vector (attention line 116)
  /** \brief Reeds-Shepp path types */
  static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

  /** \brief Complete description of a ReedsShepp path */
  class ReedsSheppPath
  {
public:
    ReedsSheppPath(
      const ReedsSheppPathSegmentType * type = reedsSheppPathType[0],
      double t = std::numeric_limits<double>::max(),
      double u = 0.0,
      double v = 0.0,
      double w = 0.0,
      double x = 0.0);

    double length() const {return totalLength_;}

    /** Path segment types */
    const ReedsSheppPathSegmentType * type_;
    /** Path segment lengths */
    /// TODO why is it fixed length
    double length_[5];
    /** Total length */
    double totalLength_;
  };

  struct StateXYT
  {
    double x;
    double y;
    double yaw;
  };

  explicit ReedsSheppStateSpace(double turningRadius)
  : turning_radius_(turningRadius) {}

  double distance(const StateXYT & s0, const StateXYT & s1);

  /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
  ReedsSheppPath reedsShepp(const StateXYT & s0, const StateXYT & s1);

protected:
  void interpolate(const StateXYT & s0, ReedsSheppPath & path, double seg, StateXYT & s_out);

  /** \brief Turning radius */
  //TODO change name
  double turning_radius_;
};

}  // namespace parking
}  // namespace planning
}  // namespace autoware

#endif  // ASTAR_SEARCH__REEDS_SHEPP_HPP_
