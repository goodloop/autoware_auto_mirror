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

#include "astar_search/reeds_shepp.hpp"

#include <cmath>

namespace autoware
{
namespace planning
{
namespace parking
{
ReedsSheppPath ReedsShepp::reedsShepp(
  const StateXYT & state_0, const StateXYT & state_1)
{
  double dx = state_1.x - state_0.x;
  double dy = state_1.y - state_0.y;
  double dth = state_1.yaw - state_0.yaw;
  double cos_yaw = std::cos(state_0.yaw);
  double sin_yaw = std::sin(state_0.yaw);

  double x = cos_yaw * dx + sin_yaw * dy;
  double y = -sin_yaw * dx + cos_yaw * dy;
  return reedsShepp(ReedsSheppNode{x / turning_radius_, y / turning_radius_, dth});
}

ReedsSheppPath ReedsShepp::reedsShepp(ReedsSheppNode node)
{
  ReedsSheppPath path;
  reeds_shepp::CSC(node, path);
  reeds_shepp::CSC(node, path);
  reeds_shepp::CCC(node, path);
  reeds_shepp::CCCC(node, path);
  reeds_shepp::CCSC(node, path);
  reeds_shepp::CCSCC(node, path);
  return path;
}

double ReedsShepp::distance(const StateXYT & state_0, const StateXYT & state_1)
{
  return turning_radius_ * reedsShepp(state_0, state_1).length();
}


void ReedsShepp::interpolate(
  const StateXYT & state_0, ReedsSheppPath & path, double seg, StateXYT & s_out)
{
  if (seg < 0.0) {
    seg = 0.0;
  }
  if (seg > path.length()) {
    seg = path.length();
  }

  s_out.x = 0.0;
  s_out.y = 0.0;
  s_out.yaw = state_0.yaw;

  for (unsigned int i = 0; i < path.length_.size() && seg > 0; ++i) {
    double v = 0.0;
    if (path.length_[i] < 0) {
      v = std::max(-seg, path.length_[i]);
      seg += v;
    } else {
      v = std::min(seg, path.length_[i]);
      seg -= v;
    }

    double phi = s_out.yaw;
    switch (path.type_[i]) {
      case COUNTERCLOCKWISE:
        s_out.x += (sin(phi + v) - std::sin(phi));
        s_out.y += (-cos(phi + v) + std::cos(phi));
        s_out.yaw = phi + v;
        break;
      case CLOCKWISE:
        s_out.x += (-sin(phi - v) + std::sin(phi));
        s_out.y += (cos(phi - v) - std::cos(phi));
        s_out.yaw = phi - v;
        break;
      case STRAIGHT:
        s_out.x += (v * std::cos(phi));
        s_out.y += (v * std::sin(phi));
        break;
      case NO_OPERATION:
        break;
    }
  }

  s_out.x = s_out.x * turning_radius_ + state_0.x;
  s_out.y = s_out.y * turning_radius_ + state_0.y;
}

}  // namespace parking
}  // namespace planning
}  // namespace autoware
