// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "common/types.hpp"
#include "lidar_utils/lidar_utils.hpp"
#include "autoware_auto_contracts/assertions.hpp"
#include "ray_ground_classifier/contract.hpp"
#include "ray_ground_classifier/ray_aggregator.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::common::types::FEPS;
using autoware::common::types::PI;
using autoware::common::types::POINT_BLOCK_CAPACITY;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using contracts_lite::gcc_7x_to_string_fix;

////////////////////////////////////////////////////////////////////////////////
RayAggregator::Config::Config(
  const Realf min_ray_angle_rad, const Realf max_ray_angle_rad,
  const StrictlyEpsPositiveRealf ray_width_rad,
  const SizeBound<POINT_BLOCK_CAPACITY> min_ray_points
)
: m_min_ray_points(min_ray_points),
  m_ray_width_rad(ray_width_rad),
  m_min_angle_rad(min_ray_angle_rad),
  m_domain_crosses_180(max_ray_angle_rad < min_ray_angle_rad),
  m_num_rays(RayAggregator::Config::compute_num_rays(m_domain_crosses_180, m_min_angle_rad,
    max_ray_angle_rad, m_ray_width_rad))
{
  // TODO(c.ho) upper limit on number of rays?
}
////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::Config::compute_num_rays(
  bool8_t domain_crosses_180,
  Realf min_ray_angle_rad,
  Realf max_ray_angle_rad,
  StrictlyEpsPositiveRealf ray_width_rad)
{
  if (domain_crosses_180) {
    const Realf angle_range = (PI - min_ray_angle_rad) + (PI + max_ray_angle_rad);
    return static_cast<std::size_t>(std::ceil(angle_range / ray_width_rad));
  }
  return static_cast<std::size_t>(std::ceil((max_ray_angle_rad - min_ray_angle_rad) /
         ray_width_rad));
}
////////////////////////////////////////////////////////////////////////////////
RayAggregator::RayAggregator(const Config & cfg)
: m_cfg(cfg),
  m_rays(m_cfg.m_num_rays),
  m_ready_indices(m_cfg.m_num_rays),
  m_ready_start_idx{},  // zero initialization
  m_num_ready{},  // zero initialization
  m_ray_state(m_cfg.m_num_rays)
  #ifdef RAY_AGGREGATOR_PARALLEL
  , m_ray_locks(m_cfg.m_num_rays),
  m_get_next_ray_lock(ATOMIC_FLAG_INIT)
  #endif
{
  m_rays.clear();  // capacity unchanged
  const std::size_t ray_size =
    std::max(m_cfg.m_min_ray_points, static_cast<std::size_t>(POINT_BLOCK_CAPACITY));
  for (std::size_t idx = 0U; idx < m_cfg.m_num_rays; ++idx) {
    m_rays.emplace_back(ray_size);
    m_rays.back().clear();
    m_ray_state.push_back(RayState::NOT_READY);
  }
  m_ready_indices.resize(m_ready_indices.capacity());
  #ifdef RAY_AGGREGATOR_PARALLEL
  m_num_ready = {0};
  for (std::size_t idx = 0U; idx < m_cfg.m_num_rays; ++idx) {
    m_ray_locks[idx].clear();
  }
  #endif
}
////////////////////////////////////////////////////////////////////////////////
void RayAggregator::end_of_scan()
{
  // all rays ready
  m_ready_start_idx = 0U;
  m_num_ready = 0U;
  for (std::size_t idx = 0U; idx < m_rays.size(); ++idx) {
    // Add all non empty "NOT_READY" rays to the ready list since the end of scan is reached.
    if (RayState::RESET != m_ray_state[idx]) {
      if (!m_rays[idx].empty()) {
        m_ready_indices[m_num_ready] = idx;
        ++m_num_ready;
      }
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::insert(const PointXYZIFR & pt)
{
  if (static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID) == pt.get_point_pointer()->id) {
    return false;
  } else {
    const std::size_t idx = bin(pt);
    #ifdef RAY_AGGREGATOR_PARALLEL
    while (m_ray_locks[idx].test_and_set()) {}  // busy wait
    #endif
    Ray & ray = m_rays[idx];
    if (RayState::RESET == m_ray_state[idx]) {
      ray.clear();  // capacity unchanged
      m_ray_state[idx] = RayState::NOT_READY;
    }
    if (ray.size() >= ray.capacity()) {
      #ifdef RAY_AGGREGATOR_PARALLEL
      m_ray_locks[idx].clear();
      #endif
    }

    DEFAULT_ENFORCE(common::contracts::assertions::vector_below_capacity(ray));

    // insert point to ray, do some presorting
    ray.push_back(pt);
    // TODO(c.ho) get push_heap working to amortize sorting burden
    // check if ray is ready
    if ((RayState::READY != m_ray_state[idx]) && (m_cfg.m_min_ray_points <= ray.size())) {
      m_ray_state[idx] = RayState::READY;
      // "push" to ring buffer
      // so long we don't fill the buffer, change m_ready_start_idx or pop, reserving m_num_ready
      // is enough to make the push thread safe
      std::size_t used_num;
      #ifdef RAY_AGGREGATOR_PARALLEL
      used_num = m_num_ready.fetch_add(1, std::memory_order_relaxed);
      #else
      used_num = m_num_ready++;
      #endif
      const std::size_t jdx = (m_ready_start_idx + used_num) % m_ready_indices.size();
      m_ready_indices[jdx] = idx;
      // TODO(c.ho) bounds check?
    }
    #ifdef RAY_AGGREGATOR_PARALLEL
    m_ray_locks[idx].clear();
    #endif
  }
  return true;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::insert(const PointXYZIF * pt)
{
  return insert(PointXYZIFR{pt});
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::insert(const PointPtrBlock & blk)
{
  bool8_t ret = true;
  for (const PointXYZIF * pt : blk) {
    ret = insert(pt);
    if (!ret || (static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID) == pt->id)) {
      break;
    }
  }
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::is_ray_ready() const
{
  return m_num_ready != 0U;
}
////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::get_ready_ray_count() const
{
  return m_num_ready;
}
////////////////////////////////////////////////////////////////////////////////
const Ray & RayAggregator::get_next_ray()
{
  #ifdef RAY_AGGREGATOR_PARALLEL
  while (m_get_next_ray_lock.test_and_set()) {}  // busy wait
  #endif
  // move the if out from the sequential section by nullifying the operations if false
  bool8_t is_ready = is_ray_ready();
  const std::size_t local_start_idx = m_ready_start_idx;
  m_ready_start_idx = (local_start_idx + is_ready) % m_ready_indices.size();
  m_num_ready -= is_ready;
  #ifdef RAY_AGGREGATOR_PARALLEL
  m_get_next_ray_lock.clear();
  #endif

  DEFAULT_ENFORCE(contracts_lite::ReturnStatus(CONTRACT_COMMENT("",
    "RayAggregator: at least one ray must be ready"),
    is_ready));

  const std::size_t idx = m_ready_indices[local_start_idx];
  #ifdef RAY_AGGREGATOR_PARALLEL
  while (m_ray_locks[idx].test_and_set()) {}  // busy wait
  #endif

  Ray & ret = m_rays[idx];
  // Sort ray
  std::sort(ret.begin(), ret.end());
  // ready to be reset on next insertion to this item
  m_ray_state[idx] = RayState::RESET;

  #ifdef RAY_AGGREGATOR_PARALLEL
  m_ray_locks[idx].clear();
  #endif
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
void RayAggregator::reset()
{
  while (is_ray_ready()) {
    const std::size_t idx = m_ready_indices[m_ready_start_idx];
    // "pop" from ring buffer
    m_ready_start_idx = (m_ready_start_idx + 1U) % m_ready_indices.size();
    --m_num_ready;
    // ready to be reset on next insertion to this item
    m_ray_state[idx] = RayState::RESET;
  }
}
////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::bin(const PointXYZIFR & pt) const
{
  const Realf x = pt.get_point_pointer()->x;
  const Realf y = pt.get_point_pointer()->y;
  // (0, 0) is always bin 0
  float32_t idx = 0.0F;
  const float32_t th = autoware::common::lidar_utils::fast_atan2(y, x);
  idx = th - m_cfg.m_min_angle_rad;
  if (m_cfg.m_domain_crosses_180 && (idx < 0.0F)) {
    // Case where receptive field crosses the +PI/-PI singularity
    // [-PI, max_angle) domain
    idx = idx + autoware::common::types::TAU;
  }
  // [min_angle, +PI) domain: normal calculation
  // normal case, no wraparound
  idx = std::floor(idx / m_cfg.m_ray_width_rad);
  // Avoid underflow
  return std::max(0, static_cast<int32_t>(idx));
}
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
