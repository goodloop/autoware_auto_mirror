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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the cloud_wrapper class.

#ifndef CLOUD_WRAPPER__CLOUD_WRAPPER_HPP_
#define CLOUD_WRAPPER__CLOUD_WRAPPER_HPP_

#include <cloud_wrapper/visibility_control.hpp>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <exception>
#include <memory>
#include "common/types.hpp"
#include "types_point_cloud2.hpp"

namespace autoware
{
namespace common
{
namespace cloud_wrapper
{

template<typename TypePoint>
class CLOUD_WRAPPER_PUBLIC CloudWrapper
{
public:
  using SharedPtr = std::shared_ptr<CloudWrapper>;
  using ConstSharedPtr = const std::shared_ptr<CloudWrapper>;
  using iterator = typename std::vector<TypePoint>::iterator;
  using const_iterator = typename std::vector<TypePoint>::const_iterator;
  using value_type = TypePoint;

  /// \brief Wrap around an existing message where the existing message is fully defined
  /// \param msg_in Message to wrap around
  explicit CloudWrapper(sensor_msgs::msg::PointCloud2 & msg_in)
  : msg_ref_(msg_in),
    size_(0)
  {
    if (msg_ref_.point_step == 0) {
      throw std::runtime_error(
              "This constructor requires message to be fully defined. Use the constructor that takes message_ptr and fields instead.");
    }
    if (sizeof(TypePoint) != msg_ref_.point_step) {
      throw std::length_error("Given point struct has different point size than message has.");
    }
    if (msg_ref_.data.size() % msg_ref_.point_step != 0) {
      throw std::length_error("Data size should be multiple of point step.");
    }
    size_ = msg_ref_.data.size() / msg_ref_.point_step;
    regenerate_iterator_begin();
    regenerate_iterator_end();
    if (static_cast<long long>(size_) != std::distance(begin_, end_)) {
      throw std::runtime_error("Message size check failed.");
    }

  }

  /// \brief Wrap around an existing Cloud message and initialize it
  /// \param msg_in Message to wrap around
  /// \param point_fields Point Fields that match with the \tparam TypePoint
  /// \param size Initial points count of the message.
  explicit CloudWrapper(
    sensor_msgs::msg::PointCloud2 & msg_in,
    const std::vector<sensor_msgs::msg::PointField> & point_fields,
    size_t size = 0UL)
  : msg_ref_(msg_in),
    size_(size)
  {
    msg_ref_.point_step = sizeof(TypePoint);
    msg_ref_.height = 1;
    msg_ref_.is_dense = false;
    msg_ref_.is_bigendian = false;
    msg_ref_.fields = point_fields;
    resize(size);

    regenerate_iterator_begin();
    regenerate_iterator_end();

    if (static_cast<long long>(size_) != std::distance(begin_, end_)) {
      throw std::runtime_error("Message size check failed.");
    }

  }

  std::vector<TypePoint> get_vector_copy() const
  {
    return std::vector<TypePoint>(begin_, end_);
  }

  const_iterator cbegin() const
  {
    return begin_;
  }

  const_iterator cend() const
  {
    return end_;
  }

  iterator begin() const
  {
    return begin_;
  }

  iterator end() const
  {
    return end_;
  }

  TypePoint & at(size_t index)
  {
    if (index >= size_) {
      throw std::out_of_range(
              "Tried to access " +
              std::to_string(index) +
              " where size is " +
              std::to_string(size_));
    }
    auto it_begin = begin_;
    std::advance(it_begin, index);
    return *(it_begin);
  }

  size_t size() const
  {
    return size_;
  }

  void resize(size_t size_new)
  {
    msg_ref_.data.resize(size_bytes_from_size_points(size_new), 0);
    size_ = size_new;
    msg_ref_.width = static_cast<uint32_t>(size_);
    msg_ref_.row_step = static_cast<uint32_t>(size_ * sizeof(TypePoint));
    regenerate_iterator_end();
  }

  void reserve(size_t size_new)
  {
    msg_ref_.data.reserve(size_bytes_from_size_points(size_new));
  }

  void push_back(const TypePoint & point)
  {
    auto iter_bytes = reinterpret_cast<const unsigned char *>(&point);
    auto iter_bytes_end = iter_bytes + sizeof(TypePoint);

    reserve(size_ + 1);
    std::copy(iter_bytes, iter_bytes_end, std::back_inserter(msg_ref_.data));

    resize(size_ + 1);
  }

  /// This can be called multiple times sequentally but update_size()
  /// should be called before doing anything else
  void push_back_without_reserve_resize(const TypePoint & point)
  {
    auto iter_bytes = reinterpret_cast<const unsigned char *>(&point);
    auto iter_bytes_end = iter_bytes + sizeof(TypePoint);

    std::copy(iter_bytes, iter_bytes_end, std::back_inserter(msg_ref_.data));
  }

  /// This should be called after push_back_without_reserve_resize is used
  /// to update message values with correct size
  void update_size()
  {
    size_ = msg_ref_.data.size() / sizeof(TypePoint);
    msg_ref_.width = static_cast<uint32_t>(size_);
    msg_ref_.row_step = static_cast<uint32_t>(size_ * sizeof(TypePoint));
    regenerate_iterator_end();
  }


  void erase_till_end(const iterator iter_end_new)
  {
    auto size_after_removal = static_cast<size_t>(std::distance(begin_, iter_end_new));
    resize(size_after_removal);
  }

  sensor_msgs::msg::PointCloud2 & get_msg_ref()
  {
    return msg_ref_;
  }

private:
  sensor_msgs::msg::PointCloud2 & msg_ref_;

  size_t size_;
  iterator begin_;
  iterator end_;

  void regenerate_iterator_begin()
  {
    iterator iter_temp(
      reinterpret_cast<TypePoint *>(&msg_ref_.data[0U]));
    begin_ = iter_temp;
  }

  void regenerate_iterator_end()
  {
    iterator iter_temp(
      reinterpret_cast<TypePoint *>(&msg_ref_.data[msg_ref_.data.size()]));

    end_ = iter_temp;
  }

  size_t size_bytes_from_size_points(size_t count_points)
  {
    return count_points * sizeof(TypePoint);
  }

};

}  // namespace cloud_wrapper
}  // namespace common
}  // namespace autoware
#endif  // CLOUD_WRAPPER__CLOUD_WRAPPER_HPP_
