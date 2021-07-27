// Copyright 2021 Apex.AI, Inc.
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

#ifndef MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
#define MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_

#include <Eigen/Geometry>


namespace autoware
{
namespace common
{
namespace state_estimation
{

enum class InputStorageOrder
{
  kRowMajor,
  kColumnMajor
};

///
/// @brief      Downscale the isometry to a lower dimension if needed.
///
/// @param[in]  isometry              The isometry transform
///
/// @tparam     kStateDimensionality  Dimensionality of the space.
/// @tparam     FloatT                Type of scalar.
///
/// @return     Downscaled isometry.
///
template<std::int32_t kStateDimensionality, typename FloatT>
static constexpr Eigen::Transform<
  FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry> downscale_isometry(
  const Eigen::Transform<FloatT, 3, Eigen::TransformTraits::Isometry> & isometry)
{
  static_assert(kStateDimensionality <= 3, "We only handle scaling the isometry down.");
  using Isometry = Eigen::Transform<
    FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry>;
  Isometry result{Isometry::Identity()};
  result.linear() = isometry.rotation()
    .template block<kStateDimensionality, kStateDimensionality>(0, 0);
  result.translation() = isometry.translation().topRows(kStateDimensionality);
  return result;
}

///
/// @brief      Transform a given array to an Eigen matrix using the specified stride and a starting
///             index within this array.
///
/// @details    The function converts a given array into an Eigen matrix using the given starting
///             index and stride. The function will take the number of rows and columns from the
///             provided parameters and will iterate the given array as many times as needed to fill
///             all elements of the resulting matrix. It is assumed that there are no gaps between
///             the elements in a single row but that the columns are spaced `stride` apart.
///
/// @throws     std::runtime_error  if there is not enough elements in the array to perform a full
///                                 conversion of all the asked elements.
///
/// @param[in]  array          The given array (e.g. a covariance array from a message)
/// @param[in]  start_index    The start index of the first element in an array to be copied
/// @param[in]  stride         How big the step to the next row is in terms of indices in the array
/// @param[in]  storage_order  The storage order of the data in the input array (row-major for ROS)
///
/// @tparam     kRows          Number of rows in the resulting matrix
/// @tparam     kCols          Number of columns in the resulting matrix
/// @tparam     ScalarT        Scalar type (inferred)
/// @tparam     kSize          Size of the input array (inferred)
///
/// @return     An Eigen matrix that stores the requested data.
///
template<std::int32_t kRows, std::int32_t kCols, typename ScalarT, std::size_t kSize>
Eigen::Matrix<ScalarT, kRows, kCols> array_to_matrix(
  const std::array<ScalarT, kSize> & array,
  const std::int32_t start_index,
  const std::int32_t stride,
  const InputStorageOrder storage_order)
{
  using Mat = Eigen::Matrix<ScalarT, kRows, kCols>;

  const auto index = [&start_index, &stride, &storage_order](auto row, auto col) {
      switch (storage_order) {
        case InputStorageOrder::kRowMajor:
          return static_cast<std::size_t>(start_index + row * stride + col);
        case InputStorageOrder::kColumnMajor:
          return static_cast<std::size_t>(start_index + col * stride + row);
      }
      throw std::runtime_error("Unexpected storage order");
    };

  Mat res{Mat::Zero()};
  const auto max_index = index(res.rows() - 1, res.cols() - 1);
  if (max_index >= array.size()) {
    throw std::runtime_error(
            "Trying to access out of bound memory at index " +
            std::to_string(max_index) + " of an array with size: " + std::to_string(array.size()));
  }

  for (auto col = 0; col < kCols; ++col) {
    for (auto row = 0; row < kRows; ++row) {
      res(row, col) = array[index(row, col)];
    }
  }
  return res;
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
