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

#include <memory>

#include "gtest/gtest.h"
#include "outlier_filter/voxel_grid_filter.hpp"

using VoxelGridFilter =
  autoware::perception::filters::outlier_filter::voxel_grid_filter::VoxelGridFilter;

TEST(VoxelGridFilterTest, test_parameter) {
  auto filter = std::make_shared<VoxelGridFilter>(1.0f, 1.0f, 1.0f, static_cast<uint32_t>(10));
}
