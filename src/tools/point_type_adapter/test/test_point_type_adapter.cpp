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

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <memory>
#include "gtest/gtest.h"
#include "point_type_adapter/point_type_adapter_node.hpp"

TEST(test_point_type_adapter, test_cloud_converter) {
  using PointSvl = autoware::tools::point_type_adapter::PointTypeAdapterNode::PointSvl;
  using PointXYZI = autoware::tools::point_type_adapter::PointTypeAdapterNode::PointXYZI;
  using sensor_msgs::msg::PointCloud2;
  PointCloud2::SharedPtr cloud_svl_ptr = std::make_shared<PointCloud2>();
  using CloudModifierSvl = point_cloud_msg_wrapper::PointCloud2Modifier<PointSvl>;
  CloudModifierSvl cloud_modifier_svl(*cloud_svl_ptr, "frame_original");
  cloud_modifier_svl.push_back(PointSvl{3.0F, 4.0F, 5.0F, 100, 123456789});
  cloud_modifier_svl.push_back(PointSvl{6.0F, 8.0F, 10.0F, 200, 123456789});

  PointXYZI point_xyzi_0{3.0F, 4.0F, 5.0F, 100};
  PointXYZI point_xyzi_1{6.0F, 8.0F, 10.0F, 200};

  PointCloud2::SharedPtr cloud_xyzi_ptr =
    autoware::tools::point_type_adapter::PointTypeAdapterNode::cloud_svl_to_cloud_xyzi(
    cloud_svl_ptr);

  using CloudViewXyzi = point_cloud_msg_wrapper::PointCloud2View<PointXYZI>;
  CloudViewXyzi cloud_view_xyzi(*cloud_xyzi_ptr);
  EXPECT_EQ(cloud_xyzi_ptr->header, cloud_svl_ptr->header);
  EXPECT_EQ(cloud_xyzi_ptr->width, cloud_svl_ptr->width);
  EXPECT_EQ(cloud_xyzi_ptr->fields.size(), 4UL);
  EXPECT_EQ(cloud_view_xyzi.at(0), point_xyzi_0);
  EXPECT_EQ(cloud_view_xyzi.at(1), point_xyzi_1);
}
