ndt_nodes
=============

# Purpose / Use cases

This package contains ndt related ROS2 nodes.

# Design

![ndt architecture](images/ndt_uml.svg)

## Map Publisher

[NDTMapPublisherNode](@ref autoware::localization::ndt_nodes::NDTMapPublisherNode) is responsible providing point cloud map data to recipients and publishing the "earth->map" transform. It does this by reading from a `.yaml` file, which contains the filename of a '.pcd' point cloud map as well as the map origin in geocentric coordinates (latitude, longitude and elevation). The transform is constructed using GeographicLib library and published as a static transform. The node reads the  point cloud file map from disk,
transforming this map into an ndt map (voxel and covariances) and then publishing this map to its recipients. The node also converts the map data into a point cloud suitable for visualization in rviz2, and publishes this point cloud.

Since the file IO means that this node cannot be used in a real time context, the dependency constraints are more relaxed and
 [pcl](https://github.com/PointCloudLibrary/pcl) is used for reading and writing of `.pcd` files.

### Algorithm Design
The workflow of the publisher can be summarized as the following:
1. Wait to discover recipients of either the ndt map or the point cloud for visualization.
2. Use [pcl](https://github.com/PointCloudLibrary/pcl) to read a `.pcd` file into a `sensor_msgs::msg::PointCloud2` message.
3. Transform the read point cloud into a ndt map using [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap).
4. Serialize the ndt map representation into a `PointCloud2` message where each point represents a single cell in the ndt map.
5. Publish the resulting PointCloud2 message containing the ndt map.
6. Convert the point cloud into a `sensor_msgs::msg::PointCloud2` with a Point type suitable to be recieved by rviz2.
7. Pubish the resulting PointCloud2 messagae containing the full point cloud

The published ndt map point cloud message has the following fields:

```
name = "x",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "y",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "z",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xx",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xy",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_yy",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_yz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_zz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cell_id",  count = 2U,     datatype = sensor_msgs::msg::PointField::UINT32
```

`cell_id` has a count of `2U` meaning it is practically an `unsigned int[2]`. This was necessary to represent 64 bit voxel indices since `sensor_msgs::msg::PointField` struct does not support 64 bit integer types.

The published point cloud message of the full point cloud has the following fields:

```
name = "x",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT32
name = "y",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT32
name = "z",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT32
```
The launch file for this node also launches a [VoxelGridNode](@ref autoware::perception::filters::VoxelGridNode) to subsample the published full point cloud to reduce the number of points to be visualized.

# Related issues
- #136: Implement NDT Map Publisher