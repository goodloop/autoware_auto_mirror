lgsvl_cloud_converter {#lgsvl_cloud_converter-package-design}
===========

This is the design document for the `lgsvl_cloud_converter` package.


# Purpose / Use cases
LGSVL Simulator outputs point clouds with following fields:

```
fields: x, y, z, intensity, timestamp
types: FLOAT32, FLOAT32, FLOAT32, UINT8, FLOAT64
offsets: 0, 4, 8, 16, 24
```

But the Autoware.Auto stack currently expects:
```
fields: x, y, z, intensity
types: FLOAT32, FLOAT32, FLOAT32, FLOAT32
offsets: 0, 4, 8, 12
```

So this node does this conversion for front and back lidars.

# Design

It simply subscribes in a PointCloud2 message with point type 
`autoware::common::types_point_cloud2::PointLgsvl` and converts it into 
a PointCloud2 message with point type
`autoware::common::types_point_cloud2::PointXYZI` and publishes it.

## Assumptions / Known limits

Input point cloud structured exactly like this:
```
fields: x, y, z, intensity, timestamp
types: FLOAT32, FLOAT32, FLOAT32, UINT8, FLOAT64
offsets: 0, 4, 8, 16, 24
```

## Inputs / Outputs / API

Input: PointCloud2 Message

Output: PointCloud2 Message

You can run it with `name_topic_cloud_in` and `name_topic_cloud_out` string ros parameters.

For simulator, simply use:
```bash
ros2 launch lgsvl_cloud_converter lgsvl_cloud_converter.launch.py
```

and it will convert topic names and types like following:
```
"/lgsvl/lidar_front/points_raw" -> "/lidar_front/points_raw"
"/lgsvl/lidar_rear/points_raw" -> "/lidar_rear/points_raw"
```

## Inner-workings / Algorithms
It just iterates over the old cloud and constructs the new cloud using `std::transform` and
`std::back_inserter`.

## Error detection and handling
If an exception occurs because the input PointCloud2 doesn't have the expected type,
it will catch it and report it in the callback.
