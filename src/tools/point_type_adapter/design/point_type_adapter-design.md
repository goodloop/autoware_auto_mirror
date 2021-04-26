point_type_adapter {#point_type_adapter-package-design}
===========

This is the design document for the `point_type_adapter` package.

# Purpose / Use cases

SVL Simulator outputs point clouds with following fields:

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

Though this node can be modified to convert from different point cloud types.


# Design

It simply subscribes in a PointCloud2 message with point type
`autoware::tools::point_type_adapter::PointTypeAdapterNode::PointSvl` and converts it into
a PointCloud2 message with point type
`autoware::common::types::PointXYZI` and publishes it.


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
It just iterates over the old cloud and constructs the new cloud using `std::transform`.


## Error detection and handling
If an exception occurs because the input PointCloud2 doesn't have the expected type,
it will catch it and report it in the callback.


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->
