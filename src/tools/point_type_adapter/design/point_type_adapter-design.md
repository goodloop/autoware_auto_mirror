point_type_adapter {#point_type_adapter-package-design}
===========

This is the design document for the `point_type_adapter` package.

# Purpose / Use cases

Autoware.Auto uses `PointCloud2` messages with following field structure and order:
| **Field Name** | **Field Datatype** |
| --- | --- |
| "x" | `FLOAT32` |
| "y" | `FLOAT32` |
| "z" | `FLOAT32` |
| "intensity" | `FLOAT32` |

The reason is, if the algoritms know this structure ahead of time, 
we can iterate over them with almost no overhead using `point_cloud_msg_wrapper`.

But various lidar drivers might output `PointCloud2` messages with extra fields
or different orders.

This node helps converting those point clouds into the type Autoware.Auto expects.

## Use case: SVL Simulator
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

And this node does this conversion for front and back lidars using 
`point_type_adapter.launch.py` file.


# Design

It simply subscribes in a PointCloud2 message with point type **should** contain:
| **Field Name** | **Field Datatype** |
| --- | --- |
| "x" | `FLOAT32` |
| "y" | `FLOAT32` |
| "z" | `FLOAT32` |
| "intensity" | `FLOAT32` or `UINT8` |

and converts it into a PointCloud2 message with point type
`autoware::common::types::PointXYZI` and publishes it.

- The input point cloud is allowed to contain extra fields.
- The input point cloud is allowed to contain required fields in different orders.

## Assumptions / Known limits

If the input point cloud doesn't have either of the fields with types required,
it will throw exceptions and won't publish anything.

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
It just iterates over the old cloud using `sensor_msgs::PointCloud2Iterator`,
creates a point made of `x,y,z,intensity` fields, puts it to the new cloud using
`point_cloud_msg_wrapper::PointCloud2Modifier` and publishes it.

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
