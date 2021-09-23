ground_truth_detections {#ground_truth_detections-package-design}
===========


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The `ground_truth_detections` package converts the ground-truth dynamic
detections from the SVL simulator into the format understood by the
Autoware.Auto perception module.

# Example Usage

The following instructions for a minimal test environment to extract ground-truth detections assume
LGSVL 2020.06.

### Configure the vehicle in LGSVL

Follow the [LGSVL instructions](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html) and launch `lgsvl` in **terminal 1**.

To extract ground-truth detections from the camera, LGSVL requires that the vehicle configuration
contain

-# a virtual sensor of type [`Color Camera`](https://www.svlsimulator.com/docs/archive/2020.06/sensor-json-options/#color-camera)
-# a virtual sensor of type [`2D Ground Truth`](https://www.svlsimulator.com/docs/archive/2020.06/sensor-json-options/#2d-ground-truth)

To extract 3D ground-truth detections, add

-# a virtual sensor of type [`3D Ground Truth`](https://www.svlsimulator.com/docs/archive/2020.06/sensor-json-options/#3d-ground-truth)

Check the file `config/lgsvl-sensors-camera.json` in this package as a basis to configure these sensors
sensors and follow the
[instructions](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html#lgsvl-configuring-vehicle)
to add them to vehicle configuration. The position (i.e. `transform`) of the camera, the other
sensors, and their other properties may have to be adjusted for a particular vehicle model to avoid
occlusions and missing detections.

### Configure the visualization

After launching the simulation, open the LGSVL window and click the eye-shaped icon in the bottom
left to open the visualization pan. With the default configuration provided in
`config/lgsvl-sensors-camera.json`, then activate

-# *Main Camera*
-# *2D Ground Truth*
-# *3D Ground Truth*

to see bounding boxes drawn around vehicles.

### LSGSVL bridge

In **terminal 2**, run the bridge to translate data from LGSVL to the ROS world

```console
ade$ sudo apt update
ade$ sudo apt install ros-$ROS_DISTRO-lgsvl-bridge
ade$ lgsvl_bridge
```

### Test output

Assuming this node has been built, start the node in **terminal 3**

```console
ade$ ros2 run ground_truth_detections ground_truth_detections_node_exe
```

and check its output in **terminal 4**

```console
ade$ ros2 topic echo /perception/ground_truth_detections_2d
ade$ ros2 topic echo /perception/ground_truth_detections_3d
```

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

## Assumptions / Known limits
<!-- Required -->

A camera sensor has to be added to the [sensor configuration of the vehicle in
SVL](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html#lgsvl-configuring-vehicle)
and the ground-truth detections have to be output.

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
This node has no ROS2 parameters.

### Input/Output

2D inputs on **topic** `/simulator/ground_truth/detections2D` of **type**
 [`lgsvl_msgs::msg::Detection2DArray`](https://github.com/lgsvl/lgsvl_msgs/blob/master/msg/Detection2DArray.msg)
 are converted to 2D outputs on **topic** `/perception/ground_truth_detections_2d` of **type**
 [`autoware_auto_msgs::msg::ClassifiedRoiArray`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/ClassifiedRoiArray.idl).

3D inputs on **topic** `/simulator/ground_truth/detections3D` of **type**
 [`lgsvl_msgs::msg::Detection3DArray`](https://github.com/lgsvl/lgsvl_msgs/blob/master/msg/Detection3DArray.msg)
 are converted to 3D outputs on **topic** `/perception/ground_truth_detections_3d` of **type**
 [`autoware_auto_msgs::msg::DetectedObjects`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/DetectedObjects.idl).

## Inner-workings / Algorithms
<!-- If applicable -->

The labels of objects are character strings in SVL and can take arbitrary values. The following
mapping to
[`autoware_auto_msgs::msg::ObjectClassification`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/ObjectClassification.idl)
values is performed:

| label in SVL  | object class |
|---------------|--------------|
| `Hatchback`   | `CAR`        |
| `Jeep`        | `CAR`        |
| `Sedan`       | `CAR`        |
| `SUV`         | `CAR`        |
| `BoxTruck`    | `TRUCK`      |
| `Pedestrian`  | `PEDESTRIAN` |
| anything else | `UNKNOWN`    |

For 2D detections, the coordinates of bounding boxes in camera coordinates are converted to polygons
in camera coordinates with four points. The points are ordered counter-clockwise:

1. bottom left
1. bottom right
1. upper right
1. upper left

The coordinates are guaranteed to be non-negative.

For 3D detections, the coordinates of bounding boxes are transformed to an oriented `Shape` object
made up of a polygon with four points and a height. The four points represent the bottom of the object
and are ordered counter-clockwise:

1. rear left
1. rear right
1. front right
1. front left

Check the `Shape` [message
definition](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/Shape.idl)
to learn more about constraints on its members. Roll, pitch, and yaw of the input bounding box are
considered but the z value of all points on output are set equal.

## Error detection and handling
<!-- Required -->
The only check is to ensure that corners of the bounding box in 2D are not negative.

# References / External links
<!-- Optional -->
- [SVL 2020.06 sensors](https://www.svlsimulator.com/docs/archive/2020.06/sensor-json-options/)
- [SVL 2020.06 ground truth](https://www.svlsimulator.com/docs/archive/2020.06/perception-ground-truth/#subscribe-to-ground-truth-messages-from-simulator)
- [SVL 2021.01 ground truth 2D](https://www.svlsimulator.com/docs/user-interface/sensor-visualizers/#2d-ground-truth)

# Future extensions / Unimplemented parts
<!-- Optional -->

1. The object type itself is not exposed by LGSVL, so the current mapping depends on the label and needs to be extended for maps with other vehicle labels.
1. The implementation assumes that all objects are level on the ground; i.e., they are not tilted and roll and pitch are ignored.

# Related issues
<!-- Required -->

- [#1335](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1335) Add 3D detections
- [#1099](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1099) Initial implementation
