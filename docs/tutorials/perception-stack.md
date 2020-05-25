Autoware.Auto 3D perception stack {#perception-stack}
============

[TOC]

# Running the Autoware.Auto 3D perception stack

The Autoware.Auto 3D perception stack consists of:

1. [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_node): Converts raw sensor data to [PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg) messages.
2. [point_cloud_filter_transform_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/point_cloud_filter_transform_nodes): Transforms output of the `velodyne_node` to a common frame.
3. [ray_ground_classifier_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/ray_ground_classifier_nodes): Classifies point cloud points to indicate whether they belong to a ground or non-ground surface.
4. [euclidean_cluster_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/segmentation/euclidean_cluster_nodes): Clusters the non-ground points into object detections.

The following subsections describe how to bring up the perception stack node by node. Follow the directions in sequence. Ensure that ADE has been started by running:

```console
$ ade start
```

## Publishing sensor data

Sensor data can be obtained by connecting the `velodyne_node` directly to the sensor, or by publishing the below `pcap` file:

- [Dual VLP-16 Hi-Res pcap file](https://drive.google.com/open?id=1vNA009j-tsVVqSeYRCKh_G_tkJQrHvP-)

To publish the `pcap` file, place it somewhere in the `adehome` directory such as `adehome/data/`, open a new terminal, then:

```console
$ ade enter
ade$ udpreplay ~/data/route_small_loop_rw-127.0.0.1.pcap -r -1
```

\note
The `-r -1` argument is optional; it just tells the player to loop playback indefinitely.

To connect to the sensor: update the IP address and port arguments in the param file for the `velodyne_node` (see next step).

## Running the velodyne node

This node consumes raw data from a velodyne sensor and publishes point cloud messages. In a new terminal, do:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_node velodyne_cloud_node_exe __ns:=/lidar_front __params:=/opt/AutowareAuto/share/velodyne_node/param/vlp16_test.param.yaml
```

When the `velodyne_node` is running, the resulting point cloud can be visualized in `rviz2` as a `sensor_msgs/PointCloud2` topic type.

## Running the point cloud filter transform node

This node transforms point clouds from the `velodyne_node` to a common frame. In a new terminal, do:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe __ns:=/lidar_front __params:=/opt/AutowareAuto/share/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml __node:=filter_transform_vlp16_front
```
## Running the rviz2 visualizer

At this point, `rviz2` can be used to visualize the point cloud data. To start it, open a new terminal, then:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ rviz2 -d /opt/AutowareAuto/share/autoware_auto_examples/rviz2/autoware.rviz
```

## Running the ray ground classifier node

This node classifies point cloud points according to whether they are ground or non-ground. In a new terminal, do:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe __params:=/opt/AutowareAuto/share/ray_ground_classifier_nodes/param/vlp16_lexus.param.yaml
```

This will create two new topics (`/nonground_points` and `/points_ground`) that output
`sensor_msgs/PointCloud2`s that we can use to segment the point clouds.

For convenience, an `rviz2` configuration is provided in `/opt/AutowareAuto/share/autoware_auto_examples/rviz2/autoware_ray_ground.rviz` that can be loaded to automatically set up the visualizations.

![Autoware.Auto ray ground filter snapshot](autoware-auto-ray-ground-filter-smaller.png)

## Running the Euclidean cluster node

This node clusters non-ground points into objects and publishes bounding boxes. In a new terminal, do:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run euclidean_cluster_nodes euclidean_cluster_exe __params:=/opt/AutowareAuto/share/euclidean_cluster_nodes/param/vlp16_lexus_cluster.param.yaml
```

After this we will have a new topic, named (`/lidar_bounding_boxes`) that we can visualize with the provided `rviz2` configuration file in `/opt/AutowareAuto/share/autoware_auto_examples/rviz2/autoware_bounding_boxes.rviz`

![Autoware.Auto bounding boxes segmentation snapshot](autoware-auto-bounding-boxes-smaller.png)
