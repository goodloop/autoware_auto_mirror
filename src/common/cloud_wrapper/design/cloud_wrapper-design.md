cloud_wrapper {#cloud_wrapper-package-design}
===========

This is the design document for the `cloud_wrapper` package.


# Purpose / Use cases

With this library it is possible to iterate points in a PointCloud2
message, add-remove points, apply std::algorithm functions to them.


# Design & Inner-workings / Algorithms

A [PointCloud2 message](https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/msg/PointCloud2.msg)
is made of following fields:
```
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```
Here, the `data` is a byte vector.

A point is made of `point_step` amount of bytes.

`fields` contain the information about structure of a point.

By using these information, it is possible to reinterpret the `data` raw byte vector
as a point cloud vector.

And this library lets the user do that by wrapping around an existing PointCloud2 message
and providing [random access iterators](http://www.cplusplus.com/reference/iterator/RandomAccessIterator/)
for points inside it. 

It also maintains the size related fields of the message in sync with the modified data.

This is done by using [reinterpret_cast](https://en.cppreference.com/w/cpp/language/reinterpret_cast) 
on the first element's address and the end address of the data vector, casting them to 
begin and end iterators respectively.

Using these begin and end iterators, the user can use std::algorithms functionality on the
point cloud data.

Also in order to be performant memory-wise, it provides with [reserve](http://www.cplusplus.com/reference/vector/vector/reserve/)
 and [resize](http://www.cplusplus.com/reference/vector/vector/resize/) functionality.

User can access & modify the elements using the [at](http://www.cplusplus.com/reference/vector/vector/at/)
 like in vectors.

## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

Use with params:
```
--ros-args --params-file /home/mfc/projects/AutowareAuto/src/common/cloud_wrapper/param/test_param.yaml
```

## Error detection and handling
<!-- Required -->


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
