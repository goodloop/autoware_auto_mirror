cloud_wrapper {#cloud_wrapper-package-design}
===========

This is the design document for the `cloud_wrapper` package.


# Purpose / Use cases

With this library it is possible to iterate points in a PointCloud2
message, add-remove points, apply [std::algorithm](https://en.cppreference.com/w/cpp/algorithm)
functions to them.


# Design & Inner-workings / Algorithms & Inputs / Outputs / API

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

Using these begin and end iterators, the user can use [std::algorithm](https://en.cppreference.com/w/cpp/algorithm)
functionality on the point cloud data.

Also in order to be performant memory-wise, it provides with [reserve](http://www.cplusplus.com/reference/vector/vector/reserve/)
 and [resize](http://www.cplusplus.com/reference/vector/vector/resize/) functionality.

User can access & modify the elements using the [at](http://www.cplusplus.com/reference/vector/vector/at/)
 like in vectors. This checks whether index is within the boundaries and advances a copy of
begin iterator and returns a reference to dereferenced iterator.

Adding a point to the cloud is done with `push_back`. This method reinterprets the given Point
struct as a byte vector and copies it into the `data` vector's end.

### Wrapping Around an Existing Message

This library has 2 implementations depending of the way it accepts the PointCloud2 messages:
- Reference
- Shared Pointer

Example:
```cpp
// Reference Example
#include <cloud_wrapper/cloud_wrapper.hpp>
void Some::callback(sensor_msgs::msg::PointCloud2 & msg)
{
  CloudWrapper<Point> cloud(msg);
  cloud.get_msg_ref().header.frame_id = "test";
  // cloud.get_msg_ref() and msg can be treated as same objects.
  std::cout << "msg frame id: " << msg.header.frame_id << std::endl;
  // output: "msg frame id: test"
}
```
```cpp
// Shared Pointer Example
#include <cloud_wrapper/cloud_ptr_wrapper.hpp>
void Some::callback(const PointCloud2::SharedPtr msg)
{
  CloudPtrWrapper<Point> cloud(msg);
  cloud.get_msg_ptr()->header.frame_id = "test";
  // cloud.get_msg_ptr() and msg can be treated as same objects.
  std::cout << "msg frame id: " << msg->header.frame_id << std::endl;
  // output: "msg frame id: test"
}
```

The important difference is, when `CloudPtrWrapper` is used, it holds a copy of `SharedPtr`
given to it. And this is held as long as `CloudPtrWrapper` object is alive. This is useful
in the regular point cloud callbacks.

But if the `CloudWrapper` is used, it holds a reference to the given `PointCloud2` object.
And if the scope of given object ends, `CloudWrapper` will point to a null object.
In order to prevent this sort of behavior happening, user should define the `CloudWrapper` in the 
same scope as the `PointCloud2` object.

### Creating a new Point Cloud

Here, a point cloud with a point is being created and published.

#### Reference Wrapper
```cpp
  using PointXYZI = autoware::common::types_point_cloud2::PointXYZI;
  using CloudXYZI = autoware::common::cloud_wrapper::CloudWrapper<PointXYZI>;
  sensor_msgs::msg::PointCloud2 msg;
  CloudXYZI cloud(msg,
  autoware::common::types_point_cloud2::fields_PointXYZI);
  cloud.push_back(PointXYZI(3.0f,4.0f,5.0f,100.0f));
  cloud.get_msg_ref().header.frame_id = "test";
  cloud.get_msg_ref().header.stamp = this->now();
  publisher->publish(cloud.get_msg_ref());
```

#### Shared Pointer Wrapper
```cpp
  using PointXYZI = autoware::common::types_point_cloud2::PointXYZI;
  using CloudXYZI = autoware::common::cloud_wrapper::CloudPtrWrapper<PointXYZI>;
  sensor_msgs::msg::PointCloud2 msg;
  CloudXYZI cloud(autoware::common::types_point_cloud2::fields_PointXYZI);
  cloud.push_back(PointXYZI(3.0f,4.0f,5.0f,100.0f));
  cloud.get_msg_ptr()->header.frame_id = "test";
  cloud.get_msg_ptr()->header.stamp = this->now();
  publisher->publish(*cloud.get_msg_ptr());
```

### Using with "STL Algorithm Library
Here only the demonstrations with shared pointer wrapper will be shown.
They can also be applied to reference wrapper.

#### std::foreach
```cpp
// apply a function to all points in a cloud
std::for_each(
  cloud.begin(),
  cloud.end(),
    [](Point & p) {
    p.z += 10.0f;
  });
```

#### std::sort
```cpp
// sort points based on their z values small to big
std::sort(
  cloud.begin(),
  cloud.end(),
    [](const Point & p_lhs, const Point & p_rhs) {
    p_lhs.z < p_rhs.z;
  });
```

#### std::remove_if
```cpp
// remove points that satisfy a certain condition
// std::remove_if removes the elements meeting the criteria in the unary predicate
// and returns the new end() which should be used to update the message size
auto end_new = std::remove_if(
  cloud.begin(),
  cloud.end(),
  [](const PointXYZI & p) {
    bool intensity_is_too_low = p.intensity < 15.0f;
    bool z_is_too_big = p.z > 10.0f;
    return intensity_is_too_low || z_is_too_big;
  });
cloud.erase_till_end(end_new);
```

#### std::copy_if
```cpp
// copy points that satisfy a certain condition to another point cloud
// std::copy_if copies the elements meeting the criteria in the unary predicate
// and returns the new end() of the target point cloud
// which should be used to update the message size

// first make sure cloud2 is the same size as cloud1
cloud2.resize(cloud1.size());

// copy from cloud1 to cloud2
auto end_new_cloud2 = std::copy_if(
  cloud1.begin(),
  cloud1.end(),
  cloud2.begin(),
  [](const PointXYZI & p) {
    bool intensity_is_too_low = p.intensity < 15.0f;
    bool z_is_too_big = p.z > 10.0f;
    return intensity_is_too_low || z_is_too_big;
  });

// remove the excess elements from cloud2 with erase_till_end method
cloud2.erase_till_end(end_new_cloud2);
```

#### std::transform
```cpp
// transform cloud1 to cloud2 by applying a function on each point

// first make sure cloud2 is the same size as cloud1
cloud2.resize(cloud1.size());

// translate cloud 10m up in z axis
std::transform(
  cloud1.begin(),
  cloud1.end(),
  cloud2.begin(),
  [](const PointXYZI & p) {
    return PointXYZI(p.x, p.y, p.z + 10.0f, p.intensity);
  });
```

## Assumptions / Known limits
Cloud wrapper requires the exact point structure of the message to be known
in advance.
This shouldn't be a problem for the Autoware.Auto framework because the drivers
are a part of the autonomous driving stack. And the exact point fields, offsets
should be defined.

The supported point structures and fields are located in `types_point_cloud2.hpp`.
This file consists of pairs of structs and fields.
For example for PointXYZI, the pair is as following:
```cpp
/// PointXYZI
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointXYZI
{
  PointXYZI() = default;
  PointXYZI(float32_t x, float32_t y, float32_t z, float32_t intensity)
  : x(x), y(y), z(z), intensity(intensity) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  float32_t intensity{0.0f};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointXYZI{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("intensity", PointField::FLOAT32, 12, 1)
};
```
Here the [__attribute__((packed))](https://gcc.gnu.org/onlinedocs/gcc-4.0.2/gcc/Type-Attributes.html)
makes sure compiler doesn't add unnecessary padding to the struct.
Because `sizeof(PointXYZI)` should be equal to `point_step` in the message.

In some instances there might be unused bytes and these should be put into the struct.
For LGSVL Point type, it is covered like so:
```cpp
/// PointLgsvl
struct __attribute__((packed)) CLOUD_WRAPPER_PUBLIC PointLgsvl
{
  PointLgsvl() = default;
  PointLgsvl(float32_t x, float32_t y, float32_t z, uint8_t intensity, float64_t timestamp)
  : x(x), y(y), z(z), intensity(intensity), timestamp(timestamp) {}
  float32_t x{0.0f};
  float32_t y{0.0f};
  float32_t z{0.0f};
  uint32_t _padding_4_01{0};
  uint8_t intensity{0};
  uint8_t _padding_1_01{0};
  uint16_t _padding_2_01{0};
  uint32_t _padding_4_02{0};
  float64_t timestamp{0.0};
};

CLOUD_WRAPPER_PUBLIC extern const std::vector<PointField> fields_PointLgsvl{
  make_point_field("x", PointField::FLOAT32, 0, 1),
  make_point_field("y", PointField::FLOAT32, 4, 1),
  make_point_field("z", PointField::FLOAT32, 8, 1),
  make_point_field("intensity", PointField::UINT8, 16, 1),
  make_point_field("timestamp", PointField::FLOAT64, 24, 1)
};
```
Here the `_padding_xx` bytes are placeholders for unused bytes.
The struct should match the point fields of the expected message.

Currently in Autoware.Auto only `PointCloud2` sources are `LGSVL` and `velodyne_nodes`.
The `velodyne_nodes` publishes the cloud in `PointXYZI` format. 
The rest of nodes use the same format for intra-node communication in Autoware.Auto.
Only LGSVL publishes in a different format and a node is written to convert its
format into the `PointXYZI` and this node is called `lgsvl_cloud_converter` located in
drivers folder.


## Error detection and handling

In the constructor, it has some sanity checking operations if they fail it throws
necessary `std::runtime_error` or `std::length_error` exceptions.

Implemented `at` operator checks for the index given to be within boundaries and throws 
`std::out_of_range` if necessary.

Just like working with a vector, the user is responsible of handling these exceptions.

# Future extensions / Unimplemented parts
<!-- To Be Listed -->


# Related issues

https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/102