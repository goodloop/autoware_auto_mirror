/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the cepton_node class.
#ifndef CEPTON_NODE__CEPTON_CLOUD_NODE_HPP_
#define CEPTON_NODE__CEPTON_CLOUD_NODE_HPP_
#include <cepton_node/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <udp_driver/udp_driver_node.hpp>
#include <cepton_node/cepton_common.hpp>
#include <cepton_sdk.hpp>
#include <cepton_sdk_util.hpp>

namespace apex_auto
{
namespace drivers
{
namespace cepton_node
{
/// \brief A class that wraps a udp receiver and conversion calls from an external library for the
///        Cepton LiDar
class CEPTON_NODE_PUBLIC CeptonCloudNode
  : public udp_driver::UdpDriverNode<Packet, sensor_msgs::msg::PointCloud2>
{
public:
  /// \brief Constructor
  /// \param[in] node_name name of the node for rclcpp internals
  /// \param[in] topic Name of the topic to publish output on
  /// \param[in] ip Expected source IP of UDP packets
  /// \param[in] port Port that this driver listens to (i.e. sensor device at ip writes to port)
  /// \param[in] timeout_ns Timeout duration after which receiving a UDP packet is said to fail
  /// \param[in] init_timeout_ms Timeout duration specific to initialization. It should be set
  /// to a sufficient duration for all necessary components to be up and running
  /// \param[in] cloud_size Number of points (16 bytes) worth of bytes to preallocate in messages
  /// \param[in] frame_id Frame id for point cloud message
  /// \param[in] max_cycle_time Maximum cycle time of this node, e.g. the time it takes to receive
  ///                           handle, and publish a packet should be no more than this
  /// \param[in] translation Translation vector, in meters
  /// \param[in] rotation Rotation quaternion in x-y-z-w order
  /// \param[in] node_namespace Namespace for this node
  /// \param[in] expected_num_subscribers The expected number of subscribers matched to this node
  /// \throw runtime error if failed to start threads or configure driver
  CeptonCloudNode(
    const apex::string_strict256_t & node_name,
    const apex::string_strict256_t & topic,
    const apex::string_strict256_t & ip,
    const uint16_t port,
    const std::chrono::nanoseconds timeout_ns,
    const std::chrono::milliseconds init_timeout_ms,
    const std::chrono::nanoseconds max_cycle_time,
    const uint32_t cloud_size,
    const apex::string_strict256_t & frame_id = "base_link",
    const std::array<float32_t, 3U> & translation = {0.0F, 0.0F, 0.0F},
    const std::array<float32_t, 4U> & rotation = {0.0F, 0.0F, 0.0F, 1.0F},
    const std::size_t expected_num_subscribers = 1U,
    const apex::string_strict256_t & node_namespace = "");

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] max_cycle_time Maximum cycle time of this node, e.g. the time it takes to receive
  ///                           handle, and publish a packet should be no more than this
  /// \param[in] node_namespace Namespace for this node
  CeptonCloudNode(
    const apex::string_strict256_t & node_name,
    const std::chrono::nanoseconds max_cycle_time,
    const apex::string_strict256_t & node_namespace = "");

  /// Destructor; calls cepton_deinitialize()
  ~CeptonCloudNode() override;

protected:
  /// \brief Does nothing. Point block is static
  void init_output(sensor_msgs::msg::PointCloud2 & output) override;
  /// \brief Dispatches to the translator's convert call
  bool8_t convert(const Packet & pkt, sensor_msgs::msg::PointCloud2 & output) override;
  /// \brief Does nothing. A packet fully fits into a point block.
  bool8_t get_output_remainder(sensor_msgs::msg::PointCloud2 & output) override;

private:
  /// \brief Ensure consistency of point cloud message fields wrt Cepton SDK point definition
  /// \param[in] pcl The message for which the fields must be checked
  /// \throw std::runtime_error If something is inconsistent
  void check_fields(const sensor_msgs::msg::PointCloud2 & pcl) const;

  const uint32_t m_cloud_size;
  const apex::string_strict256_t m_frame_id;
  cepton_sdk::util::CompiledTransform m_transform;
};  // class CeptonCloudNode
}  // namespace cepton_node
}  // namespace drivers
}  // namespace apex_auto
#endif  // CEPTON_NODE__CEPTON_CLOUD_NODE_HPP_
