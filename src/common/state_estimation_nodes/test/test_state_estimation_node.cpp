// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>
#include <state_estimation_nodes/state_estimation_node.hpp>

#include <memory>
#include <string>
#include <vector>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using autoware::common::state_estimation::StateEstimationNode;
using autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;

namespace
{
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}

tf2::TimePoint to_tf_time_point(nav_msgs::msg::Odometry::_header_type::_stamp_type & stamp)
{
  using std::chrono::seconds;
  using std::chrono::nanoseconds;
  return tf2::TimePoint{seconds{stamp.sec} + nanoseconds{stamp.nanosec}};
}

geometry_msgs::msg::TransformStamped get_dummy_transform(const nav_msgs::msg::Odometry & donor_msg)
{
  geometry_msgs::msg::TransformStamped dummy_transform{};
  dummy_transform.child_frame_id = donor_msg.child_frame_id;
  dummy_transform.header.frame_id = donor_msg.header.frame_id;
  dummy_transform.header.stamp = donor_msg.header.stamp;
  dummy_transform.transform.translation.set__x(0.0).set__y(0.0).set__z(0.0);
  dummy_transform.transform.rotation.set__x(0.0).set__y(0.0).set__z(0.0).set__w(1.0);
  return dummy_transform;
}

Odometry create_empty_odometry(const std::string & frame_id, const std::string & child_frame_id)
{
  Odometry msg{};
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 1.0;
  msg.twist.covariance[0] = 1.0;
  msg.twist.covariance[7] = 1.0;
  return msg;
}

PoseWithCovarianceStamped create_empty_pose(const std::string & frame_id)
{
  PoseWithCovarianceStamped msg{};
  msg.header.frame_id = frame_id;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 1.0;
  return msg;
}

RelativePositionWithCovarianceStamped create_empty_relative_pose(
  const std::string & frame_id,
  const std::string & child_frame_id)
{
  RelativePositionWithCovarianceStamped msg{};
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.covariance.resize(9);
  msg.covariance[0] = 1.0;
  msg.covariance[4] = 1.0;
  msg.covariance[8] = 1.0;
  return msg;
}

}  // namespace

// TODO(niosus): Re-enable tests when #488 is solved.
class DISABLED_StateEstimationNodeTest : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
    m_mock_node = std::make_shared<rclcpp::Node>("mock_node");
    const auto spin_thread = false;
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
      m_tf_buffer, m_mock_node, spin_thread);
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

  template<typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr create_publisher(
    const std::string & topic,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL},
    const std::int32_t history_size = 10)
  {
    typename rclcpp::Publisher<MsgT>::SharedPtr odometry_publisher =
      m_mock_node->create_publisher<MsgT>(topic, history_size);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (m_mock_node->count_subscribers(topic) < 1) {
      spent_time += dt;
      EXPECT_LT(spent_time, timeout) << "Nobody is listening to the mock topic we publish.";
      std::this_thread::sleep_for(dt);
    }
    return odometry_publisher;
  }

  template<typename MsgT, typename NodeT>
  typename rclcpp::Subscription<MsgT>::SharedPtr create_result_subscription(
    const std::string & topic,
    NodeT * node_under_test,
    std::function<void(const typename MsgT::SharedPtr msg)> callback,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL})
  {
    EXPECT_NE(node_under_test, nullptr);
    typename rclcpp::Subscription<MsgT>::SharedPtr subscription =
      m_mock_node->create_subscription<MsgT>(topic, 10, callback);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (node_under_test->count_publishers(topic) < 1) {
      spent_time += dt;
      EXPECT_LT(spent_time, timeout) << "The node under test is not publishing what we listen to.";
      std::this_thread::sleep_for(dt);
    }
    return subscription;
  }

  rclcpp::Node::SharedPtr get_mock_node() {return m_mock_node;}
  tf2::BufferCore & get_tf_buffer() {return m_tf_buffer;}

private:
  rclcpp::Node::SharedPtr m_mock_node{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  tf2::BufferCore m_tf_buffer;
};

INSTANTIATE_TEST_CASE_P(
  StateEstimationNodeTests,
  DISABLED_StateEstimationNodeTest,
  ::testing::Values(true, false));

/// @test Test that if we publish one message, it generates a state estimate which is sent out.
TEST_P(DISABLED_StateEstimationNodeTest, publish_and_receive_odom_message) {
  const bool publish_tf = GetParam();
  auto msg = create_empty_odometry("map", "base_link");
  msg.header.stamp.sec = 5;
  msg.header.stamp.nanosec = 12345U;

  auto dummy_transform = get_dummy_transform(msg);

  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override(
    "topics.input_odom", std::vector<std::string>{"/odom_topic_1"});
  node_options.append_parameter_override(
    "topics.input_pose", std::vector<std::string>{"/pose_topic_1"});
  node_options.append_parameter_override(
    "topics.input_twist", std::vector<std::string>{"/twist_topic_1"});
  node_options.append_parameter_override("data_driven", true);
  node_options.append_parameter_override("publish_tf", publish_tf);
  node_options.append_parameter_override("frame_id", "map");
  node_options.append_parameter_override("child_frame_id", "base_link");
  node_options.append_parameter_override("mahalanobis_threshold", 10.0);
  node_options.append_parameter_override(
    "state_variances", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  node_options.append_parameter_override(
    "process_noise_variances.acceleration", std::vector<double>{1.0, 1.0});
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  auto count_received_msgs{0};
  auto fake_odom_publisher = create_publisher<Odometry>("/odom_topic_1");
  auto result_odom_subscription = create_result_subscription<Odometry>(
    "/filtered_state", node.get(),
    [&count_received_msgs](
      const Odometry::SharedPtr) {
      count_received_msgs++;
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (count_received_msgs < 1) {
    msg.header.stamp = to_ros_time(std::chrono::system_clock::now());
    dummy_transform.header.stamp = msg.header.stamp;
    node->buffer().setTransform(dummy_transform, "test");
    fake_odom_publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_mock_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  if (publish_tf) {
    EXPECT_TRUE(
      get_tf_buffer().canTransform("map", "base_link", to_tf_time_point(msg.header.stamp)));
    const auto transform{
      get_tf_buffer().lookupTransform("map", "base_link", to_tf_time_point(msg.header.stamp))};
    EXPECT_EQ(transform.header.frame_id, "map");
    EXPECT_EQ(transform.child_frame_id, "base_link");
    EXPECT_EQ(transform.header.stamp.sec, msg.header.stamp.sec);
    EXPECT_EQ(transform.header.stamp.nanosec, msg.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(transform.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(transform.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.0);
  } else {
    EXPECT_FALSE(
      get_tf_buffer().canTransform("map", "base_link", to_tf_time_point(msg.header.stamp)));
  }
  SUCCEED();
}

/// @test Test that we can track an object moving in a straight line.
TEST_P(DISABLED_StateEstimationNodeTest, track_object_straight_line) {
  const bool publish_tf = GetParam();
  auto odom_msg = create_empty_odometry("map", "base_link");
  auto pose_msg = create_empty_pose("map");
  auto relative_pose_msg = create_empty_relative_pose("map", "base_link");

  auto dummy_transform = get_dummy_transform(odom_msg);

  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override(
    "topics.input_odom", std::vector<std::string>{"/odom_topic_1"});
  node_options.append_parameter_override(
    "topics.input_pose", std::vector<std::string>{"/pose_topic_1"});
  node_options.append_parameter_override(
    "topics.input_twist", std::vector<std::string>{"/twist_topic_1"});
  node_options.append_parameter_override(
    "topics.input_relative_pos", std::vector<std::string>{"/relative_pos_topic_1"});
  node_options.append_parameter_override("data_driven", true);
  node_options.append_parameter_override("publish_tf", publish_tf);
  node_options.append_parameter_override("frame_id", "map");
  node_options.append_parameter_override("child_frame_id", "base_link");
  node_options.append_parameter_override("mahalanobis_threshold", 10.0);
  node_options.append_parameter_override(
    "state_variances", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  node_options.append_parameter_override(
    "process_noise_variances.acceleration", std::vector<double>{1.0, 1.0});
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  std::vector<Odometry::SharedPtr> received_msgs{0};
  auto fake_odom_publisher = create_publisher<Odometry>("/odom_topic_1");
  auto fake_pose_publisher = create_publisher<PoseWithCovarianceStamped>("/pose_topic_1");
  auto fake_relative_pose_publisher = create_publisher<RelativePositionWithCovarianceStamped>(
    "/relative_pos_topic_1");
  auto result_odom_subscription = create_result_subscription<Odometry>(
    "/filtered_state", node.get(),
    [&received_msgs](
      const Odometry::SharedPtr msg) {
      received_msgs.push_back(msg);
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto speed = 1.0;
  const auto starting_time_point = std::chrono::system_clock::now();
  std::chrono::seconds total_travel_time{2LL};
  size_t messages_sent{};
  int i = 0;

  std::vector<std::string> msg_types{"odom", "pose", "relative_pose"};

  // cppcheck-suppress syntaxError // Trust me, this is valid C++ syntax.
  for (struct { std::chrono::milliseconds time_passed{}; std::size_t msg_idx{}; } iter;
    iter.time_passed <= total_travel_time;
    iter.time_passed += dt, iter.msg_idx = (iter.msg_idx + 1U) % msg_types.size())
  {
    const auto stamp = to_ros_time(starting_time_point + iter.time_passed);
    dummy_transform.header.stamp = stamp;
    node->buffer().setTransform(dummy_transform, "test");
    std::chrono::duration<double> seconds_passed{iter.time_passed};

    if (msg_types[iter.msg_idx] == "odom") {
      odom_msg.header.stamp = stamp;
      odom_msg.pose.pose.position.x = seconds_passed.count() * speed;
      odom_msg.pose.pose.position.y = seconds_passed.count() * speed;
      odom_msg.twist.twist.linear.x = speed;
      odom_msg.twist.twist.linear.y = speed;
      fake_odom_publisher->publish(odom_msg);
    } else if (msg_types[iter.msg_idx] == "pose") {
      pose_msg.header.stamp = stamp;
      pose_msg.pose.pose.position.x = seconds_passed.count() * speed;
      pose_msg.pose.pose.position.y = seconds_passed.count() * speed;
      fake_pose_publisher->publish(pose_msg);
    } else if (msg_types[iter.msg_idx] == "relative_pose") {
      relative_pose_msg.header.stamp = stamp;
      relative_pose_msg.position.x = seconds_passed.count() * speed;
      relative_pose_msg.position.y = seconds_passed.count() * speed;
      fake_relative_pose_publisher->publish(relative_pose_msg);
    }
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_mock_node());
    std::this_thread::sleep_for(dt);
    messages_sent++;
  }
  const auto distance_travelled = speed * total_travel_time.count();

  const auto max_wait_time{std::chrono::seconds{10LL}};
  std::chrono::milliseconds time_passed{0LL};
  i = 0;
  while (received_msgs.size() < messages_sent) {
    time_passed += dt;
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_mock_node());
    std::this_thread::sleep_for(dt);
    ASSERT_LT(time_passed, max_wait_time) <<
      "Some messages were dropped. Received: " << received_msgs.size();
    if (received_msgs.empty()) {continue;}
    if (publish_tf) {
      auto & msg = *received_msgs.back();
      EXPECT_TRUE(
        get_tf_buffer().canTransform(
          "map", "base_link", to_tf_time_point(msg.header.stamp)));
      const auto transform{get_tf_buffer().lookupTransform(
          "map", "base_link", to_tf_time_point(msg.header.stamp))};
      EXPECT_EQ(transform.header.frame_id, "map");
      EXPECT_EQ(transform.child_frame_id, "base_link");
      EXPECT_EQ(transform.header.stamp.sec, msg.header.stamp.sec);
      EXPECT_EQ(transform.header.stamp.nanosec, msg.header.stamp.nanosec);
      EXPECT_DOUBLE_EQ(transform.transform.translation.x, msg.pose.pose.position.x);
      EXPECT_DOUBLE_EQ(transform.transform.translation.y, msg.pose.pose.position.y);
      EXPECT_DOUBLE_EQ(transform.transform.translation.z, msg.pose.pose.position.z);
    } else {
      EXPECT_FALSE(
        get_tf_buffer().canTransform(
          "map", "base_link", to_tf_time_point(odom_msg.header.stamp)));
    }
  }
  const double epsilon = 0.2;
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.x, epsilon);
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.y, epsilon);
}

/// @test Test for the case when we publish on a timer.
TEST_F(DISABLED_StateEstimationNodeTest, publish_on_timer) {
  auto msg = create_empty_odometry("map", "base_link");
  msg.header.stamp.sec = 5;
  msg.header.stamp.nanosec = 12345U;

  auto dummy_transform = get_dummy_transform(msg);

  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override(
    "topics.input_odom", std::vector<std::string>{"/odom_topic_1"});
  node_options.append_parameter_override(
    "topics.input_pose", std::vector<std::string>{"/pose_topic_1"});
  node_options.append_parameter_override(
    "topics.input_twist", std::vector<std::string>{"/twist_topic_1"});
  node_options.append_parameter_override(
    "topics.input_relative_pos", std::vector<std::string>{"/relative_pos_topic_1"});
  node_options.append_parameter_override("frame_id", "map");
  node_options.append_parameter_override("child_frame_id", "base_link");
  node_options.append_parameter_override("mahalanobis_threshold", 10.0);
  node_options.append_parameter_override("output_frequency", 10.0);
  node_options.append_parameter_override(
    "state_variances", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  node_options.append_parameter_override(
    "process_noise_variances.acceleration", std::vector<double>{1.0, 1.0});
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  auto count_received_msgs{0};
  auto fake_odom_publisher = create_publisher<Odometry>("/odom_topic_1");
  auto result_odom_subscription = create_result_subscription<Odometry>(
    "/filtered_state", node.get(),
    [&count_received_msgs](
      const Odometry::SharedPtr) {
      count_received_msgs++;
    });

  // Check that before the node receives the first odometry message it is not publishing.
  const auto dt{std::chrono::milliseconds{100LL}};
  auto max_wait_time{std::chrono::seconds{2LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (time_passed < max_wait_time) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_mock_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (count_received_msgs > 0) {
      FAIL() << "The node should not have published before receiving an odometry message.";
    }
  }

  // Check that after the node receives the first odometry message it continuously publishes.
  max_wait_time = std::chrono::seconds{10LL};
  time_passed = std::chrono::milliseconds{0LL};
  const auto minimum_number_of_messages{20};
  while (count_received_msgs < minimum_number_of_messages) {
    if (count_received_msgs < 1) {
      // We want to stop publishing after receiving the first message as publishing is only needed
      // here to enable timer-based publishing of the node under test.
      msg.header.stamp = to_ros_time(std::chrono::system_clock::now());
      dummy_transform.header.stamp = msg.header.stamp;
      node->buffer().setTransform(dummy_transform, "test");
      fake_odom_publisher->publish(msg);
    }
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_mock_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive enough messages.";
    }
  }
  SUCCEED();
}
