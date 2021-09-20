Safety Monitor {#safety_monitor-package-design}
===========

This is the design document for the `safety_monitor` package.

# Purpose / Use cases

When running Autoware, there is a need to ensure all nodes have been given sufficient resources and are able to run at the intended frequency.
Any processing pipeline involving multiple nodes should also adhere to a latency quota overall to ensure successful operation.
Hence, a certain monitoring functionality is required to detect when nodes are performing below tolerance and react appropriately.

# Design

The safety monitor will have 2 operating modes:

- Self-monitoring: nodes monitor their own performance using ROS timer API and take appropriate action in the event of a failure.
- Safety Monitor: All monitored nodes emit events on an event topic. This topic is monitored using a dedicated safety monitor node.
  This dedicated node is in charge of detecting failures and taking appropriate action.

The user can choose between modes by setting parameters in launch files.

Each publishing node shall, alongside the main topic, publish the intended publishing interval of the topic.
On the subscriber side, this information can be used to make sure the subscriber callback is invoked at the intended rate.
The subscriber callback should take less time than the minimum message interval in order to keep up with the rest of the system.
This condition can be checked automatically.
A further time limit can be imposed onto the subscriber callback by the programmer.
This is to deal with situations where the end-to-end latency in a processing chain is limited and each step in the chain gets a quota.

## Assumptions / Known limits

Assumptions:

- ROS timers are accurate.
- The DDS implementation suffers minimum delay

## Inputs / Outputs / API

Instead of inheriting rclcpp::Node class to build the nodes, the user would use

```cpp
class MyNode : public MonitoredNode {
  MyNode (const rclcpp::NodeOptions & options)
    : MonitoredNode("listener", options)
  {
    ...
  }
}
```

Publishers and subscribers are created using the `create_monitored_<>` api.
`MonitoredSubscription` and `MonitoredPublisher` are wrappers around the ROS base classes to implement timing checks transparently.
`MonitoredSubscription` object also contains information about the intended publishing interval of the subscribed topic.
This information can be used to determine the intended publishing interval of the publishers in the current node.
A filter node, for example, subscribes to a incoming topic and publishes on the outgoing topic.
The intended rate of the outgoing topic depends on the rate of the incoming topic.

```cpp
MonitoredSubscription::SharedPtr m_sub = create_monitored_subscription<MessageTypeT>("TOPIC", QoS, max_callback_time_ms);
MonitoredPublisher::SharedPtr m_pub = create_monitored_publisher<MessageTypeT>("TOPIC", QoS, m_sub->min_interval_ms, m_sub->max_interval_ms);
```

## Inner-workings / Algorithms

### Detecting Anomalies

1. Monitor subscription callback frequency: At the start of a callback, a timer is started.
   The time is set to expire at max_interval specified in the API.
   If a second callback comes in before the timer expires, the elapsed time on the timer is checked.
   If a node is operating normally, The elapsed time should be larger than the min_interval.
   If the timer expires before a second callback occurs or the min_interval check fails, a handler is triggered to print a warning and optionally shutdown the node.
   Otherwise, the timer is reset.
2. Monitor subscription callback duration: same method applies. A timer is started before the callback is invoked, and reset at the end of the callback.
3. Monitor end to end latency: If an external safety monitor node is used, there exists the possibility to register extra rules with the node to monitor delay in a whole processing pipeline by monitoring the input and output topics.
   This is useful for enforcing a limit on the delay between the sensors and the controls.

Timers and handlers can exist either as part of the monitored node or a separate safety monitor node.

### Communication

If an external safety monitor node is present, monitored nodes communicate with the monitor node via a single topic.
Each time an event happens, eg. callback_started, callback_ended, a message containing the event name and timestamp is sent to the topic.
The monitor node analyses this stream of events and starts/stops timers appropriately.
The safety monitor node can be run on dedicated safety-hardened hardware to ensure its correct operation.

### Interval Propagation

The min/max_interval_ms members of the monitored subscriber are held as futures. The value is only set after the relevant information is received from an upstream node.
This should always happen before the first message is published from the current node when the interval values are propagated to downstream nodes via the publisher.

## Error detection and handling

Safety monitor functionality wraps around a normal node and should not interfere with the normal error handling mechanism of the node.

# Security considerations

- A dedicated safety monitor node is a high value target as it ensures the normal running of the whole system and thus prevents certain types of attacks.
Hence, the usual steps should be taken to secure Linux and ROS in order to prevent attacks.
The safety monitor should be run on dedicated safety hardware on a high value deployment.
- The safety monitoring functionality relies on the ROS API to work.
Therefore, usual practices to secure ROS and Linux apply.

# Future extensions / Unimplemented parts

The external monitor will be implemented as a future step.
Nodes will operate in self monitor mode only to start with.

# Related issues

- [#821](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/821) - Detect when nodes' incoming messages are skipped
