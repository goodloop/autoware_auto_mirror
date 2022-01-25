Error Monitor {#error_monitor}
===========

This is the design document for the `error_monitor` package.


# Purpose / Use cases
The purpose of the `error_monitor` is to monitor all Autoware nodes implementing the `Monitored Node API` from the `monitored_node` package, detect when some of them are not running at their intended frequency and, if needed, estimate the gravity of the situation. This is part of the wider `Autoware Monitoring System`. If a node is not implemented using the `API` mentioned above, the `error_monitor` can't monitor that node.


# Design
The monitored Autoware nodes monitor their own performance and publish data on the `/diagnostic` topic. This data can be used to establish whether a node is running normally or is running too slow.
The `error_monitor` subscribes to this topic and analyzes its content, publishing on `/vehicle/driving_capability`, which is then consumed by the `emergency_handler`.
By default, the `error_monitor` checks the contents of the `/diagnostic`, treating all nodes as critical. The user can specify which topics/nodes they are not interested in, putting this information in a configuration file.


## Assumptions / Known limits
If a self-monitoring node crashes, it stops publishing on the `/diagnostic` topic without publishing an error message first, in this case the `error_monitor` doesn't get informed of the problem. A possible solution to this issue is discussed in the Future extensions section.


## Inputs / Outputs / API
Inputs
* `diagnostic_msgs/msg/DiagnosticStatus` is published by the self-monitoring nodes, and is used to check which nodes are functioning correctly and which are not.

Outputs
* `autoware_auto_msgs/msg/DrivingCapability`.
As part of `DrivingCapability`, we have `HazardStatus`:
  - `NO_FAULT`: No error detected.
  - `SAFE_FAULT`: One or more nodes are not behaving as expected, but there is no danger.
  - `LATENT_FAULT`: One or more nodes are not behaving as expected, but we are able to keep driving (e.g. slowing down).
  - `SINGLE_POINT_FAULT`: Some critical parts are broken and we are not able to keep driving.
  - We also have `emergency` and `emergency_holding`, the latter one is for keeping the error status until an operator manually clears it.


## Inner-workings / Algorithms
The input is analyzed and a state machine is employed to keep track of the current state. According to this, the output topic is published.
Many possibilities can be taken into consideration here, for example we might have a node that is running with a lower frequency than the expected one. In this situation, depending on the node, the car might still be drivable or we might need to switch to "emergency mode".
By default, the `error_monitor` treats all topics/nodes as having critical importance, but the user can provide a configuration file containing the list of topics/nodes that they want to exclude from the monitoring. The file has this structure:
```
/**:
  ros__parameters:
    # don't monitor certain topics from certain nodes
    exclusion_list_topics_nodes: [
      [topic, node],
      [topic, node],
      [topic, node],
      [topic, node]
    ]
    # exclude certain topics from the monitoring
    exclusion_list_topics_nodes: [
      topic,
      topic,
      topic
    ]
    # exclude certain nodes from the monitoring
    exclusion_list_topics_nodes: [
      node,
      node
    ]
```


# Security considerations
A dedicated error monitor node is a high value target as it ensures the normal operation of the whole system and thus prevents certain types of attacks. Hence, the usual steps should be taken to secure Linux and ROS in order to prevent attacks.
The error monitor should/might be run on a dedicated safety hardware on a high value deployment.


# Future extensions / Unimplemented parts
A Possible extension of the functionality would be letting the user filter out specific `/diagnostic` messages, for example a filter which excludes `callback_start` or `callback_end` messages from a specific node.
In the future, we might also add a heartbeat checker for all the nodes we are monitoring, to check if the nodes are still running or have completely stopped/crashed. This check might be performed in the `error_monitor`, and a possible solution is asking the user to specify a maximum time we can wait for a node to publish its topics. After this time passes without the node publishing anything, we can say the node has stopped.


# Related issues
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/821
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1233
