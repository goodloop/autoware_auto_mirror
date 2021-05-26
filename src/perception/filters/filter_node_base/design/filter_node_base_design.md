filter_node_base {#filter-node-base-package-design}
===========


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The `FilterNodeBase` class provides a base implementation for all filter nodes from the
[AutowareArchitectureProposal repository](https://github.com/tier4/AutowareArchitectureProposal.iv).

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The FilterNodeBase provides a generic filter implementation that takes an input point cloud,
performs some manipulation to the point cloud and outputs the point cloud on a different topic.
This allows the filter implementations to have the same basic set of APIs and standardises
the data types used for input and outputs of these processes.

## Inner-workings / Algorithms
<!-- If applicable -->
Developers writing a class that inherits from the FilterNodeBase will need to implement two methods:
 * `filter` - processes the input pointcloud and modifies the reference output pointcloud
 * `get_node_parameters` - processes the parameters during a parameter change event

These virtual functions are called by the parent class in the following methods:
 * `pointcloud_callback` - calls `filter`
 * `param_callback` - calls `get_node_parameters`

### pointcloud_callback

The `pointcloud_callback` method is bound to a subscriber object at during the construction of the
class. This method determines the validity of the point cloud then calls the `filter` method to
process the pointcloud. The following is the definition of the `pointcloud_callback` method:

```{cpp}
FILTER_NODE_BASE_LOCAL void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
```

### filter

The `filter` method is a virtual method in the FilterNodebase class. The main filter algorithm is
implemented in this method and the result of the filtering is returned via the `output`

```{cpp}
FILTER_NODE_BASE_LOCAL virtual void filter(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output) = 0;
```

There are two inputs to this method with one modified output representing the filtered PointCloud:
- `input` - point cloud passed by reference provided by the callback; not modified. 
- `output` - filter algorithm in this method modifies this argument which is passed by reference
from the `pointcloud_callback` method

### param_callback

The `param_callback` method is bound to a parameter service callback which is triggered when the
node parameters are changed. The method returns a `SetParametersResult` 

```{cpp}
FILTER_NODE_BASE_LOCAL rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & p);
```

There is one input to this method:
 - p - List of parameters declared in the node namespace.

The method returns the SetParameterResult which will determine the event successfully completed.
The return is passed from the the `get_node_parameters` where the value is determined by successful
parameter retrieval.

### get_node_parameters

The `get_node_parameters` method retrieves the node parameters from the input parameter vector,
creates the return result structure, and returns the structure. 

```{cpp}
FILTER_NODE_BASE_LOCAL virtual rcl_interfaces::msg::SetParametersResult get_node_parameters(
    const std::vector<rclcpp::Parameter> & p) = 0;
```

There is one input to this method:
 - p - List of parameters declared in the node namespace passed from `param_callback`

The method returns the SetParameterResult which will determine the event successfully completed.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
### Inputs

A node inheriting from @ref autoware::perception::filters::filter_node_base::FilterNodeBase
requires the following parameter to be set:
- `max_queue_size` - defines the maximum size of queues

Child classes inheriting from the `FilterNodeBase` may declare additional parameters in the
constructor of the child class. Any parameter declared should be retrieved in the
`get_node_parameters` method - unless the parameter is not intended to change (e.g.
`max_queue_size`).


## Assumptions / Known limits
<!-- Required -->

* No transformation of frames is required to be done by the filter since this will be handled by
an external node purely for transforming topics.


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
See issue:
- #917
