filter_node_base {#filter-node-base-package-design}
===========

This is the design document for the `filter_node_base` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The FilterNodeBase class provides a base implementation for all filter nodes from the
AutowareArchitectureProposal repository. 


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The FilterNodeBase provides a generic filter implementation that takes an input pointcloud,
performs some manipulation to the pointcloud and outputs the pointcloud on a different topic.
This allows for the the filter implementations to have the same basic set of APIs and standardises
the data types used for input and outputs of these processes.

## Inner-workings / Algorithms
<!-- If applicable -->

### filter

The `filter` method is a virtual method in the FilterNodebase class.
Child classes inheriting from this base class need to implement the `filter` method.
The `filter` method is defined as follows:

```{cpp}
virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;
```

There are three inputs to this method with one modified output representing the filtered PointCloud:
- `input`: `sensor_msgs::msg::PointCloud2::ConstSharedPtr` - pointcloud as received by the callback to be filter
- `indices`: `pcl::IndicesPtr` - pointer to the indices of a pointcloud [not always used]
- `output`: `sensor_msgs::msg::PointCloud2` - reference to the pointcloud object which will be modified by the filter method.


### computePublish

The `computePublish` method calls the child `filter` method to process the pointcloud, then
publishes the resulting output on a topic.

```{cpp}
void computePublish(const PointCloud2ConstPtr & input, const IndicesPtr & indices);
```

There are two inputs to this method:
- `input`: `sensor_msgs::msg::PointCloud2::ConstSharedPtr` - pointcloud as received by the callback to be filter
- `indices`: `pcl::IndicesPtr` - pointer to the indices of a pointcloud [not always used]


### input_indices_callback

The `input_indices_callback` method is bound to a subscriber object at during the construction of the class.
Depending on the flags selected, up to two topics may be bound to this method.
The following is the definition of the `input_indices_callback` method:

```{cpp}
void input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
```

There are two inputs to this class:
- `cloud`: `sensor_msgs::msg::PointCloud2::ConstSharedPtr` - pointcloud as received by the callback to be filter
- `indices`: `pcl::IndicesConstPtr` - pointer to the indices of a pointcloud [not always used]

This method determines the validity of the pointcloud and indices, transforms the indices if they
are passed as an argument, then calls the `computePublish` method above to process the messages.
Because there is a possibility of more than one ROS2 topic bound to the same callback, the
synchronization policy will need to be decided during constructions these are determined by the
parameters sets as input, see the below for more information.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
### Inputs

A node inheriting from @ref autoware::perception::filters::filter_node_base::FilterNodeBase require
the following parameters to be set:
- `max_queue_size` - defines the maximum size of queues
- `use_indices` - binds an `indices` (`pcl_msgs::msg::PointIndices`) topic to the callback
  - `false` = Use only the pointcloud as input
  - `true` = Bind the callback `input_indices_callback` to a pointcloud and indices topic
- `approximate_sync` - defines the synchronization policy for two topics being received by a single callback
  - `false` = `message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>`
  - `true` = `message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>`

@note `approximate_sync` is only used is `use_indices` is set to `true`.


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
