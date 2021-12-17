apollo_lidar_segmentation {#apollo-lidar-segmentation-design}
==================

# Design

## Usage {#apollo-lidar-segmentation-design-usage}

### Neural network

This package will not build without a neural network for its inference.
The network is provided by the [neural_networks package](@ref neural-networks-design).
See its design page for more information on how to enable downloading pre-compiled networks (by setting the `DOWNLOAD_ARTIFACTS` cmake variable), or how to handle user-compiled networks.

### Backend

The backend used for the inference can be selected by setting the `apollo_lidar_segmentation_BACKEND` cmake variable.
The current available options are `llvm` for a CPU backend, and `vulkan` for a GPU backend.
It defaults to `llvm`.

## Convolutional Neural Networks (CNN) Segmentation

See the [original design](https://github.com/ApolloAuto/apollo/blob/3422a62ce932cb1c0c269922a0f1aa59a290b733/docs/specs/3d_obstacle_perception.md#convolutional-neural-networks-cnn-segmentation) by Apollo.
The paragraph of interest goes up to, but excluding, the "MinBox Builder" paragraph.
This package instead uses the Autoware BoundingBox message type for the bounding boxes.

Note: the parameters described in the original design have been modified and are out of date.

## Bounding Box

The lidar segmentation node establishes a bounding box for the detected obstacles.
The `L-fit` method of fitting a bounding box to a cluster is used for that.

## API

For more details on the API, see the
[documentation](@ref autoware::perception::segmentation::apollo_lidar_segmentation::ApolloLidarSegmentation).

# Reference

Lidar segmentation is based off a core algorithm by [Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md), with modifications from [TierIV] (https://github.com/tier4/lidar_instance_segmentation_tvm) for the TVM backend.
