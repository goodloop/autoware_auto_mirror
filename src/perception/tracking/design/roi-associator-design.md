ROI Associator {#roi-associator}
============

The tracks within the tracker represents 3D objects around the robot. If the 2D measurements of 
any of these objects is available, then they can be associated to reinforce the track information.

# Purpose / Use cases

Tracking can be reinforced by fusing the data from multiple modalities that complement each 
other. Computer vision is one of such modalities that can offer certain strengths like being 
able to extract semantic information better than other sensors in properly lighted conditions.

In order to use the visual detections to reinforce the track information, an associator that 
can associate the ROIs from images and tracks is required.

# Design

[GreedyRoiAssociator](@ref autoware::perception::tracking::GreedyRoiAssociator) 
will associate tracked objects with ROIs from an image by first 
projecting each track to the image plane and then selecting the best match according to a 
match metric which is currently set to [IOUHeuristic](@ref 
autoware::perception::tracking::IOUHeuristic).

## Inner-workings / Algorithms

See [Projection](@ref projection) document for more details on how the projection is executed.

* Tracks that are not on the image plane are not associated
* Tracks that do not have matching ROI counterparts are not associated
* ROIs that do not have matching ROI counterparts are not associated
* To consider a ROI and a track to be a match, the computed IOU between them must be greater 
than a threshold.

## Inputs / Outputs / API

Inputs:
* Camera intrinsics
* Camera transformation
* `autoware_auto_msgs::msg::TrackedObjects`
* `autoware_auto_msgs::msg::ClassifiedRoiArray`

Outputs:
* [AssociatorResult](@ref autoware::perception::tracking::AssociatorResult)


## Related issues

- #983: Integrate vision detections in object tracker 