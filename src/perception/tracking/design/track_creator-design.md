Track Creator {#tracking-track-creator}
===================

# Purpose / Use cases

Object tracking for an autonomous driving system involves associating new detections in every frame to tracks that the tracker keeps account of. New objects enter the field of view of the vehicle every frame and the perception sensor may also have false positives every frame. This means the tracker needs a module that will assess these detections that do not meet the matching criteria to be matched with any of the existing tracks. This module ideally needs to create new tracks only for detections that are from real objects which the tracker can take into its operation. 

# Design

[TrackCreator](@ref autoware::perception::tracking::TrackCreator) will keep hold of the [vision detections](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/ClassifiedRoiArray.idl) and [lidar clusters](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/DetectedObjects.idl) that are not associated with any of the tracks and then, based on a configurable policy, it will create new tracks from these.  
If the policy dictates, the association between vision detections and lidar clusters will be carried out using [GreedyRoiAssociator](@ref autoware::perception::tracking::GreedyRoiAssociator)  
This module is designed to be triggered to create new tracks after every new lidar clusters message. Hence it keeps track of only the last lidar clusters message but it holds a cache of the vision detections message

## Inner-workings / Algorithms

The logic used to create new tracks is controlled by [TrackCreationPolicy](@ref autoware::perception::tracking::TrackCreationPolicy).  

### LidarClusterOnly  
- Call `add_unassigned_lidar_clusters()` with the lidar clusters message and result from the lidar-track association. This method will go through every lidar cluster that was not associated to a track and store it internally  
- Call `create_tracks()`. This method will create one track per cluster that was stored internally in the previous step  
- Using this policy does not require a valid `GreedyRoiAssociator` object pointer. It can be initialized to nullptr

### LidarClusterIfVision
- Call `add_unassigned_lidar_clusters()` with the lidar clusters message and result from the lidar-track association. This method will go through every lidar cluster that was not associated to a track and store it internally  
- Call `add_unassigned_vision_detections()` with the vision detections message and result from the vision-track association. This method will go through every vision detection that was not associated to a track and store it internally. Every time this function is called a new vision detection message is created internally and pushed to a cache 
- Call `create_tracks()`. This method will first try to find a vision detection that is within `kMaxVisionLidarStampDiffMs` from the lidar cluster msg stamp. If it finds such a message it will try to associate the lidar clusters and the vision detections. New tracks will be created only from lidar clusters that are matched with a vision detection
- Using this policy requires a valid `GreedyRoiAssociator` object pointer

## Parameters
- TrackCreationPolicy - Choose one of the available policies based on the above explanation
- Default variance & Noise variance - These are values to be used in the initialization of new [TrackedObject](@ref autoware::perception::tracking::TrackedObject) objects
- kMaxVisionLidarStampDiffMs - Maximum difference between vision detections message stamp and lidar clusters message stamp to be considered for association

## Assumptions / Known Limitations
- This module assumes `create_tracks` will be called every time a lidar objects message is received
- All the functions in the module are assumed to be called from the same thread except for `add_unassigned_vision_detections()` which uses thread-safe data structures and can be called from a different thread 

# Future extensions / Unimplemented parts
- Support for radar detections
- Support for lidar detections

# Related issues:
- #1265 - Initial implementation
- #1120 - Add support for vision detection

