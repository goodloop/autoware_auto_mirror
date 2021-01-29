Planning {#autoware-planning-design}
========

## Domain Description

The `planning` sub-directory contains nodes and libraries related to producing a trajectory towards the goal destination. These include packages may not necessarily be able to produce a refined trajectory but are involved in the planning process in order to alleviate the complexity of the path planning optimization problem. In addition, this directory also contains nodes used for testing and recording of the trajectory generation process.

## Subpages

- Behavior planner
  - @subpage behavior-planner
  - @subpage behavior-planner-node
- RecordReplay planner
  - @subpage recordreplay-planner
  - @subpage recordreplay-planner-nodes
- Lane planner
  - @subpage lane-planner
  - @subpage lane-planner-nodes
- Object Collision Estimator
  - @subpage object-collision-estimator
  - @subpage object-collision-estimator-nodes
- Parking planner
  - @subpage parking-planner
  - @subpage parking-planner-nodes
- @subpage osm-planner
- @subpage trajectory-planner-node-base
- @subpage trajectory-smoother-package-design
- @subpage trajectory-spoofer-design
