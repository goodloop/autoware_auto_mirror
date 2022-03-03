Map Creation {#map-creation-howto}
==============================

# Overview
Autoware requires two types of maps for autonomous driving:
* Geometric information of the environment (pointcloud map)
* Semantic information of roads (lanelet2 map)

The maps are typically created by mapping companies, but users can also generate their own maps using free tools available online.
This page shares information about the tools that you may use to create above maps for Autoware.Auto.

# Creating Pointcloud Map

You can use the following tools to generate pointcloud maps from rosbags that includes LiDAR data:
* [Interactive SLAM](https://github.com/SMRT-AIST/interactive_slam)
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

If you are generating a map for simulation environment, you may can follow the following instruction from LGSVL README to export a pointcloud map:
* https://www.svlsimulator.com/docs/simulation-content/pointcloud-export/

# Creating Lanelet2 Map
You can use the following tools to generate lanelet2 map:
* [JOSM](https://josm.openstreetmap.de/), See [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_maps/README.md#editing-lanelet2-maps) for adding Lanelet2 specific presets
* [Tier IV VectorMapBuilder](https://tools.tier4.jp/vector_map_builder_ll2/), See [here](https://tools.tier4.jp/static/manuals/vector_map_builder_ll2_user_guide.pdf) for usage.
