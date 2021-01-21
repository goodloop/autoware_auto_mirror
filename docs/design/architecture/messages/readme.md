# Creating message diagrams

Data and scripts in this folder exist to create overview diagrams for messages sent out by components of Autoware.Auto.

The source of truth is in the `*.csv` files. The expected format is three columns, where the first line is ignored. The file name without the `.csv` extension is interpreted as the name of the component. For example in `Sensing.csv`:

```csv
"Sub-module Pipeline","Message Type","Topic"
"3D Lidar","sensor_msgs/msg/PointCloud2","/sensors/lidar/points_processed"
"2D Lidar","sensor_msgs/msg/LaserScan","/sensors/lidar/scans_processed"
```

The script `convert.sh` takes all `csv` files in this directory as input and generates graphs in the graphviz language with extension `.gv`. Modify its source to invoke `dot` (part of `graphviz`) to generate an image (e.g. PNG, SVG, PDF) in this folder for testing purposes. Only add the `*.gv` files to git as `doxygen` creates the images automatically.

The topology and formatting of node labels in the graph is defined inside `create_dot.py`.
