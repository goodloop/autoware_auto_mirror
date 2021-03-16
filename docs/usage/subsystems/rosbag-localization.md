Localization Demo using rosbag {#rosbag-localization-howto}
==============================

@tableofcontents

# Running the Localization Demo

1. Setup the Autoware environment following instructions in the [installation guide](@ref installation).

2. In the console, download and extract a rosbag.
```{bash}
> cd ~
> curl https://autoware-auto.s3.us-east-2.amazonaws.com/rosbag2/rosbag2-astuff-1-lidar-only.tar.gz | tar xz
```
3. Launch the demo.
   ```{bash}
   > ros2 launch autoware_demos localization_rosbag.launch.py
   ```
   There are a few options to the launch script.
   You can see more details by running:
   ```{bash}
   > ros2 launch -s autoware_demos localization_rosbag.launch.py
   ```
