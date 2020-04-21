Instructions
============


This tool takes the rosbag2 version of the Moriyama dataset and transforms it to match the Autoware.Auto conventions for types, frame ids and topic names.

Once we have downloaded the rosbag2 file from https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/ROSBAG-Demo#demo-data, we will need to start three ADE terminals.


Terminal 1:

- Run the dataset converter:

```shell
$ ros2 run rosbag2_moriyama_converter dataset_converter
```

Terminal 2:

- Record the target bag:

```shell
$ ros2 bag record -o sample_moriyama_150324_autowareauto /gnss/nmea_raw /lidar_front/points_raw /nmea_sentence
[INFO] [rosbag2_storage]: Opened database 'sample_moriyama_150324_autowareauto'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/nmea_sentence'
[INFO] [rosbag2_transport]: Subscribed to topic '/lidar_front/points_raw'
[INFO] [rosbag2_transport]: Subscribed to topic '/gnss/nmea_raw'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

Terminal 3:

- Replay the original bag:

```shell
$ ros2 bag play sample_moriyama_150324.bag2/
```

Once the terminal 3 is done replaying the bag, just Ctrl-C the recorder and the converter.

The new bag should look like the following:

```shell
$ ros2 bag info sample_moriyama_150324_autowareauto/

Files:             sample_moriyama_150324_autowareauto.db3
Bag size:          7.9 GiB
Storage id:        sqlite3
Duration:          479.95s
Start:             Apr 17 2020 17:27:20.631 (1587137240.631)
End                Apr 17 2020 17:35:19.726 (1587137719.726)
Messages:          19163
Topic information: Topic: /gnss/nmea_raw | Type: sensor_msgs/msg/NavSatFix | Count: 2396 | Serialization Format: cdr
                   Topic: /lidar_front/points_raw | Type: sensor_msgs/msg/PointCloud2 | Count: 4787 | Serialization Format: cdr
                   Topic: /nmea_sentence | Type: nmea_msgs/msg/Sentence | Count: 11980 | Serialization Format: cdr
```
