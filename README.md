Slam - kit
---



Several docker are needed
- slam-kit-core
- glim_core
- velodyne-core
- rs-core


Commands in several docker containers


## slam-kit-core
```` bash
ros2 launch slam_kit_core bringup.launch.py
````

## glim_core

```` bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/root/ros2_ws/src/glim/config
````
### Important changes
* topic and frame names at `config_ros.json`
```json
{
  /*** ROS-related config ***/
  // --- Backend modules ---
  // If both local and global mapping are disabled, odometry estimation runs.
  // enable_local_mapping   : Enable local mapping module
  // enable_global_mapping  : Enable global mapping module
  //
  // --- Points config ---
  // keep_raw_points        : Keep raw points in the map (required for only extension modules)
  //
  // --- IMU config ---
  // imu_time_offset        : Time offset for IMU data (in seconds)
  // acc_scale              : Accelerometer scale factor (Set to 9.80665 for Livox sensors)
  //
  // --- TF config ---
  // GLIM publishes TF subtrees "map -> odom -> base_frame" and "imu -> lidar".
  // imu_frame_id           : IMU frame ID
  // lidar_frame_id         : LiDAR frame ID
  // base_frame_id          : Base frame ID. If blank, IMU frame ID is used.
  // odom_frame_id          : Odometry frame ID
  // map_frame_id           : Map frame ID
  // publish_imu2lidar      : If false, do not publish "imu -> lidar" TF
  // tf_time_offset         : Time offset for TF data (in seconds)
  //
  // --- Extension modules ---
  // extension_modules      : List of extension modules to load
  //
  // --- Topics ---
  // imu_topic              : IMU topic
  // points_topic           : Points topic
  // image_topic            : Image topic (required for only extension modules)
  "glim_ros": {
    // Backend modules
    "enable_local_mapping": true,
    "enable_global_mapping": true,
    // Points config
    "keep_raw_points": false,
    // IMU config
    "imu_time_offset": 0.0,
    "points_time_offset": 0.0,
    "acc_scale": 1.0,
    // TF config
    "imu_frame_id": "camera_imu_frame",
    "lidar_frame_id": "lidar2D_1_link",
    "base_frame_id": "base_link",
    "odom_frame_id": "odom",
    "map_frame_id": "map",
    "publish_imu2lidar": true,
    "tf_time_offset": 1e-6,
    // Extension modules
    "extension_modules": [
      "libmemory_monitor.so",
      "libstandard_viewer.so",
      "librviz_viewer.so",
      "libscan_context_loop_detector.so"
      // "libimu_validator.so"
    ],
    // Topics
    "imu_topic": "/camera/camera/imu",
    "points_topic": "/velodyne_points",
    "image_topic": "/camera/camera/color/image_raw"
  }
}
```

* tf between lidar and imu at `config_sensors.json`
```json
   "T_lidar_imu": [
      0.07,
      0.0,
      -0.06,
      0.0,
      0.0,
      0.0,
      1.0
    ],
```
Change value of k_correspondences: 10->20 in `config_preprocessing.json`

## velodyne-core
```` bash
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py 

````

## rs-core

```` bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=false enable_gyro:=true enable_accel:=true unite_imu_method:=2
````

Pointcloud with realsense for this case isn't really needed.










