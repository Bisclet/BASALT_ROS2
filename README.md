## Basalt ROS2 Wrapper

This is a ROS2 wrapper for the Basalt VIO pipeline. Its only purpose is to extend the Basalt code base with ROS2 compatibility. For more information about the original project and standalone applications, see: https://github.com/VladyslavUsenko/basalt-mirror

## Requisites

- Ubuntu 22+
- ROS2 Humble

## Usage

Build the package
```bash
    cd /your/workspace/src
    git clone --recursive https://github.com/Bisclet/BASALT_ROS2.git
    cd BASALT_ROS2
    ./scripts/install_deps.sh
    cd /your/workspace
    colcon build
    source install/setup.bash
```

## How to Launch

This launch file starts the Basalt VIO ROS2 node and exposes several launch arguments for configuring input topics and file paths.

### Running With Custom Topics or Paths

You can override launch arguments at runtime:
```bash
    ros2 launch basalt basalt_vio.launch.py \
        left_image_topic:=<left_cam_topic> \
        right_image_topic:=<right_cam_topic> \
        imu_topic:=<imu_topic> \
        odometry_topic:=<output_odom_topic> \
        cam_calib:=</path/to/cam_calib.json> \
        config_path:=</path/to/config.json>
```
### Example for TUM-VI-512
Assuming you are in the root of your workspace, where your build and install folders are.
```bash
    ros2 launch basalt basalt_vio.launch.py \
        left_image_topic:=/cam0/image_raw \
        right_image_topic:=/cam1/image_raw \
        imu_topic:=/imu0 \
        odometry_topic:=/basalt/odom \
        cam_calib:=$(realpath ./install/basalt/share/basalt/config/tumvi_512_ds_calib.json) \
        config_path:=$(realpath ./install/basalt/share/basalt/config/tumvi_512_config.json)
```
Then run your TUM-VI-512 rosbag.

### Launch Arguments

left_image_topic  
Topic for the left camera image.

right_image_topic  
Topic for the right camera image.

imu_topic  
Topic for IMU messages.

odometry_topic  
Topic where Basalt publishes estimated odometry.

cam_calib  
Path to the camera calibration JSON file.

config_path  
Path to the Basalt VIO configuration JSON file.


### WARNING
This wrapper is highly experimental, it does not guarantee stability at high frame rates and does not provide any calibration tools. For more information visit https://github.com/VladyslavUsenko/basalt-mirror. 