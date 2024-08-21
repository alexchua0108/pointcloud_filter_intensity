# PointCloud Intensity Filter

This package is a ROS driver for filtering point clouds based on intensity values. It reads in PointCloud2 messages, filters out points below a specified intensity threshold, and publishes the filtered point cloud while preserving all original fields.

## System Requirements

This package is tested under the following environment setup:

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Getting Started

1. **Create catkin workspace and clone the repository as a package:**

    ```shell
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/alexchua0108/pointcloud_filter_intensity.git pointcloud_intensity_filter
    ```

2. **Build the package:**

    ```shell
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. **Launch the node with the default intensity threshold:**

    ```shell
    roslaunch pointcloud_intensity_filter pointcloud_intensity_filter.launch
    ```

4. **Launch the node with custom intensity threshold and topics:**

    ```shell
    roslaunch pointcloud_intensity_filter pointcloud_intensity_filter.launch intensity_threshold:=100 input_topic:=/ouster/points output_topic:=/ouster/filtered_points
    ```

## ROS Interfaces

### Parameters

* **`intensity_threshold`**: The minimum intensity value to retain points in the filtered point cloud. Points with intensity values below this threshold will be removed. The default value is `0.5`.

* **`input_topic`**: The ROS topic from which the node subscribes to PointCloud2 messages. The default value is `/ouster/points`.

* **`output_topic`**: The ROS topic to which the node publishes the filtered PointCloud2 messages. The default value is `/ouster/filtered_points`.

### Published Topics

* **`/ouster/filtered_points`** (`sensor_msgs/PointCloud2`): The filtered point cloud, preserving the original fields such as `x`, `y`, `z`, `intensity`, `t`, `reflectivity`, `ring`, `ambient`, and `range`.

### Subscribed Topics

* **`/ouster/points`** (`sensor_msgs/PointCloud2`): The input point cloud to be filtered. This can be customized using the `input_topic` parameter.


## Code Structure

```plaintext
.
├── CMakeLists.txt
├── include
│   └── pointcloud_intensity_filter
│       └── pointcloud_intensity_filter_node.h
├── launch
│   └── pointcloud_intensity_filter.launch
├── package.xml
└── src
    └── pointcloud_intensity_filter_node.cpp

```