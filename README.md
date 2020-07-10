Real-time occupancy grid mapping using SuperRay library on ROS
================================================================================

This ROS package provides an example for real-time occupancy mapping using [SuperRay library](https://github.com/PinocchioYS/SuperRay).
The example node subscribes a pointcloud(sensor_msgs::PointCloud2) and updates the occupancy gridmap in real-time.
In this package, we provide one example using a simple Grid3D, but it can be applied to other types of the occupancy maps in SuperRay library.

BUILD
-----
This package requires to install [SuperRay library](https://github.com/PinocchioYS/SuperRay).
For example of installing the library, run:

    cd ~/Downloads && git clone https://github.com/PinocchioYS/SuperRay.git
    mkdir SuperRay/build && cd SuperRay/build
    cmake ..
    sudo make install

You can build the visualization through the following commands:

    cd ~/catkin_ws/src
    git clone https://github.com/PinocchioYS/realtime_occupancy_mapping.git
    cd ~/catkin_ws && catkin_make
    
RUN EXAMPLES
------------
We provide one bag file for you to check the availability of this package.

    roslaunch realtime_occupancy_mapping example.launch
    roscd realtime_occupancy_mapping && rosbag play bag/example.bag

RViz that we provides will visualize the occupied cells of occupancy grid in real time.
The visualization option can be found in the source code, which is written as "#define VIS_OCCUPIED_CELLS".

NOTE
----
Please check the fixed frame id and the topic name of pointcloud carefully when using this package in your system.
You can find how to set these values in the example launch file.
  
If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/realtime_occupancy_mapping/issues).
