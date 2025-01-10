Real-time occupancy grid mapping using SuperRay library on ROS
================================================================================

This ROS package provides an example for real-time occupancy mapping using [SuperRay library](https://github.com/PinocchioYS/SuperRay).
The example node subscribes a pointcloud(sensor_msgs::PointCloud2) and updates the occupancy gridmap in real-time.
In this package, we provide one example using a simple Grid3D, but it can be applied to other types of the occupancy maps in SuperRay library.

BUILD
-----
This package requires to install [SuperRay library](https://github.com/PinocchioYS/SuperRay).
For example of installing the library, run:

    git clone https://github.com/PinocchioYS/SuperRay.git
    mkdir SuperRay/build && cd SuperRay/build
    cmake ..
    sudo make install

You can build the visualization through the following commands:

    cd ~/catkin_ws/src
    git clone https://github.com/PinocchioYS/realtime_occupancy_mapping.git
    cd ~/catkin_ws && catkin_make
    
RUN EXAMPLES
------------
We provide one bag file (30sec, 90MB) for you to check the availability of this package.

    roslaunch realtime_occupancy_mapping example.launch

RViz will visualize the occupied cells of occupancy grid in real time.

NOTE
----
Please check the fixed frame id and the topic name of pointcloud carefully when using this package in your system.
Refer the example launch file to set the rosbag file and the parameters.
  
If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/realtime_occupancy_mapping/issues).
