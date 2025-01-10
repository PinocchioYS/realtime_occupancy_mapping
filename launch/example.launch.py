# !/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
                package='realtime_occupancy_mapping',
                executable='realtime_occupancy_mapping',
                name='realtime_occupancy_mapping',
                parameters=[
                    {'resolution': 0.1},
                    {'max_range': 5.0},
                    {'hit_prob': 0.8},
                    {'miss_prob': 0.2},
                    {'fixed_frame_id': "map"},
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('/realtime_occupancy_mapping/pointcloud_in', '/velodyne_points')
                ]
            )
    
    node2 = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [os.path.join(get_package_share_directory('realtime_occupancy_mapping'), 'rviz', 'example.rviz')]]
            )
    
    ld.add_action(node1)
    ld.add_action(node2)

    return ld