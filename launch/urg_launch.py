from launch import LaunchDescription
from launch_ros.actions import Node
from math import pi
# from rosbag2.launch_actions import Record
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Ref
# https://github.com/foxglove/ros-foxglove-bridge

world2laser = "--x {x} --y {y} --z {z} ".format(x=0.0, y=0.0, z=0.2)
world2laser += "--yaw {yaw} --pitch {pitch} --roll {roll} ".format(yaw=0*pi/180, pitch=0*pi/180, roll=0*pi/180)
world2laser += "--child-frame-id {child} --frame-id {parent}".format(child="lidar", parent="world")
world2laser = world2laser.split(' ')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[
                {"port": 8765},
                {"address": "0.0.0.0"},
                {"tls": False},
                {"topic_whitelist": [".*"]},
                {"send_buffer_limit": 10000000},
                {"use_sim_time": False},
                {"num_threads": 0}
            ]
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = world2laser
        ),
        Node(
            package='rtf_lidar',
            executable='rtf_urg',
            name='lidar',
            # parameters=[
            #     {'turtlename': 'turtle1'}
            # ]
        ),
        # Record(
        #     name='my_recorder',
        #     parameters=[{
        #         'topics': LaunchConfiguration('my_topic_list')
        #     }]
        # ),
    ])