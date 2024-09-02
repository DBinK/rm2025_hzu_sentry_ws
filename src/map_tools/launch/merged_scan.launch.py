from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():


    node_name = 'merger_node'

    merged_scan = LifecycleNode(package='map_tools',
                                executable='merger_node',
                                name='merger_node',
                                namespace='/',
                                output='screen',
                                emulate_tty=True)

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser_merged',
                    arguments=['0.115', '0', '0', '0', '0', '0', '1', 'base_link', 'merged_laser'],
                    )

    return LaunchDescription([
        merged_scan,
        tf2_node,
    ])