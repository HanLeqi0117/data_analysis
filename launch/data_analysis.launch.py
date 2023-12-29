import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    data_analysis_node = Node(
        package="data_analysis",
        executable="data_analysis_node",
        name="data_analysis_node",
        parameters=[
            {"topic_list" : ["ublox/fix", "gps/filtered", "nmea", "odometry/filtered/global", "odometry/filtered/local", "odometry/gps"]},
            {"file_path" : os.path.join(
                os.environ["HOME"],
                "data_analysis", "doc"
            )}
        ]
    )
    ld.add_action(data_analysis_node)
    
    return ld
