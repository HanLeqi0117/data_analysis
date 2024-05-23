import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from whill_navi2.modules.ros2_launch_utils import DataPath, RegisterEventHandler, OnProcessStart, ExecuteProcess, FindExecutable

def generate_launch_description():
    
    data_path = DataPath()
    
    data_play_rate = "3.0"
    file_path = os.path.join(data_path.base_path, "etc")
    
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="data_play_rate", default_value=data_play_rate))
    data_play_rate = LaunchConfiguration("data_play_rate")
    
    data_analysis_node = Node(
        package="data_analysis",
        executable="data_analysis_node",
        name="data_analysis_node",
        parameters=[
            {
                "topic_list" : [
                    "gnss/fix", "gnss/filtered", "ublox/nmea", 
                    "odometry/filtered/global", "odometry/filtered/local", 
                    "odometry/gnss"
                ],
                "file_path" : file_path
            },
        ]
    )
    ld.add_action(data_analysis_node)
    
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=data_analysis_node,
            on_start=[
                ExecuteProcess(cmd=[
                    FindExecutable(name="ros2"), "bag", "play",
                    "--rate", data_play_rate,
                    data_path.bag_file_path
                ])
            ]
        )
    ))
    
    return ld
