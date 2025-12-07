import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ublox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ublox_gps"), "launch"), "/ublox_gps_node_zedf9p-launch.py"]
        )
    )

    ublox_w_ns = GroupAction(
     actions=[
         PushRosNamespace('/gps'),
         ublox,
      ]
    )

    return LaunchDescription(
        [
            ublox
        ]
    )
