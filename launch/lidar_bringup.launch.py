import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')),
    )

    mixer_dir = get_package_share_directory('laserscan_mixer')
    mixer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mixer_dir, 'launch', 'laserscan_mixer_launch.py')),
    )

    return LaunchDescription([
        ydlidar_launch,
        mixer_launch,
    ])
