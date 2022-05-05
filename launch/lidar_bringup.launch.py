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

    correction_dir = get_package_share_directory('scan_correction')
    correction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(correction_dir, 'launch', 'scan_correction.launch.py')),
    )

    return LaunchDescription([
        ydlidar_launch,
        correction_launch,
    ])
