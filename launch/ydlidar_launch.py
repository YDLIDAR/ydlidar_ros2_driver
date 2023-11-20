#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    node_name = 'gole_lidar_node'

    backright_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='gole_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                            {"port": f"/dev/ttyLIDARBR", "frame_id": f"lidar_RR", "ignore_array": ""},
                                            {"baudrate": 512000, "lidar_type": 1, "device_type": 0, "sample_rate": 6, "abnormal_check_count": 4,
                                            "intensity_bit": 8},
                                            {"resolution_fixed": False, "auto_reconnect": True, "reversion": True, "inverted": True,
                                            "isSingleChannel": False, "intensity": False, "support_motor_dtr": False,
                                            "invalid_range_is_inf": False, "point_cloud_preservative": False},
                                            {"angle_min": -126.0, "angle_max": 126.0, "range_min": 0.1, "range_max": 15.0, "frequency": 12.0},
                                        ],
                                namespace='/backright',
                                )
    
    backleft_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='gole_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                            {"port": f"/dev/ttyLIDARBL", "frame_id": f"lidar_RL", "ignore_array": ""},
                                            {"baudrate": 512000, "lidar_type": 1, "device_type": 0, "sample_rate": 6, "abnormal_check_count": 4,
                                            "intensity_bit": 8},
                                            {"resolution_fixed": False, "auto_reconnect": True, "reversion": True, "inverted": True,
                                            "isSingleChannel": False, "intensity": False, "support_motor_dtr": False,
                                            "invalid_range_is_inf": False, "point_cloud_preservative": False},
                                            {"angle_min": -126.0, "angle_max": 126.0, "range_min": 0.1, "range_max": 15.0, "frequency": 12.0},
                                        ],
                                namespace='/backleft',
                                )
    
    frontleft_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='gole_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                            {"port": f"/dev/ttyLIDARFL", "frame_id": f"lidar_FL", "ignore_array": ""},
                                            {"baudrate": 512000, "lidar_type": 1, "device_type": 0, "sample_rate": 6, "abnormal_check_count": 4,
                                            "intensity_bit": 8},
                                            {"resolution_fixed": False, "auto_reconnect": True, "reversion": True, "inverted": True,
                                            "isSingleChannel": False, "intensity": False, "support_motor_dtr": False,
                                            "invalid_range_is_inf": False, "point_cloud_preservative": False},
                                            {"angle_min": -126.0, "angle_max": 126.0, "range_min": 0.1, "range_max": 15.0, "frequency": 12.0},
                                        ],
                                namespace='/frontleft',
                                )
    
    frontright_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='gole_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                            {"port": f"/dev/ttyLIDARFR", "frame_id": f"lidar_FR", "ignore_array": ""},
                                            {"baudrate": 512000, "lidar_type": 1, "device_type": 0, "sample_rate": 6, "abnormal_check_count": 4,
                                            "intensity_bit": 8},
                                            {"resolution_fixed": False, "auto_reconnect": True, "reversion": True, "inverted": True,
                                            "isSingleChannel": False, "intensity": False, "support_motor_dtr": False,
                                            "invalid_range_is_inf": False, "point_cloud_preservative": False},
                                            {"angle_min": -126.0, "angle_max": 126.0, "range_min": 0.1, "range_max": 15.0, "frequency": 12.0},
                                        ],
                                namespace='/frontright',
                                )
    
    return LaunchDescription([
        frontright_node,
        frontleft_node,
        backright_node,
        backleft_node,
    ])