#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


rviz_config_path = os.path.join(get_package_share_directory('amr_control'), 'config', 'rviz_walle_config.rviz')

print(rviz_config_path)
