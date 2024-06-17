#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file_dir = os.path.join(get_package_share_directory("diff_wheeled_smooth_ctrl"), 'params', 'smooth_ctrl_params.yaml')

    return LaunchDescription([
      Node(
          package="diff_wheeled_smooth_ctrl",
          executable="smooth_ctrl",
          respawn=True,
          output='screen',
          parameters=[params_file_dir]
      )
    ])
