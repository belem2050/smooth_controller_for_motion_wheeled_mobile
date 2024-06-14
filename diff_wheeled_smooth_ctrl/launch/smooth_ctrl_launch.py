# #!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  
  dir = get_package_share_directory("diff_wheeled_smooth_ctrl")
  params_file_dir = os.path.join(dir, 'params', 'smooth_ctrl_params.yaml')

  return LaunchDescription(
    [ 
      Node(
        package     = "diff_wheeled_smooth_ctrl",
        executable  = "smooth_ctrl",
        respawn     = True,
        output      = 'screen',
        parameters  = [LaunchConfiguration('params_file', default = params_file_dir)]
      )
    ]
    )
