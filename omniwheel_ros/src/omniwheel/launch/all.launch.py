import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   workstation_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('omniwheel'), 'launch'),
         '/workstation_launch.py'])
      )
   visualization_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('omniwheel_visualization'), 'launch'),
         '/display.launch.py'])
      )
   slam_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('lidarslam'), 'launch'),
         '/lidarslam.launch.py'])
  )

   return LaunchDescription([
      workstation_node,
      visualization_node,
      slam_node,
   ])