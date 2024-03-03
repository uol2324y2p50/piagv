import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   apriltag_pipeline = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('piagv'), 'launch'),
         '/apriltag_pipeline.launch.py'])
      )
   map_server = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('piagv'), 'launch'),
         '/map_server.launch.py'])
      )
   vehicle_interface = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('piagv'), 'launch'),
         '/vehicle_interface.launch.py'])
      )
   localization = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('piagv'), 'launch'),
         '/localization.launch.py'])
      )
   planning = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('piagv'), 'launch'),
         '/planning.launch.py'])
      )

   return LaunchDescription([
      apriltag_pipeline,
      map_server,
      vehicle_interface,
      localization,
      planning,
   ])
