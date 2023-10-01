from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    gesture_path = get_package_share_path('dexhand_gesture_controller')

    # Including the launch file from the dependent package
    gesture_sim_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         gesture_path, 'launch'),
         '/simulation.launch.py'])
      )
    
    
    return LaunchDescription([
        gesture_sim_launch,
        Node(
            package='dexhand_llm_control',
            executable='llm_control',
            name='llm_control',
            output='screen'),
        
    ])