from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():

    stm32_broker_spawn = Node(package='krabi_python_serial_broker',
              executable='simple_stm32_broker.py',
              output='both',
              namespace="krabi_ns",
              )
    

    return LaunchDescription([stm32_broker_spawn])