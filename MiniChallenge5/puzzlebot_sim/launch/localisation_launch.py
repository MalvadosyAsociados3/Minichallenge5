"""
PARTE 2 - Launch con SIMULACION + LOCALISATION.

Arranca:
  - simulator (publish_tf=false; ahora el TF lo publica localisation)
  - localisation (publica /odom y TF odom->base_footprint)
  - robot_state_publisher (URDF con STL del puzzlebot)
  - RViz2


"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')
    params_file = os.path.join(pkg, 'config', 'puzzlebot_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='puzzlebot_sim',
            executable='simulator',
            name='puzzlebot_sim',
            parameters=[params_file],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            parameters=[params_file],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
        ),
    ])
