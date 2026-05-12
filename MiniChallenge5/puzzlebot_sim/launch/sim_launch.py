"""
PARTE 1 - Launch para solo la SIMULACION.

Arranca:
  - simulator (publish_tf=true para que RViz muestre la pose del robot)
  - robot_state_publisher (URDF)
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
            parameters=[{
                'wheel_radius': 0.05,
                'wheel_base': 0.19,
                'update_rate': 50.0,
                'publish_tf': True,   # Solo para Parte 1: el sim publica la TF
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
        ),
    ])
