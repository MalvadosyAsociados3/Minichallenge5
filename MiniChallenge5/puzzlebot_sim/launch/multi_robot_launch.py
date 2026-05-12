"""
MINI CHALLENGE 3 - LAUNCH MULTI-ROBOT

Lanza dos puzzlebots con namespaces /robot1 y /robot2. Cada robot tiene su
propio grupo de nodos: simulator, localisation, point_generator, control y
robot_state_publisher (Joint State publisher implicito en simulator).

Cada robot:
  - usa frames TF prefijados (robot1/base_footprint, robot1/odom, ...).
  - publica TF estatico world -> robotN/odom (desde el nodo localisation).
  - tiene pose inicial y waypoints distintos definidos en su YAML.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def robot_group(namespace, params_file, urdf_xml):
    """Crea el GroupAction con los 5 nodos del robot dentro del namespace."""
    return GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_xml,
                # frame_prefix prepende namespace/ a TODOS los frame_id de los
                # links del URDF, asi robot1/base_link, robot1/wheel_l_link
                'frame_prefix': f'{namespace}/',
            }],
            output='screen',
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
            package='puzzlebot_sim',
            executable='point_generator',
            name='point_generator',
            parameters=[params_file],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='control',
            name='control_node',
            parameters=[params_file],
            output='screen',
        ),
    ])


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'multi_robot.rviz')
    robot1_params_default = os.path.join(pkg, 'config', 'robot1_params.yaml')
    robot2_params_default = os.path.join(pkg, 'config', 'robot2_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot1_params_arg = DeclareLaunchArgument(
        'robot1_params',
        default_value=robot1_params_default,
        description='YAML con parametros del robot1.'
    )
    robot2_params_arg = DeclareLaunchArgument(
        'robot2_params',
        default_value=robot2_params_default,
        description='YAML con parametros del robot2.'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz (true/false).'
    )
    use_rqt_graph_arg = DeclareLaunchArgument(
        'use_rqt_graph',
        default_value='true',
        description='Lanzar rqt_graph (true/false).'
    )

    robot1_params = LaunchConfiguration('robot1_params')
    robot2_params = LaunchConfiguration('robot2_params')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    rqt_graph = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rqt_graph',
                executable='rqt_graph',
                name='rqt_graph',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_rqt_graph')),
                additional_env={
                    'GTK_PATH': '',
                    'GIO_MODULE_DIR': '',
                    'GTK_EXE_PREFIX': '',
                    'GTK_IM_MODULE_FILE': '',
                },
            ),
        ],
    )

    return LaunchDescription([
        robot1_params_arg,
        robot2_params_arg,
        use_rviz_arg,
        use_rqt_graph_arg,
        robot_group('robot1', robot1_params, robot_description),
        robot_group('robot2', robot2_params, robot_description),
        rviz,
        rqt_graph,
    ])
