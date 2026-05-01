import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_gazebo = LaunchConfiguration('launch_gazebo', default='true')

    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py',
            )
        ),
        condition=IfCondition(launch_gazebo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='true',
            description='Set false to skip launching TurtleBot3 in Gazebo',
        ),
        tb3_gazebo,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='obstacle_sim_node',
            name='obstacle_sim_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='cbf_controller_node',
            name='cbf_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
