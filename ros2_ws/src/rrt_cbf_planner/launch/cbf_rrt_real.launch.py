import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Edit these to match your Motive rigid body names and physical radii
OBSTACLE_BODIES = ['obs_1', 'obs_2', 'obs_3']
OBSTACLE_RADII  = [0.15, 0.15, 0.15]

# Environment bounds (metres) — match your real room size
X_MIN, X_MAX = 0.0, 7.0
Y_MIN, Y_MAX = 0.0, 7.0


def generate_launch_description():
    launch_rviz  = LaunchConfiguration('launch_rviz', default='false')
    rviz_default = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'rviz', 'ethan_rrt.rviz'
    )
    rviz_config  = LaunchConfiguration('rviz_config', default=rviz_default)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set true to launch RViz automatically',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_default,
            description='Full path to RViz config file',
        ),
        slam,
        Node(
            package='rrt_cbf_planner',
            executable='optitrack_obstacle_node',
            name='optitrack_obstacle_node',
            output='screen',
            parameters=[{
                'obstacle_bodies': OBSTACLE_BODIES,
                'obstacle_radii':  OBSTACLE_RADII,
                'vrpn_prefix':     '/vrpn_client_node',
                'publish_rate':    20.0,
                'max_stale_sec':   0.5,
            }],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
            parameters=[{
                'x_min': X_MIN, 'x_max': X_MAX,
                'y_min': Y_MIN, 'y_max': Y_MAX,
            }],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='cbf_controller_node',
            name='cbf_controller_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
