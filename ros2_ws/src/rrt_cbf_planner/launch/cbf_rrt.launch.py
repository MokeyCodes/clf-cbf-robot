import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Environment bounds (metres) — change these to match your Gazebo world size
X_MIN, X_MAX = 0.0, 7.0
Y_MIN, Y_MAX = 0.0, 7.0


def generate_launch_description():
    launch_gazebo = LaunchConfiguration('launch_gazebo', default='true')
    gazebo_gui    = LaunchConfiguration('gazebo_gui',    default='false')
    launch_rviz   = LaunchConfiguration('launch_rviz',   default='false')
    launch_slam   = LaunchConfiguration('launch_slam',   default='false')
    rviz_default  = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'rviz', 'ethan_rrt.rviz'
    )
    rviz_config   = LaunchConfiguration('rviz_config', default=rviz_default)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(launch_slam),
    )

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'empty_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(launch_gazebo),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gazebo_gui),
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(launch_gazebo),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        condition=IfCondition(launch_gazebo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='true',
            description='Set false to skip launching TurtleBot3 in Gazebo',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='false',
            description='Set true to show the Gazebo GUI window',
        ),
        DeclareLaunchArgument(
            'launch_slam',
            default_value='false',
            description='Set true to run slam_toolbox instead of static map->odom transform',
        ),
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
        gzserver,
        gzclient,
        robot_state_pub,
        spawn_tb3,
        slam,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}],
            condition=UnlessCondition(launch_slam),
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
            parameters=[{
                'use_sim_time': True,
                'x_min': X_MIN, 'x_max': X_MAX,
                'y_min': Y_MIN, 'y_max': Y_MAX,
            }],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='cbf_controller_node',
            name='cbf_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
            parameters=[{'use_sim_time': True}],
        ),
    ])
