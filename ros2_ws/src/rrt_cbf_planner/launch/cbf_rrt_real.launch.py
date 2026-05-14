"""
Real-world launch for the RRT*-CBF planner on TurtleBot4 'cascades' (tb_117).

Wiring assumptions:
  * Robot pose comes from slam_toolbox at /pose (PoseWithCovarianceStamped).
    Swap to OptiTrack later by changing POSE_TOPIC / POSE_TYPE below to
    '/tb_117/pose_stamped' / 'PoseStamped'.
  * Obstacle poses come from the NatNet ROS2 bridge at /<body>/pose_stamped.
    The NatNet client must be launched separately before this launch file.
  * Velocity commands publish to /cascades/cmd_vel.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- Edit these to match the physical setup --------------------------------
# Default: full NatNet (robot + obstacles via OptiTrack).
# Slam fallback: POSE_TOPIC='/pose', POSE_TYPE='PoseWithCovarianceStamped', WORLD_FRAME='map'.
POSE_TOPIC       = '/tb_117/pose_stamped'
POSE_TYPE        = 'PoseStamped'
CMD_VEL_TOPIC    = '/cascades/cmd_vel'
OBSTACLE_TOPICS  = ['/box/pose_stamped', '/helmet/pose_stamped', '/box2/pose_stamped', '/box3/pose_stamped', '/box4/pose_stamped', '/tb_116/pose_stamped']
OBSTACLE_RADII   = [0.30, 0.30, 0.30, 0.30, 0.30, 0.22]
WORLD_FRAME      = 'world'                       # verify via `ros2 topic echo /tb_117/pose_stamped --once`
X_MIN, X_MAX     = -3.0, 3.0
Y_MIN, Y_MAX     = -2.0, 2.0
NATNET_SERVER_IP = '192.168.0.108'               # Motive PC's Local Interface
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------


def generate_launch_description():
    launch_rviz  = LaunchConfiguration('launch_rviz', default='false')
    rviz_default = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'rviz', 'ethan_rrt.rviz'
    )
    rviz_config  = LaunchConfiguration('rviz_config', default=rviz_default)

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set true to launch RViz automatically (off by default — no display over SSH)',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_default,
            description='Full path to RViz config file',
        ),
        Node(
            package='natnet',
            executable='client',
            name='natnet_client',
            output='screen',
            parameters=[{'server_address': NATNET_SERVER_IP}],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='optitrack_obstacle_node',
            name='optitrack_obstacle_node',
            output='screen',
            parameters=[{
                'obstacle_topics': OBSTACLE_TOPICS,
                'obstacle_radii':  OBSTACLE_RADII,
                'world_frame':     WORLD_FRAME,
                'publish_rate':    30.0,
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
                'pose_topic':  POSE_TOPIC,
                'pose_type':   POSE_TYPE,
                'world_frame': WORLD_FRAME,
            }],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='cbf_controller_node',
            name='cbf_controller_node',
            output='screen',
            parameters=[{
                'pose_topic':    POSE_TOPIC,
                'pose_type':     POSE_TYPE,
                'cmd_vel_topic': CMD_VEL_TOPIC,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
