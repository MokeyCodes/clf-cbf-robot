"""
Real-robot launch file.

Differences from cbf_rrt.launch.py (simulation):
  - No Gazebo
  - obstacle_sim_node replaced by optitrack_obstacle_node
  - No fake map->odom static transform — run your localization stack separately
    (e.g. `ros2 launch slam_toolbox online_async_launch.py` or AMCL)

Before launching, start vrpn_client_ros2 pointed at your Motive PC:
    ros2 run vrpn_client_ros vrpn_client_node \
        --ros-args -p server:=<MOTIVE_IP> -p port:=3883

Then launch this file:
    ros2 launch rrt_cbf_planner cbf_rrt_real.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Edit these to match your Motive rigid body names and physical radii.
    obstacle_bodies = ['obs_1', 'obs_2', 'obs_3']
    obstacle_radii = [0.15, 0.15, 0.15]

    return LaunchDescription([
        Node(
            package='rrt_cbf_planner',
            executable='optitrack_obstacle_node',
            name='optitrack_obstacle_node',
            output='screen',
            parameters=[{
                'obstacle_bodies': obstacle_bodies,
                'obstacle_radii': obstacle_radii,
                'vrpn_prefix': '/vrpn_client_node',
                'publish_rate': 20.0,
                'max_stale_sec': 0.5,
            }],
        ),
        Node(
            package='rrt_cbf_planner',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
        ),
        Node(
            package='rrt_cbf_planner',
            executable='cbf_controller_node',
            name='cbf_controller_node',
            output='screen',
        ),
    ])
