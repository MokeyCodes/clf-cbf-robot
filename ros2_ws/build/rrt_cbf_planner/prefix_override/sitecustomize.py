import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/turtlebot/ethan_RRT_ws/clf-cbf-robot/ros2_ws/install/rrt_cbf_planner'
