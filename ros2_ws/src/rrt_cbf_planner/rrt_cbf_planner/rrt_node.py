# rrt_node.py
import rclpy
from rclpy.node import Node
from rrt_cbf_planner.rrt_star import RRTStar
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.get_logger().info("RRT node started")
        self.goal = None
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacles', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.current_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.start_marker_pub = self.create_publisher(Marker, '/start_marker', 10)

        self.timer = self.create_timer(2.0, self.run_planner)

    def run_planner(self):


        if self.current_pose is None:
            self.get_logger().warn("Waiting for odom...")
            return

        start = self.current_pose
        self.publish_start_marker(start)
        goal = (5.0, 5.0)
        obstacles = [
            (2.5, 2.5, 0.5),
            (3.5, 3.0, 0.5),
            (1.0, 3.0, 1.0)
        ]

        self.publish_obstacles(obstacles)

        planner = RRTStar(start, goal, obstacles)
        path = planner.plan()

        if not path:
            self.get_logger().warn("No path found")
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for (x, y) in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses")

    def goal_callback(self,msg):
        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        self.get_logger().info(f"New goal: {self.goal}")

    def publish_start_marker(self, start):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "start"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(start[0])
        marker.pose.position.y = float(start[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.start_marker_pub.publish(marker)
    
    def publish_obstacles(self, obstacles):
        marker_array = MarkerArray()

        for i, (ox, oy, r) in enumerate(obstacles):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = "obstacles"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(ox)
            m.pose.position.y = float(oy)
            m.pose.position.z = 0.0

            m.pose.orientation.w = 0.0

            m.scale.x = 2.0 * float(r)
            m.scale.y = 2.0 * float(r)
            m.scale.z = 0.1

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.8

            marker_array.markers.append(m)

        self.obstacle_pub.publish(marker_array)

    def odom_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()