import rclpy
from rclpy.node import Node
from rrt_cbf_planner.rrt_star import RRTStar
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.goal = None
        self.current_pose = None
        self.obstacles = []

        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 7.0)
        self.declare_parameter('y_min', 0.0)
        self.declare_parameter('y_max', 7.0)

        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(MarkerArray, '/obstacle_states', self._obs_cb, 10)

        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.start_marker_pub = self.create_publisher(Marker, '/start_marker', 10)

        self.create_timer(1.0, self._run_planner)
        self.get_logger().info('RRT node started')

    def _run_planner(self):
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odom...')
            return
        if self.goal is None:
            return

        self._publish_start_marker(self.current_pose)
        self._publish_obstacles(self.obstacles)

        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        planner = RRTStar(self.current_pose, self.goal, self.obstacles,
                          x_bounds=(x_min, x_max), y_bounds=(y_min, y_max))
        path = planner.plan()

        if not path:
            self.get_logger().warn('RRT*: no path found')
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Path published: {len(path)} waypoints')

    def _goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'New goal: {self.goal}')

    def _odom_cb(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _obs_cb(self, msg):
        self.obstacles = [
            (m.pose.position.x, m.pose.position.y, m.scale.x / 2.0)
            for m in msg.markers
        ]

    def _publish_start_marker(self, start):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'start'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(start[0])
        m.pose.position.y = float(start[1])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.g = 1.0
        m.color.a = 1.0
        self.start_marker_pub.publish(m)

    def _publish_obstacles(self, obstacles):
        msg = MarkerArray()
        for i, (ox, oy, r) in enumerate(obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(ox)
            m.pose.position.y = float(oy)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * float(r)
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.a = 0.8
            msg.markers.append(m)
        self.obstacle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
