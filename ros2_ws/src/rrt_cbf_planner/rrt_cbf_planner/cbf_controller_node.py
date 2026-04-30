import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from rrt_cbf_planner.cbf_controller import CBFController


class CBFControllerNode(Node):
    def __init__(self):
        super().__init__('cbf_controller_node')

        self.controller = CBFController()
        self._prev_obs = {}   # marker id -> (x, y, timestamp)
        self._vel_obs = {}    # marker id -> (vx, vy) low-pass filtered
        self._vel_alpha = 0.3  # low-pass weight for new measurement
        self.dt = 0.05

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/rrt_path', self._path_cb, 10)
        self.create_subscription(MarkerArray, '/obstacle_states', self._obs_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('CBF controller node started')

    def _odom_cb(self, msg):
        self.controller.x = msg.pose.pose.position.x
        self.controller.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.controller.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _path_cb(self, msg):
        self.controller.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.controller.wp_index = 0

    def _obs_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        new_obs = []
        for m in msg.markers:
            ox, oy, r = m.pose.position.x, m.pose.position.y, m.scale.x / 2.0
            mid = m.id
            vx, vy = self._vel_obs.get(mid, (0.0, 0.0))
            if mid in self._prev_obs:
                px, py, pt = self._prev_obs[mid]
                dt = now - pt
                if dt > 0.01:
                    raw_vx = (ox - px) / dt
                    raw_vy = (oy - py) / dt
                    a = self._vel_alpha
                    vx = a * raw_vx + (1 - a) * vx
                    vy = a * raw_vy + (1 - a) * vy
            self._prev_obs[mid] = (ox, oy, now)
            self._vel_obs[mid] = (vx, vy)
            new_obs.append([ox, oy, r, vx, vy])
        self.controller.obstacles = new_obs

    def _control_loop(self):
        c = self.controller
        if not c.waypoints:
            self._stop()
            return

        gx, gy = c.waypoints[-1]
        if math.hypot(c.x - gx, c.y - gy) < c.wp_threshold:
            self.get_logger().info('Goal reached')
            c.waypoints = []
            self._stop()
            return

        v, w = c.solveCBFQP()
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CBFControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
