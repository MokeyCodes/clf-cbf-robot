import math
import cvxpy as cp
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray


class CBFControllerNode(Node):
    def __init__(self):
        super().__init__('cbf_controller_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.waypoints = []
        self.wp_index = 0
        self.wp_threshold = 0.2

        # obstacle list: [[x, y, r, vx, vy], ...]
        self.obstacles = []
        self._prev_obs = {}  # marker id -> (x, y, timestamp)

        # CBF parameters
        self.alpha = 3.0
        self.l = 0.1          # lookahead distance

        # Nominal controller gains
        self.Kp = 1.0
        self.Kw = 2.0

        # TurtleBot3 Burger velocity limits
        self.dt = 0.05
        self.v_min = 0.0
        self.v_max = 0.22
        self.w_min = -2.84
        self.w_max = 2.84

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/rrt_path', self._path_cb, 10)
        self.create_subscription(MarkerArray, '/obstacle_states', self._obs_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('CBF controller node started')

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _path_cb(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.wp_index = 0

    def _obs_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        new_obs = []
        for m in msg.markers:
            ox, oy, r = m.pose.position.x, m.pose.position.y, m.scale.x / 2.0
            mid = m.id
            vx, vy = 0.0, 0.0
            if mid in self._prev_obs:
                px, py, pt = self._prev_obs[mid]
                dt = now - pt
                if dt > 0.01:
                    vx = (ox - px) / dt
                    vy = (oy - py) / dt
            self._prev_obs[mid] = (ox, oy, now)
            new_obs.append([ox, oy, r, vx, vy])
        self.obstacles = new_obs

    def _control_loop(self):
        if not self.waypoints:
            self._stop()
            return

        # Advance waypoint index
        while self.wp_index < len(self.waypoints) - 1:
            wx, wy = self.waypoints[self.wp_index]
            if math.hypot(self.x - wx, self.y - wy) < self.wp_threshold:
                self.wp_index += 1
            else:
                break

        # Check if final goal is reached
        wx, wy = self.waypoints[self.wp_index]
        if (self.wp_index == len(self.waypoints) - 1
                and math.hypot(self.x - wx, self.y - wy) < self.wp_threshold):
            self.get_logger().info('Goal reached')
            self.waypoints = []
            self._stop()
            return

        v, w = self._solve_cbf_qp()
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _calc_nom(self):
        wx, wy = self.waypoints[self.wp_index]
        dx, dy = wx - self.x, wy - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.02:
            return 0.0, 0.0
        e_theta = self._wrap(math.atan2(dy, dx) - self.theta)
        return self.Kp * dist * math.cos(e_theta), self.Kw * e_theta

    def _solve_cbf_qp(self):
        v = cp.Variable()
        w = cp.Variable()
        v_nom, w_nom = self._calc_nom()

        constraints = [
            v >= self.v_min, v <= self.v_max,
            w >= self.w_min, w <= self.w_max,
        ]

        xa = self.x + self.l * math.cos(self.theta)
        ya = self.y + self.l * math.sin(self.theta)

        for obs in self.obstacles:
            xo, yo, r, vox, voy = obs
            h = (xa - xo)**2 + (ya - yo)**2 - r**2
            A = 2*(xa-xo)*math.cos(self.theta) + 2*(ya-yo)*math.sin(self.theta)
            B = -2*self.l*(xa-xo)*math.sin(self.theta) + 2*self.l*(ya-yo)*math.cos(self.theta)
            C = -2*(xa-xo)*vox - 2*(ya-yo)*voy
            constraints.append(A*v + B*w + C + self.alpha*h >= 0)

        prob = cp.Problem(cp.Minimize((v - v_nom)**2 + (w - w_nom)**2), constraints)
        prob.solve(solver='Clarabel')

        if v.value is None or w.value is None:
            self.get_logger().warn('CBF-QP infeasible, stopping')
            return 0.0, 0.0
        return float(v.value), float(w.value)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _wrap(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = CBFControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
