import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleSimNode(Node):
    """
    Simulates dynamic and static obstacles, publishing their states to
    /obstacle_states.  In real deployment, replace this node with your
    optical tracker publisher — same topic, same MarkerArray format.

    MarkerArray encoding per obstacle:
        pose.position.{x,y}  — centre position
        scale.x / 2          — radius
    """

    def __init__(self):
        super().__init__('obstacle_sim_node')

        # [x, y, r, vx, vy]
        self.obstacles = [
            # --- static wall ---
            [3.5, 1.5, 0.5,  0.0,  0.0],
            [3.5, 2.5, 0.5,  0.0,  0.0],
            [2.5, 3.5, 0.5,  0.0,  0.0],
            [3.5, 4.5, 0.5,  0.0,  0.0],
            [3.5, 5.5, 0.5,  0.0,  0.0],
            [1.5, 3.0, 0.45, 0.0,  0.0],
            [5.5, 3.0, 0.45, 0.0,  0.0],
            # --- dynamic ---
            [1.0, 2.0, 0.3,  0.7,  0.0],
            [1.0, 4.0, 0.3, -0.7,  0.0],
            [5.0, 2.0, 0.3, -0.7,  0.0],
            [5.0, 4.0, 0.3,  0.7,  0.0],
            [1.2, 1.0, 0.25, 0.0,  0.6],
            [5.8, 5.0, 0.25, 0.0, -0.6],
        ]

        self.dt = 0.05
        self.x_bounds = (0.0, 7.0)
        self.y_bounds = (0.0, 7.0)

        self.pub = self.create_publisher(MarkerArray, '/obstacle_states', 10)
        self.create_timer(self.dt, self._update)
        self.get_logger().info('Obstacle sim node started')

    def _update(self):
        for obs in self.obstacles:
            obs[0] += self.dt * obs[3]
            obs[1] += self.dt * obs[4]

            if obs[0] < self.x_bounds[0] or obs[0] > self.x_bounds[1]:
                obs[3] *= -1
                obs[0] = max(self.x_bounds[0], min(self.x_bounds[1], obs[0]))
            if obs[1] < self.y_bounds[0] or obs[1] > self.y_bounds[1]:
                obs[4] *= -1
                obs[1] = max(self.y_bounds[0], min(self.y_bounds[1], obs[1]))

        self._publish()

    def _publish(self):
        msg = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(obs[0])
            m.pose.position.y = float(obs[1])
            m.pose.position.z = 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * float(obs[2])
            m.scale.y = 2.0 * float(obs[2])
            m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 0.5 if obs[3] != 0.0 or obs[4] != 0.0 else 0.0
            m.color.b = 0.0
            m.color.a = 0.8
            msg.markers.append(m)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
