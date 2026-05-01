"""
Drop-in replacement for obstacle_sim_node when using OptiTrack.

OptiTrack → Motive → vrpn_client_ros2 publishes each rigid body as:
    /vrpn_client_node/<BodyName>/pose   (geometry_msgs/PoseStamped)

This node subscribes to one topic per obstacle body, then republishes
all of them as a MarkerArray on /obstacle_states — the same topic the
CBF controller and RRT node already consume. No other nodes need changes.

ROS2 parameters (set in the launch file or a YAML):
    obstacle_bodies  (string list) — Motive rigid body names, e.g. ['obs_1', 'obs_2']
    obstacle_radii   (double list) — physical radius (m) for each body, same order
    vrpn_prefix      (string)      — topic namespace, default '/vrpn_client_node'
    publish_rate     (double)      — Hz for the MarkerArray output, default 20.0
    max_stale_sec    (double)      — drop a body from output if no msg for this long
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class OptitrackObstacleNode(Node):
    def __init__(self):
        super().__init__('optitrack_obstacle_node')

        self.declare_parameter('obstacle_bodies', ['obs_1', 'obs_2'])
        self.declare_parameter('obstacle_radii', [0.15, 0.15])
        self.declare_parameter('vrpn_prefix', '/vrpn_client_node')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_stale_sec', 0.5)

        bodies = self.get_parameter('obstacle_bodies').value
        radii = self.get_parameter('obstacle_radii').value
        prefix = self.get_parameter('vrpn_prefix').value
        rate = self.get_parameter('publish_rate').value
        self.max_stale = self.get_parameter('max_stale_sec').value

        if len(radii) != len(bodies):
            self.get_logger().error(
                f'obstacle_bodies ({len(bodies)}) and obstacle_radii ({len(radii)}) '
                'must have the same length — node will not publish'
            )
            return

        # body name → {x, y, radius, stamp}
        self._states: dict[str, dict] = {
            name: {'x': 0.0, 'y': 0.0, 'r': radii[i], 'stamp': None}
            for i, name in enumerate(bodies)
        }

        for name in bodies:
            topic = f'{prefix}/{name}/pose'
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self._pose_cb(msg, n),
                10,
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self.pub = self.create_publisher(MarkerArray, '/obstacle_states', 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info('OptiTrack obstacle node started')

    def _pose_cb(self, msg: PoseStamped, name: str):
        s = self._states[name]
        s['x'] = msg.pose.position.x
        s['y'] = msg.pose.position.y
        s['stamp'] = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()
        array = MarkerArray()

        for i, (name, s) in enumerate(self._states.items()):
            if s['stamp'] is None:
                continue
            age = (now - s['stamp']).nanoseconds * 1e-9
            if age > self.max_stale:
                self.get_logger().warn(f'Stale OptiTrack data for {name} ({age:.2f}s)', throttle_duration_sec=2.0)
                continue

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now.to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(s['x'])
            m.pose.position.y = float(s['y'])
            m.pose.position.z = 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * float(s['r'])
            m.scale.y = 2.0 * float(s['r'])
            m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            array.markers.append(m)

        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
