"""
Drop-in replacement for obstacle_sim_node when using OptiTrack via NatNet.

The NatNet ROS2 client publishes each Motive rigid body as a PoseStamped on
a per-body topic (e.g. /box/pose_stamped, /helmet/pose_stamped). This node
subscribes to one topic per obstacle, then republishes all of them as a
MarkerArray on /obstacle_states — the same topic the CBF controller and RRT
node consume. No other nodes need changes.

ROS2 parameters:
    obstacle_topics  (string list) — full PoseStamped topic per obstacle,
                                     e.g. ['/box/pose_stamped', '/helmet/pose_stamped']
    obstacle_radii   (double list) — radius (m) for each obstacle, same order
    world_frame      (string)      — frame_id to stamp on output markers
    publish_rate     (double)      — Hz for the MarkerArray output
    max_stale_sec    (double)      — drop an obstacle from output if no msg for this long
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class OptitrackObstacleNode(Node):
    def __init__(self):
        super().__init__('optitrack_obstacle_node')

        self.declare_parameter('obstacle_topics', ['/box/pose_stamped', '/helmet/pose_stamped'])
        self.declare_parameter('obstacle_radii', [0.20, 0.15])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_stale_sec', 0.5)

        topics = self.get_parameter('obstacle_topics').value
        radii = self.get_parameter('obstacle_radii').value
        self.world_frame = self.get_parameter('world_frame').value
        rate = self.get_parameter('publish_rate').value
        self.max_stale = self.get_parameter('max_stale_sec').value

        if len(radii) != len(topics):
            self.get_logger().error(
                f'obstacle_topics ({len(topics)}) and obstacle_radii ({len(radii)}) '
                'must have the same length — node will not publish'
            )
            return

        # topic name → {x, y, radius, stamp}
        self._states: dict[str, dict] = {
            topic: {'x': 0.0, 'y': 0.0, 'r': radii[i], 'stamp': None}
            for i, topic in enumerate(topics)
        }

        for topic in topics:
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, t=topic: self._pose_cb(msg, t),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self.pub = self.create_publisher(MarkerArray, '/obstacle_states', 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info('OptiTrack obstacle node started')

    def _pose_cb(self, msg: PoseStamped, topic: str):
        s = self._states[topic]
        s['x'] = msg.pose.position.x
        s['y'] = msg.pose.position.y
        s['stamp'] = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()
        array = MarkerArray()

        for i, (topic, s) in enumerate(self._states.items()):
            if s['stamp'] is None:
                continue
            age = (now - s['stamp']).nanoseconds * 1e-9
            if age > self.max_stale:
                self.get_logger().warn(f'Stale OptiTrack data for {topic} ({age:.2f}s)', throttle_duration_sec=2.0)
                continue

            m = Marker()
            m.header.frame_id = self.world_frame
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
