import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
BEAMS = 360
PEER_RINGS = (0.0, 0.04, 0.08, 0.12)
PEER_Z = 0.1


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def footprint(x, y):
    pts = [(x, y, PEER_Z)]
    for r in PEER_RINGS[1:]:
        n = max(8, int(2.0 * math.pi * r / 0.03))
        pts += [(x + r * math.cos(2.0 * math.pi * k / n), y + r * math.sin(2.0 * math.pi * k / n), PEER_Z)
                for k in range(n)]
    return pts


def strip(frame, prefix):
    frame = frame.lstrip('/')
    while frame.startswith(prefix):
        frame = frame[len(prefix):]
    return frame


class Robot:
    def __init__(self, node, name, x, y, yaw):
        self.node = node
        self.name = name
        self.prefix = name + '/'
        self.tf_pub = node.create_publisher(TFMessage, f'/{name}/tf', 100)
        self.static_pub = node.create_publisher(TFMessage, f'/{name}/tf_static', STATIC_QOS)
        self.scan_pub = node.create_publisher(LaserScan, f'/{name}/scan_fixed', qos_profile_sensor_data)
        self.peers_pub = node.create_publisher(PointCloud2, f'/{name}/peers', 5)
        node.create_subscription(LaserScan, f'/{name}/scan', self.scan_cb, qos_profile_sensor_data)
        node.create_subscription(Odometry, f'/{name}/odom', self.odom_cb, 50)
        node.create_subscription(TFMessage, f'/{name}/tf', self.ns_tf_cb, 100)
        self.start = (x, y, yaw)
        self.map_godom = (0.0, 0.0, 0.0)
        self.odom = None
        self.statics = {}
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'global_odom'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.add_statics([t])

    def owns(self, t):
        return t.child_frame_id.lstrip('/').startswith(self.prefix)

    def relabel(self, t):
        t.header.frame_id = strip(t.header.frame_id, self.prefix)
        t.child_frame_id = strip(t.child_frame_id, self.prefix)
        return t

    def add_statics(self, transforms):
        for t in transforms:
            self.statics[t.child_frame_id] = t
        self.static_pub.publish(TFMessage(transforms=list(self.statics.values())))

    def ns_tf_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'global_odom':
                tr = t.transform
                self.map_godom = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def pose(self):
        if self.odom is None:
            return None
        return compose(compose(self.map_godom, self.start), self.odom)

    def odom_cb(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        t = TransformStamped()
        t.header = m.header
        t.header.frame_id = strip(m.header.frame_id, self.prefix)
        t.child_frame_id = strip(m.child_frame_id, self.prefix)
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def scan_cb(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        k = r.size
        if k == 0 or m.angle_increment <= 0.0:
            return
        inc = 2.0 * math.pi / BEAMS
        rel = (np.arange(BEAMS) * inc - m.angle_min) % (2.0 * math.pi)
        idx = np.rint(rel / m.angle_increment).astype(np.int64)
        gap = np.abs(rel - idx * m.angle_increment) > m.angle_increment
        idx %= k
        out = r[idx]
        out[gap] = np.inf
        m.intensities = (np.asarray(m.intensities, dtype=np.float32)[idx].tolist()
                         if len(m.intensities) == k else [])
        m.ranges = out.tolist()
        m.angle_min = 0.0
        m.angle_increment = inc
        m.angle_max = inc * (BEAMS - 1)
        m.time_increment = m.scan_time / BEAMS if m.scan_time > 0.0 else 0.0
        m.header.frame_id = strip(m.header.frame_id, self.prefix)
        self.scan_pub.publish(m)


class Relay(Node):
    def __init__(self, specs):
        super().__init__('multi_robot_relay')
        self.robots = [Robot(self, *s) for s in specs]
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 200)
        self.create_subscription(TFMessage, '/tf_static', self.static_cb, STATIC_QOS)
        self.fleet_pub = self.create_publisher(PoseArray, '/fleet/poses', 5)
        self.create_timer(0.1, self.publish_peers)

    def publish_peers(self):
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        poses = {r.name: r.pose() for r in self.robots}
        fleet = PoseArray(header=header)
        for r in self.robots:
            p = poses[r.name]
            q = Pose()
            if p is not None:
                q.position.x, q.position.y = p[0], p[1]
                q.orientation.z, q.orientation.w = math.sin(p[2] / 2.0), math.cos(p[2] / 2.0)
            fleet.poses.append(q)
            pts = []
            for name, other in poses.items():
                if name != r.name and other is not None:
                    pts += footprint(other[0], other[1])
            r.peers_pub.publish(create_cloud_xyz32(header, pts))
        self.fleet_pub.publish(fleet)

    def route(self, msg):
        out = {r.name: [] for r in self.robots}
        for t in msg.transforms:
            for r in self.robots:
                if r.owns(t):
                    out[r.name].append(r.relabel(t))
                    break
        return out

    def tf_cb(self, msg):
        routed = self.route(msg)
        for r in self.robots:
            ts = [t for t in routed[r.name]
                  if not (t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint')]
            if ts:
                r.tf_pub.publish(TFMessage(transforms=ts))

    def static_cb(self, msg):
        routed = self.route(msg)
        for r in self.robots:
            if routed[r.name]:
                r.add_statics(routed[r.name])


def parse(arg):
    name, x, y, yaw = arg.split(':')
    return name, float(x), float(y), math.radians(float(yaw))


def main():
    rclpy.init()
    specs = [parse(a) for a in sys.argv[1:] if ':' in a and not a.startswith('-')]
    rclpy.spin(Relay(specs))


if __name__ == '__main__':
    main()
