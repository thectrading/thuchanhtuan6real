"""ROS adapter for the tested controller; the only HRI node publishing cmd_vel."""
import json
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector

from .control import Controller, Settings


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        defaults = Settings()
        for key, value in vars(defaults).items():
            self.declare_parameter(key, value)
        self.declare_parameter('event_log', '/tmp/tb4_gesture_behavior.json')
        self.engine = Controller(Settings(**{k: self.get_parameter(k).value for k in vars(defaults)}))
        self.log_path = Path(self.get_parameter('event_log').value).expanduser()
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = self.create_publisher(String, '/behavior_state', state_qos)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.input_subscriptions = [
            self.create_subscription(String, '/detected_gesture', self.gesture, command_qos),
            self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data),
            self.create_subscription(DockStatus, '/dock_status', self.dock, qos_profile_sensor_data),
            self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazards,
                                     qos_profile_sensor_data)]
        self.last_odom_stamp = self.last_scan_stamp = -1
        self.last_save = self.last_state = 0.0
        self.saved_events = 0
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Waiting for fresh sensors and gestures. No motion without selection + Thumb_Up.')

    def gesture(self, msg):
        was_running = self.engine.mode == 'RUNNING'
        self.engine.on_gesture(msg.data, time.monotonic())
        if msg.data in ('Open_Palm', 'Closed_Fist') or (was_running and self.engine.mode != 'RUNNING'):
            self.velocity.publish(Twist())  # Do not wait for the next 20 Hz tick.

    def odometry(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last_odom_stamp:
            return  # Repeated simulator timestamps do not refresh the watchdog.
        self.last_odom_stamp = stamp
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.engine.on_pose(p.x, p.y, yaw, time.monotonic())

    def scan(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last_scan_stamp or not msg.ranges:
            return
        front, rear, all_ranges = [], [], []
        for i, radius in enumerate(msg.ranges):
            if math.isnan(radius) or radius < msg.range_min or radius <= 0:
                continue
            radius = min(radius, msg.range_max) if math.isfinite(radius) else math.inf
            angle = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                               math.cos(msg.angle_min + i * msg.angle_increment))
            all_ranges.append(radius)
            if abs(angle) < math.pi / 3:
                front.append(radius)
            if abs(angle) > 2 * math.pi / 3:
                rear.append(radius)
        if len(front) < 3 or len(rear) < 3:
            return
        self.last_scan_stamp = stamp
        self.engine.on_scan(min(front), min(rear), min(all_ranges), time.monotonic())

    def dock(self, msg):
        self.engine.docked = msg.is_docked
        self.engine.last_dock = time.monotonic()

    def hazards(self, msg):
        self.engine.hazard = any(h.type in (1, 2, 3, 4, 5) for h in msg.detections)
        self.engine.last_hazard = time.monotonic()

    def save(self):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.log_path.with_suffix('.tmp')
        temp.write_text(json.dumps({'method': 'robot_behavior_node: real subscribed inputs',
                                   'events': self.engine.events,
                                   'state': self.engine.status(time.monotonic())}, indent=2))
        temp.replace(self.log_path)

    def tick(self):
        now = time.monotonic()
        v, w = self.engine.tick(now)
        msg = Twist(); msg.linear.x, msg.angular.z = v, w
        self.velocity.publish(msg)
        if now - self.last_state >= 0.10:
            self.state.publish(String(data=json.dumps(self.engine.status(now))))
            self.last_state = now
        if self.saved_events != len(self.engine.events):
            self.get_logger().info(self.engine.reason)
            self.save()
            self.saved_events = len(self.engine.events)

    def close(self):
        self.engine.stop(time.monotonic(), 'Behavior node shutting down', clear=True)
        self.velocity.publish(Twist())
        self.save()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        node = RobotBehaviorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
