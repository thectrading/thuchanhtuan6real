#!/usr/bin/env python3
"""Read-only ROS audit: camera latency, recognized labels, robot state and RTF.

Publishes no gestures or velocities. Useful while a human operates the webcam.
This records metadata only, never camera frames or a replacement demo video.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(); node = rclpy.create_node('audit_live_gesture_pipeline')
    result = {'method': 'Read-only ROS audit; no fake inputs, no camera frames saved',
              'labels': {}, 'states': []}
    clock_samples, ages, inference = [], [], []
    def debug(msg):
        d = json.loads(msg.data)
        key = d['raw_label'] + ' => ' + d['gesture']
        result['labels'][key] = result['labels'].get(key, 0) + 1
        result['last_debug'] = d
        if d.get('frame_age_s') is not None: ages.append(d['frame_age_s'])
        if d.get('capture_to_result_ms') is not None: inference.append(d['capture_to_result_ms'])
    def state(msg):
        d = json.loads(msg.data)
        if not result['states'] or result['states'][-1]['reason'] != d['reason']:
            result['states'].append(d)
        result['last_state'] = d
    def odom(msg):
        p,q = msg.pose.pose.position, msg.pose.pose.orientation
        result['odom'] = {'x': p.x, 'y': p.y,
            'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            'linear': msg.twist.twist.linear.x, 'angular': msg.twist.twist.angular.z}
    def scan(msg):
        result['scan_min_m'] = min((v for v in msg.ranges if math.isfinite(v) and v>0), default=None)
    def clock(msg): clock_samples.append((time.monotonic(), msg.clock.sec+msg.clock.nanosec/1e9))
    subs = [node.create_subscription(String, '/gesture_debug', debug, 10),
        node.create_subscription(String, '/behavior_state', state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)]
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
        if len(clock_samples)>2:
            a,b = clock_samples[0],clock_samples[-1]
            result['real_time_factor'] = (b[1]-a[1])/(b[0]-a[0])
        result['gesture_publishers'] = [p.node_name for p in node.get_publishers_info_by_topic('/detected_gesture')]
        if ages: result['mean_frame_age_s'] = sum(ages)/len(ages)
        if inference: result['mean_capture_to_result_ms'] = sum(inference)/len(inference)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2))
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
