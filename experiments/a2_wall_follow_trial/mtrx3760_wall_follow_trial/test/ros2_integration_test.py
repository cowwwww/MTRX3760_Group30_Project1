#!/usr/bin/env python3
"""Run the production ROS adapter against synthetic data on loopback only.

Each child uses a private namespace; no publisher uses the robot's /cmd_vel.
These are message/adapter checks, not physical-motion or Gazebo maze tests.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest

# Configure before importing ROS, and inherit these settings into every child.
os.environ['ROS_DOMAIN_ID'] = '201'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def scan_for(node, distance=0.30, yaw=0.0, corrupt=False):
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    scan.header.frame_id = 'base_scan'
    scan.angle_min = -math.pi
    scan.angle_increment = math.pi / 180.0
    scan.angle_max = scan.angle_min + 359 * scan.angle_increment
    scan.range_min = 0.02
    scan.range_max = 12.0
    scan.ranges = []
    for index in range(360):
        angle = scan.angle_min + index * scan.angle_increment + yaw
        sine = math.sin(angle)
        room = min(5.0 / max(abs(math.cos(angle)), 1e-9), 5.0 / max(abs(sine), 1e-9))
        value = min(room, distance / -sine) if sine < -1e-9 else room
        scan.ranges.append(math.nan if corrupt else value if value <= scan.range_max else math.inf)
    return scan


class AdapterTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.namespace = '/group30_offline/' + self._testMethodName
        self.node = rclpy.create_node('validator', namespace=self.namespace)
        self.commands = []
        self.paths = []
        self.diagnostics = []
        self.diag_sub = self.node.create_subscription(DiagnosticArray, 'controller_diagnostics', self.diagnostics.append, 10)
        self.process = None
        self.log = None
        self.scan_pub = self.node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 3.0
            while self.process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=3)
        if self.log is not None:
            self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def start(self, stamped=False, parameters=None, wait=True):
        message_type = TwistStamped if stamped else Twist
        self.command_sub = self.node.create_subscription(
            message_type, 'cmd_vel',
            lambda message: self.commands.append((time.monotonic(), message)), 10)
        params = {'stamped_cmd_vel': 'true' if stamped else 'false'}
        params.update(parameters or {})
        command = [str(EXECUTABLE), '--ros-args', '-r', '__ns:=' + self.namespace]
        for name, value in params.items():
            command.extend(['-p', name + ':=' + str(value)])
        self.log = (ARTIFACT_DIR / (self._testMethodName + '.log')).open('w')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        if wait:
            self.wait_for(lambda: self.scan_pub.get_subscription_count() > 0
                          and self.command_sub.get_publisher_count() > 0)

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail('Timed out; inspect ' + str(ARTIFACT_DIR))
            if self.process is not None and self.process.poll() is not None:
                self.fail('Controller exited early; inspect ' + str(ARTIFACT_DIR))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def phase(self, factory, duration=0.8):
        started = time.monotonic()
        next_scan = started
        while time.monotonic() - started < duration:
            if time.monotonic() >= next_scan:
                self.fresh_odom()
                self.scan_pub.publish(factory())
                next_scan = time.monotonic() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.02)
        messages = [msg for stamp, msg in self.commands if stamp >= started + duration - 0.25]
        self.assertGreaterEqual(len(messages), 3, 'Insufficient stable command samples')
        return messages

    def fresh_odom(self):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)

    def publish_odom(self, x, seconds, frame='odom'):
        self.wait_for(lambda: self.odom_pub.get_subscription_count() > 0)
        message = Odometry()
        message.header.stamp.sec = seconds
        message.header.frame_id = frame
        message.child_frame_id = 'base_footprint'
        message.pose.pose.position.x = float(x)
        message.pose.pose.orientation.w = 1.0
        for _ in range(4):
            self.odom_pub.publish(message)
            self.spin_for(0.06)

    def test_twist_corrections_and_invalid_data(self):
        self.start()
        close = self.phase(lambda: scan_for(self.node, distance=0.25))
        self.assertTrue(all(msg.linear.x > 0 and msg.angular.z > 0 for msg in close))
        # Restart so these are independent static correction checks.
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=3)
        self.log.close()
        self.commands.clear()
        self.start(parameters={'wall_distance': 0.30})
        distant = self.phase(lambda: scan_for(self.node, distance=0.50))
        self.assertTrue(all(msg.linear.x > 0 and msg.angular.z < 0 for msg in distant))
        bad = self.phase(lambda: scan_for(self.node, corrupt=True))
        self.assertTrue(all(msg.linear.x == 0 and msg.angular.z == 0 for msg in bad))

    def test_stamped_yaw_frame_and_timestamp(self):
        self.start(stamped=True, parameters={'laser_yaw': math.pi / 2, 'command_frame': 'offline_base'})
        messages = self.phase(lambda: scan_for(self.node, yaw=math.pi / 2))
        self.assertTrue(all(msg.twist.linear.x > 0.09 and abs(msg.twist.angular.z) < 0.01 for msg in messages))
        self.assertTrue(all(msg.header.frame_id == 'offline_base' for msg in messages))
        stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec for msg in messages]
        self.assertGreater(stamps[0], 0)
        self.assertTrue(all(later > earlier for earlier, later in zip(stamps, stamps[1:])))

    def test_path_retention_filtering_and_resets(self):
        self.start()
        self.publish_odom(0.0, 10)
        # Subscribe after the first publication: the retained path must arrive.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.node.create_subscription(RosPath, 'trajectory', self.paths.append, qos)
        self.wait_for(lambda: bool(self.paths))
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.publish_odom(0.01, 11)
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.publish_odom(0.05, 12)
        self.assertEqual(len(self.paths[-1].poses), 2)
        self.assertAlmostEqual(self.paths[-1].poses[-1].pose.position.x, 0.05)
        self.publish_odom(math.nan, 13)
        self.publish_odom(0.1, 14, frame='')
        self.assertEqual(len(self.paths[-1].poses), 2)
        self.publish_odom(1.0, 20, frame='new_odom')
        self.assertEqual(self.paths[-1].header.frame_id, 'new_odom')
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.publish_odom(1.1, 5, frame='new_odom')
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.assertEqual(self.paths[-1].header.stamp.sec, 5)

    def test_sigint_zero_while_scans_are_fresh(self):
        self.start(stamped=True)
        messages = self.phase(lambda: scan_for(self.node))
        self.assertTrue(all(msg.twist.linear.x > 0 for msg in messages))
        signalled = time.monotonic()
        self.process.send_signal(signal.SIGINT)
        # Continue supplying valid scans: zero cannot be attributed to timeout.
        deadline = signalled + 0.35
        while time.monotonic() < deadline:
            self.fresh_odom()
            self.scan_pub.publish(scan_for(self.node))
            self.spin_for(0.02)
        self.assertEqual(self.process.wait(timeout=3), 0)
        stopped = [(stamp, msg) for stamp, msg in self.commands if stamp >= signalled
                   and msg.twist.linear.x == 0 and msg.twist.angular.z == 0]
        self.assertTrue(stopped, 'No shutdown zero received over local DDS')
        self.assertLess(stopped[0][0] - signalled, 0.35)

    def test_diagnostics_unknown_and_recovery(self):
        self.start(stamped=True)
        self.phase(lambda: scan_for(self.node))
        bad = self.phase(lambda: scan_for(self.node, corrupt=True))
        self.assertTrue(all(msg.twist.linear.x == 0 and msg.twist.angular.z == 0 for msg in bad))
        self.assertTrue(any('scan_invalid' in item.message
                            for message in self.diagnostics for item in message.status))
        recovered = self.phase(lambda: scan_for(self.node))
        self.assertTrue(all(msg.twist.linear.x > 0 for msg in recovered))

    def test_missing_odometry_and_old_scan_headers(self):
        self.start(stamped=True)
        # Supply scans with no odometry; the core must keep publishing zeros.
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            self.scan_pub.publish(scan_for(self.node))
            self.spin_for(0.08)
        self.assertTrue(self.commands)
        self.assertTrue(all(msg.twist.linear.x == 0 and msg.twist.angular.z == 0
                            for _, msg in self.commands))
        def old_scan():
            message = scan_for(self.node)
            message.header.stamp.sec -= 10
            return message
        messages = self.phase(old_scan)
        self.assertTrue(all(msg.twist.linear.x == 0 and msg.twist.angular.z == 0 for msg in messages))
        self.assertTrue(any('scan_header_age_or_clock_mismatch' in item.message
                            for message in self.diagnostics for item in message.status))
        self.assertTrue(all(msg.twist.linear.x > 0 for msg in self.phase(lambda: scan_for(self.node))))

    def test_invalid_configuration_exits_without_motion(self):
        self.start(parameters={'forward_speed': 0.0}, wait=False)
        self.assertNotEqual(self.process.wait(timeout=5), 0)
        self.spin_for(0.15)
        self.assertFalse(self.commands, 'Invalid configuration published commands')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True, type=Path)
    parser.add_argument('--artifact-dir', required=True, type=Path)
    args, remaining = parser.parse_known_args()
    EXECUTABLE = args.executable.resolve()
    ARTIFACT_DIR = args.artifact_dir.resolve()
    if not EXECUTABLE.is_file():
        parser.error('Controller executable does not exist')
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(ARTIFACT_DIR / 'ros_logs')
    (ARTIFACT_DIR / 'ros_logs').mkdir(exist_ok=True)
    (ARTIFACT_DIR / 'isolation.json').write_text(json.dumps({
        'executable': str(EXECUTABLE), 'domain': 201, 'discovery': 'localhost',
        'namespace_root': '/group30_offline', 'hardware': False,
    }, indent=2) + '\n')
    unittest.main(argv=[sys.argv[0], '-v', *remaining])
