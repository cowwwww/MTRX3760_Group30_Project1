#!/usr/bin/env python3
"""Record one default-maze run using the repository's real Gazebo launch and A1 node.

Source Jazzy and the built A1 workspace before running this file. No physical
robot is used. Exit detection is specific to default_maze's east exit.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

os.environ.update(ROS_DOMAIN_ID='202', ROS_LOCALHOST_ONLY='1',
                  RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                  TURTLEBOT3_MODEL='burger_cam', GZ_PARTITION='group30_offline_maze_check')

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as RosImage, LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=420)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    processes, handles = [], []

    def start(name, command):
        handle = (output / (name + '.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    rclpy.init()
    node = rclpy.create_node('maze_run_recorder')
    state = {'sim_time': 0.0, 'scan_count': 0, 'odom_count': 0, 'camera_count': 0,
             'command_count': 0, 'odom': None, 'world': None, 'v': 0.0, 'w': 0.0,
             'scan_min': None, 'pose_count': 0}
    rows = []

    def clock(message):
        state['sim_time'] = message.clock.sec + message.clock.nanosec / 1e9

    def scan(message):
        state['scan_count'] += 1
        values = [value for value in message.ranges if math.isfinite(value)
                  and message.range_min <= value <= message.range_max]
        state['scan_min'] = min(values) if values else None
        if state['scan_count'] == 1:
            (output / 'scan_metadata.json').write_text(json.dumps({
                'frame': message.header.frame_id, 'rays': len(message.ranges),
                'angle_min': message.angle_min, 'angle_max': message.angle_max,
                'angle_increment': message.angle_increment, 'range_min': message.range_min,
                'range_max': message.range_max}, indent=2) + '\n')

    def odom(message):
        state['odom_count'] += 1
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        state['odom'] = [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                           1 - 2 * (q.y*q.y + q.z*q.z))]

    def command(message):
        state['command_count'] += 1
        state['v'], state['w'] = message.twist.linear.x, message.twist.angular.z

    def read_ground_truth(process):
        # Pose_V -> TFMessage loses Gazebo model names in this bridge version.
        # Read named JSON poses directly so the world pose is unambiguous.
        for line in process.stdout:
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue
            for pose in message.get('pose', []):
                if pose.get('name') == 'burger_cam':
                    p = pose['position']
                    state['world'] = [p.get('x', 0.0), p.get('y', 0.0)]
                    state['pose_count'] += 1
                    if state['pose_count'] == 1:
                        (output / 'first_ground_truth.json').write_text(json.dumps(message, indent=2)+'\n')

    def camera(message):
        state['camera_count'] += 1
        if state['camera_count'] in (1, 100, 500):
            modes = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'), 'mono8': ('L', 'L')}
            if message.encoding in modes:
                mode, raw = modes[message.encoding]
                image = Image.frombytes(mode, (message.width, message.height),
                                        bytes(message.data), 'raw', raw, message.step)
                image.save(output / ('camera_%04d.png' % state['camera_count']))

    subscriptions = [
        node.create_subscription(Clock, '/clock', clock, 10),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', odom, 10),
        node.create_subscription(TwistStamped, '/cmd_vel', command, 10),
        node.create_subscription(RosImage, '/camera/image_raw', camera, qos_profile_sensor_data),
    ]
    result = 'STARTUP_TIMEOUT'
    started = time.monotonic()
    controller = None
    initial_world = None
    initial_odom = None
    next_sample = 0.0
    next_progress = 0.0
    try:
        simulation = start('maze_launch', ['ros2', 'launch', 'tb3_maze', 'maze.launch.py'])
        pose_log = (output / 'ground_truth_reader.log').open('w')
        handles.append(pose_log)
        pose_reader = subprocess.Popen(['gz', 'topic', '-e', '--json-output', '-t',
            '/world/default/dynamic_pose/info'], stdout=subprocess.PIPE, stderr=pose_log,
            text=True, start_new_session=True)
        processes.append(pose_reader)
        threading.Thread(target=read_ground_truth, args=(pose_reader,), daemon=True).start()
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
            if simulation.poll() is not None:
                result = 'SIMULATION_EXITED'
                break
            if controller is None and state['scan_count'] and state['odom_count'] and state['world']:
                initial_world = state['world']
                initial_odom = state['odom']
                controller = start('wall_follower', ['ros2', 'run', 'tb3_maze', 'turtlebot3_drive',
                    '--ros-args', '-p', 'use_sim_time:=true'])
                result = 'TIME_LIMIT'
                print('Sensors ready; started production A1 right-wall follower.', flush=True)
            if controller is not None and controller.poll() is not None:
                result = 'CONTROLLER_EXITED'
                break
            if time.monotonic() >= next_progress:
                next_progress = time.monotonic() + 15
                print('sim=%.1fs world=%s scans=%d camera=%d commands=%d' %
                      (state['sim_time'], state['world'], state['scan_count'],
                       state['camera_count'], state['command_count']), flush=True)
            if state['odom'] is not None and time.monotonic() >= next_sample:
                next_sample = time.monotonic() + 0.1
                x, y, yaw = state['odom']
                wx, wy = state['world'] or (None, None)
                rows.append([state['sim_time'], x, y, yaw, wx, wy, state['v'], state['w'],
                             state['scan_min']])
                if wx is not None and wx > 2.60 and 0.85 < wy < 1.55:
                    result = 'EXIT_REACHED'
                    break
        print(result, flush=True)
    except KeyboardInterrupt:
        result = 'INTERRUPTED'
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        for handle in handles:
            handle.close()
        with (output / 'trajectory.csv').open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['sim_time', 'odom_x', 'odom_y', 'yaw', 'world_x', 'world_y',
                             'linear_x', 'angular_z', 'scan_min'])
            writer.writerows(rows)
        report = dict(state, result=result, initial_world=initial_world,
                      initial_odom=initial_odom, wall_seconds=time.monotonic()-started,
                      hardware=False, ros_domain=202, gz_partition=os.environ['GZ_PARTITION'],
                      world='default_maze', model='burger_cam', controller='tb3_maze/turtlebot3_drive',
                      exit_criterion='Ground-truth x > 2.60 m and 0.85 < y < 1.55 m')
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if result == 'EXIT_REACHED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
