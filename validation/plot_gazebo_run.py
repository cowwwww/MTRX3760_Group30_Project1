#!/usr/bin/env python3
"""Plot recorded Gazebo ground truth over the actual SDF collision geometry."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--world', type=Path, default=Path(__file__).resolve().parents[1] /
                        'A1 Maze Simulation/tb3_maze/worlds/default_maze.world')
    args = parser.parse_args()
    with (args.run / 'trajectory.csv').open() as handle:
        rows = [{key: float(value) if value else None for key, value in row.items()}
                for row in csv.DictReader(handle)]
    points = [(row['world_x'], row['world_y']) for row in rows if row['world_x'] is not None]
    if len(points) < 2:
        parser.error('Insufficient ground-truth points')
    fig, ax = plt.subplots(figsize=(9, 6), constrained_layout=True)
    world = ET.parse(args.world)
    for collision in world.findall(".//model[@name='default_maze']/link/collision"):
        pose = list(map(float, collision.findtext('pose').split()))
        width, height, _ = map(float, collision.findtext('geometry/box/size').split())
        assert abs(pose[5]) < 1e-8, 'Plot expects axis-aligned walls'
        ax.add_patch(Rectangle((pose[0]-width/2, pose[1]-height/2), width, height,
                               facecolor='#655b50', edgecolor='none'))
    ax.plot(*zip(*points), color='#007fac', linewidth=1.7, label='Gazebo ground-truth route')
    ax.scatter(*points[0], color='#198754', s=60, zorder=3, label='Start')
    ax.scatter(*points[-1], color='#c73e3e', s=60, zorder=3, label='Recorded exit')
    ax.set(xlabel='World x (m)', ylabel='World y (m)', xlim=(-2.7, 3.0), ylim=(-1.9, 1.9),
           title='A1 right-wall follower — default Gazebo maze, 7 October 2026')
    ax.set_aspect('equal')
    ax.grid(alpha=0.15)
    ax.legend(loc='upper left', fontsize=8)
    fig.savefig(args.run / 'ground_truth_route.png', dpi=180)
    plt.close(fig)
    distance = sum(math.hypot(x2-x1, y2-y1) for (x1,y1),(x2,y2) in zip(points, points[1:]))
    mins = [row['scan_min'] for row in rows if row['scan_min'] is not None]
    metrics = {'ground_truth_samples': len(points), 'route_length_m': distance,
               'last_ground_truth_m': list(points[-1]), 'last_simulation_time_s': rows[-1]['sim_time'],
               'minimum_sampled_lidar_return_m': min(mins) if mins else None,
               'max_abs_command_linear_m_s': max(abs(row['linear_x']) for row in rows),
               'max_abs_command_angular_rad_s': max(abs(row['angular_z']) for row in rows),
               'collision_verification': 'Not established: no contact sensor was recorded.',
               'route_length_note': 'Approximate polyline length sampled at about 10 Hz wall time.'}
    (args.run / 'calculated_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
