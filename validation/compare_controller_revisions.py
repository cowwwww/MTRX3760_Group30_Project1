#!/usr/bin/env python3
"""Compare deterministic production-controller outputs across Git revisions.

Source Jazzy first. Builds both revisions in an output directory and compares
2,000 sequential generated inputs per controller, using the same trace source.
No ROS nodes or hardware connections are created.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', default='ed9200438d0fa73787733dccec85b87fb23e82cb')
    parser.add_argument('--candidate', default='HEAD')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    trace = repo / 'validation/controller_behaviour_trace.cpp'
    ros_prefixes = [Path(p) for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]
    includes = [str(folder) for prefix in ros_prefixes for folder in (prefix/'include').glob('*')
                if folder.is_dir()]
    if not includes:
        parser.error('Source ROS 2 Jazzy before running this comparison')
    report = {'baseline': args.baseline, 'candidate': args.candidate, 'seed': 30,
              'sequential_inputs_per_controller': 2000,
              'trace_source_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(), 'controllers': {}}
    for label, revision in [('baseline', args.baseline), ('candidate', args.candidate)]:
        sha = subprocess.check_output(['git','rev-parse',revision],cwd=repo,text=True).strip()
        report[label+'_sha'] = sha
        # A3's final packages moved out of the historical A1/A2 folders.
        # Select paths from each revision so old evidence remains reproducible.
        has_a3 = subprocess.run(
            ['git', 'cat-file', '-e', sha+':A3 Refactor/Physical ROS/CMakeLists.txt'],
            cwd=repo, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        physical_root = 'A3 Refactor/Physical ROS' if has_a3 else 'A2 ROS'
        simulation_root = 'A3 Refactor/Simulation/tb3_maze' if has_a3 else 'A1 Maze Simulation/tb3_maze'
        report[label+'_source_roots'] = [physical_root, simulation_root]
        source = output / (label+'_source')
        source.mkdir(exist_ok=True)
        archive = subprocess.check_output(['git','archive',sha,physical_root,simulation_root],cwd=repo)
        with tarfile.open(fileobj=io.BytesIO(archive)) as handle:
            handle.extractall(source, filter='data')
        with (output/(label+'_build.log')).open('w') as log:
            def run(command):
                subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
            build = output / (label+'_core')
            run(['cmake','-S',str(source/physical_root),'-B',str(build),'-DPROJECT1_ROS_VERSION=',
                 '-DBUILD_TESTING=OFF','-DCMAKE_BUILD_TYPE=Debug'])
            run(['cmake','--build',str(build),'-j4'])
            exe = output/(label+'_a2_trace')
            run(['c++','-std=c++14','-O0','-I'+str(source/physical_root/'include'),str(trace),
                 str(build/'libwall_controller.a'),'-o',str(exe)])
            (output/(label+'_a2_trace.txt')).write_bytes(subprocess.check_output([str(exe)]))
            package = source/simulation_root
            sources = [package/'src/c_robot.cpp',package/'src/c_sensor.cpp']
            strategy = package/'src/c_right_wall_follower_robot.cpp'
            if strategy.exists(): sources.append(strategy)
            exe = output/(label+'_a1_trace')
            run(['c++','-std=c++17','-O0','-DTRACE_A1','-I'+str(package/'include'),
                 *['-I'+p for p in includes],str(trace),*[str(p) for p in sources],'-o',str(exe)])
            (output/(label+'_a1_trace.txt')).write_bytes(subprocess.check_output([str(exe)]))
    for component in ['a1','a2']:
        before = (output/('baseline_'+component+'_trace.txt')).read_bytes()
        after = (output/('candidate_'+component+'_trace.txt')).read_bytes()
        report['controllers'][component] = {'identical':before==after,
            'baseline_sha256':hashlib.sha256(before).hexdigest(),
            'candidate_sha256':hashlib.sha256(after).hexdigest(),
            'lines':len(after.splitlines())}
    (output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if all(value['identical'] for value in report['controllers'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
