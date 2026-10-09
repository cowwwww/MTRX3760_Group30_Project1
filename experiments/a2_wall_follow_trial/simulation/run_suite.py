#!/usr/bin/env python3
"""Run the unchanged controllers; retain unsuccessful navigation outcomes."""
import argparse
import csv
import json
from pathlib import Path
import subprocess


def assess(name, config, version, rows, code):
    final = rows[-1]
    faults = [r for r in rows if r['fault'] == '1']
    contact = any(r['contact'] == '1' for r in rows)
    complete = final['completed'] == '1'
    state_changes = []
    previous = None
    for row in rows:
        if row['state'] != previous:
            state_changes.append({k: row[k] for k in ('time', 'state', 'reason', 'yaw_deg', 'x', 'y')})
            previous = row['state']
    turn_episodes = []
    start = None
    for index, row in enumerate(rows):
        if row['state'] == 'TurnLeft' and start is None:
            start = row
        if start and (row['state'] != 'TurnLeft' or index == len(rows)-1):
            turn_episodes.append(dict(start_time=float(start['time']), end_time=float(row['time']),
                                      angle_degrees=float(row['yaw_deg'])-float(start['yaw_deg'])))
            start = None
    classification = ('contact' if contact else 'goal_completed' if complete else
                      'blind_sector_hold' if config.get('fault') == 'blind' and faults else 'navigation_incomplete')
    return dict(name=name, version=version, config=config, outcome=classification,
                process_code=code, completed=complete, entered=final['entered']=='1',
                approached=final['approached']=='1', returned=final['returned']=='1',
                contact=contact, final_time=float(final['time']), final_yaw=float(final['yaw_deg']),
                final_reason=final['reason'], fault_scans=sum(r['fault']=='1' for r in rows[::2]),
                min_cap_gap=float(final['min_cap_gap']) if config.get('dead-end') else None,
                fault_zero_commands=bool(faults) and all(abs(float(r['v']))<1e-8 and abs(float(r['w']))<1e-8 for r in faults),
                fault_motion=any(abs(float(r['v']))>.001 or abs(float(r['w']))>.01 for r in faults),
                left_turn_episodes=turn_episodes, state_changes=state_changes,
                csv=f'{name}_{version}.csv')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenarios', type=Path, default=Path(__file__).with_name('scenarios.json'))
    args=parser.parse_args()
    cases=json.loads(args.scenarios.read_text())
    build=args.build.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    results=[]
    for case in cases:
        for version in ('original','new'):
            dest=out/f"{case['name']}_{version}.csv"
            cmd=[str(build/f'{version}_runner'),'--output',str(dest)]
            for key,value in case['config'].items():
                cmd += ['--'+key,str(value).lower() if isinstance(value,bool) else str(value)]
            process=subprocess.run(cmd,capture_output=True,text=True)
            if not dest.exists() or process.returncode not in (0,2):
                raise RuntimeError(f'{case["name"]}/{version}: {process.stderr}')
            with dest.open() as f:rows=list(csv.DictReader(f))
            result=assess(case['name'],case['config'],version,rows,process.returncode)
            results.append(result)
            print(f"{case['name']:32} {version:8} {result['outcome']:22} {result['final_time']:6.1f}s {result['final_reason']}",flush=True)
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    with (out/'summary.csv').open('w',newline='') as f:
        fields=['name','version','outcome','completed','entered','approached','returned','contact',
                'final_time','final_yaw','final_reason','fault_scans','fault_zero_commands','min_cap_gap']
        writer=csv.DictWriter(f,fields,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    print('\nActual navigation outcomes (not an all-pass score):')
    for version in ('original','new'):
        subset=[r for r in results if r['version']==version]
        print(version,{key:sum(r['outcome']==key for r in subset) for key in sorted(set(r['outcome'] for r in subset))})


if __name__=='__main__':
    main()
