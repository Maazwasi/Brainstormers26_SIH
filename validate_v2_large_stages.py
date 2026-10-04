"""Run in Ubuntu Terminal: sequential gates, never advance after a FAIL.

Evidence is newly recorded for each attempt; old passes are not transplanted.
The agent must diagnose a failed gate before restarting from that multiplier.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--from-speed',type=float,default=.6,
                        choices=(.6,.9,1.2,1.5,1.8))
    parser.add_argument('--visible',action='store_true')
    args=parser.parse_args()
    workspace=Path(__file__).resolve().parent
    attempt=time.strftime('%Y%m%d-%H%M%S')
    stages=[]
    for speed in (.6,.9,1.2,1.5,1.8):
        if speed<args.from_speed:continue
        for scenario in ('motion','priority','reroute','charging_gap'):
            report=f'SWARMX_LARGE_{attempt}_{speed:.1f}_{scenario}.json'
            command=[sys.executable,str(workspace/'validate_v2_large.py'),scenario,
                     '--speed',str(speed),'--report',report]
            if args.visible:command.append('--visible')
            print(f'GATE: {speed/.6:.1f}x / {speed:.2f} m/s / {scenario}',flush=True)
            process=subprocess.run(command,cwd=workspace)
            evidence=json.loads((workspace/report).read_text()) if (workspace/report).exists() else {}
            stages.append(dict(speed=speed,multiplier=round(speed/.6,1),scenario=scenario,
                               status=evidence.get('status','FAIL'),report=report))
            (workspace/'SWARMX_LARGE_STAGE_PROGRESS.json').write_text(json.dumps(stages,indent=2))
            if process.returncode or evidence.get('status')!='PASS':
                print('SAFE GATE: failed stage; no higher speed started.',flush=True)
                return 1
            # Shutdown owns the whole process group; give DDS cleanup a short
            # bounded interval before the next new simulation launch.
            time.sleep(3.)
    report=f'SWARMX_LARGE_{attempt}_1.8_fleet.json'
    command=[sys.executable,str(workspace/'validate_v2_large.py'),'fleet',
             '--speed','1.8','--report',report]
    if args.visible:command.append('--visible')
    return subprocess.run(command,cwd=workspace).returncode


if __name__=='__main__':
    raise SystemExit(main())
