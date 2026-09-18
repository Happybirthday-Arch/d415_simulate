#!/usr/bin/env python3
"""Exercise Fortress world reset through its real transport service."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    directory=ROOT/'validation/reset';directory.mkdir(exist_ok=True)
    csv=directory/'steps.csv'
    if csv.exists():csv.unlink()
    env=os.environ.copy();env.update(IGN_PARTITION=f'magnetic_reset_{os.getpid()}',MAGNETIC_ROBOT_DIAGNOSTICS=str(csv))
    with (directory/'server.log').open('w') as log:
        proc=subprocess.Popen([str(ROOT/'run.sh'),'robot_bench','-s'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                if csv.exists() and csv.stat().st_size>100000:break
                if proc.poll() is not None:raise RuntimeError('Server exited')
                time.sleep(.1)
            else:raise RuntimeError('Diagnostics not ready')
            snapshot=csv.read_bytes()
            # The writer can be between columns; retain only complete CSV rows.
            snapshot=snapshot[:snapshot.rfind(b'\n')+1]
            (directory/'before_reset.csv').write_bytes(snapshot)
            command=['ign','service','-s','/world/robot_bench/control','--reqtype','ignition.msgs.WorldControl',
                     '--reptype','ignition.msgs.Boolean','--timeout','5000','--req','reset: {all: true}']
            response=subprocess.run(command,env=env,capture_output=True,text=True,timeout=10)
            (directory/'service_response.txt').write_text(response.stdout+response.stderr)
            time.sleep(3)
        finally:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    d=np.atleast_1d(np.genfromtxt(csv,delimiter=',',names=True))
    before=np.atleast_1d(np.genfromtxt(directory/'before_reset.csv',delimiter=',',names=True))
    # Depending on Fortress reset implementation the plugin can be recreated,
    # or time can move backwards within the same diagnostic stream.
    resets=np.where(np.diff(d['time'])<0)[0]
    restarted=(len(resets)>0 or d['time'][-1]<before['time'][-1]+2.8)
    after=d[resets[-1]+1:] if len(resets) else d
    settled=after[after['time']>.2]
    passed=bool(response.returncode==0 and 'true' in response.stdout and restarted and len(settled)>500
                and np.all(np.abs(settled['steer'])<=np.pi/12)
                and np.all(np.abs(settled['front_force']-200)<.1)
                and np.all(np.abs(settled['rear_force']-200)<.1)
                and not np.any(np.abs(settled['z']-.0315)>.001))
    report={'passed':passed,'service_response':response.stdout.strip(),'time_reset_observed':bool(restarted),
            'backward_time_indices':resets.tolist(),'final_time':float(d['time'][-1]),'after_reset_samples':len(after)}
    (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
