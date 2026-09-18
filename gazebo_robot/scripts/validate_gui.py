#!/usr/bin/env python3
"""Launch only this project's GUI and capture its window as visual evidence."""
import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('world',default='robot_bench',nargs='?');args=parser.parse_args()
    directory=ROOT/'validation/gui';directory.mkdir(exist_ok=True)
    env=os.environ.copy();env['IGN_PARTITION']=f'magnetic_gui_{os.getpid()}'
    with (directory/f'{args.world}.log').open('w') as log:
        proc=subprocess.Popen([str(ROOT/'run.sh'),args.world],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(10)
            if proc.poll() is not None:raise RuntimeError('Gazebo GUI exited; inspect log')
            windows=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
            matches=re.findall(r'(0x[0-9a-f]+) "Gazebo"',windows)
            if not matches:raise RuntimeError('Gazebo window not found')
            from PyQt5.QtGui import QGuiApplication
            app=QGuiApplication([])
            picture=app.primaryScreen().grabWindow(int(matches[0],16))
            if picture.isNull():raise RuntimeError('Empty window capture')
            path=directory/f'{args.world}.png'
            if not picture.save(str(path)):raise RuntimeError('Screenshot save failed')
            print(path)
        finally:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)


if __name__=='__main__':main()
