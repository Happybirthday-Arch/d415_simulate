#!/usr/bin/env python3
"""Full recording with existing readiness/closure logic and explicit extra topics."""
import argparse,importlib.util,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix

def main():
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--manifest',required=True)
    args,rest=parser.parse_known_args()
    path=Path(get_package_prefix('bridge_d415_sim'))/'lib/bridge_d415_sim/record.py'
    spec=importlib.util.spec_from_file_location('camera_record',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    extra=[]
    for topic in ['/joint_states','/robot/drive_command','/robot/drive_status','/robot/ground_truth']:
        extra+=['--extra-topic',topic]
    sys.argv=[sys.argv[0],*rest,'--assembly',args.manifest,*extra];module.main()
if __name__=='__main__':main()
