#!/usr/bin/env python3
"""Consolidate saved integration evidence without rerunning expensive tests."""
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
repo=Path(__file__).resolve().parents[4];root=repo/'gazebo_robot/validation/integration'
load=lambda path:json.loads((root/path).read_text())
reports={name:load(path) for name,path in {
    'attachment':'attachment_report.json','bench':'robot_bench_rgb/report.json',
    'bridge':'bridge_robot_rgb/report.json','keyboard':'gui/report.json',
    'lifecycle':'lifecycle/report.json','full':'robot_bench_full/report.json'}.items()}
reports['holding']=load('holding/report.json')
metadata=max((root/'robot_bench_full/recordings').glob('*/bag/metadata.yaml'),key=lambda p:p.stat().st_mtime)
bag=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
counts={entry['topic_metadata']['name']:entry['message_count'] for entry in bag['topics_with_message_count']}
required=['/d415/color/image_raw','/d415/depth/image_raw','/d415/color/camera_info','/d415/depth/camera_info','/tf','/tf_static','/clock','/joint_states','/robot/drive_command','/robot/ground_truth']
reports['full_bag_topics']=counts
checks={name:report['passed'] for name,report in reports.items() if name not in ('holding','full_bag_topics')}
checks.update({name:r['passed'] for name,r in reports['holding'].items()})
checks['full_topics_present']=all(counts.get(topic,0)>0 for topic in required)
checks['rgb_only_files']=all(not any(name.endswith(('.db3','.npy','.bag')) for name in reports[key]['recorded_files']) and reports[key]['recording']['error'] is None for key in ('bench','bridge'))
poses=np.loadtxt(root/'robot_bench_rgb/poses.csv',delimiter=',');yaw=np.unwrap(Rotation.from_quat(poses[:,4:8]).as_euler('xyz')[:,2])
turns={}
for name in ('left','right'):
    start,end=reports['bench']['phases'][name];indices=[np.argmin(abs(poses[:,0]-t)) for t in (start,end)]
    turns[name]=float((yaw[indices[1]]-yaw[indices[0]])*180/np.pi)
checks['left_turn_direction']=turns['left']>0;checks['right_turn_direction']=turns['right']<0
summary={'passed':all(checks.values()),'checks':checks,'support_length_m':.03,'camera_bottom_xyz_m':[.04213,0,.087],
         'straight_speed_m_s':reports['bridge']['forward_measured_m_s'],'turn_yaw_changes_deg':turns,
         'max_steer_deg':reports['bench']['max_steer_deg'],
         'depth_valid_fraction_in_bridge':reports['bridge']['depth_valid_fraction'],
         'full_recorded_topics':counts,
         'limitations':['Fortress sensor scheduling may pause after a world reset until simulation catches up to the previous next-frame time. Restart for immediate fresh capture.',
                        'No arbitrary edge transitions or full-bridge route coverage are claimed.'],
         'reports':{name:path for name,path in [('bench','robot_bench_rgb/report.json'),('bridge','bridge_robot_rgb/report.json'),('keyboard','gui/report.json'),('holding','holding/report.json'),('lifecycle','lifecycle/report.json')]}}
(root/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));assert summary['passed']
