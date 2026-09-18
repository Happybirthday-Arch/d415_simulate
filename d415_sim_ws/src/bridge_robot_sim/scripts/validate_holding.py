#!/usr/bin/env python3
"""60 simulated seconds of finite-torque parking with added camera/support."""
import json,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from generate_assembly import generate
repo=Path(__file__).resolve().parents[4];root=repo/'gazebo_robot/validation/integration/holding';root.mkdir(parents=True,exist_ok=True)
reports={}
for world in ('adhesion_slope','adhesion_wall','adhesion_ceiling'):
    out=root/world;generate(repo,repo/'d415_sim_ws/src/bridge_robot_sim/config/assembly.yaml',out,world)
    tree=ET.parse(out/'world.sdf');w=tree.getroot().find('world')
    for plug in list(w.findall('plugin')):
        if plug.get('name','').endswith('::Sensors'):w.remove(plug)
    for link in w.findall('model/link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type')!='contact':link.remove(sensor)
    tree.write(out/'physics.sdf')
    env=os.environ.copy();env.update(IGN_PARTITION=f'holding_{os.getpid()}_{world}',
        MAGNETIC_ROBOT_RESOURCE_ROOT=str(repo/'gazebo_robot'),MAGNETIC_ROBOT_DIAGNOSTICS=str(out/'physics.csv'),
        IGN_GAZEBO_SYSTEM_PLUGIN_PATH=str(repo/'gazebo_robot/build'),
        IGN_GAZEBO_RESOURCE_PATH=str(repo/'gazebo_robot/models')+':'+str(repo/'gazebo_bridge/models'))
    with (out/'server.log').open('w') as log:
        subprocess.run(['ign','gazebo','-r','-s',str(out/'physics.sdf'),'--iterations','61000'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180,start_new_session=True)
    data=np.genfromtxt(out/'physics.csv',delimiter=',',names=True)
    positions=np.column_stack([data[k] for k in ('x','y','z')]);stable=data['time']>1
    drift=np.linalg.norm(positions[stable]-positions[stable][0],axis=1)
    recent=data['time']>51;last_drift=np.linalg.norm(positions[recent]-positions[recent][0],axis=1)
    r={'seconds':float(data['time'][-1]),'max_drift_after_settle_m':float(drift.max()),'last_10s_drift_m':float(last_drift.max()),
       'min_front_force':float(data['front_force'][stable].min()),'min_rear_force':float(data['rear_force'][stable].min()),
       'max_steer_deg':float(np.abs(data['steer']).max()*180/np.pi)}
    r['passed']=r['seconds']>=60 and r['last_10s_drift_m']<.002 and min(r['min_front_force'],r['min_rear_force'])>199.9 and r['max_steer_deg']<=15
    reports[world]=r;print(world,r,flush=True)
(root/'report.json').write_text(json.dumps(reports,indent=2));assert all(r['passed'] for r in reports.values())
