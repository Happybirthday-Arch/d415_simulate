#!/usr/bin/env python3
"""Run real Fortress/DART experiments, retaining per-step evidence and SDFs."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def run_case(name,world,seconds=3,brake=False,steering_torque=0,wheel_torque=0,
             initial_speed=0,step=None,expect='attached',modify=None):
    directory=ROOT/'validation/physics'/name
    directory.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(ROOT/'worlds'/f'{world}.sdf')
    w=tree.getroot().find('world');w.set('name',name)
    w.find('physics/real_time_factor').text='0'
    if step:w.find('physics/max_step_size').text=str(step)
    dt=float(w.findtext('physics/max_step_size'))
    model=w.find("model[@name='magnetic_robot']")
    p=ET.SubElement(model,'plugin',filename='libMagneticRobot.so',name='magnetic_robot::TestRig')
    for k,v in [('brake',str(brake).lower()),('steering_torque',steering_torque),
                ('wheel_torque',wheel_torque),('initial_steering_speed',initial_speed)]:
        ET.SubElement(p,k).text=str(v)
    if modify:modify(w,model)
    ET.indent(tree,space='  ')
    sdf=directory/'world.sdf';tree.write(sdf,encoding='utf-8',xml_declaration=True)
    env=os.environ.copy()
    env.update(MAGNETIC_ROBOT_RESOURCE_ROOT=str(ROOT),MAGNETIC_ROBOT_DIAGNOSTICS=str(directory/'steps.csv'),
               IGN_PARTITION=f'magnetic_validation_{os.getpid()}_{name}',
               IGN_GAZEBO_RESOURCE_PATH=f'{ROOT}/models:{ROOT.parent}/gazebo_bridge/models',
               IGN_GAZEBO_SYSTEM_PLUGIN_PATH=str(ROOT/'build'),
               IGN_GAZEBO_LOG_PATH=str(directory/'gazebo_logs'))
    command=['ign','gazebo','-s','-r','-v','3','--iterations',str(round(seconds/dt)),str(sdf)]
    start=time.monotonic()
    with (directory/'server.log').open('w') as log:
        result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=max(180,seconds*8))
    log=(directory/'server.log').read_text()
    report={'name':name,'world':world,'seconds_requested':seconds,'step':dt,'brake_fixture':brake,
            'steering_torque_Nm':steering_torque,'wheel_torque_Nm':wheel_torque,
            'initial_steering_speed_rad_s':initial_speed,'returncode':result.returncode,
            'wall_seconds':time.monotonic()-start,'expect':expect}
    csv=directory/'steps.csv'
    if not csv.exists():
        report.update(passed=False,error='No plugin diagnostics; inspect server.log')
    else:
        data=np.atleast_1d(np.genfromtxt(csv,delimiter=',',names=True))
        if not len(data):raise RuntimeError('Empty diagnostics')
        tail=data[data['time']>min(.5,seconds/2)]
        if not len(tail):tail=data
        complete=data['time'][-1]>=seconds-dt*1.1
        no_errors=not any(s in log for s in ['[Err]','STEERING_LIMIT_VIOLATION','Creating a FIXED'])
        q=float(np.max(np.abs(data['steer']))*180/math.pi)
        gap=np.concatenate([tail['front_gap'],tail['rear_gap']])
        forces=np.column_stack([tail['front_force'],tail['rear_force']])
        attached=np.all(np.abs(forces-200)<.1) and np.max(np.abs(gap))<.001
        report.update(samples=len(data),final_time=float(data['time'][-1]),complete=bool(complete),
                      max_steering_deg=q,force_min_N=forces.min(axis=0).tolist(),force_max_N=forces.max(axis=0).tolist(),
                      gap_range_m=[float(gap.min()),float(gap.max())],
                      final_position_m=[float(data[k][-1]) for k in ['x','y','z']],
                      final_rpy_rad=[float(data[k][-1]) for k in ['roll','pitch','yaw']],
                      wheel_angle_change_rad=[float(data[k][-1]-data[k][0]) for k in ['front_angle','rear_angle']],
                      contact_steps=int(np.count_nonzero(tail['contacts'])),no_server_errors=no_errors)
        check=attached if expect=='attached' else np.all(forces<.1) if expect=='detached' else True
        report['passed']=bool(result.returncode==0 and complete and no_errors and q<=15 and check)
    (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return report


def main():
    p=argparse.ArgumentParser();p.add_argument('--suite',choices=['smoke','full','limits','negative','bridge','rolling','release','stress','single'],default='smoke');p.add_argument('--seconds',type=float)
    args=p.parse_args();results=[]
    if args.suite in ['smoke','full']:
        for name,world in [('floor','robot_bench'),('wall','adhesion_wall'),('ceiling','adhesion_ceiling'),('slope','adhesion_slope')]:
            results.append(run_case(name,world,args.seconds or (60 if args.suite=='full' else 3),brake=True))
    if args.suite in ['limits','full']:
        for dt in [.001,.0005]:
            for sign in [-1,1]:
                results.append(run_case(f'limit_{sign}_{dt}','steering_fixture',args.seconds or 3,
                                        steering_torque=sign*.5,initial_speed=sign*2,step=dt,expect='detached'))
    if args.suite in ['negative','full']:
        for name in ['nonmagnetic','hole']:
            results.append(run_case(name,name,args.seconds or 1,expect='detached'))
        # A wheel initially outside capture can fall back into capture. Disable gravity
        # for the range-only negative experiment rather than incorrectly expecting release forever.
        def no_gravity(w,m):w.find('gravity').text='0 0 0'
        results.append(run_case('out_of_range','out_of_range',args.seconds or 1,expect='detached',modify=no_gravity))
    if args.suite in ['stress','full']:
        for sign in [-1,1]:
            results.append(run_case(f'limit_stress_{sign}','steering_fixture',args.seconds or 2,
                                    steering_torque=sign*5,initial_speed=sign*20,expect='detached'))
    if args.suite in ['single','full']:
        r=run_case('single_wheel','single_wheel_fixture',args.seconds or 1,expect='observe')
        d=np.atleast_1d(np.genfromtxt(ROOT/'validation/physics/single_wheel/steps.csv',delimiter=',',names=True))
        independent=np.all(np.abs(d['front_force']-200)<.1) and np.all(d['rear_force']<.1)
        r.update(single_wheel_verified=bool(independent),passed=bool(r['passed'] and independent))
        (ROOT/'validation/physics/single_wheel/report.json').write_text(json.dumps(r,indent=2)+'\n');results.append(r)
    if args.suite in ['release','full']:
        def pull(w,m):
            ET.SubElement(m.find("plugin[@name='magnetic_robot::TestRig']"),'pull_force').text='600'
        r=run_case('pull_off','robot_bench',args.seconds or .5,expect='observe',modify=pull)
        d=np.atleast_1d(np.genfromtxt(ROOT/'validation/physics/pull_off/steps.csv',delimiter=',',names=True))
        before=d[(d['time']>.1)&(d['time']<.2)];after=d[d['time']>.35]
        released=all(np.all(before[k]>199.9) and np.all(after[k]<.1) for k in ['front_force','rear_force'])
        r.update(release_verified=bool(released),passed=bool(r['passed'] and released))
        (ROOT/'validation/physics/pull_off/report.json').write_text(json.dumps(r,indent=2)+'\n');results.append(r)
    if args.suite in ['bridge','full']:
        results.append(run_case('bridge_floor','bridge_robot',args.seconds or 10,brake=True))
        for world in ['bridge_ceiling','bridge_roof','bridge_underside','bridge_left_wall','bridge_right_wall','bridge_panel','bridge_seam']:
            results.append(run_case(world,world,args.seconds or 3,brake=True))
        results.append(run_case('bridge_hole','bridge_hole',args.seconds or 1,expect='detached',
                                modify=lambda w,m:setattr(w.find('gravity'),'text','0 0 0')))
    if args.suite in ['rolling','full']:
        for name,world,torque in [('roll_floor','robot_bench',.01),('roll_ceiling','adhesion_ceiling',.01),
                                  ('roll_wall','adhesion_wall',-.43),('roll_slope','adhesion_slope',-.31),
                                  ('roll_bridge','bridge_robot',.01)]:
            r=run_case(name,world,args.seconds or 2,wheel_torque=torque)
            d=np.atleast_1d(np.genfromtxt(ROOT/'validation/physics'/name/'steps.csv',delimiter=',',names=True))
            distance=float(np.linalg.norm([d[k][-1]-d[k][0] for k in ['x','y','z']]))
            wheel_travel=float(np.mean(np.abs(r['wheel_angle_change_rad']))*.0315)
            r.update(distance_m=distance,wheel_travel_m=wheel_travel,
                     rolling_passed=bool(distance>.02 and abs(distance-wheel_travel)<.02+.2*wheel_travel))
            r['passed'] &= r['rolling_passed']
            (ROOT/'validation/physics'/name/'report.json').write_text(json.dumps(r,indent=2)+'\n')
            results.append(r)
    (ROOT/'validation'/f'{args.suite}_report.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),'cases':results},indent=2)+'\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
