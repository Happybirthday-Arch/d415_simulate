#!/usr/bin/env python3
"""Check stale commands through pause/reset and RGB segment closure on reset."""
import json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from validate_integration import Probe
repo=Path(__file__).resolve().parents[4];out=repo/'gazebo_robot/validation/integration/lifecycle';out.mkdir(parents=True,exist_ok=True)
os.environ['IGN_PARTITION']=f'lifecycle_{os.getpid()}';os.environ['ROS_DOMAIN_ID']=str(70+os.getpid()%100)
log=(out/'launch.log').open('w');proc=subprocess.Popen([str(repo/'d415_sim_ws/run_robot.sh'),'world:=robot_bench','gui:=false','rviz:=false','keyboard:=false','record_mode:=rgb',f'recordings:={out}/recordings'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Probe();report={}
def service(req):
    result=subprocess.run(['ign','service','-s','/world/bridge_robot_camera/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,check=True)
    if 'data: true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
def wall(seconds,x=0):
    end=time.monotonic()+seconds
    while time.monotonic()<end:node.step(x)
try:
    deadline=time.monotonic()+90
    while node.t is None or len(node.counts)<6:
        node.step()
        if time.monotonic()>deadline:raise RuntimeError('No sensor streams')
    node.phase(.5);node.phase(2,1)
    service('pause: true');wall(.5,1);paused_time=node.t;wall(.7,1)
    report['pause_time_static']=abs(node.t-paused_time)<.002
    service('pause: false');wall(.2,1);node.phase(1,1)
    report['pause_stale_command_blocked']=node.status.z==1 and max(abs(x) for x in node.rows[-1][4:6])<.04
    node.phase(.5);node.phase(1,1);before=node.t
    service('reset: {all: true}');deadline=time.monotonic()+5
    while node.t>=before:
        node.step(1)
        if time.monotonic()>deadline:raise RuntimeError('Reset time did not rewind')
    node.phase(1,1)
    report['reset_stale_command_blocked']=node.status.z==1 and max(abs(x) for x in node.rows[-1][4:6])<.04
    node.phase(.5);node.phase(1,1);report['neutral_rearm']=node.status.z==0 and node.status.x>14.9
    images_after_reset=node.counts['/d415/color/image_raw']
    # Fortress's Sensors system resumes at the pre-reset scheduled time.
    node.phase(max(1,before-node.t+1))
    report['sensors_resume_after_reset']=node.counts['/d415/color/image_raw']>images_after_reset
finally:
    node.pub.publish(Vector3(z=1.));node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=35)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
    log.close()
    sessions=list((out/'recordings').glob('*'))
    if sessions:
        latest=max(sessions,key=lambda p:p.stat().st_mtime);video=json.loads((latest/'recording.json').read_text())
        report['reset_video_segments']=len(video['segments']);report['encoder_error']=video['error']
    report['passed']=all(report.get(k,False) for k in ['pause_time_static','pause_stale_command_blocked','reset_stale_command_blocked','neutral_rearm']) and report.get('reset_video_segments',0)>=2 and report.get('encoder_error') is None
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
assert report['passed']
