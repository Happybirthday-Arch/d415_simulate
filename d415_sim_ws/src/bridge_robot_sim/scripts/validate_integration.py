#!/usr/bin/env python3
"""Own an isolated launch, exercise control and verify data and RGB-only files."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Vector3,PoseStamped
from sensor_msgs.msg import JointState,Image,PointCloud2,CameraInfo
from tf2_ros import Buffer,TransformListener
from cv_bridge import CvBridge
import cv2

class Probe(Node):
    def __init__(self):
        super().__init__('integration_probe');self.pub=self.create_publisher(Vector3,'/robot/drive_command',1)
        self.t=None;self.rows=[];self.poses=[];self.counts={};self.images={};self.status=None;self.last_status=0
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/robot/ground_truth',self.pose,qos_profile_sensor_data)
        self.create_subscription(Vector3,'/robot/drive_status',self.drive_status,qos_profile_sensor_data)
        for topic,cls in [('/d415/color/image_raw',Image),('/d415/depth/image_raw',Image),('/d415/depth/preview',Image),('/d415/depth/points',PointCloud2),('/d415/color/camera_info',CameraInfo),('/d415/depth/camera_info',CameraInfo)]:
            self.create_subscription(cls,topic,lambda m,t=topic:self.sensor(t,m),qos_profile_sensor_data)
    def drive_status(self,m):self.status=m;self.last_status=time.monotonic()
    def joints(self,m):
        self.t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.rows.append([self.t,*m.position,*m.velocity])
    def pose(self,m):
        p=m.pose.position;q=m.pose.orientation
        self.poses.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def sensor(self,t,m):
        self.counts[t]=self.counts.get(t,0)+1
        if isinstance(m,Image):self.images[t]=m
    def step(self,x=0.,y=0.,z=0.,publish=True):
        if publish:self.pub.publish(Vector3(x=float(x),y=float(y),z=float(z)))
        until=time.monotonic()+.02
        while time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.002)
    def phase(self,duration,x=0,y=0,z=0,publish=True):
        start=self.t;wall=time.monotonic()
        while self.t-start<duration:
            if time.monotonic()-wall>150:raise RuntimeError('Simulation did not advance')
            self.step(x,y,z,publish)
        return [start,self.t]

def main():
    p=argparse.ArgumentParser();p.add_argument('--world',default='robot_bench');p.add_argument('--smoke',action='store_true');p.add_argument('--mode',default='rgb');p.add_argument('--gui',action='store_true');a=p.parse_args()
    repo=Path(__file__).resolve().parents[4];out=repo/'gazebo_robot/validation/integration'/f'{a.world}_{a.mode}';out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['IGN_PARTITION']=f'robot_test_{os.getpid()}';env['ROS_DOMAIN_ID']=str(70+os.getpid()%100)
    env['MAGNETIC_ROBOT_DIAGNOSTICS']=str(out/'physics.csv');os.environ.update({k:env[k] for k in ('IGN_PARTITION','ROS_DOMAIN_ID')})
    log=(out/'launch.log').open('w')
    process=subprocess.Popen([str(repo/'d415_sim_ws/run_robot.sh'),f'world:={a.world}',f'gui:={str(a.gui).lower()}',
          'rviz:=false','keyboard:=false',f'record_mode:={a.mode}',f'recordings:={out}/recordings'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report={'world':a.world,'mode':a.mode,'phases':{}};rclpy.init();node=Probe();wall=time.monotonic()
    try:
        while node.t is None or len(node.counts)<6:
            if process.poll() is not None:raise RuntimeError('Launch exited; inspect launch.log')
            if time.monotonic()-wall>100:raise RuntimeError(f'No complete streams: {node.counts}')
            node.step()
        phases=[('neutral',1,0,0),('forward',3 if a.smoke else 4,1,0),('stop',2,0,0)]
        if not a.smoke:phases += [('reverse',3,-1,0),('stop_reverse',1,0,0),('left',4,1,1),('stop_left',1,0,0),('right',4,1,-1),('stop_right',2,0,0),('before_timeout',1,1,0)]
        for name,duration,x,y in phases:
            report['phases'][name]=node.phase(duration,x,y)
            print(a.world,name,node.t,flush=True)
        if not a.smoke:
            report['phases']['timeout']=node.phase(2,publish=False)
            report['phases']['neutral_rearm']=node.phase(.5)
            report['phases']['before_brake']=node.phase(1,1)
            report['phases']['brake']=node.phase(1,z=1)
        # Let transforms catch up to an existing image stamp while stationary.
        img=node.images['/d415/color/image_raw'];depth=node.images['/d415/depth/image_raw']
        node.phase(.2)
        tf=node.buffer.lookup_transform('world',img.header.frame_id,Time.from_msg(img.header.stamp))
        report['tf_at_rgb_stamp']=True;report['rgb_frame']=img.header.frame_id
        bridge=CvBridge();cv2.imwrite(str(out/'rgb.png'),bridge.imgmsg_to_cv2(img,'bgr8'))
        array=bridge.imgmsg_to_cv2(depth);np.save(out/'depth.npy',array)
        report['depth_encoding']=depth.encoding;report['depth_valid_fraction']=float(np.isfinite(array).mean())
        report['streams']=node.counts
        rows=np.array(node.rows);np.savetxt(out/'joints.csv',rows,delimiter=',',header='time,front_angle,rear_angle,steer,front_velocity,rear_velocity,steer_velocity')
        poses=np.array(node.poses);np.savetxt(out/'poses.csv',poses,delimiter=',')
        report['max_steer_deg']=float(np.max(np.abs(rows[:,3]))*180/np.pi)
        for name,(begin,end) in report['phases'].items():
            steady=rows[(rows[:,0]>max(begin+.5*(end-begin),end-.5))&(rows[:,0]<=end)]
            report[name]={'mean_rpm':(np.mean(steady[:,4:6],axis=0)*30/np.pi).tolist(),'mean_steer_deg':float(np.mean(steady[:,3])*180/np.pi)}
        start,end=report['phases']['forward'];p0=poses[np.argmin(abs(poses[:,0]-(start+1)))];p1=poses[np.argmin(abs(poses[:,0]-end))]
        report['forward_measured_m_s']=float(np.linalg.norm(p1[1:4]-p0[1:4])/(p1[0]-p0[0]))
        checks={'steering_limit':report['max_steer_deg']<=15,
                'straight_wheel_speed':max(abs(x-15) for x in report['forward']['mean_rpm'])<.3,
                'straight_ground_speed':abs(report['forward_measured_m_s']-.0494800843)<.0494800843*.05,
                'stop':max(abs(x) for x in report['stop']['mean_rpm'])<.3,
                'image_tf':report['tf_at_rgb_stamp'],'depth_encoding':depth.encoding=='32FC1'}
        if not a.smoke:
            checks.update(left_angle=abs(report['left']['mean_steer_deg']-14)<.5,
                          right_angle=abs(report['right']['mean_steer_deg']+14)<.5,
                          timeout_stop=max(abs(x) for x in report['timeout']['mean_rpm'])<.3,
                          brake_stop=max(abs(x) for x in report['brake']['mean_rpm'])<.3)
        report['checks']=checks;report['passed']=all(checks.values())
        report['wall_seconds']=time.monotonic()-wall
    except Exception as exc:
        report['error']=str(exc);raise
    finally:
        node.pub.publish(Vector3(z=1.));node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=45)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        log.close()
        report['launch_returncode']=process.returncode
        recordings=list((out/'recordings').glob('*')) if (out/'recordings').exists() else []
        if a.mode=='rgb' and recordings:
            latest=max(recordings,key=lambda p:p.stat().st_mtime);report['recording']=json.loads((latest/'recording.json').read_text())
            report['recorded_files']=[p.name for p in latest.iterdir()]
            video=latest/report['recording']['segments'][0]['video']
            result=subprocess.run(['ffprobe','-v','error','-show_entries','stream=codec_name,width,height,nb_frames,duration','-of','json',str(video)],capture_output=True,text=True,check=True)
            report['video']=json.loads(result.stdout);report['video_bytes']=video.stat().st_size
        (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    if not report.get('passed'):raise RuntimeError(f'Failed checks: {report.get("checks")}')
if __name__=='__main__':main()
