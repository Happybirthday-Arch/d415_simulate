#!/usr/bin/env python3
"""Only RGB -> bounded queue -> H.264 MP4. No rosbag or depth subscriptions."""
import argparse,csv,json,queue,signal,subprocess,threading,time
from datetime import datetime
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge
import yaml

class Recorder(Node):
    def __init__(self,args):
        super().__init__('record_rgb');self.args=args;self.bridge=CvBridge()
        cfg=yaml.safe_load(Path(args.config).read_text());self.fps=float(cfg['fps']);self.bitrate=str(cfg['bitrate'])
        self.queue=queue.Queue(maxsize=int(cfg['queue_size']));self.received=0;self.dropped=0;self.written=0
        self.error=None;self.session=None;self.process=None;self.segments=[];self.epoch=0;self.clock_time=None
        self.worker=threading.Thread(target=self.encode,daemon=True);self.worker.start()
        self.create_subscription(Image,'/d415/color/image_raw',self.image,qos_profile_sensor_data)
        # Read clock only to recognize reset even when Fortress pauses sensor output
        # until its previous scheduled frame time. No clock stream is recorded.
        self.create_subscription(Clock,'/clock',self.clock,1)
    def clock(self,msg):
        now=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock_time is not None and now<self.clock_time:self.epoch+=1
        self.clock_time=now
    def image(self,msg):
        self.received+=1
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.epoch and self.clock_time is not None and stamp>self.clock_time+.2:
            self.dropped+=1;return
        try:self.queue.put_nowait((self.epoch,msg))
        except queue.Full:self.dropped+=1
    def encode(self):
        start=last=None;index=0;prev=None;shape=None;file=None;writer=None;log=None;epoch=-1
        def close_segment():
            nonlocal file,log
            if self.process:
                self.process.stdin.close()
                code=self.process.wait(timeout=20)
                self.process=None
                if code:raise RuntimeError(f'ffmpeg failed: {code}')
            if file:file.close();file=None
            if log:log.close();log=None
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                current_epoch,msg=item
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                frame=self.bridge.imgmsg_to_cv2(msg,'rgb8');newshape=(msg.width,msg.height)
                if last is not None and stamp==last and epoch==current_epoch:continue
                # Reset, resolution changes or long missing intervals start new segments.
                if start is None or epoch!=current_epoch or stamp<last or stamp-last>2 or shape!=newshape:
                    close_segment()
                    if self.session is None:
                        self.session=Path(self.args.root)/datetime.now().strftime('%Y%m%d_%H%M%S_%f_rgb')
                        self.session.mkdir(parents=True,exist_ok=False)
                        print(f'RGB RECORDING: {self.session}',flush=True)
                    n=len(self.segments);name='rgb' if n==0 else f'rgb_{n:03d}'
                    log=(self.session/f'{name}_encoder.log').open('w')
                    cmd=['ffmpeg','-hide_banner','-loglevel','warning','-nostdin','-f','rawvideo','-pix_fmt','rgb24',
                         '-s',f'{msg.width}x{msg.height}','-r',str(self.fps),'-i','pipe:0','-an','-c:v','libx264',
                         '-preset','veryfast','-b:v',self.bitrate,'-pix_fmt','yuv420p','-movflags','+faststart',str(self.session/f'{name}.mp4')]
                    self.process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log,start_new_session=True)
                    file=(self.session/f'{name}_timestamps.csv').open('w');writer=csv.writer(file)
                    writer.writerow(['video_frame','simulation_time','source_time','duplicated'])
                    start=stamp;index=0;prev=None;shape=newshape;epoch=current_epoch
                    self.segments.append({'video':f'{name}.mp4','start_sim_time':stamp,'frames':0})
                target=round((stamp-start)*self.fps)
                # Fill genuine missing simulation frames, never wall-clock pauses.
                while index<target and prev is not None:
                    self.process.stdin.write(prev);writer.writerow([index,start+index/self.fps,last,True]);index+=1;self.written+=1
                if target>=index:
                    data=frame.tobytes();self.process.stdin.write(data)
                    writer.writerow([index,stamp,stamp,False]);index+=1;self.written+=1;prev=data
                last=stamp;self.segments[-1]['frames']=index;self.segments[-1]['end_sim_time']=stamp
            close_segment()
        except Exception as exc:
            self.error=str(exc)
            if self.process:
                self.process.kill();self.process.wait();self.process=None
            if file:file.close()
            if log:log.close()
    def finish(self):
        if self.worker.is_alive():
            try:self.queue.put(None,timeout=2)
            except queue.Full:self.error='Encoder queue did not drain at shutdown'
            self.worker.join(timeout=25)
        if self.worker.is_alive():
            self.error='Encoder shutdown timed out'
            if self.process:self.process.kill()
            self.worker.join(timeout=2)
        if self.session:
            report={'mode':'rgb','topic':'/d415/color/image_raw','fps':self.fps,'bitrate':self.bitrate,
                    'received':self.received,'queue_dropped':self.dropped,'written':self.written,
                    'segments':self.segments,'error':self.error,'depth_recorded':False,'clock_resets':self.epoch}
            (self.session/'recording.json').write_text(json.dumps(report,indent=2))
            print(f'RGB RECORDING CLOSED: {self.session}',flush=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',required=True);parser.add_argument('--config',required=True)
    a,rest=parser.parse_known_args();rclpy.init(args=rest);node=Recorder(a);stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    begin=time.monotonic()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.1)
            if node.error:raise RuntimeError(node.error)
            if not node.received and time.monotonic()-begin>90:raise RuntimeError('90 秒内未收到 RGB，录制未启动')
    finally:
        node.finish();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if node.error:raise RuntimeError(node.error)
if __name__=='__main__':main()
