#!/usr/bin/env python3
"""Focused key-down/up WASD control. Wall-clock heartbeat, no terminal repeat reliance."""
import signal,time,tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

class Keyboard:
    def __init__(self):
        self.node=Node('keyboard_drive');self.pub=self.node.create_publisher(Vector3,'/robot/drive_command',1)
        self.root=tk.Tk();self.root.title('小车 WASD 控制');self.root.geometry('520x290')
        self.keys=set();self.suppressed=set();self.pending={};self.locked=False;self.feedback=None;self.last_feedback=0;self.joint_text='等待关节状态'
        self.node.create_subscription(Vector3,'/robot/drive_status',self.status,qos_profile_sensor_data)
        self.node.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        tk.Label(self.root,text='W/S 前进、后退　A/D 前轮左打、右打',font=('',15)).pack(pady=15)
        tk.Label(self.root,text='按住行驶，松键停车；支持 W+A 等组合键\nSpace 制动锁定；松开 WASD 后 Enter 解锁\n窗口失焦即停车。倒车时车头偏航方向相反。').pack()
        self.label=tk.Label(self.root,text='',font=('',12));self.label.pack(pady=15)
        self.root.bind('<KeyPress>',self.press);self.root.bind('<KeyRelease>',self.release)
        self.root.bind('<FocusOut>',lambda e:self.root.after_idle(self.focus_check))
        self.root.protocol('WM_DELETE_WINDOW',self.close);self.running=True;self.root.after(20,self.tick)
    def status(self,msg):
        if msg.z>.5 and not self.locked:
            self.suppressed.update(self.keys);self.keys.clear()
        self.feedback=msg;self.last_feedback=time.monotonic()
    def joints(self,msg):
        import math
        v=dict(zip(msg.name,msg.velocity));p=dict(zip(msg.name,msg.position))
        self.joint_text=f"实际轮速 前 {v.get('front_wheel_joint',0)*30/math.pi:.2f} / 后 {v.get('rear_wheel_joint',0)*30/math.pi:.2f} rpm\n实际转角 {p.get('front_steering_joint',0)*180/math.pi:.2f}°"
    def press(self,event):
        key=event.keysym.lower()
        if key in self.pending:self.root.after_cancel(self.pending.pop(key))
        if key=='space':
            self.suppressed.update(self.keys);self.keys.clear();self.locked=True
        elif key=='return' and not self.keys and not self.suppressed:self.locked=False
        elif key in 'wasd' and len(key)==1 and key not in self.suppressed:self.keys.add(key)
    def release(self,event):
        key=event.keysym.lower()
        def finish():self.pending.pop(key,None);self.keys.discard(key);self.suppressed.discard(key)
        if key in self.pending:self.root.after_cancel(self.pending[key])
        self.pending[key]=self.root.after_idle(finish)
    def focus_check(self):
        if self.root.focus_get() is None:
            self.suppressed.update(self.keys);self.keys.clear()
    def tick(self):
        if not self.running:return
        rclpy.spin_once(self.node,timeout_sec=0)
        connected=time.monotonic()-self.last_feedback<2.0
        x=float(('w' in self.keys)-('s' in self.keys));y=float(('a' in self.keys)-('d' in self.keys))
        # Slow rendering is not a key release. Actual pause/reset/timeout is
        # reported by the controller; focus loss is handled independently.
        if self.locked or self.feedback is None:x=y=0.
        self.pub.publish(Vector3(x=x,y=y,z=float(self.locked)))
        state='制动锁定' if self.locked else ('已连接' if connected else '等待仿真 / 已暂停')
        target=f'目标 {self.feedback.x:.2f} rpm / {self.feedback.y:.1f}°' if self.feedback else ''
        self.label.config(text=f'{state}　按键：{" ".join(sorted(self.keys)) or "无"}\n{target}\n{self.joint_text}')
        self.root.after(20,self.tick)
    def close(self):
        if not self.running:return
        self.running=False;self.pub.publish(Vector3(z=1.));self.root.destroy()
    def run(self):
        try:self.root.mainloop()
        finally:
            self.pub.publish(Vector3(z=1.));self.node.destroy_node()

def main():
    rclpy.init();ui=Keyboard()
    signal.signal(signal.SIGINT,lambda *_:ui.close())
    signal.signal(signal.SIGTERM,lambda *_:ui.close())
    try:ui.run()
    except KeyboardInterrupt:ui.close()
    finally:
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
