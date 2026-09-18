#!/usr/bin/env python3
"""Broadcast actual dynamic robot TF and generated static camera extrinsics."""
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

class StateTF(Node):
    def __init__(self,manifest):
        super().__init__('robot_state_tf');self.cfg=json.load(open(manifest))
        self.dynamic=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        frames=[self.tf(x['parent'],x['child'],x['xyz'],x['quat'],self.get_clock().now().to_msg()) for x in self.cfg['static_tf']]
        self.static.sendTransform(frames)
        self.create_subscription(PoseStamped,'/robot/ground_truth',self.pose,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
    def tf(self,parent,child,xyz,quat,stamp):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
        t.transform.translation.x=float(xyz[0]);t.transform.translation.y=float(xyz[1]);t.transform.translation.z=float(xyz[2])
        t.transform.rotation.x=float(quat[0]);t.transform.rotation.y=float(quat[1]);t.transform.rotation.z=float(quat[2]);t.transform.rotation.w=float(quat[3]);return t
    def pose(self,msg):
        p=msg.pose.position;q=msg.pose.orientation
        self.dynamic.sendTransform(self.tf('world','base_link',[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],msg.header.stamp))
    def joints(self,msg):
        q=dict(zip(msg.name,msg.position));orig=self.cfg['link_origins'];frames=[]
        for parent,child,joint,axis in [('base_link','front_steering_link','front_steering_joint','z'),
                                      ('base_link','rear_wheel_link','rear_wheel_joint','y'),
                                      ('front_steering_link','front_wheel_link','front_wheel_joint','y')]:
            if joint not in q:return
            delta=np.array(orig[child])-np.array(orig[parent]);quat=Rotation.from_euler(axis,q[joint]).as_quat()
            frames.append(self.tf(parent,child,delta,quat,msg.header.stamp))
        self.dynamic.sendTransform(frames)

def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',required=True);a,rest=p.parse_known_args()
    rclpy.init(args=rest);node=StateTF(a.manifest)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
