#!/usr/bin/env python3
"""Measure live streams, geometry, and TF; write evidence instead of assuming 30 Hz."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plane-distance', type=float)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = Node('d415_validation')
    bridge = CvBridge(); stamps = {}; latest = {}; first_wall = {}
    types = {'color/image_raw': Image, 'depth/image_raw': Image,
             'color/camera_info': CameraInfo, 'depth/camera_info': CameraInfo,
             'depth/points': PointCloud2}
    def receive(msg, key):
        latest[key] = msg
        first_wall.setdefault(key, time.monotonic())
        stamps.setdefault(key, []).append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    for key, cls in types.items():
        qos = qos_profile_sensor_data if cls == PointCloud2 else 100
        node.create_subscription(cls, '/d415/' + key, lambda msg, k=key: receive(msg, k), qos)
    tf = Buffer(); listener = TransformListener(tf, node)
    start = time.monotonic(); ready = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if len(latest) == len(types) and ready is None: ready = now
        if ready is not None and now - ready >= args.seconds: break
        if ready is None and now - start > 45: break
    report = {'wall_seconds': time.monotonic()-start, 'streams': {}, 'errors': []}
    for key in types:
        ss = stamps.get(key, [])
        if len(ss) < 2:
            report['errors'].append('Missing stream: ' + key); continue
        msg = latest[key]; delta = np.diff(ss)
        report['streams'][key] = {'count': len(ss), 'frame_id': msg.header.frame_id,
            'sim_hz': (len(ss)-1)/(ss[-1]-ss[0]) if ss[-1] > ss[0] else 0,
            'wall_hz': (len(ss)-1)/(time.monotonic()-first_wall[key]),
            'max_gap_sim_s': float(max(delta)), 'nonincreasing_stamps': int(np.count_nonzero(delta <= 0))}
        expected = 'd415_color_optical_frame' if key.startswith('color') else 'd415_depth_optical_frame'
        if msg.header.frame_id != expected: report['errors'].append(f'{key}: wrong frame {msg.header.frame_id}')
        if hasattr(msg, 'width'):
            report['streams'][key]['size'] = [msg.width, msg.height]
            if [msg.width,msg.height] != [640,360]: report['errors'].append(key + ': wrong size')
    for kind in ['color','depth']:
        if kind+'/image_raw' not in latest: continue
        msg=latest[kind+'/image_raw']; report['streams'][kind+'/image_raw']['encoding']=msg.encoding
        a=bridge.imgmsg_to_cv2(msg, 'bgr8' if kind=='color' else 'passthrough')
        if kind=='color': cv2.imwrite(str(args.output/'rgb.png'), a)
        else:
            if msg.encoding=='16UC1': a=a.astype(np.float32)*0.001
            np.save(args.output/'depth.npy', a)
            valid=np.isfinite(a) & (a > 0)
            report['depth_valid_fraction']=float(valid.mean())
            center=a[a.shape[0]//2-5:a.shape[0]//2+5,a.shape[1]//2-5:a.shape[1]//2+5]
            med=float(np.median(center[np.isfinite(center)])) if np.isfinite(center).any() else None
            report['center_depth_m']=med
            if args.plane_distance is not None and (med is None or abs(med-args.plane_distance)>0.02):
                report['errors'].append('Known-plane center depth error > 2 cm')
        info=latest.get(kind+'/camera_info')
        if info:
            report[kind+'_intrinsics']={'k': list(info.k), 'p':list(info.p)}
            if info.p[3] != 0 or info.p[7] != 0:
                report['errors'].append(kind + ': unexpected projection translation')
    if 'depth/points' in latest:
        cloud = latest['depth/points']
        offsets = {field.name: field.offset for field in cloud.fields}
        dtype = np.dtype({'names': ['x','y','z'], 'formats': ['<f4']*3,
                          'offsets': [offsets[k] for k in ('x','y','z')], 'itemsize':cloud.point_step})
        points = np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,
                            strides=(cloud.row_step,cloud.point_step))
        center = points[cloud.height//2,cloud.width//2]
        report['point_cloud_center_xyz'] = [float(center[k]) if np.isfinite(center[k]) else None for k in ('x','y','z')]
        if args.plane_distance is not None:
            if abs(float(center['x'])) > 0.002 or abs(float(center['y'])) > 0.002 or abs(float(center['z'])-args.plane_distance)>0.02:
                report['errors'].append('Point cloud optical center is incorrect')
            if not (points['x'][cloud.height//2,-1]>0 and points['y'][-1,cloud.width//2]>0):
                report['errors'].append('Point cloud optical axes are incorrect')
    try:
        transform=tf.lookup_transform('world','d415_depth_optical_frame',rclpy.time.Time())
        report['world_to_depth']={'translation':str(transform.transform.translation),
                                  'rotation':str(transform.transform.rotation)}
    except Exception as e: report['errors'].append('TF: '+str(e))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(report,indent=2,allow_nan=False))
    node.destroy_node();rclpy.shutdown()
    if report['errors']:raise SystemExit(1)


if __name__=='__main__': main()
