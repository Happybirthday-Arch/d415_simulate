#!/usr/bin/env python3
"""Depth preview and independent-camera CameraInfo normalization (no rendering).

Fortress Sensors.cc assigns a stereo baseline to cameras sharing a rigid link.
These independent image streams are expressed in their own optical frames; the
extrinsic baseline lives in TF, so P[3] and P[7] must be zero for these topics.
Keep all native intrinsics, frame IDs and timestamps unchanged.
"""
import cv2
import numpy as np
import yaml
import rclpy
import signal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

cv2.setNumThreads(1)


def depth_preview(depth, near, far):
    valid = np.isfinite(depth) & (depth >= near) & (depth <= far)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[valid] = np.clip((depth[valid] - near) * 255 / (far - near), 0, 255).astype(np.uint8)
    rgb = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
    rgb[~valid] = 0
    return rgb


class CameraSupport(Node):
    def __init__(self):
        super().__init__('camera_support')
        self.declare_parameter('config_file', '')
        self.cfg = yaml.safe_load(open(self.get_parameter('config_file').value))
        self.cfg = self.cfg.get('camera', self.cfg)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/d415/depth/preview', qos_profile_sensor_data)
        for stream in ('color', 'depth'):
            pub = self.create_publisher(CameraInfo, f'/d415/{stream}/camera_info', 30)
            self.create_subscription(CameraInfo, f'/d415/native/{stream}/camera_info',
                                     lambda msg, p=pub: self.normalize_info(msg, p), 30)
        self.create_subscription(Image, '/d415/depth/image_raw', self.callback, qos_profile_sensor_data)

    def normalize_info(self, msg, pub):
        msg.p[3] = 0.0
        msg.p[7] = 0.0
        pub.publish(msg)

    def callback(self, msg):
        if self.pub.get_subscription_count() == 0:
            return
        data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == '16UC1':
            data = data.astype(np.float32) * 0.001
        elif msg.encoding != '32FC1':
            self.get_logger().error(f'Unsupported depth encoding: {msg.encoding}')
            return
        preview = self.bridge.cv2_to_imgmsg(depth_preview(data, self.cfg['depth_near'], self.cfg['depth_far']), encoding='bgr8')
        preview.header = msg.header
        self.pub.publish(preview)


def main():
    rclpy.init()
    node = CameraSupport()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch may forward SIGINT after the terminal has already delivered it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
