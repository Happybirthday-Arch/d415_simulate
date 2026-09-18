#!/usr/bin/env python3
"""Record existing ROS streams with rosbag2; never replace raw depth with video."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

TOPICS = ['/d415/color/image_raw', '/d415/color/camera_info',
          '/d415/depth/image_raw', '/d415/depth/camera_info',
          '/tf', '/tf_static', '/clock']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--world', default='bridge_camera')
    parser.add_argument('--bridge-models', required=True)
    parser.add_argument('--extra-topic', action='append', default=[])
    parser.add_argument('--assembly', help='Optional moving robot assembly manifest')
    args, ros_args = parser.parse_known_args()
    topics = list(dict.fromkeys(TOPICS + args.extra_topic))
    rclpy.init(args=ros_args)
    node = Node('record_readiness')
    seen = set()
    for topic in TOPICS[:4]:
        cls = CameraInfo if topic.endswith('camera_info') else Image
        node.create_subscription(cls, topic, lambda msg, t=topic: seen.add(t), qos_profile_sensor_data)
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print('等待 RGB、Depth 和两路 CameraInfo；尚未开始录制。', flush=True)
    deadline = time.monotonic() + 90
    while len(seen) < 4 and not stop and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise RuntimeError(f'Camera not ready; missing {set(TOPICS[:4]) - seen}')
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    if stop: return
    root = Path(args.root).resolve()
    session = root / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    session.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load(Path(args.config).read_text())
    config = {'camera': cfg, 'world': args.world, 'topics': topics,
              'depth_units': '32FC1 metres; verify bag encoding',
              'upstream_commit': 'bf438856e58ec0af2ff2be04066b6942a77eadf3'}
    if args.world == 'calibration':
        config['mount_pose'] = [0., 0., 1., 0., 0., 0.]
    else:
        config['mount_pose'] = [cfg[k] for k in ('x','y','z','roll','pitch','yaw')]
    if args.assembly:
        config['assembly'] = json.loads(Path(args.assembly).read_text())
        config['mount_pose'] = config['assembly']['camera_bottom_xyz'] + [0., 0., 0.]
        config['mount_pose_reference'] = 'base_link; moving with robot'
    bridge = Path(args.bridge_models) / 'bridge/model.sdf'
    config['bridge_model_sha256'] = hashlib.sha256(bridge.read_bytes()).hexdigest()
    versions = subprocess.run(['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\n',
                               'libignition-gazebo6', 'libignition-sensors6',
                               'ros-humble-realsense2-description', 'ros-humble-ros-gz-bridge'],
                              capture_output=True, text=True)
    config['installed_versions'] = versions.stdout.splitlines()
    (session / 'run_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    qos = {topic: {'reliability': 'reliable', 'durability': 'volatile',
                   'history': 'keep_last', 'depth': 100} for topic in TOPICS[:4]}
    qos['/tf_static'] = {'reliability': 'reliable', 'durability': 'transient_local',
                         'history': 'keep_last', 'depth': 100}
    (session / 'qos.yaml').write_text(yaml.safe_dump(qos))
    cmd = ['ros2', 'bag', 'record', '--use-sim-time', '--storage', 'sqlite3',
           '--max-cache-size', '134217728', '--qos-profile-overrides-path', str(session / 'qos.yaml'),
           '-o', str(session / 'bag'), *topics]
    process = subprocess.Popen(cmd, start_new_session=True)
    print(f'RECORDING: {session}\n停止后执行 ./export.sh 导出两路视频。', flush=True)
    try:
        while process.poll() is None and not stop:
            time.sleep(0.2)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                process.terminate(); process.wait(timeout=5)
        (session / 'record_status.json').write_text(json.dumps({'returncode': process.returncode}))
        print(f'RECORDING CLOSED: {session}', flush=True)
    if process.returncode not in (0, -signal.SIGINT):
        raise SystemExit(process.returncode)


if __name__ == '__main__': main()
