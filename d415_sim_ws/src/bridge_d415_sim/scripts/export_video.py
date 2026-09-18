#!/usr/bin/env python3
"""Export timestamp-resampled previews from a closed rosbag; raw data stays intact."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera_support import depth_preview


class Video:
    def __init__(self, path, width, height, fps):
        self.path = path
        self.tmp = path.with_name(path.stem + '.partial.mp4')
        self.fps = fps
        self.proc = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '18', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(self.tmp)], stdin=subprocess.PIPE)
        self.first = self.previous_stamp = None
        self.last = None
        self.frames = self.sources = 0

    def add(self, frame, stamp):
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            raise ValueError('Non-increasing image timestamp; split recording at simulation reset.')
        if self.first is None:
            self.first = stamp
        if self.last is not None:
            # Hold last sample across gaps, preserving simulated elapsed time.
            while self.first + round(self.frames * 1e9 / self.fps) < stamp:
                self.proc.stdin.write(self.last.tobytes()); self.frames += 1
        self.last = np.ascontiguousarray(frame)
        self.previous_stamp = stamp
        self.sources += 1

    def close(self):
        if self.last is not None:
            self.proc.stdin.write(self.last.tobytes()); self.frames += 1
        self.proc.stdin.close()
        if self.proc.wait() != 0: raise RuntimeError('FFmpeg encoding failed')
        self.tmp.replace(self.path)
        return {'source_frames': self.sources, 'video_frames': self.frames,
                'first_stamp_ns': self.first, 'last_stamp_ns': self.previous_stamp,
                'video_fps': self.fps}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    session = args.session.resolve()
    if not (session / 'bag/metadata.yaml').is_file():
        raise RuntimeError('Bag must be stopped and metadata.yaml present before export.')
    for name in ['rgb.mp4', 'depth_preview.mp4']:
        if (session / name).exists() and not args.force:
            raise FileExistsError(f'{name} exists; use --force to replace derived videos.')
    cfg = yaml.safe_load((session / 'run_config.yaml').read_text())['camera']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    bridge = CvBridge(); writers = {}; stats = {}
    topics = {'/d415/color/image_raw': 'rgb', '/d415/depth/image_raw': 'depth_preview'}
    try:
        with (session / 'frame_timestamps.csv').open('w', newline='') as f:
            csvout = csv.writer(f); csvout.writerow(['topic','source_frame','header_stamp_ns','bag_stamp_ns'])
            while reader.has_next():
                topic, data, bag_stamp = reader.read_next()
                if topic not in topics: continue
                msg = deserialize_message(data, Image)
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                name = topics[topic]
                if name not in writers:
                    writers[name] = Video(session / (name + '.mp4'), msg.width, msg.height, cfg['fps'])
                if name == 'rgb':
                    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                else:
                    depth = bridge.imgmsg_to_cv2(msg, 'passthrough')
                    if msg.encoding == '16UC1': depth = depth.astype(np.float32) * 0.001
                    elif msg.encoding != '32FC1': raise ValueError(msg.encoding)
                    frame = depth_preview(depth, cfg['depth_near'], cfg['depth_far'])
                csvout.writerow([topic, writers[name].sources, stamp, bag_stamp])
                writers[name].add(frame, stamp)
        if len(writers) != 2: raise RuntimeError('Both RGB and depth images are required.')
        for name, writer in writers.items(): stats[name] = writer.close()
        (session / 'export_report.json').write_text(json.dumps(stats, indent=2))
        print(f'视频与时间戳已导出：{session}')
    finally:
        for writer in writers.values():
            if writer.proc.poll() is None:
                writer.proc.terminate(); writer.proc.wait()


if __name__ == '__main__': main()
