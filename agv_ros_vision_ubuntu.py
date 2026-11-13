#!/usr/bin/env python3
"""Native ROS 2 Jazzy AGV waste vision on Ubuntu Server/Raspberry Pi 5.

Captures headless 640x480 MJPEG frames using the already tested rpicam-apps
camera stack (no Picamera2 bindings, Docker, or local display required).
Runs the exact trained model through agv_waste_yolo26.process and publishes:
  /camera/image_raw      sensor_msgs/msg/Image, bgr8
  /waste/annotated_image sensor_msgs/msg/Image, bgr8
  /waste/detections      std_msgs/msg/String JSON, unchanged from old bridge
  /waste/class           std_msgs/msg/String with plastic/organic/metal/none

Start after `source /opt/ros/jazzy/setup.bash` and activating the AGV venv.
Stop with Ctrl+C. Publish images using --no-images false (default).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from agv_waste_yolo26 import CLASSES, open_model, process


def mjpeg_frames(stream, process_handle):
    """Yield JPEG payloads from an MJPEG byte stream, with a bounded buffer."""
    buffer = bytearray()
    soi = b'\xff\xd8'
    eoi = b'\xff\xd9'
    while True:
        chunk = os.read(stream.fileno(), 65536)
        if not chunk:
            if process_handle.poll() is None:
                raise RuntimeError('Camera closed its video stream unexpectedly')
            raise RuntimeError(f'Camera stopped with exit code {process_handle.returncode}')
        buffer.extend(chunk)
        while True:
            start = buffer.find(soi)
            if start == -1:
                # Keep one byte for a marker that spans chunks.
                if len(buffer) > 1:
                    del buffer[:-1]
                break
            if start:
                del buffer[:start]
            end = buffer.find(eoi, 2)
            if end == -1:
                if len(buffer) > 8_000_000:
                    raise RuntimeError('MJPEG frame exceeded 8 MB')
                break
            jpeg = bytes(buffer[:end + 2])
            del buffer[:end + 2]
            yield jpeg


def image_message(bgr, stamp):
    h, w = bgr.shape[:2]
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = 'camera_optical_frame'
    msg.height = h
    msg.width = w
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg


def choose_waste_label(detections, margin: float = 0.15) -> str:
    """Choose one safe label. If detections disagree strongly, return 'none'.

    detections: iterable of (class_id, confidence, xyxy), as returned by
    agv_waste_yolo26.process().  No object also returns 'none'.
    """
    if not detections:
        return 'none'
    best_by_class = {}
    for class_id, score, _rect in detections:
        key = int(class_id)
        if key not in range(len(CLASSES)):
            continue
        best_by_class[key] = max(best_by_class.get(key, 0.0), float(score))
    if not best_by_class:
        return 'none'
    ranked = sorted(best_by_class.items(), key=lambda x: x[1], reverse=True)
    if len(ranked) > 1 and ranked[0][1] - ranked[1][1] < margin:
        return 'none'  # Overlapping conflicting classes: do not guess.
    return CLASSES[ranked[0][0]]


def main():
    parser = argparse.ArgumentParser(description='Headless AGV camera + ONNX -> native ROS 2')
    parser.add_argument('--model', type=Path,
                        default=Path.home() / 'agv_project/models/waste.onnx')
    parser.add_argument('--fps', type=float, default=3.0,
                        help='Camera MJPEG target FPS; start at 3 for Pi 5 CPU budget')
    parser.add_argument('--conf', type=float, default=0.35)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--class-margin', type=float, default=0.15,
                        help='Confidence lead needed between conflicting classes')
    parser.add_argument('--print-labels', action='store_true',
                        help='Print just the class label for each processed camera frame')
    parser.add_argument('--no-images', action='store_true',
                        help='Publish only detections; reduce CPU and ROS traffic')
    args = parser.parse_args()
    if args.fps <= 0 or not (0 < args.conf <= 1 and 0 < args.iou <= 1) or args.class_margin < 0:
        parser.error('fps must be positive, conf and iou in (0,1], class-margin >= 0')
    if not args.model.is_file():
        parser.error(f'Cannot find ONNX model: {args.model}')
    command = shutil.which('rpicam-vid')
    if command is None:
        parser.error('rpicam-vid missing; check rpicam-apps-rpi installation')

    session, input_name = open_model(args.model)

    rclpy.init()
    node = Node('agv_waste_camera_vision')
    raw_pub = None
    annotated_pub = None
    if not args.no_images:
        raw_pub = node.create_publisher(Image, '/camera/image_raw', 10)
        annotated_pub = node.create_publisher(Image, '/waste/annotated_image', 10)
    detection_pub = node.create_publisher(String, '/waste/detections', 10)
    class_pub = node.create_publisher(String, '/waste/class', 10)

    cmd = [command, '--nopreview', '--timeout', '0', '--width', '640',
           '--height', '480', '--framerate', str(args.fps), '--codec', 'mjpeg',
           '--output', '-']
    log_dir = Path.home() / 'agv_project/logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    camera_log = log_dir / 'rpicam_ubuntu.log'
    camera_process = None
    count = 0
    begun = time.monotonic()
    try:
        with camera_log.open('ab', buffering=0) as log:
            camera_process = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                              stderr=log, bufsize=0)
            if camera_process.stdout is None:
                raise RuntimeError('Could not open camera output')
            node.get_logger().info(f'Capturing OV5647 MJPEG at {args.fps:g} fps; log: {camera_log}')
            node.get_logger().info('Publishing /waste/detections, /waste/class'
                                   + ('' if args.no_images else ', /camera/image_raw, /waste/annotated_image'))
            for jpeg in mjpeg_frames(camera_process.stdout, camera_process):
                if not rclpy.ok():
                    break
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    node.get_logger().warning('Could not decode an MJPEG frame')
                    continue
                annotated, detections, infer_ms = process(
                    frame, session, input_name, args.conf, args.iou)
                stamp = node.get_clock().now().to_msg()

                metadata = {
                    'detections': [
                        {'class_id': int(cls), 'class': CLASSES[cls],
                         'confidence': round(float(score), 4),
                         'xyxy': [int(x) for x in rect]}
                        for cls, score, rect in detections
                    ],
                    'inference_ms': round(float(infer_ms), 1),
                }
                event = String()
                event.data = json.dumps(metadata, separators=(',', ':'))
                detection_pub.publish(event)

                # A simple, independent ROS topic for the behavior controller.
                # Each frame gives EXACTLY one of: metal, organic, plastic, none.
                label = choose_waste_label(detections, args.class_margin)
                class_msg = String()
                class_msg.data = label
                class_pub.publish(class_msg)
                if args.print_labels:
                    print(label, flush=True)

                if not args.no_images:
                    raw_pub.publish(image_message(frame, stamp))
                    cv2.putText(annotated, f'ONNX: {infer_ms:.0f} ms',
                                (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    annotated_pub.publish(image_message(annotated, stamp))
                count += 1
                if count % 30 == 0:
                    duration = time.monotonic() - begun
                    node.get_logger().info(
                        f'Processed {count} frames, avg {count / duration:.2f} FPS; '
                        f'last inference {infer_ms:.1f} ms, objects {len(detections)}')
    except KeyboardInterrupt:
        node.get_logger().info('Stopping vision node')
    finally:
        if camera_process is not None:
            if camera_process.stdout is not None:
                camera_process.stdout.close()
            if camera_process.poll() is None:
                camera_process.terminate()
                try:
                    camera_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    camera_process.kill()
                    camera_process.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as ex:
        print(f'AGV vision error: {ex}', file=sys.stderr)
        raise
