#!/usr/bin/env python3
"""Serve the manual-assisted dashboard and MJPEG camera streams."""

import argparse
import os
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, send_file


app = Flask(__name__)
CONFIG = None


def parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def dashboard_path():
    installed = Path(get_package_share_directory("sub_hardware")) / "web" / "manual_assisted_dashboard.html"
    if installed.exists():
        return installed

    source = Path(__file__).resolve().parent.parent / "web" / "manual_assisted_dashboard.html"
    if source.exists():
        return source
    raise FileNotFoundError("manual_assisted_dashboard.html is not installed")


def placeholder_frame(camera_index, message):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, f"Camera {camera_index}", (36, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (230, 230, 230), 2)
    cv2.putText(frame, message, (36, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 170, 255), 2)
    return frame


def open_camera(camera_index):
    camera = cv2.VideoCapture(camera_index)
    if camera.isOpened():
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return camera
    camera.release()
    return None


def encode_mjpeg(frame):
    success, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), CONFIG.jpeg_quality],
    )
    if not success:
        return None
    return b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"


def save_frame(frame, camera_index):
    camera_dir = Path(CONFIG.output) / f"camera_{camera_index}" / CONFIG.label
    camera_dir.mkdir(parents=True, exist_ok=True)
    filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")[:-3] + ".jpg"
    cv2.imwrite(str(camera_dir / filename), frame)


def gen_frames(camera_index):
    camera = None
    frame_counter = 0
    retry_at = 0.0

    try:
        while True:
            now = time.monotonic()
            if camera is None and now >= retry_at:
                camera = open_camera(camera_index)
                retry_at = now + CONFIG.camera_retry_sec

            success = False
            frame = None
            if camera is not None:
                success, frame = camera.read()
                if not success:
                    camera.release()
                    camera = None
                    retry_at = now + CONFIG.camera_retry_sec

            if not success:
                frame = placeholder_frame(camera_index, "Unavailable - retrying")
            else:
                frame_counter += 1
                if CONFIG.save_video and frame_counter % CONFIG.save_every_n_frames == 0:
                    save_frame(frame, camera_index)

            encoded = encode_mjpeg(frame)
            if encoded is not None:
                yield encoded
            time.sleep(CONFIG.frame_interval)
    finally:
        if camera is not None:
            camera.release()


@app.route("/")
def index():
    return send_file(dashboard_path())


@app.route("/video_feed1")
def video_feed1():
    return Response(
        gen_frames(CONFIG.camera_1),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/video_feed2")
def video_feed2():
    return Response(
        gen_frames(CONFIG.camera_2),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/health")
def health():
    return jsonify(
        status="ok",
        camera_1=CONFIG.camera_1,
        camera_2=CONFIG.camera_2,
        save_video=CONFIG.save_video,
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-1", type=int, default=0)
    parser.add_argument("--camera-2", type=int, default=4)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=6969)
    parser.add_argument("--save-video", type=parse_bool, default=False)
    parser.add_argument("--output", default="/workspace/output/manual_assisted")
    parser.add_argument("--label", default="pool-test")
    parser.add_argument("--frame-interval", type=float, default=0.13)
    parser.add_argument("--camera-retry-sec", type=float, default=1.0)
    parser.add_argument("--save-every-n-frames", type=int, default=5)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args, _ = parser.parse_known_args()

    args.frame_interval = max(0.02, args.frame_interval)
    args.camera_retry_sec = max(0.2, args.camera_retry_sec)
    args.save_every_n_frames = max(1, args.save_every_n_frames)
    args.jpeg_quality = min(100, max(20, args.jpeg_quality))
    os.makedirs(args.output, exist_ok=True)
    return args


def main():
    global CONFIG
    CONFIG = parse_args()
    app.run(host=CONFIG.host, port=CONFIG.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
