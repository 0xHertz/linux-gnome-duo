#!/usr/bin/env python3
"""摄像头开合角检测守护进程 (camera-based lid-fold angle daemon).

打开笔记本摄像头检测屏幕开合：
1. 检测到人脸 → 屏幕正对用户（展开），不折叠（角度 1.0）。
2. 无人脸时，用「相对亮度 + 自适应基线」判断：展开时画面较亮，合拢进入
   屏幕与键盘间暗区时变暗，当前亮度相对历史最亮基线的下降比例决定合拢程度。
角度经 Unix 域套接字以 ~60 FPS 广播给 GNOME Shell 扩展。
"""

import os
import signal
import sys
import threading
import time

import cv2
import numpy as np

from common import SOCKET_PATH, serve, setup_socket

FRAME_INTERVAL = 0.05
SMOOTH_ALPHA = 0.03
FACE_ALPHA = 0.3
BASELINE_DECAY = 0.001
DROP_FLOOR = 0.35
EPS = 1e-6
CAMERA_INDEX = 0
DEBUG_INTERVAL = 0.5


class CameraTracker:
    def __init__(self, debug=False):
        self._lock = threading.Lock()
        self._smooth = 1.0
        self.current_angle = 1.0
        self._baseline = 0.0
        self._cascade = self._load_cascade()
        if self._cascade is None:
            print("[camera_daemon] 警告: 无法加载人脸检测级联文件", file=sys.stderr)
        self._debug = debug
        self._last_debug = 0.0

    @staticmethod
    def _load_cascade():
        paths = []
        try:
            paths.append(
                os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
            )
        except AttributeError:
            pass
        paths.extend(
            [
                "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
                "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            ]
        )
        for path in paths:
            if os.path.isfile(path):
                return cv2.CascadeClassifier(path)
        return None

    def process(self, frame):
        small = cv2.resize(frame, None, fx=0.5, fy=0.5)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray)) / 255.0
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        self._baseline = max(brightness, self._baseline * (1.0 - BASELINE_DECAY))

        has_face = False
        if self._cascade is not None:
            faces = self._cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=4, minSize=(24, 24)
            )
            has_face = len(faces) > 0

        if has_face:
            raw = 1.0
            alpha = FACE_ALPHA
        else:
            ratio = brightness / (self._baseline + EPS)
            raw = np.clip((ratio - DROP_FLOOR) / (1.0 - DROP_FLOOR), 0.0, 1.0)
            alpha = SMOOTH_ALPHA

        self._smooth = self._smooth * (1.0 - alpha) + raw * alpha
        with self._lock:
            self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

        if self._debug:
            now = time.monotonic()
            if now - self._last_debug >= DEBUG_INTERVAL:
                self._last_debug = now
                print(
                    f"[camera_daemon] face={int(has_face)} brightness={brightness:.3f} "
                    f"baseline={self._baseline:.3f} sharpness={sharpness:.1f} "
                    f"angle={self.current_angle:.3f}",
                    file=sys.stderr,
                )

    def set_angle(self, val):
        val = float(val)
        with self._lock:
            self._smooth = val
            self.current_angle = val

    def get_angle(self):
        with self._lock:
            return self.current_angle


def main():
    debug = "--debug" in sys.argv
    stop_event = threading.Event()

    def _request_stop(signum, frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[camera_daemon] 无法打开摄像头 /dev/video{CAMERA_INDEX}", file=sys.stderr)
        return

    server = setup_socket()
    tracker = CameraTracker(debug=debug)

    threading.Thread(
        target=serve, args=(server, tracker, stop_event), daemon=True
    ).start()

    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if ok:
                tracker.process(frame)
            time.sleep(FRAME_INTERVAL)
    finally:
        stop_event.set()
        cap.release()
        server.close()
        try:
            os.remove(SOCKET_PATH)
        except OSError:
            pass


if __name__ == "__main__":
    main()
