#!/usr/bin/env python3
"""摄像头开合角检测守护进程 (camera-based lid-fold angle daemon).

打开笔记本摄像头检测屏幕开合：
1. 检测到人脸 → 快速恢复到完全展开状态（角度 1.0）。
2. 无人脸时 → 在人脸区域之外选择多个特征点，通过中位数纵向位移计算屏幕开合程度。
3. 方案2：引入边缘预警与多点无缝基准继承 (Baseline Handover)，当点靠近边缘或移出画面时，
   自动补点并继承之前的位移历史，彻底解决特征点全灭导致的角度跳变。
角度经 Unix 域套接字广播给 GNOME Shell 扩展。
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
SMOOTH_ALPHA = 0.04
FACE_ALPHA = 0.5
CAMERA_INDEX = 0
DEBUG_INTERVAL = 0.5


class CameraTracker:
    """
    通过“人脸区域以外的多个特征点”的纵向位移估算屏幕开合程度。
    具备多点中位数抗噪与无缝基准继承 (Seamless Handover) 功能。
    """

    def __init__(self, debug=False):
        self._lock = threading.Lock()

        self._smooth = 1.0
        self.current_angle = 1.0

        self._cascade = self._load_cascade()
        if self._cascade is None:
            print(
                "[camera_daemon] 警告: 无法加载人脸检测级联文件",
                file=sys.stderr,
            )

        self._debug = debug
        self._last_debug = 0.0

        # 光流多点状态
        self._prev_gray = None
        self._points = None  # shape: (N, 1, 2)
        self._base_ys = None  # shape: (N,)

        # 记录最后一次有效的纵向位移 (dy)，用于无缝继承
        self._last_dy = 0.0

        # 达到完全折叠时所对应的纵向像素位移
        self._max_vertical_displacement = 500.0

        # 光流参数
        self._lk_win_size = (21, 21)
        self._lk_max_level = 3

        # 多点跟踪参数
        self._max_corners = 15  # 最多寻找特征点数
        self._min_corners = 3  # 触发补点的最小有效点数

    @staticmethod
    def _load_cascade():
        paths = []
        try:
            paths.append(
                os.path.join(
                    cv2.data.haarcascades,
                    "haarcascade_frontalface_default.xml",
                )
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
                cascade = cv2.CascadeClassifier(path)
                if not cascade.empty():
                    return cascade

        return None

    def _detect_face_mask(self, gray):
        """返回人脸区域 mask，人脸只用于排除。"""
        mask = np.zeros_like(gray)
        if self._cascade is None:
            return mask

        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=4,
            minSize=(24, 24),
        )

        for x, y, w, h in faces:
            pad_x = int(w * 0.20)
            pad_y = int(h * 0.25)

            x0 = max(0, x - pad_x)
            y0 = max(0, y - pad_y)
            x1 = min(gray.shape[1], x + w + pad_x)
            y1 = min(gray.shape[0], y + h + pad_y)

            mask[y0:y1, x0:x1] = 255

        return mask

    def _find_tracking_points(self, gray, face_mask):
        """在人脸之外寻找多点，增加 10% 的边缘安全缓冲区。"""
        h, w = gray.shape
        feature_mask = np.full_like(gray, 255)

        # 设置边缘安全缓冲区 (10%)，提前排除快要离开画面的边缘区域
        margin_x = max(12, int(w * 0.10))
        margin_y = max(12, int(h * 0.10))

        feature_mask[:margin_y, :] = 0
        feature_mask[h - margin_y :, :] = 0
        feature_mask[:, :margin_x] = 0
        feature_mask[:, w - margin_x :] = 0

        # 排除人脸区域
        feature_mask[face_mask > 0] = 0

        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self._max_corners,
            qualityLevel=0.01,
            minDistance=15,
            blockSize=15,
            mask=feature_mask,
            useHarrisDetector=False,
        )

        return points  # N x 1 x 2 或 None

    def _reset_tracking(self, gray, face_mask=None, inherit_dy=0.0):
        """
        重新寻找特征点。
        如果指定了 inherit_dy，则新特征点的 base_y 将继承之前的位移历史 (Handover)。
        """
        if face_mask is None:
            face_mask = self._detect_face_mask(gray)

        points = self._find_tracking_points(gray, face_mask)

        if points is None or len(points) == 0:
            self._points = None
            self._base_ys = None
            return False

        self._points = points.astype(np.float32)
        # 核心继承逻辑: base_y = current_y - dy
        current_ys = points[:, 0, 1]
        self._base_ys = current_ys - inherit_dy
        self._prev_gray = gray.copy()
        return True

    def _calculate_angle_from_displacement(self, dy):
        """根据纵向位移 dy 计算开合比例 0.0 ~ 1.0。"""
        displacement = abs(dy)
        ratio = np.clip(
            displacement / self._max_vertical_displacement,
            0.0,
            1.0,
        )
        fold = ratio * ratio * (3.0 - 2.0 * ratio)
        return float(np.clip(1.0 - fold, 0.0, 1.0))

    def process(self, frame):
        small = cv2.resize(
            frame,
            None,
            fx=0.5,
            fy=0.5,
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        face_mask = self._detect_face_mask(gray)
        has_face = np.count_nonzero(face_mask) > (gray.shape[0] * gray.shape[1] * 0.02)

        # ============================================================
        # 1. 人脸检测最高优先级
        # ============================================================
        if has_face:
            self._smooth = self._smooth * (1.0 - FACE_ALPHA) + 1.0 * FACE_ALPHA
            with self._lock:
                self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

            self._last_dy = 0.0  # 人脸出现归位，位移重置为 0

            if self._points is None or self._base_ys is None:
                self._prev_gray = gray.copy()
                self._reset_tracking(gray, face_mask, inherit_dy=0.0)
            else:
                # 人脸期间持续维护已有角点的光流跟踪，但不更新 _base_ys
                next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                    self._prev_gray,
                    gray,
                    self._points,
                    None,
                    winSize=self._lk_win_size,
                    maxLevel=self._lk_max_level,
                    criteria=(
                        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                        20,
                        0.03,
                    ),
                )

                valid_pts = []
                valid_bases = []
                if next_points is not None and status is not None:
                    for i, pt in enumerate(next_points):
                        if status[i, 0] == 1:
                            px, py = pt[0]
                            if 0 <= px < w and 0 <= py < h:
                                valid_pts.append(pt)
                                valid_bases.append(self._base_ys[i])

                if len(valid_pts) > 0:
                    self._points = np.array(valid_pts, dtype=np.float32)
                    self._base_ys = np.array(valid_bases, dtype=np.float32)
                else:
                    self._reset_tracking(gray, face_mask, inherit_dy=0.0)

                self._prev_gray = gray.copy()

            if self._debug:
                now = time.monotonic()
                if now - self._last_debug >= DEBUG_INTERVAL:
                    self._last_debug = now
                    print(
                        f"[camera_daemon] face detected -> fast recovery, angle={self.current_angle:.3f}",
                        file=sys.stderr,
                    )
            return

        # ============================================================
        # 2. 无人脸：多点跟踪与无缝基准继承 (方案2)
        # ============================================================
        if self._prev_gray is None or self._points is None or self._base_ys is None:
            self._prev_gray = gray.copy()
            if not self._reset_tracking(gray, face_mask, inherit_dy=self._last_dy):
                return
            return

        # 计算 LK 光流
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray,
            self._points,
            None,
            winSize=self._lk_win_size,
            maxLevel=self._lk_max_level,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )

        valid_pts = []
        valid_bases = []
        valid_dys = []

        margin_x = int(w * 0.05)
        margin_y = int(h * 0.05)

        if next_points is not None and status is not None:
            for i, pt in enumerate(next_points):
                if status[i, 0] == 1:
                    px, py = pt[0]
                    # 剔除移出画面或进入安全边界外的点
                    if (
                        margin_x <= px < w - margin_x
                        and margin_y <= py < h - margin_y
                        and face_mask[int(py), int(px)] == 0
                    ):
                        dy = py - self._base_ys[i]
                        valid_dys.append(dy)
                        valid_pts.append(pt)
                        valid_bases.append(self._base_ys[i])

        # ------------------------------------------------------------
        # 核心判断逻辑
        # ------------------------------------------------------------
        if len(valid_pts) >= self._min_corners:
            # A. 存活点数量充足：保留有效点，使用中位数计算 dy
            self._points = np.array(valid_pts, dtype=np.float32)
            self._base_ys = np.array(valid_bases, dtype=np.float32)
            self._prev_gray = gray.copy()

            self._last_dy = float(np.median(valid_dys))

        else:
            # B. 存活点不足（或点移出画面全灭）：触发无缝接力补点 (Baseline Handover)
            self._prev_gray = gray.copy()
            # 在当前位置寻找新点，并让新点继承上一帧的 _last_dy
            if not self._reset_tracking(gray, face_mask, inherit_dy=self._last_dy):
                # 如果暂时找不到新点，则保留最后的 _last_dy 冻结当前角度，不发生跳变
                pass

        # 根据最新（或继承冻结的）dy 计算角度
        raw = self._calculate_angle_from_displacement(self._last_dy)

        # 指数平滑
        alpha = SMOOTH_ALPHA

        if abs(raw - self._smooth) < 0.008:
            self._smooth = raw
        else:
            self._smooth = self._smooth * (1.0 - alpha) + raw * alpha

        with self._lock:
            self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

        # self._smooth = self._smooth * (1.0 - alpha) + raw * alpha

        # with self._lock:
        #    self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

        if self._debug:
            now = time.monotonic()
            if now - self._last_debug >= DEBUG_INTERVAL:
                self._last_debug = now
                pts_cnt = len(self._points) if self._points is not None else 0
                print(
                    f"[camera_daemon] pts={pts_cnt} dy={self._last_dy:.1f} angle={self.current_angle:.3f}",
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
        print(
            f"[camera_daemon] 无法打开摄像头 /dev/video{CAMERA_INDEX}", file=sys.stderr
        )
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
