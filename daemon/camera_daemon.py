#!/usr/bin/env python3
"""摄像头开合角检测守护进程 (camera-based lid-fold angle daemon).

打开笔记本摄像头检测屏幕开合：
1. 检测到人脸 → 快速恢复到完全展开状态（角度 1.0）。
2. 无人脸时 → 在人脸区域之外选择一个特征点，通过该点的纵向位移
   计算屏幕开合程度。
3. 人脸检测优先于特征点跟踪；再次检测到人脸时立即触发快速恢复，
   并重新建立人脸外特征点基准。
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
SMOOTH_ALPHA = 0.03
FACE_ALPHA = 0.5
BASELINE_DECAY = 0.001
DROP_FLOOR = 0.35
EPS = 1e-6
CAMERA_INDEX = 0
DEBUG_INTERVAL = 0.5


class CameraTracker:
    """
    通过“人脸区域以外的固定画面特征点”的纵向位移估算屏幕开合程度。

    这里不再使用画面亮度判断角度：
      1. 人脸检测只用于排除人脸区域，避免把人脸上的特征当作固定点。
      2. 在人脸之外寻找一个 Shi-Tomasi 角点。
      3. 使用 Lucas-Kanade 光流逐帧跟踪该点。
      4. 只取 Y 方向位移。
      5. 以启动/重新锁定时的位置作为基准，通过像素位移映射为 0~1。
         1.0 = 完全展开，0.0 = 达到设定的最大纵向位移。
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

        # 光流状态
        self._prev_gray = None
        self._point = None

        # 纵向位移基准
        self._base_y = None

        # 达到完全折叠时所对应的纵向像素位移。
        #
        # 这个值需要根据你的摄像头安装位置实测。
        # 数值越小，同样的实际位移产生的“折叠程度”越大。
        self._max_vertical_displacement = 220.0

        # 光流参数
        self._lk_win_size = (21, 21)
        self._lk_max_level = 3

        # 特征点重新寻找间隔
        self._reselect_interval = 30
        self._frames_since_reselect = 0

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
                "/usr/share/opencv4/haarcascades/"
                "haarcascade_frontalface_default.xml",
                "/usr/share/opencv/haarcascades/" "haarcascade_frontalface_default.xml",
            ]
        )

        for path in paths:
            if os.path.isfile(path):
                cascade = cv2.CascadeClassifier(path)
                if not cascade.empty():
                    return cascade

        return None

    def _detect_face_mask(self, gray):
        """
        返回人脸区域 mask。
        人脸只用于“排除”，不参与角度计算。
        """
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
            # 稍微扩大人脸排除区域，避免选到头发、眼睛等特征。
            pad_x = int(w * 0.20)
            pad_y = int(h * 0.25)

            x0 = max(0, x - pad_x)
            y0 = max(0, y - pad_y)
            x1 = min(gray.shape[1], x + w + pad_x)
            y1 = min(gray.shape[0], y + h + pad_y)

            mask[y0:y1, x0:x1] = 255

        return mask

    def _find_tracking_point(self, gray, face_mask):
        """
        在人脸之外寻找一个稳定角点。

        只选择一个点，而不是多个点平均。
        """
        h, w = gray.shape

        # 避开画面最边缘，防止角点在边框/黑边上。
        feature_mask = np.full_like(gray, 255)

        margin_x = max(8, int(w * 0.05))
        margin_y = max(8, int(h * 0.05))

        feature_mask[:margin_y, :] = 0
        feature_mask[h - margin_y :, :] = 0
        feature_mask[:, :margin_x] = 0
        feature_mask[:, w - margin_x :] = 0

        # 排除人脸。
        feature_mask[face_mask > 0] = 0

        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=1,
            qualityLevel=0.01,
            minDistance=30,
            blockSize=15,
            mask=feature_mask,
            useHarrisDetector=False,
        )

        if points is None or len(points) == 0:
            return None

        return points[0].reshape(1, 1, 2).astype(np.float32)

    def _reset_tracking(self, gray, face_mask=None):
        """
        重新寻找一个人脸外的固定特征点，并以当前位置建立纵向位移基准。
        """
        if face_mask is None:
            face_mask = self._detect_face_mask(gray)

        point = self._find_tracking_point(gray, face_mask)

        if point is None:
            self._point = None
            self._base_y = None
            return False

        self._point = point
        self._base_y = float(point[0, 0, 1])
        self._prev_gray = gray.copy()
        self._frames_since_reselect = 0
        return True

    def _calculate_angle_from_vertical_displacement(self, y):
        """
        根据相对于基准位置的纵向位移计算 0~1。

        注意：
        这里输出的是“开合比例”，供 GNOME Shell 使用。
        它不是光学意义上经过相机内参标定的绝对机械角度。

        假设：
          dy = 0                       -> 完全展开 -> 1.0
          |dy| >= max_displacement    -> 完全折叠 -> 0.0
        """
        if self._base_y is None:
            return self._smooth

        dy = float(y - self._base_y)

        # 只使用纵向位移的绝对值。
        # 这样无论镜头运动方向是向上还是向下，都能工作。
        displacement = abs(dy)

        ratio = np.clip(
            displacement / self._max_vertical_displacement,
            0.0,
            1.0,
        )

        # 用 smoothstep 消除轻微光流抖动。
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

        # ============================================================
        # 人脸检测最高优先级
        #
        # 检测到人脸：
        #   1. 快速恢复到完全展开状态
        #   2. 丢弃旧的光流点
        #   3. 在人脸之外重新寻找特征点
        #   4. 当前画面作为新的“完全展开”参考
        #
        # 人脸出现时不再使用纵向位移计算角度。
        # ============================================================
        face_mask = self._detect_face_mask(gray)

        has_face = np.any(face_mask > 0)

        if has_face:
            # 保留原来的快速恢复功能。
            # FACE_ALPHA 越大，恢复越快。
            self._smooth = self._smooth * (1.0 - FACE_ALPHA) + 1.0 * FACE_ALPHA

            with self._lock:
                self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

            # 人脸出现时，不能每一帧都重新建立特征点基准。
            #
            # 如果每帧都 _reset_tracking()：
            #   人脸期间 base_y 会不断被更新；
            #   人脸消失后的第一帧又从“当前位置 = 0 位移”开始；
            #   因此角度必然先保持 1，再过一帧才开始识别位移。
            #
            # 正确做法：
            #   - 第一次检测到人脸：建立一次基准点
            #   - 后续人脸帧：继续跟踪这个点，但强制 angle=1
            #   - 人脸消失：直接拿已经跟踪到的点计算相对 base_y 的位移
            #
            # 这样人脸消失后的第一帧就可以参与位移识别。
            if self._point is None or self._base_y is None:
                self._prev_gray = gray.copy()
                self._reset_tracking(gray, face_mask)
            else:
                # 人脸存在期间也继续更新光流点的位置，
                # 但绝不修改 _base_y。
                next_point, status, error = cv2.calcOpticalFlowPyrLK(
                    self._prev_gray,
                    gray,
                    self._point,
                    None,
                    winSize=self._lk_win_size,
                    maxLevel=self._lk_max_level,
                    criteria=(
                        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                        20,
                        0.03,
                    ),
                )

                if (
                    next_point is not None
                    and status is not None
                    and int(status[0, 0]) == 1
                    and np.isfinite(next_point[0, 0]).all()
                ):
                    x, y = next_point[0, 0]

                    if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                        self._point = next_point.astype(np.float32)
                    else:
                        self._point = None

                self._prev_gray = gray.copy()

            self._frames_since_reselect = 0

            if self._debug:
                now = time.monotonic()
                if now - self._last_debug >= DEBUG_INTERVAL:
                    self._last_debug = now
                    print(
                        "[camera_daemon] "
                        f"face detected -> fast recovery, "
                        f"angle={self.current_angle:.3f}",
                        file=sys.stderr,
                    )

            return

        # ============================================================
        # 无人脸：使用人脸之外的单个特征点进行纵向位移检测
        # ============================================================

        # 第一次运行才需要建立基准。
        # 正常情况下，人脸存在期间已经持续维护 _point，
        # 所以人脸刚消失时这里可以直接进入光流计算，
        # 不会重新把当前位置当成 base_y。
        if self._prev_gray is None or self._point is None or self._base_y is None:
            self._prev_gray = gray.copy()

            if not self._reset_tracking(gray, face_mask):
                return

            # 没有上一帧可用于光流，因此这一帧只能建立参考。
            # 从下一帧开始正式计算位移。
            return

        next_point, status, error = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray,
            self._point,
            None,
            winSize=self._lk_win_size,
            maxLevel=self._lk_max_level,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )

        valid = next_point is not None and status is not None and int(status[0, 0]) == 1

        if valid:
            x, y = next_point[0, 0]

            if not (
                np.isfinite(x)
                and np.isfinite(y)
                and 0 <= x < gray.shape[1]
                and 0 <= y < gray.shape[0]
            ):
                valid = False

        if not valid:
            self._prev_gray = gray.copy()

            # 光流丢失时重新找点。
            # 注意：只有真正丢失时才重新建立基准，
            # 避免正常跟踪过程中角度突然归零/归一。
            if not self._reset_tracking(gray, face_mask):
                return

            return

        self._point = next_point
        self._prev_gray = gray.copy()
        self._frames_since_reselect += 1

        raw = self._calculate_angle_from_vertical_displacement(float(y))

        # 无人脸时保持原来的平滑角度变化。
        alpha = SMOOTH_ALPHA
        self._smooth = self._smooth * (1.0 - alpha) + raw * alpha

        with self._lock:
            self.current_angle = float(np.clip(self._smooth, 0.0, 1.0))

        # 定期检查当前跟踪点是否进入人脸区域。
        # 虽然本帧已经没有检测到人脸，但这里保留检查逻辑，
        # 防止下一帧出现人脸后旧点继续参与计算。
        if self._frames_since_reselect >= self._reselect_interval:
            self._frames_since_reselect = 0

            px = int(round(float(x)))
            py = int(round(float(y)))

            if (
                0 <= px < gray.shape[1]
                and 0 <= py < gray.shape[0]
                and face_mask[py, px] > 0
            ):
                self._reset_tracking(gray, face_mask)

        if self._debug:
            now = time.monotonic()

            if now - self._last_debug >= DEBUG_INTERVAL:
                self._last_debug = now

                base_y = self._base_y if self._base_y is not None else float("nan")

                dy = (
                    float(y) - self._base_y
                    if self._base_y is not None
                    else float("nan")
                )

                print(
                    f"[camera_daemon] "
                    f"point=({x:.1f},{y:.1f}) "
                    f"base_y={base_y:.1f} "
                    f"dy={dy:.1f} "
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
