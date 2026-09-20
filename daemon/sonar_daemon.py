#!/usr/bin/env python3
"""近超声波开合角声纳守护进程。

角度检测逻辑与 camera_daemon.py 对齐：
1. 使用固定声学基准位置计算相对位移，不再通过 current_angle 持续积分。
2. 保留有符号位移，因此基准位置两侧的移动方向可以区分。
3. 位移长时间基本不变时，自动把当前位置重新设为新的基准位置，
   并将该位置的开合角同时重置为 1.0，从而避免声学相位长期漂移导致的累计误差。
4. 信号质量不足或发生异常跳变时，冻结上一有效位移，不让角度突然跳变。
5. 对有效位移做中值滤波，再通过平滑映射得到 0.0~1.0 开合角。
6. RESET:<value> 可以手动把当前位置设为新的基准，并指定当前位置角度。
"""

import math
import os
import signal
import sys
import threading
import time
from collections import deque

import numpy as np
import sounddevice as sd

from common import SOCKET_PATH, serve, setup_socket


FS = 48000
FREQ = 19500.0
CHUNK = 1024
SPEED_OF_SOUND = 343.0
TX_AMPLITUDE = 0.35

# 一个完整检测范围对应的声学位移。
# 需要根据实际机器的测量结果校准。
MAX_DISPLACEMENT = 0.25

# 与 camera_daemon.py 一致。
SMOOTH_ALPHA = 0.04

# 信号质量门限。
MIN_COHERENCE = 0.20
MIN_RMS = 0.003

# 单个音频块允许的最大位移变化。
MAX_STEP_DISPLACEMENT = 0.025

# 中值滤波窗口。
MEDIAN_WINDOW = 5

# ------------------------------------------------------------
# 自动基准重置
# ------------------------------------------------------------

# 连续多长时间位移基本没有变化后，认为设备已经稳定。
BASELINE_STABLE_SECONDS = 3.0

# “基本没有变化”的位移阈值。
BASELINE_STABLE_DISPLACEMENT = 0.004

# 自动重置时，基准不能过于频繁移动。
BASELINE_RESET_COOLDOWN = 2.0

# 用于判断稳定状态的历史窗口。
BASELINE_HISTORY_SECONDS = 3.0


class SonarTracker:
    """在 PortAudio 音频线程与 socket 主线程间安全共享角度状态。"""

    def __init__(self, debug=False):
        self._lock = threading.Lock()

        self._smooth = 1.0
        self.current_angle = 1.0

        self._debug = debug
        self._last_debug = 0.0

        # ============================================================
        # 相位状态
        # ============================================================

        self._prev_phase = None
        self._unwrapped_phase = 0.0

        # 固定基准相位。
        # 所有位移均相对于这个位置计算。
        self._base_unwrapped_phase = None

        # 上一次有效位移。
        self._last_displacement = 0.0

        # 最近有效位移，用于中值滤波。
        self._displacement_history = deque(
            maxlen=MEDIAN_WINDOW
        )

        # 最近一段时间的“稳定位移”历史。
        self._baseline_history = deque()

        self._last_baseline_reset = time.monotonic()

        self._sample_index = 0

        # 手动 RESET 后当前位置对应的角度。
        self._reference_angle = 1.0

        # 19.5 kHz 声波：
        # λ = c / f
        # 这里按照原代码的单程 λ/2 关系换算。
        wavelength = SPEED_OF_SOUND / FREQ
        self._phase_to_displacement = (
            (wavelength / 2.0)
            / (2.0 * math.pi)
        )

    @staticmethod
    def _wrap_phase_delta(delta):
        """把相位差限制到 [-π, π)。"""
        return (delta + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _extract_phase(rx, t):
        """提取 19.5 kHz 分量相位及信号质量。"""
        iq = rx * np.exp(
            -1j * 2.0 * np.pi * FREQ * t
        )

        iq_mean = np.mean(iq)

        phase = float(np.angle(iq_mean))
        magnitude = float(abs(iq_mean))
        rms = float(np.sqrt(np.mean(rx * rx)))

        # IQ 平均幅值相对于整体 RMS 的比例，
        # 用于判断当前载波是否具有足够的稳定性。
        coherence = magnitude / (rms + 1e-9)

        return phase, coherence, rms

    def _calculate_angle_from_displacement(self, displacement):
        """根据有符号相对位移计算开合比例。

        基准位置对应 _reference_angle。

        正方向：
            向折叠方向移动 -> angle 下降

        负方向：
            向展开方向移动 -> angle 上升

        这样基准点两侧的运动不会被 abs() 抹掉。
        """

        # 将有符号位移归一化。
        ratio = displacement / MAX_DISPLACEMENT

        # 分别限制到 [-1, 1]。
        ratio = float(np.clip(ratio, -1.0, 1.0))

        # 对正负方向分别做 smoothstep。
        if ratio >= 0.0:
            fold = ratio * ratio * (3.0 - 2.0 * ratio)
            raw = self._reference_angle - fold * self._reference_angle
        else:
            open_ratio = -ratio
            opening = (
                open_ratio
                * open_ratio
                * (3.0 - 2.0 * open_ratio)
            )

            # 向基准的展开方向移动时增加角度。
            raw = self._reference_angle + (
                opening * (1.0 - self._reference_angle)
            )

        return float(np.clip(raw, 0.0, 1.0))

    def _reset_baseline(self, angle=None):
        """把当前位置重新设置为新的声学基准。"""

        if self._base_unwrapped_phase is None:
            return

        self._base_unwrapped_phase = self._unwrapped_phase
        self._last_displacement = 0.0
        self._displacement_history.clear()
        self._baseline_history.clear()
        self._last_baseline_reset = time.monotonic()

        if angle is not None:
            self._reference_angle = float(
                np.clip(angle, 0.0, 1.0)
            )

        with self._lock:
            self._smooth = self._reference_angle
            self.current_angle = self._reference_angle

        if self._debug:
            print(
                "[sonar_daemon] baseline reset: "
                f"angle={self._reference_angle:.3f}",
                file=sys.stderr,
            )

    def _update_auto_baseline(self, displacement):
        """检测位移长期稳定，自动建立新的基准。

        注意：
        自动重置不会改变当前角度，只是把“当前声学位置”
        重新定义为零位移。
        """

        now = time.monotonic()

        self._baseline_history.append(
            (now, displacement)
        )

        # 清理时间窗口之外的数据。
        cutoff = now - BASELINE_HISTORY_SECONDS

        while (
            self._baseline_history
            and self._baseline_history[0][0] < cutoff
        ):
            self._baseline_history.popleft()

        if (
            now - self._last_baseline_reset
            < BASELINE_RESET_COOLDOWN
        ):
            return

        if not self._baseline_history:
            return

        elapsed = (
            self._baseline_history[-1][0]
            - self._baseline_history[0][0]
        )

        if elapsed < BASELINE_STABLE_SECONDS:
            return

        values = np.array(
            [item[1] for item in self._baseline_history],
            dtype=np.float32,
        )

        # 用最大值-最小值判断“长时间基本没有变化”。
        variation = float(
            np.max(values) - np.min(values)
        )

        if variation <= BASELINE_STABLE_DISPLACEMENT:
            # 当前位移稳定了，说明笔记本已经停止运动。
            #
            # 与“重新建立基准”的定义一致：
            # 当前稳定位置同时被定义为新的“完全展开/零位移”状态，
            # 因此角度也必须重置为 1.0。
            #
            # 后续再次发生位移时，从这个新基准重新计算：
            #   0 位移 -> 1.0
            #   向折叠方向 -> 角度下降
            #   向展开方向 -> 仍保持/趋向 1.0
            self._reset_baseline(
                angle=1.0
            )

    def audio_callback(
        self,
        indata,
        outdata,
        frames,
        time_info,
        status,
    ):
        n = frames

        t = (
            self._sample_index
            + np.arange(n)
        ) / FS

        self._sample_index += n

        outdata[:, 0] = (
            TX_AMPLITUDE
            * np.sin(
                2.0 * np.pi * FREQ * t
            )
        ).astype(np.float32)

        rx = indata[:, 0].astype(
            np.float32
        )

        phase, coherence, rms = (
            self._extract_phase(rx, t)
        )

        # ============================================================
        # 1. 第一次有效采样：建立初始基准
        # ============================================================

        if self._prev_phase is None:
            self._prev_phase = phase
            self._unwrapped_phase = phase
            self._base_unwrapped_phase = phase
            return

        # ============================================================
        # 2. 计算连续相位变化
        # ============================================================

        d_phase = self._wrap_phase_delta(
            phase - self._prev_phase
        )

        signal_valid = (
            coherence >= MIN_COHERENCE
            and rms >= MIN_RMS
        )

        if not signal_valid:
            # 信号无效：
            # 不推进相位，不更新位移。
            # 等价于 Camera 的特征点暂时全部失效。
            return

        self._prev_phase = phase
        self._unwrapped_phase += d_phase

        # ============================================================
        # 3. 计算相对于基准位置的有符号位移
        # ============================================================

        if self._base_unwrapped_phase is None:
            self._base_unwrapped_phase = (
                self._unwrapped_phase
            )

        phase_from_base = (
            self._unwrapped_phase
            - self._base_unwrapped_phase
        )

        displacement = (
            phase_from_base
            * self._phase_to_displacement
        )

        # ============================================================
        # 4. 异常跳变保护
        # ============================================================

        step = (
            displacement
            - self._last_displacement
        )

        if abs(step) > MAX_STEP_DISPLACEMENT:
            # 本次相位变化认为异常。
            # 回退本次相位累计。
            self._unwrapped_phase -= d_phase
            return

        # 防止超出物理检测范围。
        displacement = float(
            np.clip(
                displacement,
                -MAX_DISPLACEMENT,
                MAX_DISPLACEMENT,
            )
        )

        # ============================================================
        # 5. 中值滤波
        # ============================================================

        self._displacement_history.append(
            displacement
        )

        filtered_displacement = float(
            np.median(
                np.asarray(
                    self._displacement_history,
                    dtype=np.float32,
                )
            )
        )

        self._last_displacement = (
            filtered_displacement
        )

        # ============================================================
        # 6. 自动检测“停止运动”
        # ============================================================

        self._update_auto_baseline(
            filtered_displacement
        )

        # 自动重置可能刚刚把当前位移归零。
        displacement = self._last_displacement

        # ============================================================
        # 7. 位移 -> 开合角
        # ============================================================

        raw = (
            self._calculate_angle_from_displacement(
                displacement
            )
        )

        # ============================================================
        # 8. 与 Camera 一致的平滑
        # ============================================================

        if abs(raw - self._smooth) < 0.008:
            self._smooth = raw
        else:
            self._smooth = (
                self._smooth
                * (1.0 - SMOOTH_ALPHA)
                + raw * SMOOTH_ALPHA
            )

        with self._lock:
            self.current_angle = float(
                np.clip(
                    self._smooth,
                    0.0,
                    1.0,
                )
            )

        if self._debug:
            now = time.monotonic()

            if (
                now - self._last_debug
                >= 0.5
            ):
                self._last_debug = now

                print(
                    "[sonar_daemon] "
                    f"disp={self._last_displacement:.4f}m "
                    f"angle={self.current_angle:.3f} "
                    f"coherence={coherence:.3f}",
                    file=sys.stderr,
                )

    def set_angle(self, val):
        """处理 RESET:<value>。

        RESET 时：
        1. 当前声学位置成为新的基准。
        2. 当前角度立即设为指定值。
        """

        val = float(
            np.clip(val, 0.0, 1.0)
        )

        if self._prev_phase is not None:
            self._base_unwrapped_phase = (
                self._unwrapped_phase
            )

        self._last_displacement = 0.0
        self._displacement_history.clear()
        self._baseline_history.clear()

        self._reference_angle = val
        self._last_baseline_reset = (
            time.monotonic()
        )

        with self._lock:
            self._smooth = val
            self.current_angle = val

    def get_angle(self):
        with self._lock:
            return self.current_angle


def run_audio_loop(tracker, stop_event):
    with sd.Stream(
        channels=1,
        samplerate=FS,
        blocksize=CHUNK,
        dtype="float32",
        callback=tracker.audio_callback,
    ):
        while not stop_event.is_set():
            time.sleep(0.1)


def run_demo(tracker, stop_event):
    start = time.monotonic()

    while not stop_event.is_set():
        elapsed = (
            time.monotonic()
            - start
        )

        tracker.set_angle(
            0.5
            + 0.5
            * math.sin(
                2.0
                * math.pi
                * 0.2
                * elapsed
            )
        )

        time.sleep(0.02)


def main():
    demo = "--demo" in sys.argv
    debug = "--debug" in sys.argv

    stop_event = threading.Event()

    def _request_stop(signum, frame):
        stop_event.set()

    signal.signal(
        signal.SIGTERM,
        _request_stop,
    )
    signal.signal(
        signal.SIGINT,
        _request_stop,
    )

    server = setup_socket()
    tracker = SonarTracker(
        debug=debug
    )

    threading.Thread(
        target=serve,
        args=(
            server,
            tracker,
            stop_event,
        ),
        daemon=True,
    ).start()

    try:
        if demo:
            run_demo(
                tracker,
                stop_event,
            )
        else:
            run_audio_loop(
                tracker,
                stop_event,
            )
    except Exception as exc:  # noqa: BLE001
        print(
            f"[sonar_daemon] 启动失败: {exc}",
            file=sys.stderr,
        )
    finally:
        stop_event.set()
        server.close()

        try:
            os.remove(
                SOCKET_PATH
            )
        except OSError:
            pass


if __name__ == "__main__":
    main()
