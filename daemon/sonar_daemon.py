#!/usr/bin/env python3
"""近超声波开合角声纳守护进程 (near-ultrasonic lid-fold sonar daemon).

无硬件角度传感器环境下，通过笔记本「扬声器-麦克风」近超声波声纳追踪开合状态：
全双工播放 19.5 kHz 载波并同步采集麦克风信号，对接收信号做 IQ 正交解调提取
回波相位，相位变化换算为相对声学位移，再归一化为开合角 (1.0=展开, 0.0=折叠)。
角度经 Unix 域套接字以 ~60 FPS 广播给 GNOME Shell 扩展，并接收扩展下发的
`RESET:<value>` 零点/满位校准指令。
"""

import math
import os
import signal
import sys
import threading
import time

import numpy as np
import sounddevice as sd

from common import SOCKET_PATH, serve, setup_socket

FS = 48000
FREQ = 19500.0
CHUNK = 1024
SPEED_OF_SOUND = 343.0
MAX_DISPLACEMENT = 0.25
TX_AMPLITUDE = 0.35


class OneDimensionalKalman:
    def __init__(self, q=1e-4, r=1e-2, initial_val=1.0):
        self.q = q
        self.r = r
        self.x = initial_val
        self.p = 1.0

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1.0 - k)
        return self.x


class SonarTracker:
    """在 PortAudio 音频线程与 socket 主线程间安全共享角度状态。"""

    def __init__(self):
        self.kalman = OneDimensionalKalman(initial_val=1.0)
        self._lock = threading.Lock()
        self.current_angle = 1.0
        self._prev_phase = 0.0
        self._sample_index = 0

    def audio_callback(self, indata, outdata, frames, time_info, status):
        n = frames
        t = (self._sample_index + np.arange(n)) / FS
        self._sample_index += n

        outdata[:, 0] = (TX_AMPLITUDE * np.sin(2.0 * np.pi * FREQ * t)).astype(np.float32)

        rx = indata[:, 0].astype(np.float32)
        iq = rx * np.exp(-1j * 2.0 * np.pi * FREQ * t)
        phase = np.angle(np.mean(iq))

        d_phase = phase - self._prev_phase
        self._prev_phase = phase
        d_phase = (d_phase + np.pi) % (2.0 * np.pi) - np.pi

        # 一个波长的相位变化对应 wavelength / 2 的单程声学位移
        wavelength = SPEED_OF_SOUND / FREQ
        delta_d = (d_phase / (2.0 * np.pi)) * (wavelength / 2.0)

        raw_angle = np.clip(self.current_angle + delta_d / MAX_DISPLACEMENT, 0.0, 1.0)
        with self._lock:
            self.current_angle = float(self.kalman.update(raw_angle))

    def set_angle(self, val):
        val = float(val)
        with self._lock:
            self.current_angle = val
            self.kalman.x = val

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
        elapsed = time.monotonic() - start
        tracker.set_angle(0.5 + 0.5 * math.sin(2.0 * math.pi * 0.2 * elapsed))
        time.sleep(0.02)


def main():
    demo = "--demo" in sys.argv
    stop_event = threading.Event()

    def _request_stop(signum, frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    server = setup_socket()
    tracker = SonarTracker()

    threading.Thread(
        target=serve, args=(server, tracker, stop_event), daemon=True
    ).start()

    try:
        if demo:
            run_demo(tracker, stop_event)
        else:
            run_audio_loop(tracker, stop_event)
    except Exception as exc:  # noqa: BLE001
        print(f"[sonar_daemon] 启动失败: {exc}", file=sys.stderr)
    finally:
        stop_event.set()
        server.close()
        try:
            os.remove(SOCKET_PATH)
        except OSError:
            pass


if __name__ == "__main__":
    main()
