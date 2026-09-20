# Linux GNOME Duo

**中文**：一个 GNOME Shell 扩展，通过测量笔记本盖子的真实开合角度，在普通（非 MacBook）Linux 笔记本上复刻 iPhone Duo 式的折叠视差动画，并据此折叠整个 GNOME Shell 界面。

**English**: A GNOME Shell extension that recreates the iPhone Duo style "fold" parallax animation on ordinary (non-MacBook) Linux laptops by measuring the real lid angle and folding the entire GNOME Shell UI accordingly.

- **UUID**: `linux-duo@kechen.github`
- **Extension name**: Duo Fold Animation
- **GNOME Shell**: 45, 46

---

## Preview / 预览

**中文**：效果演示视频：

**English**: Demo video:

https://github.com/user-attachments/assets/2fb03812-6c76-407d-ba0c-07aac362981c

---

## Features / 功能

| 功能 / Feature | 说明 / Description |
| --- | --- |
| 真实角度驱动 / Real-angle driven | 依据盖子实际开合角度（0.0 到 1.0）驱动界面折叠，而非模拟固定动画。<br>Drives the fold from the actual lid angle (0.0 to 1.0) instead of a fixed simulated animation. |
| 双检测方案 / Two detection methods | 可在设置中切换摄像头（默认）或超声波声纳。<br>Switch between camera (default) and ultrasonic sonar in Settings. |
| 渐变模糊 / Gradient blur | 折叠时屏幕顶部模糊最强，向下逐渐减弱，底部保持清晰。<br>When folded, blur is strongest at the top, fades downward, and stays clear at the bottom. |
| 淡出与黑底 / Fade and black backdrop | 折叠过程中叠加垂直渐变淡出与黑色背景，营造翻盖观感。<br>A vertical fade overlay plus a black backdrop appear during folding for a clamshell look. |
| 实时设置 / Live settings | 检测方案与模糊强度修改后立即生效，无需重启。<br>Changing the detection method or blur strength takes effect immediately, no restart needed. |
| 自动校准 / Auto calibration | 扩展监听 UPower 的 `LidIsClosed`，在盖子开合时发送校准指令。<br>The extension watches UPower `LidIsClosed` and sends calibration commands as the lid opens or closes. |

---

## How It Works / 工作原理

**中文**：扩展本身不直接采集传感器数据。它启动一个 Python 守护进程（`daemon/`），通过位于 `/tmp/gnome_lid_sonar.sock` 的 Unix 域套接字接收角度值，每秒约 60 次。守护进程持续广播 0.0（完全合上）到 1.0（完全展开）的浮点数，扩展据此实时折叠界面。

**English**: The extension does not read sensors directly. It spawns a Python daemon (`daemon/`) and receives the angle over a Unix domain socket at `/tmp/gnome_lid_sonar.sock`, roughly 60 times per second. The daemon broadcasts a float from 0.0 (fully closed) to 1.0 (fully open), and the extension folds the UI in real time.

**中文**：同时，扩展通过 D-Bus 监听 UPower 的 `LidIsClosed` 属性。盖子状态变化时，扩展向守护进程发送 `RESET:0`（合上）或 `RESET:1`（展开），把当前位置重新设为基准。

**English**: In parallel, the extension watches UPower's `LidIsClosed` property over D-Bus. When the lid state changes, it sends `RESET:0` (closed) or `RESET:1` (open) to the daemon, re-anchoring the current position as the baseline.

### Camera detection / 摄像头检测

**中文**：打开笔记本摄像头，用 OpenCV 加光流法估算折叠程度：

1. 检测到人脸时，快速恢复到完全展开（角度趋向 1.0），因为屏幕打开时通常会看到用户的脸。
2. 未检测到人脸时，在检测到的人脸区域之外选取多个特征点，用 Lucas-Kanade 光流跟踪它们。
3. 取所有有效跟踪点纵向位移的中位数作为折叠位移，再经 smoothstep 映射到 0.0 到 1.0 的开合角。
4. 采用「无缝基准继承」（Baseline Handover）：当跟踪点即将离开画面或数量不足时，自动补充新特征点，并让新点继承此前的位移历史，避免角度跳变。

**English**: Opens the laptop webcam and estimates the fold with OpenCV plus optical flow:

1. When a face is detected, it recovers quickly to fully open (angle toward 1.0), since seeing a face usually means the screen is open.
2. When no face is found, it selects feature points outside the detected face region and tracks them with Lucas-Kanade optical flow.
3. The median vertical displacement of all valid tracked points becomes the fold displacement, mapped through a smoothstep curve to a 0.0 to 1.0 lid angle.
4. It uses "Seamless Baseline Handover": when points approach the edge or become too few, new points are seeded automatically and inherit the previous displacement history, so the angle does not jump.

### Sonar detection / 超声波检测

**中文**：通过扬声器发出 19.5 kHz 的超声波，用麦克风（PortAudio）接收回声并测量其相位：

1. 相位相对一个固定声学基准计算，因此不需要持续积分，避免长期漂移。
2. 保留有符号位移，基准位置两侧的移动方向可以区分，而不是用绝对值抹平。
3. 位移长时间基本不变时（设备已稳定），自动把当前位置设为新基准，并把该位置的角度重置为 1.0。
4. 信号质量不足（相干性 / RMS 过低）或出现异常跳变时，冻结上一个有效位移，不让角度突然跳动。
5. 对有效位移做中值滤波，再经平滑映射得到 0.0 到 1.0 的开合角。

**English**: Emits a 19.5 kHz ultrasonic tone through the speakers and measures the phase of the echo through the microphone (PortAudio):

1. Phase is measured against a fixed acoustic baseline, so it needs no continuous integration and avoids long-term drift.
2. Signed displacement is preserved, so movement on either side of the baseline can be told apart instead of being flattened by an absolute value.
3. When displacement stays essentially unchanged for a while (the device is stable), the current position becomes the new baseline and its angle is reset to 1.0.
4. When signal quality is poor (low coherence or RMS) or a sudden jump occurs, the last valid displacement is frozen so the angle does not jump.
5. Valid displacement goes through a median filter, then a smooth mapping to a 0.0 to 1.0 lid angle.

### Rendering / 渲染

**中文**：角度到达后，扩展对 `Main.layoutManager.uiGroup` 施加变换：以屏幕底部为轴心（`0.5, 1.0`）绕 X 轴旋转，最大约 40 度，同时按开合比例调整整体不透明度，并更新 16 层渐变模糊层（顶部最强、向下递减）、垂直淡出层和黑色背景。

**English**: Once the angle arrives, the extension transforms `Main.layoutManager.uiGroup`: it pivots at the bottom of the screen (`0.5, 1.0`) and rotates around the X axis up to about 40 degrees, scales overall opacity with the open ratio, and updates 16 gradient blur layers (strongest at the top, fading down), a vertical fade overlay, and a black backdrop.

---

## Installation / 安装

**中文**：运行安装脚本，它会安装系统依赖、部署扩展到 GNOME 扩展目录、编译 GSettings schema、安装 Python 库并启用扩展：

**English**: Run the setup script. It installs system dependencies, deploys the extension into the GNOME extensions directory, compiles the GSettings schema, installs the Python libraries, and enables the extension:

```shell
./setup.sh
```

**中文**：安装完成后需要重载 GNOME Shell：

- X11：按 `Alt+F2`，输入 `r`，回车。
- Wayland：注销后重新登录。

**English**: After setup, reload GNOME Shell:

- X11: press `Alt+F2`, type `r`, and press Enter.
- Wayland: log out and log back in.

> [!note]
> **中文**：如果你在 Wayland 下，无法用 `Alt+F2` 重载，必须重新登录。
>
> **English**: On Wayland you cannot reload with `Alt+F2`; you must log out and log back in.

---

## Configuration / 配置

**中文**：打开扩展设置（GNOME Extensions 应用或 `gnome-extensions prefs linux-duo@kechen.github`）。

**English**: Open the extension settings (GNOME Extensions app, or `gnome-extensions prefs linux-duo@kechen.github`).

| 设置 / Setting | 键名 / Key | 取值 / Values | 默认 / Default | 说明 / Description |
| --- | --- | --- | --- | --- |
| 检测方案 / Detection Method | `method` | `camera`, `sonar` | `camera` | 选择开合角度检测方案。<br>Choose the lid-angle detection method. |
| 模糊强度 / Blur Strength | `blur-strength` | 0.0 到 0.2 | 0.08 | 折叠时屏幕顶部的模糊程度，越大越模糊，底部逐渐减弱。<br>Blur amount at the top when folded; higher means blurrier, fading toward the bottom. |

> [!note]
> **中文**：如果你的麦克风位于屏幕顶部，请在设置中把检测方案设为 `Sonar`（默认是 `Camera`）。
>
> **English**: If your microphone is at the top of the screen, set Detection Method to `Sonar` in Settings (the default is `Camera`).

---

## Requirements & Dependencies / 依赖

**系统依赖 / System dependencies** (installed by `setup.sh`):

- `python3`
- `python3-pip`
- `portaudio19-dev`
- `libasound2-dev`
- `libglib2.0-bin`

**Python 库 / Python packages** (`daemon/requirements.txt`):

- `numpy`
- `sounddevice`
- `scipy`
- `opencv-python-headless`

---

## Project Structure / 项目结构

```text
linux-duo@kechen.github/
├── metadata.json                 # 扩展元数据 / Extension metadata
├── extension.js                  # 主扩展逻辑，套接字、D-Bus、渲染 / Main logic: socket, D-Bus, rendering
├── prefs.js                      # 设置界面 / Preferences UI
├── setup.sh                      # 安装脚本 / Installer
├── schemas/
│   └── org.gnome.shell.extensions.linux-duo.gschema.xml   # GSettings schema
└── daemon/
    ├── common.py                 # 套接字服务与 IPC 工具 / Socket server and IPC helpers
    ├── camera_daemon.py          # 摄像头检测守护进程 / Camera detection daemon
    ├── sonar_daemon.py           # 超声波检测守护进程 / Sonar detection daemon
    └── requirements.txt          # Python 依赖 / Python dependencies
```

---

## Troubleshooting & Notes / 疑难解答与说明

**中文**：

- **检测方案选择**：麦克风在屏幕顶部的机器更适合 `Sonar`；其他情况默认 `Camera`。
- **摄像头不可用**：`camera_daemon.py` 打开 `/dev/video0`。若打不开，请确认摄像头未被其他程序占用且当前用户有访问权限。
- **守护进程日志**：守护进程的标准错误会输出到 GNOME Shell 日志，可用 `journalctl` 查看。给守护进程加 `--debug` 可打印更多信息。
- **角度跳变**：两种方案都内置了抗跳变机制（摄像头用基准继承，声纳用异常跳变保护与冻结），若仍有抖动，可检查光照或环境噪声。
- **套接字路径**：扩展与守护进程通过 `/tmp/gnome_lid_sonar.sock` 通信；异常退出后残留的套接字会被下次启动自动清理。
- **切换方案**：在设置中切换后，扩展会重新启动对应的守护进程。

**English**:

- **Choosing a method**: Machines with the microphone at the top of the screen suit `Sonar`; otherwise `Camera` is the default.
- **Camera unavailable**: `camera_daemon.py` opens `/dev/video0`. If it fails, make sure no other program is using the camera and your user has access.
- **Daemon logs**: The daemon's stderr goes to the GNOME Shell log, visible through `journalctl`. Pass `--debug` to the daemon for verbose output.
- **Angle jumps**: Both methods have anti-jump handling (baseline handover for the camera, jump protection and freezing for sonar). If jitter persists, check lighting or ambient noise.
- **Socket path**: The extension and daemon talk over `/tmp/gnome_lid_sonar.sock`; a stale socket from an abnormal exit is cleaned up on the next start.
- **Switching methods**: After you change the setting, the extension restarts the matching daemon.
