笔记本开合角实时追踪与视差折叠动画系统（GNOME 46+）开发指南
    
本指南用于指导 AI Agent 或开发人员构建一套在无硬件角度传感器环境下，通过笔记本“扬声器-麦克风”近超声波声纳追踪开合状态，并由 GNOME Shell Extension 自动管理后台 Daemon、进行零点校准与渲染 iPhone / Surface Duo 风格转轴视差与渐变模糊动画 的完整工程架构。

### 一、 系统整体架构设计

┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              GNOME Shell Extension (GJS)                                │
│                                                                                        │
│  ┌───────────────────────────┐     Spawn/Kill     ┌─────────────────────────────────┐  │
│  │   PythonProcessManager    │ ─────────────────> │      Python Sonar Daemon        │  │
│  └───────────────────────────┘                    └─────────────────────────────────┘  │
│                │                                                   │                   │
│         DBus Signal Listener                                Unix Socket                │
│ (org.freedesktop.UPower / LidIsClosed)               (/tmp/gnome_lid_sonar.sock)       │
│                │                                                   │                   │
│                ▼                                                   ▼                   │
│  ┌───────────────────────────┐                    ┌─────────────────────────────────┐  │
│  │      Zero-Calibration     │ ─────────────────> │   Duo Hinge Renderer & Shader   │  │
│  │    (零点/满位硬件漂移校准)   │   Trigger Re-Cal  │   (3D Perspective & Gradient Blur)  │  │
│  └───────────────────────────┘                    └─────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────────────────────┘

### 二、 模块一：Python 近超声波 DSP 守护进程 (sonar_daemon.py)

创建目录 daemon/，并在其中放入以下文件。
1. #### 依赖配置 (requirements.txt)

  Plaintext

​	numpy>=1.24.0
​	sounddevice>=0.4.6
​	scipy>=1.10.0

2. #### DSP 与 Socket 逻辑 (sonar_daemon.py)

  ```Python
  import os
  import sys
  import time
  import socket
  import numpy as np
  import sounddevice as sd
  
  # ---------------- 配置参数 ----------------
  
  FS = 48000           # 采样率
  FREQ = 19500         # 超声波频率 (19.5kHz)
  CHUNK = 1024         # 音频处理块大小
  SOCKET_PATH = "/tmp/gnome_lid_sonar.sock"
  SPEED_OF_SOUND = 343.0
  MAX_DISPLACEMENT = 0.25  # 最大估计开合声学位移(米)
  
  # ---------------- 卡尔曼滤波器 ----------------
  
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
      self.p *= (1 - k)
      return self.x
  
  kalman = OneDimensionalKalman(initial_val=1.0)
  current_angle = 1.0  # 1.0 = 完全平铺展开 (180°), 0.0 = 完全开合折叠
  prev_phase = 0.0
  
  # 预生成载波
  
  t_chunk = np.arange(CHUNK) / FS
  tx_signal = (0.35 * np.sin(2 * np.pi * FREQ * t_chunk)).astype(np.float32)
  
  def audio_callback(indata, outdata, frames, time_info, status):
  global prev_phase, current_angle
  
  outdata[:, 0] = tx_signal
  rx = indata[:, 0]
  
  # IQ 正交解调
  
  t = (np.arange(frames) + time_info.inputBufferAdcTime * FS) / FS
  i_q = rx * np.exp(-1j * 2 * np.pi * FREQ * t)
  
  phase = np.angle(np.mean(i_q))
  d_phase = np.unwrap([prev_phase, phase])[1] - prev_phase
  prev_phase = phase
  
  # 相位转相对位移
  
  wavelength = SPEED_OF_SOUND / FREQ
  delta_d = (d_phase / (2 * np.pi)) * (wavelength / 2.0)
  
  raw_angle = np.clip(current_angle + (delta_d / MAX_DISPLACEMENT), 0.0, 1.0)
  current_angle = kalman.update(raw_angle)
  
  def main():
  if os.path.exists(SOCKET_PATH):
      try:
          os.remove(SOCKET_PATH)
      except OSError:
          pass
  
  server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  server.bind(SOCKET_PATH)
  server.listen(1)
  server.settimeout(0.5)
  
  with sd.Stream(channels=1, samplerate=FS, blocksize=CHUNK, callback=audio_callback):
      while True:
          try:
              conn, _ = server.accept()
          except socket.timeout:
              continue
  
          conn.setblocking(True)
          try:
              while True:
                  # 检查是否有控制指令 (如校准指令 RESET:0.0 / RESET:1.0)
                  conn.settimeout(0.001)
                  try:
                      cmd = conn.recv(64).decode('utf-8').trim()
                      if cmd.startswith("RESET:"):
                          val = float(cmd.split(":")[1])
                          global current_angle
                          current_angle = val
                          kalman.x = val
                  except (socket.timeout, BlockingIOError):
                      pass
      
                  # 广播角度比例
                  payload = f"{current_angle:.4f}\n".encode('utf-8')
                  conn.sendall(payload)
                  time.sleep(0.016)  # ~60 FPS
          except (BrokenPipeError, ConnectionResetError):
              conn.close()
  
  if __name__ == '__main__':
  main()
  ```

### 三、 模块二：GNOME 46+ Extension 实现 (lid-fold-sonar@local)

在目录下配置如下文件。
1. #### metadata.json

  ```JSON
  {
  "uuid": "linux-duo@kechen.github",
  "name": "Ultrasonic Duo Fold Animation",
  "description": "Real-time iPhone/Duo style parallax and gradient blur fold animation using ultrasonic sonar.",
  "shell-version": ["45", "46"],
  "version": 1.0
  }
  ```

2. #### extension.js

  ```JavaScript
  import Clutter from 'gi://Clutter';
  import Gio from 'gi://Gio';
  import GLib from 'gi://GLib';
  import Graphene from 'gi://Graphene';
  import * as Main from 'resource:///org/gnome/shell/ui/main.js';
  import { Extension } from 'resource:///org/gnome/shell/extensions/extension.js';
  
  const SOCKET_PATH = '/tmp/gnome_lid_sonar.sock';
  const UPOWER_BUS_NAME = 'org.freedesktop.UPower';
  const UPOWER_OBJECT_PATH = '/org/freedesktop/UPower';
  const UPOWER_INTERFACE = 'org.freedesktop.UPower';
  
  // 自定义 GLSL 渐变模糊 Shader：距离转轴（底部 y=1.0）越远，模糊程度更高
  const HINGE_GRADIENT_BLUR_SHADER = `
  uniform sampler2D tex;
  uniform float progress; // 1.0 (全开) ~ 0.0 (全关)
  
  void main() {
      vec2 st = cogl_tex_coord0_in.xy;
  
      // 距离转轴 (y=1.0) 的距离
      float distFromHinge = 1.0 - st.y; 
      
      // 动态模糊因子：越靠近顶部 (st.y -> 0)，模糊扩散越剧烈
      float blurRadius = distFromHinge * (1.0 - progress) * 0.025;
      
      vec4 color = vec4(0.0);
      float totalWeight = 0.0;
      
      // 9-Tap 高斯模糊采样
      for (float x = -2.0; x <= 2.0; x += 1.0) {
          for (float y = -2.0; y <= 2.0; y += 1.0) {
              float weight = 1.0 - (abs(x) + abs(y)) * 0.2;
              vec2 offset = vec2(x, y) * blurRadius;
              color += texture2D(tex, st + offset) * weight;
              totalWeight += weight;
          }
      }
      
      cogl_color_out = color / totalWeight;
  
  }
  `;
  
  export default class DuoFoldExtension extends Extension {
  enable() {
      this._cancellable = new Gio.Cancellable();
      this._subprocess = null;
      this._socketConn = null;
      this._dataStream = null;
      this._upowerProxy = null;
  
      // 1. 设置 Shader Effect
      this._shaderEffect = new Clutter.ShaderEffect({
          shader_type: Clutter.ShaderType.FRAGMENT_SHADER
      });
      this._shaderEffect.set_shader_source(HINGE_GRADIENT_BLUR_SHADER);
      
      const uiGroup = Main.layoutManager.uiGroup;
      uiGroup.add_effect_with_name('hinge-blur', this._shaderEffect);
      
      // 2. 启动 Python 子进程
      this._spawnDaemon();
      
      // 3. 监听硬件开关盖 (UPower DBus) 进行校准
      this._initDBusCalibration();
      
      // 4. 连接 Socket 流
      this._connectSocket();
  
  }
  
  _spawnDaemon() {
      const daemonPath = GLib.build_filenamev([this.path, 'daemon', 'sonar_daemon.py']);
      try {
          this._subprocess = new Gio.Subprocess({
              argv: ['python3', daemonPath],
              flags: Gio.SubprocessFlags.NONE,
          });
          this._subprocess.init(this._cancellable);
      } catch (e) {
          console.error(`[DuoFold] Failed to launch daemon: ${e.message}`);
      }
  }
  
  _initDBusCalibration() {
      this._upowerProxy = new Gio.DBusProxy({
          g_connection: Gio.DBus.system,
          g_name: UPOWER_BUS_NAME,
          g_object_path: UPOWER_OBJECT_PATH,
          g_interface_name: UPOWER_INTERFACE,
      });
  
      this._upowerProxy.init_async(Gio.PRIORITY_DEFAULT, this._cancellable, () => {
          this._signalId = this._upowerProxy.connect('g-properties-changed', (proxy, changedProps) => {
              const unpacked = changedProps.unpack();
              if ('LidIsClosed' in unpacked) {
                  const isClosed = unpacked['LidIsClosed'].unpack();
                  // 硬件开关盖触发零点强行校准
                  this._sendCalibration(isClosed ? 0.0 : 1.0);
              }
          });
      });
  
  }
  
  _sendCalibration(val) {
      if (this._socketConn) {
          try {
              const outputStream = this._socketConn.get_output_stream();
              outputStream.write_all(`RESET:${val}\n`, null);
          } catch (e) {
              // Ignore output errors
          }
      }
  }
  
  _connectSocket() {
      const client = new Gio.SocketClient();
      const address = Gio.UnixSocketAddress.new(SOCKET_PATH);
  
      client.connect_async(address, this._cancellable, (obj, res) => {
          try {
              this._socketConn = client.connect_finish(res);
              const inputStream = this._socketConn.get_input_stream();
              this._dataStream = new Gio.DataInputStream({ base_stream: inputStream });
              this._readNextFrame();
          } catch (e) {
              if (e.matches(Gio.IOErrorEnum, Gio.IOErrorEnum.CANCELLED)) return;
      
              // 守护进程尚未就绪，重试连接
              GLib.timeout_add_seconds(GLib.PRIORITY_DEFAULT, 1, () => {
                  if (this._cancellable && !this._cancellable.is_cancelled()) {
                      this._connectSocket();
                  }
                  return GLib.SOURCE_REMOVE;
              });
          }
      });
  
  }
  
  _readNextFrame() {
      if (!this._dataStream) return;
  
      this._dataStream.read_line_async(GLib.PRIORITY_DEFAULT, this._cancellable, (stream, res) => {
          try {
              const [line] = stream.read_line_finish_utf8(res);
              if (line !== null) {
                  const ratio = parseFloat(line.trim());
                  if (!isNaN(ratio)) {
                      this._renderDuoTransform(ratio);
                  }
                  this._readNextFrame();
              }
          } catch (e) {
              if (!this._cancellable.is_cancelled()) {
                  this._connectSocket();
              }
          }
      });
  
  }
  
  _renderDuoTransform(progress) {
      // progress: 1.0 (完全平铺展开) -> 0.0 (完全合拢折叠)
      const actor = Main.layoutManager.uiGroup;
  
      // 设置旋转锚点为屏幕正下方中轴 (转轴位置)
      actor.set_pivot_point(0.5, 1.0);
      
      // 视差变换计算：沿 X 轴后倾倾斜
      const angleX = (1.0 - progress) * -40.0; 
      const translateY = (1.0 - progress) * -60.0;
      const translateZ = (1.0 - progress) * -120.0;
      
      let transform = new Graphene.Matrix();
      transform.init_identity();
      
      // 注入透视距 (Perspective)
      let perspective = new Graphene.Matrix();
      perspective.init_perspective(700, 0.1, 1000);
      
      transform.multiply(perspective);
      transform.rotate_x(angleX);
      transform.translate(new Graphene.Point3D({ x: 0, y: translateY, z: translateZ }));
      
      // 应用矩阵变动与透明度渐变
      actor.set_child_transform(transform);
      actor.opacity = Math.floor(progress * 255);
      
      // 更新 Shader progress 变量
      if (this._shaderEffect) {
          this._shaderEffect.set_uniform_value_float('progress', 1, [progress]);
      }
  
  }
  
  disable() {
      // 1. 取消异步 Cancellable
      if (this._cancellable) {
          this._cancellable.cancel();
          this._cancellable = null;
      }
  
      // 2. 清理 Socket Stream
      if (this._dataStream) {
          this._dataStream.close(null);
          this._dataStream = null;
      }
      
      // 3. 杀掉 Python 子进程
      if (this._subprocess) {
          this._subprocess.send_signal(GLib.SYS_SIGTERM);
          GLib.timeout_add(GLib.PRIORITY_DEFAULT, 300, () => {
              if (this._subprocess) {
                  this._subprocess.force_exit();
                  this._subprocess = null;
              }
              return GLib.SOURCE_REMOVE;
          });
      }
      
      // 4. 断开 DBus 信号
      if (this._upowerProxy && this._signalId) {
          this._upowerProxy.disconnect(this._signalId);
      }
      
      // 5. 还原 UI Group 状态与 Shader
      const actor = Main.layoutManager.uiGroup;
      actor.set_child_transform(null);
      actor.set_pivot_point(0.5, 0.5);
      actor.opacity = 255;
      actor.remove_effect_by_name('hinge-blur');
  
  }
  }
  
  
  ```

### 四、 部署与自动化安装脚本 (setup.sh)

Agent 可在扩展目录下创建并运行 setup.sh 脚本以完成系统的准备工作：
```Bash
#!/usr/bin/env bash
set -e

EXT_DIR="$HOME/.local/share/gnome-shell/extensions/linux-duo@kechen.github"

echo "[1/3] 检查并安装 Python 及 PortAudio 依赖..."
sudo apt-get update && sudo apt-get install -y \
python3 \
python3-pip \
portaudio19-dev \
libasound2-dev

echo "[2/3] 安装 Python DSP 第三方库..."
pip3 install --user -r "$EXT_DIR/daemon/requirements.txt"

echo "[3/3] 启用 GNOME Extension..."
gnome-extensions enable "linux-duo@kechen.github"

echo "安装完毕！请按 Alt+F2 输入 r 回车 (X11) 或重新登录 (Wayland) 重载 Shell 运行。"


```

### 五、 Agent 执行与构建验收 Checklists

当 AI Agent 执行本任务时，需依次验证以下节点：

[ ] Daemon 独立性：检查扩展禁用（disable()）时，Python 子进程是否被 SIGTERM 正常回收，防止后台驻留超声波进程占用麦克风。

[ ] 渲染性能：确保 Shader 中的高斯采样步长控制在 9-Tap 以内，避免在高分辨率屏幕上引发 GNOME Shell 掉帧。

[ ] 防失真：确认 set_pivot_point(0.5, 1.0) 的 X 轴偏转效果作用在 Main.layoutManager.uiGroup 上，且没有被容器 clip 裁剪。xxxxxxxxxx5 1GJS 垃圾回收 (GC) 防脱钩：Gio.Subprocess 实例、Cancellable 和动画中的 onUpdate 闭包引用务必挂载在 Extension 类的主生命周期变量（this）上，否则容易在长时间运行后被 JS 引擎强行回收导致子进程僵死。23Wayland 合成渲染限制：直接操作 set_child_transform 可能会触发硬件裁剪。若动画边界处被切断，需在父容器设置 actor.set_clip_to_allocation(false)。45坐标校准：set_pivot_point(0.5, 1.0) 是基于归一化坐标的（(0.5, 1.0) 即正下方轴线）。如果窗口尺寸在动画过程中发生变化，需重新计算 set_pivot_point。
