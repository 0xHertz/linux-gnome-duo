"""公共 IPC 工具模块。 (Common IPC utility module.)

本模块通过 Unix 域套接字暴露盖角跟踪器： (Exposes a lid-angle tracker over a Unix domain socket:)
- 扩展端连接后发送 `RESET:<angle>` 重置角度； (the extension connects and sends `RESET:<angle>` to reset the angle;)
- 服务端持续回传当前角度，用于驱动折叠动画。 (the server continuously replies with the current angle used to drive the fold animation.)
"""

import os
import socket
import time

# IPC 套接字路径。 (Path of the IPC socket.)
SOCKET_PATH = "/tmp/gnome_lid_sonar.sock"
# 角度广播间隔（秒），约 60Hz。 (Angle broadcast interval in seconds, ~60Hz.)
BROADCAST_INTERVAL = 0.016


def setup_socket():
    # 清理残留套接字并创建、绑定、监听 Unix 域套接字。
    # (Remove any stale socket, then create, bind and listen on the Unix domain socket.)
    if os.path.exists(SOCKET_PATH):
        try:
            os.remove(SOCKET_PATH)
        except OSError:
            pass
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    server.settimeout(0.5)
    return server


def handle_command(tracker, line):
    # 解析一行客户端指令；`RESET:<angle>` 用于重置跟踪器角度。
    # (Parse one client command line; `RESET:<angle>` resets the tracker angle.)
    line = line.decode("utf-8", errors="ignore").strip()
    if line.startswith("RESET:"):
        try:
            tracker.set_angle(float(line.split(":", 1)[1]))
        except ValueError:
            pass


def serve(server, tracker, stop_event):
    # 主服务循环：接受单个客户端，处理其指令并周期性回传当前角度，直到停止。
    # (Main serve loop: accept a single client, handle its commands and periodically
    #  broadcast the current angle until stopped.)
    while not stop_event.is_set():
        try:
            conn, _ = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        conn.setblocking(True)
        conn.settimeout(BROADCAST_INTERVAL)
        buf = b""
        try:
            while not stop_event.is_set():
                try:
                    # 接收并按换行符切分指令。 (Receive data and split commands on newlines.)
                    data = conn.recv(64)
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        handle_command(tracker, line)
                except (socket.timeout, BlockingIOError):
                    pass
                except OSError:
                    break
                # 将当前角度格式化为 4 位小数回传。 (Format the current angle to 4 decimals and send it back.)
                payload = f"{tracker.get_angle():.4f}\n".encode("utf-8")
                try:
                    conn.sendall(payload)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
                time.sleep(BROADCAST_INTERVAL)
        finally:
            # 无论正常结束还是异常，都关闭连接。 (Always close the connection, on either normal exit or error.)
            try:
                conn.close()
            except OSError:
                pass
