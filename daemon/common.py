import os
import socket
import time

SOCKET_PATH = "/tmp/gnome_lid_sonar.sock"
BROADCAST_INTERVAL = 0.016


def setup_socket():
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
    line = line.decode("utf-8", errors="ignore").strip()
    if line.startswith("RESET:"):
        try:
            tracker.set_angle(float(line.split(":", 1)[1]))
        except ValueError:
            pass


def serve(server, tracker, stop_event):
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
                payload = f"{tracker.get_angle():.4f}\n".encode("utf-8")
                try:
                    conn.sendall(payload)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
                time.sleep(BROADCAST_INTERVAL)
        finally:
            try:
                conn.close()
            except OSError:
                pass
