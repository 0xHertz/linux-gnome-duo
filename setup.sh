#!/usr/bin/env bash
set -euo pipefail

UUID="linux-duo@kechen.github"
EXT_DIR="$HOME/.local/share/gnome-shell/extensions/$UUID"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/5] 检查并安装系统依赖 (Python, PortAudio, ALSA, GSettings)..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    portaudio19-dev \
    libasound2-dev \
    libglib2.0-bin

echo "[2/5] 部署扩展到 GNOME 扩展目录..."
mkdir -p "$EXT_DIR"
cp -a "$SRC_DIR/metadata.json" "$EXT_DIR/"
cp -a "$SRC_DIR/extension.js"  "$EXT_DIR/"
cp -a "$SRC_DIR/prefs.js"      "$EXT_DIR/"
cp -a "$SRC_DIR/daemon"        "$EXT_DIR/"
cp -a "$SRC_DIR/schemas"       "$EXT_DIR/"

echo "[3/5] 编译 GSettings schema..."
glib-compile-schemas "$EXT_DIR/schemas"

echo "[4/5] 安装 Python 第三方库..."
python3 -m pip install --user -r "$EXT_DIR/daemon/requirements.txt" --break-system-packages

echo "[5/5] 启用 GNOME Extension..."
gnome-extensions enable "$UUID" || true

echo ""
echo "安装完毕！请按 Alt+F2 输入 r 回车 (X11) 或重新登录 (Wayland) 重载 Shell 运行。"
