#!/usr/bin/env bash
# 安装脚本：部署扩展、安装依赖并启用。 (Install script: deploy the extension, install dependencies and enable it.)
set -euo pipefail

# 扩展 UUID 与安装目录。 (Extension UUID and install directory.)
UUID="linux-duo@kechen.github"
EXT_DIR="$HOME/.local/share/gnome-shell/extensions/$UUID"
# 脚本所在源码目录。 (Source directory where this script resides.)
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 步骤 1/5：检查并安装系统依赖。 (Step 1/5: check and install system dependencies.)
echo "[1/5] 检查并安装系统依赖 (Python, PortAudio, ALSA, GSettings)..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    portaudio19-dev \
    libasound2-dev \
    libglib2.0-bin

# 步骤 2/5：复制文件到 GNOME 扩展目录。 (Step 2/5: copy files into the GNOME extensions directory.)
echo "[2/5] 部署扩展到 GNOME 扩展目录 (Deploying extension to the GNOME extensions directory)..."
mkdir -p "$EXT_DIR"
cp -a "$SRC_DIR/metadata.json" "$EXT_DIR/"
cp -a "$SRC_DIR/extension.js"  "$EXT_DIR/"
cp -a "$SRC_DIR/prefs.js"      "$EXT_DIR/"
cp -a "$SRC_DIR/daemon"        "$EXT_DIR/"
cp -a "$SRC_DIR/schemas"       "$EXT_DIR/"

# 步骤 3/5：编译 GSettings schema。 (Step 3/5: compile the GSettings schema.)
echo "[3/5] 编译 GSettings schema (Compiling GSettings schema)..."
glib-compile-schemas "$EXT_DIR/schemas"

# 步骤 4/5：安装 Python 第三方库。 (Step 4/5: install Python third-party libraries.)
echo "[4/5] 安装 Python 第三方库 (Installing Python dependencies)..."
python3 -m pip install --user -r "$EXT_DIR/daemon/requirements.txt" --break-system-packages

# 步骤 5/5：启用 GNOME 扩展。 (Step 5/5: enable the GNOME extension.)
echo "[5/5] 启用 GNOME Extension (Enabling GNOME Extension)..."
gnome-extensions enable "$UUID" || true

# 完成提示：说明如何重载 Shell。 (Done notice: how to reload the Shell.)
echo ""
echo "安装完毕！请按 Alt+F2 输入 r 回车 (X11) 或重新登录 (Wayland) 重载 Shell 运行。 (Installation complete! Press Alt+F2, type r and Enter (X11), or log out and back in (Wayland) to reload the Shell.)"
