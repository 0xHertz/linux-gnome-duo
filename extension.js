import Clutter from 'gi://Clutter';
import Gio from 'gi://Gio';
import GLib from 'gi://GLib';
import GObject from 'gi://GObject';
import St from 'gi://St';
import Shell from 'gi://Shell';
import * as Main from 'resource:///org/gnome/shell/ui/main.js';
import { Extension } from 'resource:///org/gnome/shell/extensions/extension.js';

const SOCKET_PATH = '/tmp/gnome_lid_sonar.sock';
const UPOWER_BUS_NAME = 'org.freedesktop.UPower';
const UPOWER_OBJECT_PATH = '/org/freedesktop/UPower';
const UPOWER_INTERFACE = 'org.freedesktop.UPower';
const SIGTERM = 15;
const DAEMON_RETRY_MS = 1000;
const DEMO_MODE = false;

export default class DuoFoldExtension extends Extension {
    enable() {
        this._settings = this.getSettings('org.gnome.shell.extensions.linux-duo');
        this._cancellable = new Gio.Cancellable();
        this._subprocess = null;
        this._socketConn = null;
        this._dataStream = null;
        this._upowerProxy = null;
        this._signalId = 0;
        this._methodChangedId = 0;
        this._blurStrengthChangedId = 0;
        this._blurLayers = [];
        this._blurStrength = this._settings.get_double('blur-strength');
        this._blurLayerCount = 16;
        this._maxBlurRadius = 3000;
        this._fadeOverlay = null;
        this._blackBg = null;
        this._lastLoggedAngle = undefined;

        const uiGroup = Main.layoutManager.uiGroup;
        uiGroup.set_clip_to_allocation(false);

        this._blackBg = new St.Widget({
            style: 'background-color: #000000;',
            reactive: false,
        });
        this._blackBg.set_size(global.stage.width, global.stage.height);
        global.stage.insert_child_below(this._blackBg, uiGroup);
        this._blackBg.hide();

        this._createBlurLayers();

        this._fadeOverlay = new St.Widget({
            style: 'background-gradient-direction: vertical; background-gradient-start: rgba(0,0,0,0.35); background-gradient-end: rgba(0,0,0,0);',
            reactive: false,
        });
        this._fadeOverlay.set_size(global.stage.width, global.stage.height);
        global.stage.add_child(this._fadeOverlay);
        this._fadeOverlay.hide();

        this._spawnDaemon();
        this._initDBusCalibration();
        this._connectSocket();

        this._methodChangedId = this._settings.connect('changed::method', () => {
            this._spawnDaemon();
        });
        this._blurStrengthChangedId = this._settings.connect('changed::blur-strength', () => {
            this._blurStrength = this._settings.get_double('blur-strength');
            this._updateBlurLayers(0.0);
        });
    }

    _spawnDaemon() {
        this._killDaemon();

        const method = this._settings.get_string('method');
        const daemonDir = GLib.build_filenamev([this.path, 'daemon']);
        if (method === 'sonar') {
            const argv = ['python3', GLib.build_filenamev([daemonDir, 'sonar_daemon.py'])];
            if (DEMO_MODE)
                argv.push('--demo');
            this._launchSubprocess(argv);
        } else {
            this._launchSubprocess(['python3', GLib.build_filenamev([daemonDir, 'camera_daemon.py'])]);
        }
    }

    _killDaemon() {
        if (!this._subprocess)
            return;
        const subprocess = this._subprocess;
        this._subprocess = null;
        subprocess.send_signal(SIGTERM);
        GLib.timeout_add(GLib.PRIORITY_DEFAULT, 300, () => {
            subprocess.force_exit();
            return GLib.SOURCE_REMOVE;
        });
    }

    _launchSubprocess(argv) {
        let subprocess = null;
        try {
            subprocess = Gio.Subprocess.new(argv, Gio.SubprocessFlags.STDERR_PIPE);
        } catch (e) {
            console.error(`[DuoFold] Failed to launch daemon: ${e.message}`);
            return;
        }

        if (!subprocess) {
            console.error('[DuoFold] Gio.Subprocess.new returned null');
            return;
        }

        this._subprocess = subprocess;
        this._logDaemonStderr(subprocess);
    }

    _logDaemonStderr(subprocess) {
        const stderr = subprocess.get_stderr_pipe();
        if (!stderr)
            return;

        const stream = new Gio.DataInputStream({ base_stream: stderr });
        const readLine = () => {
            stream.read_line_async(GLib.PRIORITY_DEFAULT, null, (s, res) => {
                try {
                    const [line] = stream.read_line_finish_utf8(res);
                    if (line !== null) {
                        console.error(`[DuoFold daemon] ${line.trim()}`);
                        readLine();
                    }
                } catch (e) {
                    // pipe closed
                }
            });
        };
        readLine();
    }

    _initDBusCalibration() {
        this._upowerProxy = new Gio.DBusProxy({
            g_connection: Gio.DBus.system,
            g_name: UPOWER_BUS_NAME,
            g_object_path: UPOWER_OBJECT_PATH,
            g_interface_name: UPOWER_INTERFACE,
        });

        this._upowerProxy.init_async(GLib.PRIORITY_DEFAULT, this._cancellable, () => {
            this._signalId = this._upowerProxy.connect(
                'g-properties-changed',
                (proxy, changedProps) => {
                    const changed = changedProps.deep_unpack();
                    if ('LidIsClosed' in changed)
                        this._sendCalibration(changed.LidIsClosed ? 0.0 : 1.0);
                });
        });
    }

    _sendCalibration(val) {
        if (!this._socketConn)
            return;
        try {
            const outputStream = this._socketConn.get_output_stream();
            outputStream.write_all(`RESET:${val}\n`, null);
        } catch (e) {
            // ignore output errors
        }
    }

    _connectSocket() {
        if (!this._cancellable || this._cancellable.is_cancelled())
            return;

        const client = new Gio.SocketClient();
        const address = Gio.UnixSocketAddress.new(SOCKET_PATH);

        client.connect_async(address, this._cancellable, (obj, res) => {
            try {
                this._socketConn = client.connect_finish(res);
                const inputStream = this._socketConn.get_input_stream();
                this._dataStream = new Gio.DataInputStream({ base_stream: inputStream });
                this._readNextFrame();
            } catch (e) {
                if (e.matches(Gio.IOErrorEnum, Gio.IOErrorEnum.CANCELLED))
                    return;
                this._retryConnect();
            }
        });
    }

    _retryConnect() {
        GLib.timeout_add(GLib.PRIORITY_DEFAULT, DAEMON_RETRY_MS, () => {
            if (this._cancellable && !this._cancellable.is_cancelled())
                this._connectSocket();
            return GLib.SOURCE_REMOVE;
        });
    }

    _readNextFrame() {
        if (!this._dataStream || !this._cancellable || this._cancellable.is_cancelled())
            return;

        this._dataStream.read_line_async(GLib.PRIORITY_DEFAULT, this._cancellable, (stream, res) => {
            try {
                const [line] = stream.read_line_finish_utf8(res);
                if (line !== null) {
                    const ratio = parseFloat(line.trim());
                    if (!isNaN(ratio))
                        this._renderDuoTransform(ratio);
                    this._readNextFrame();
                } else {
                    this._retryConnect();
                }
            } catch (e) {
                if (this._cancellable && !this._cancellable.is_cancelled())
                    this._retryConnect();
            }
        });
    }

    _renderDuoTransform(progress) {
        const actor = Main.layoutManager.uiGroup;
        const fold = 1.0 - progress;

        actor.set_pivot_point(0.5, 1.0);
        actor.set_rotation_angle(Clutter.RotateAxis.X_AXIS, fold * 40.0);
        actor.opacity = Math.floor(progress * 255);

        const folding = progress < 0.99;

        this._updateBlurLayers(fold);

        if (this._fadeOverlay) {
            this._fadeOverlay.visible = folding;
            this._fadeOverlay.opacity = Math.floor(fold * 255);
        }

        if (this._blackBg)
            this._blackBg.visible = folding;

        this._logAngle(progress);
    }

    _createBlurLayers() {
        const width = global.stage.width;
        const height = global.stage.height;

        this._blurLayers = [];

        for (let i = 0; i < this._blurLayerCount; i++) {
            const layer = new St.Widget({
                reactive: false,
                style: 'background-color: rgba(0,0,0,0);',
            });

            const y0 = Math.floor(height * i / this._blurLayerCount);
            const y1 = Math.ceil(height * (i + 1) / this._blurLayerCount);
            layer.set_position(0, y0);
            layer.set_size(width, Math.max(1, y1 - y0 + 1));

            const effect = new Shell.BlurEffect({
                mode: 1,
                radius: 0,
                brightness: 1.0,
            });
            layer.add_effect(effect);

            global.stage.add_child(layer);
            layer.hide();

            this._blurLayers.push({ layer, effect, index: i });
        }
    }

    _updateBlurLayers(fold) {
        if (!this._blurLayers.length)
            return;

        const enabled = fold > 0.001;
        const angleFactor = Math.pow(Math.max(0, Math.min(1, fold)), 1.35);
        const strength = Math.max(0, this._blurStrength);

        for (const item of this._blurLayers) {
            const t = item.index / Math.max(1, this._blurLayerCount - 1);
            const verticalFactor = Math.pow(1.0 - t, 1.35);

            // 顶部最大，向下逐渐减弱；同时折叠越小，整体模糊越强。
            const radius = this._maxBlurRadius * strength * angleFactor *
                (0.08 + 0.92 * verticalFactor);

            item.effect.radius = Math.round(radius);
            item.layer.visible = enabled && radius > 0.1;
        }
    }

    _logAngle(progress) {
        if (this._lastLoggedAngle !== undefined &&
            Math.abs(progress - this._lastLoggedAngle) < 0.01)
            return;
        this._lastLoggedAngle = progress;
        console.log(`[DuoFold] lid angle: ${progress.toFixed(4)}`);
    }

    disable() {
        if (this._cancellable) {
            this._cancellable.cancel();
            this._cancellable = null;
        }

        if (this._dataStream) {
            try {
                this._dataStream.close(null);
            } catch (e) {
                // close() reports PENDING while a read_line_async is still in flight
            }
            this._dataStream = null;
        }
        if (this._socketConn) {
            try {
                this._socketConn.close(null);
            } catch (e) {
                // ignore teardown errors
            }
            this._socketConn = null;
        }

        if (this._methodChangedId) {
            this._settings.disconnect(this._methodChangedId);
            this._methodChangedId = 0;
        }
        if (this._blurStrengthChangedId) {
            this._settings.disconnect(this._blurStrengthChangedId);
            this._blurStrengthChangedId = 0;
        }

        this._killDaemon();

        if (this._upowerProxy && this._signalId)
            this._upowerProxy.disconnect(this._signalId);
        this._signalId = 0;
        this._upowerProxy = null;

        const actor = Main.layoutManager.uiGroup;
        for (const item of this._blurLayers) {
            try {
                item.layer.destroy();
            } catch (e) {
                // ignore teardown errors
            }
        }
        this._blurLayers = [];
        if (this._fadeOverlay) {
            this._fadeOverlay.destroy();
            this._fadeOverlay = null;
        }
        if (this._blackBg) {
            this._blackBg.destroy();
            this._blackBg = null;
        }
        actor.set_rotation_angle(Clutter.RotateAxis.X_AXIS, 0);
        actor.set_pivot_point(0.5, 0.5);
        actor.opacity = 255;
        actor.set_clip_to_allocation(true);
    }
}
