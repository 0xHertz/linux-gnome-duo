import Adw from 'gi://Adw';
import Gtk from 'gi://Gtk';
import { ExtensionPreferences } from 'resource:///org/gnome/Shell/Extensions/js/extensions/prefs.js';

export default class LinuxDuoPreferences extends ExtensionPreferences {
    fillPreferencesWindow(window) {
        const settings = this.getSettings('org.gnome.shell.extensions.linux-duo');

        const page = new Adw.PreferencesPage();
        window.add(page);

        const group = new Adw.PreferencesGroup();
        page.add(group);

        const model = new Gtk.StringList();
        model.append('camera');
        model.append('sonar');

        const row = new Adw.ComboRow({
            title: '检测方案 (Detection Method)',
            subtitle: '选择开合角度检测方案（默认摄像头） (Choose the lid-angle detection method, default: camera)',
        });
        row.model = model;
        row.selected = settings.get_string('method') === 'sonar' ? 1 : 0;
        row.connect('notify::selected', () => {
            settings.set_string('method', row.selected === 1 ? 'sonar' : 'camera');
        });
        group.add(row);

        const blurRow = new Adw.SpinRow({
            title: '模糊强度 (Blur Strength)',
            subtitle: '折叠时屏幕顶部的模糊程度（越大越模糊，底部逐渐减弱） (Blur amount at the top of the screen when folded; higher means blurrier, fading toward the bottom)',
            adjustment: new Gtk.Adjustment({
                lower: 0.0,
                upper: 0.2,
                step_increment: 0.01,
                page_increment: 0.05,
            }),
            digits: 2,
            value: settings.get_double('blur-strength'),
        });
        blurRow.connect('notify::value', () => {
            settings.set_double('blur-strength', blurRow.value);
        });
        group.add(blurRow);
    }
}
