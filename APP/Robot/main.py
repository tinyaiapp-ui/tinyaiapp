from translations import TRANSLATIONS
import os
import sys
import argparse
import json as _json
from pathlib import Path
if getattr(sys, 'frozen', False):
    script_dir = Path(getattr(sys, '_MEIPASS', os.path.dirname(sys.executable))).resolve()
else:
    script_dir = Path(__file__).resolve().parent
try:
    if not getattr(sys, 'frozen', False):
        os.chdir(str(script_dir))
except Exception as err:
    print(f'[warn] Failed to set working directory: {err}')

# U ovoj funkciji je inicijalizacija i priprema potrebnih resursa.
def _create_default_data(target_root: Path) -> None:
    try:
        target_root = target_root.resolve()
        machine_dir = target_root / 'machine'
        work_dir = target_root / 'work'
        seq_dir = target_root / 'seq'
        machine_dir.mkdir(parents=True, exist_ok=True)
        work_dir.mkdir(parents=True, exist_ok=True)
        seq_dir.mkdir(parents=True, exist_ok=True)
        last_settings_path = machine_dir / 'last_settings.json'
        if not last_settings_path.exists():
            default = {'max_angles_deg': [180.0] * 6, 'min_angles_deg': [-180.0] * 6, 'arm_lengths': [50, 80, 50, 30, 20], 'gripper_length': 25.0, 'last_positions': [511, 511, 511, 511, 511, 511], 'acceleration': 100.0, 'speed': 100, 'home_positions': [511, 511, 511, 511, 511, 511], 'language': 'hr'}
            try:
                with open(last_settings_path, 'w', encoding='utf-8') as f:
                    _json.dump(default, f, indent=2)
                print(f'[info] Created default last_settings.json at: {last_settings_path}')
            except Exception as err:
                print(f'[warn] Failed to write default last_settings.json: {err}')
        else:
            print(f'[info] last_settings.json already exists at: {last_settings_path}')
    except Exception as err:
        print(f'[warn] _create_default_data failed: {err}')

# U ovoj funkciji je obrada argumenata komandne linije.
def _maybe_handle_cli_and_exit():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--create-data', action='store_true', help='Create machine/work/seq folders and a default last_settings.json then exit')
    p.add_argument('--use-appdata', action='store_true', help='When used with --create-data, create folders under %%APPDATA%% instead of next to the exe')
    args, _rest = p.parse_known_args()
    if args.create_data:
        if args.use_appdata:
            appdata = os.getenv('APPDATA') or Path.home()
            root = Path(appdata) / 'RobotRuka'
        else:
            root = script_dir
        _create_default_data(root)
        print('[info] Data creation finished. Exiting.')
        sys.exit(0)
_maybe_handle_cli_and_exit()
import tkinter as tk
from tkinter import filedialog, ttk, messagebox, simpledialog
try:
    import serial
    import serial.tools.list_ports
except Exception as err:
    print(f'[warn] pyserial not available: {err}')
    serial = None
import datetime
import json
import math
import sys
import time
import inspect
import numpy as np
from typing import Any, Dict, List, Optional, Tuple
try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    MATPLOTLIB_OK = True
except Exception as err:
    print(f'[warn] matplotlib unavailable: {err}')
    MATPLOTLIB_OK = False
try:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
except Exception as err:
    print(f'[warn] Poly3DCollection unavailable: {err}')
    Poly3DCollection = None
BAUD_RATE = 115200
class ArbotiXGUI:

    # U ovoj metodi je matrične transformacije (translacija/rotacija).
    def _t(self, x: float=0, y: float=0, z: float=0) -> List[List[float]]:
        return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
    def _rot(self, axis: str, deg: float) -> List[List[float]]:
        r = math.radians(deg)
        c, s = (math.cos(r), math.sin(r))
        if axis == 'x':
            return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]
        if axis == 'y':
            return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]
        return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    # U ovoj metodi je matrične transformacije (translacija/rotacija).
    def _matmul(self, A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
        return [[sum((A[i][k] * B[k][j] for k in range(4))) for j in range(4)] for i in range(4)]
    def _deg(self, pos_raw: float, idx: int) -> float:
        try:
            maxa = float(self.max_angles_deg[idx]) if idx < len(self.max_angles_deg) else 180.0
        except Exception as err:
            self._log_exception(None, err)
            maxa = 180.0
        try:
            mina = float(self.min_angles_deg[idx]) if idx < len(self.min_angles_deg) else -180.0
        except Exception as err:
            self._log_exception(None, err)
            mina = -180.0
        span = max(maxa - mina, 1e-06)
        norm = max(0.0, min(1.0, float(pos_raw) / 1023.0))
        ang = mina + norm * span
        signs = getattr(self, 'joint_sign', [1] * 6)
        offsets = getattr(self, 'joint_offsets_deg', [0.0] * 6)
        sgn = signs[idx] if idx < len(signs) else 1
        off = offsets[idx] if idx < len(offsets) else 0.0
        return sgn * ang + off

    # U ovoj metodi je naprijedna kinematika i izračun položaja.
    def _fk(self, positions: List[int]) -> Tuple[List[float], List[float], List[float], Tuple[float, float, float], List[List[float]]]:
        T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        xs, ys, zs = ([0], [0], [0])
        axes = getattr(self, 'joint_axes', ['z'] * 6)
        for i, raw in enumerate(positions[:5]):
            ang = self._deg(raw, i)
            axis = axes[i] if i < len(axes) else 'z'
            T = self._matmul(T, self._rot(axis, ang))
            L = self.arm_lengths[i] if i < len(self.arm_lengths) else 0
            T = self._matmul(T, self._t(0, 0, L) if i == 0 else self._t(L, 0, 0))
            xs.append(T[0][3])
            ys.append(T[1][3])
            zs.append(T[2][3])
        origin = (T[0][3], T[1][3], T[2][3])
        R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]
        return (xs, ys, zs, origin, R)

    # U ovoj metodi je rad s TCP-om (pomicanje/računanje vrha alata).
    def _tcp_move_horizontal(self, direction: str, step_mm: float) -> bool:
        return False
        try:
            current_pos = self._get_current_positions()
            if len(current_pos) < 5:
                self._log('ERROR: Nedovoljno pozicija motora')
                return False
            self._log(f'DEBUG: Početne pozicije: {current_pos[:6]}')
            xs, ys, zs, origin, R = self._fk(current_pos)
            start_tcp_x, start_tcp_y, start_tcp_z = origin
            self._log(f'DEBUG: Početni TCP: X={start_tcp_x:.1f}, Y={start_tcp_y:.1f}, Z={start_tcp_z:.1f}')
            target_x = start_tcp_x + (step_mm if direction == 'tcp_forward' else -step_mm)
            self._log(f'DEBUG: Target X: {target_x:.1f} (delta: {target_x - start_tcp_x:.1f})')
            motor_step = max(1, min(3, int(step_mm / 8)))
            self._log(f'DEBUG: Motor step size: {motor_step}')
            best_pos = current_pos.copy()
            best_score = float('inf')
            for motor_idx in [1, 2, 3]:
                for direction_sign in [1, -1]:
                    test_pos = current_pos.copy()
                    test_pos[motor_idx] += direction_sign * motor_step
                    if test_pos[motor_idx] < 0 or test_pos[motor_idx] > 1023:
                        continue
                    try:
                        _, _, _, test_origin, _ = self._fk(test_pos)
                        test_x, test_y, test_z = test_origin
                        distance_to_target_x = abs(test_x - target_x)
                        z_change = abs(test_z - start_tcp_z)
                        score = distance_to_target_x + z_change * 10.0
                        self._log(f'DEBUG: Motor {motor_idx} step {direction_sign * motor_step}: TCP=({test_x:.1f},{test_y:.1f},{test_z:.1f}), dist_x={distance_to_target_x:.2f}, z_change={z_change:.2f}, score={score:.2f}')
                        if score < best_score and z_change < 5.0:
                            best_pos = test_pos.copy()
                            best_score = score
                            self._log(f'DEBUG: NEW BEST! Motor {motor_idx}, score: {score:.2f}')
                    except Exception as e:
                        self._log(f'DEBUG: FK error for motor {motor_idx}: {e}')
                        continue
            if best_score == float('inf'):
                self._log('DEBUG: No acceptable move found')
                return False
            changed_motors = []
            for i in range(6):
                if best_pos[i] != current_pos[i]:
                    changed_motors.append(i)
                    self._log(f'DEBUG: Changing motor {i}: {current_pos[i]} -> {best_pos[i]}')
                    if i < len(self.sliders):
                        self.sliders[i].set(best_pos[i])
                        self.safe_config(self.value_labels[i], text=str(best_pos[i]))
            _, _, _, final_origin, _ = self._fk(best_pos)
            final_x, final_y, final_z = final_origin
            actual_dx = final_x - start_tcp_x
            actual_dy = final_y - start_tcp_y
            actual_dz = final_z - start_tcp_z
            self._log(f'DEBUG: Changed motors: {changed_motors}')
            self._log(f'DEBUG: Final TCP: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}')
            self._log(f'DEBUG: Actual movement: dX={actual_dx:.1f}, dY={actual_dy:.1f}, dZ={actual_dz:.1f}')
            self._log(f'DEBUG: Final score: {best_score:.2f}')
            self._schedule_robot_draw(delay=1)
            return True
        except Exception as err:
            self._log_exception('_tcp_move_horizontal', err)
            return False
        except Exception as err:
            self._log_exception('_tcp_move_horizontal', err)
            return False

    # U ovoj metodi je pretvorbe i računanje kuteva.
    def _deg_to_raw(self, angle_deg: float, motor_idx: int) -> int:
        try:
            min_deg = float(self.min_angles_deg[motor_idx]) if motor_idx < len(self.min_angles_deg) else -180.0
            max_deg = float(self.max_angles_deg[motor_idx]) if motor_idx < len(self.max_angles_deg) else 180.0
            signs = getattr(self, 'joint_sign', [1] * 6)
            offsets = getattr(self, 'joint_offsets_deg', [0.0] * 6)
            sgn = signs[motor_idx] if motor_idx < len(signs) else 1
            off = offsets[motor_idx] if motor_idx < len(offsets) else 0.0
            corrected_angle = (angle_deg - off) / sgn
            if corrected_angle < min_deg or corrected_angle > max_deg:
                return None
            span = max_deg - min_deg
            norm = (corrected_angle - min_deg) / span
            raw_pos = int(round(norm * 1023.0))
            return max(0, min(1023, raw_pos))
        except Exception as err:
            self._log_exception('_deg_to_raw', err)
            return None

    # U ovoj metodi je logiranje i ispis poruka.
    def _log(self, msg: str) -> None:
        if getattr(self, 'verbose', True):
            print(msg)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _has_3d(self) -> bool:
        if hasattr(self, 'canvas3d'):
            return True
        ax = getattr(self, 'ax3d', None)
        return bool(getattr(getattr(ax, 'figure', None), 'canvas', None))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i upravljanje ubrzanjem.
    def _edit_acceleration(self) -> None:
        return
    def flush(self) -> None:
        pass

    # U ovoj metodi je logiranje i ispis poruka.
    def _log_exception(self, context: Optional[str], err: Exception) -> None:
        if not context:
            try:
                frame = inspect.currentframe().f_back
                context = frame.f_code.co_name
            except Exception:
                context = '<unknown>'
        self._log(f'[warn] {context}: {err}')

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def safe_config(self, widget: tk.Misc, **kw: Any) -> None:
        try:
            widget.config(**kw)
        except tk.TclError as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _cancel_after(self, attr: str) -> None:
        handle = getattr(self, attr, None)
        if handle is not None:
            try:
                self.root.after_cancel(handle)
            except Exception as err:
                self._log_exception(None, err)
            setattr(self, attr, None)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _tr_console(self, msg: str) -> str:
        m = msg.lstrip('?').lstrip()
        try:
            import unicodedata
            m = ''.join((ch for ch in m if unicodedata.category(ch) not in ('So', 'Sk')))
        except Exception as err:
            self._log_exception(None, err)
        return m

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _seq_move_item(self, direction: int) -> None:
        idx = self.seq_get_selected_index()
        seq = self.get_active_sequence()
        if idx is not None and 0 <= idx + direction < len(seq):
            seq[idx], seq[idx + direction] = (seq[idx + direction], seq[idx])
            self.seq_refresh()
            target = idx + direction
            self.seq_listbox.selection_clear(0, tk.END)
            self.seq_listbox.select_set(target)
            self.seq_listbox.see(target)
            self.set_current_index(target)
            self.update_pose_label()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _seq_insert_item(self, item: Dict[str, Any]) -> None:
        idx = self.seq_get_selected_index()
        seq = self.get_active_sequence()
        if idx is not None:
            seq.insert(idx + 1, item)
            insert_idx = idx + 1
        else:
            seq.append(item)
            insert_idx = len(seq) - 1
        self.seq_refresh()
        self.seq_listbox.selection_clear(0, tk.END)
        self.seq_listbox.select_set(insert_idx)
        self.seq_listbox.see(insert_idx)
        self.set_current_index(insert_idx)
        self.update_pose_label()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _get_machine_settings(self) -> Dict[str, Any]:
        return {'max_angles_deg': list(self.max_angles_deg), 'min_angles_deg': list(self.min_angles_deg), 'arm_lengths': list(self.arm_lengths[:5]), 'gripper_length': float(self.gripper_length), 'last_positions': self._get_current_positions() if hasattr(self, '_get_current_positions') else [int(s.get()) for s in getattr(self, 'sliders', [])], 'acceleration': float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get()), 'speed': int(getattr(self, 'speed_var', tk.DoubleVar(value=100)).get()), 'home_positions': [int(x) for x in getattr(self, 'home_positions', [511] * 6)], 'language': getattr(self, 'language', 'hr')}
    def _get_mechanical_settings(self) -> Dict[str, Any]:
        return {'max_angles_deg': list(self.max_angles_deg), 'min_angles_deg': list(self.min_angles_deg), 'arm_lengths': list(self.arm_lengths[:5]), 'gripper_length': float(self.gripper_length), 'acceleration': float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get())}

    # U ovoj metodi je matrične transformacije (translacija/rotacija).
    def t(self, key: str, **kwargs) -> str:
        lang_map = self.translations.get(self.language, {})
        text = lang_map.get(key)
        if text is None:
            text = self.translations.get('en', {}).get(key) or self.translations.get('hr', {}).get(key) or key
        if kwargs:
            try:
                text = text.format(**kwargs)
            except Exception as err:
                self._log_exception(f't.format.{key}', err)
        return text

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _default_sequence_name(self, index: int) -> str:
        base = self.t('sequence')
        return f'{base} {index}'

    # U ovoj metodi je učitavanje podataka ili konfiguracije.
    def load_last_settings(self) -> None:
        try:
            if os.path.exists(self.last_settings_path):
                with open(self.last_settings_path, 'r') as f:
                    data = json.load(f)
                settings = None
                if isinstance(data, dict):
                    if 'settings' in data and isinstance(data['settings'], dict):
                        settings = data['settings']
                    else:
                        settings = data
                if settings is not None:
                    self._apply_machine_settings(settings)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _apply_machine_settings(self, data: Dict[str, Any]) -> None:
        try:
            mad = data.get('max_angles_deg')
            if isinstance(mad, list) and len(mad) >= 1:
                self.max_angles_deg = [float(x) for x in mad[:6]] + [180.0] * max(0, 6 - len(mad))
            self.max_angles_deg = [min(180.0, float(x)) for x in self.max_angles_deg[:6]] + [180.0] * max(0, 6 - len(self.max_angles_deg))
            if hasattr(self, 'max_angle_vars'):
                for i, v in enumerate(self.max_angle_vars):
                    try:
                        v.set(str(int(self.max_angles_deg[i])) + '°')
                    except Exception as err:
                        self._log_exception(None, err)
            mind = data.get('min_angles_deg')
            if isinstance(mind, list) and len(mind) >= 1:
                self.min_angles_deg = [float(x) for x in mind[:6]]
            else:
                self.min_angles_deg = []
            self.min_angles_deg = self.min_angles_deg[:6]
            while len(self.min_angles_deg) < 6:
                idx = len(self.min_angles_deg)
                default_min = -abs(self.max_angles_deg[idx]) if idx < len(self.max_angles_deg) else -180.0
                self.min_angles_deg.append(default_min)
            for i in range(6):
                if self.min_angles_deg[i] >= self.max_angles_deg[i]:
                    self.min_angles_deg[i] = self.max_angles_deg[i] - 1.0
                if self.min_angles_deg[i] < -180.0:
                    self.min_angles_deg[i] = -180.0
            if hasattr(self, 'min_angle_vars'):
                for i, v in enumerate(self.min_angle_vars):
                    try:
                        v.set(str(int(self.min_angles_deg[i])) + '°')
                    except Exception as err:
                        self._log_exception(None, err)
            al = data.get('arm_lengths')
            if isinstance(al, list) and len(al) >= 1:
                for i in range(min(5, len(al))):
                    try:
                        self.arm_lengths[i] = float(al[i])
                        if hasattr(self, 'length_vars') and i < len(self.length_vars):
                            lv = self.length_vars[i]
                            if lv is not None:
                                try:
                                    lv.set(str(int(self.arm_lengths[i])))
                                except Exception as err:
                                    self._log_exception(None, err)
                    except Exception as err:
                        self._log_exception(None, err)
            if 'gripper_length' in data:
                try:
                    self.gripper_length = float(data.get('gripper_length'))
                    if getattr(self, 'gripper_length_var', None) is not None:
                        try:
                            self.gripper_length_var.set(str(int(self.gripper_length)))
                        except Exception as err:
                            self._log_exception(None, err)
                except Exception as err:
                    self._log_exception(None, err)
            try:
                lang = data.get('language')
                if isinstance(lang, str) and lang.strip().lower() in getattr(self, 'translations', {}):
                    self.language = lang.strip().lower()
                    try:
                        self.retranslate_ui()
                    except Exception as err:
                        self._log_exception(None, err)
            except Exception as err:
                self._log_exception(None, err)
            try:
                if 'acceleration' in data:
                    try:
                        accel_val = float(data.get('acceleration', 100.0))
                        if getattr(self, 'accel_var', None) is not None:
                            try:
                                self.accel_var.set(accel_val)
                            except Exception:
                                pass
                    except Exception as err:
                        self._log_exception('_apply_machine_settings.acceleration', err)
            except Exception:
                pass
            try:
                if 'speed' in data:
                    try:
                        sp = int(data.get('speed', 100))
                        if getattr(self, 'speed_var', None) is not None:
                            try:
                                self.speed_var.set(sp)
                                self.update_speed_label(sp)
                            except Exception:
                                pass
                    except Exception as err:
                        self._log_exception('_apply_machine_settings.speed', err)
            except Exception:
                pass
            try:
                hp = data.get('home_positions')
                if isinstance(hp, list) and len(hp) >= 1:
                    self.home_positions = [int(x) for x in hp[:6]] + [511] * max(0, 6 - len(hp))
            except Exception:
                pass
            try:
                lang = data.get('language')
                if isinstance(lang, str) and lang.strip():
                    self._set_language(lang.strip(), persist=False)
            except Exception:
                pass
            lp = data.get('last_positions')
            prev_suppress = getattr(self, '_suppress_slider_send', False)
            self._suppress_slider_send = True
            if isinstance(lp, list) and lp:
                for i, val in enumerate(lp):
                    try:
                        if i < len(getattr(self, 'sliders', [])):
                            self.sliders[i].set(int(val))
                            if i < len(getattr(self, 'value_labels', [])) and self.value_labels[i] is not None:
                                self.safe_config(self.value_labels[i], text=str(int(val)))
                    except Exception as err:
                        self._log_exception('_apply_machine_settings.last_positions.item', err)
                if getattr(self, 'online_mode', False) and getattr(self, 'ser', None) is not None:
                    for i, val in enumerate(lp):
                        if i < 6:
                            try:
                                speed = int(getattr(self, 'speed_var', tk.DoubleVar(value=100)).get())
                            except Exception:
                                speed = 100
                            try:
                                accel_val = float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get())
                            except Exception:
                                accel_val = None
                            try:
                                self.send_position(i + 1, int(val), speed, accel=accel_val)
                            except Exception as err:
                                self._log_exception('_apply_machine_settings.last_positions.send', err)
            try:
                self._schedule_robot_draw(delay=1)
            except Exception as err:
                self._log_exception(None, err)
            self._suppress_slider_send = prev_suppress
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je spremanje podataka ili konfiguracije.
    def save_last_settings(self) -> None:
        self._ensure_machine_dir()
        try:
            with open(self.last_settings_path, 'w') as f:
                json.dump(self._get_machine_settings(), f, indent=2)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _set_language(self, new_lang: str, persist: bool=False) -> None:
        code = (new_lang or '').strip().lower()
        if not code:
            return
        if code not in self.translations:
            supported = ', '.join(sorted(self.translations.keys()))
            self._log(self.t('log.language.unsupported', code=code, supported=supported))
            return
        if getattr(self, 'language', None) == code:
            return
        self.language = code
        if hasattr(self, 'lang_menu_var') and self.lang_menu_var.get() != code:
            try:
                self.lang_menu_var.set(code)
            except Exception as err:
                self._log_exception('_set_language.menu_var', err)
        try:
            self.retranslate_ui()
        except Exception as err:
            self._log_exception('_set_language.retranslate', err)
        if persist:
            try:
                self.save_last_settings()
            except Exception as err:
                self._log_exception('_set_language.save', err)

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def _init_defaults(self) -> None:
        self.preserve_view = True
        self.initial_view_zoom = 0.45
        self.initial_view_dist = 1.5
        self.minimal_view = True
        self.ui_scale = 1
        self.sequence_thread_active = False
        self.paused = False
        self.online_mode = False
        self.ser = None
        self.time_per_unit = 0.0008
        self.anim_fps = 60  # Increased from 30 to 60 for faster serial updates (smoother motion)
        self.arm_lengths = [50, 80, 50, 30, 20, 10]
        self.grid_cell_size = 50.0
        self.joystick_step_mm = 5.0
        self.joint_axes = ['z', 'y', 'y', 'y', 'x', 'z']
        self.joint_offsets_deg = [0, -90, 0, 0, 0, 0]
        self.joint_sign = [+1, +1, +1, +1, +1, +1]
        self._arm_line3d = None
        self._tcp_quivers = []
        self._base_patch = None
        self._arm_patches = []
        self._transient_artists = []
        self._ucs_quivers = []
        self._suppress_slider_send = False
        self._gripper_lines = []
        self._draw_after_id = None
        self._draw_pending = False
        self.low_latency = True
        self._last_draw_time = 0.0
        self._show_fps = False
        self._fps_label = None
        self._fps_ema = 0.0
        self._fps_last_ts = None
        self.detail_mode = 'auto'
        self.aspect_ratio_xy = 20 / 9
        self._z_aspect_scale = 0.8
        self.max_angles_deg = [180.0] * 6
        self.min_angles_deg = [-180.0] * 6
        self.gripper_length = 25.0
        self.script_dir = script_dir
        self.machine_dir = os.path.join(self.script_dir, 'machine')
        self.work_dir = os.path.join(self.script_dir, 'work')
        self.sequence_dir = os.path.join(self.script_dir, 'seq')
        self.last_settings_path = os.path.join(self.machine_dir, 'last_settings.json')
        self._sequence_after_id = None
        self._interp_after_id = None
        self._interp_state = None

    # U ovoj metodi je spremanje podataka ili konfiguracije.
    def _restore_factory_defaults(self) -> None:
        try:
            self.arm_lengths = [50, 80, 50, 30, 20, 10]
            self.gripper_length = 25.0
            self.max_angles_deg = [180.0] * 6
            self.min_angles_deg = [-180.0] * 6
            self.speed_var = getattr(self, 'speed_var', tk.DoubleVar(value=100))
            try:
                self.speed_var.set(100)
            except Exception:
                pass
            self.accel_var = getattr(self, 'accel_var', tk.DoubleVar(value=100))
            try:
                self.accel_var.set(100.0)
            except Exception:
                pass
            self.home_positions = [511] * 6
            if getattr(self, 'sliders', None):
                for i, s in enumerate(self.sliders):
                    try:
                        s.set(511)
                    except Exception:
                        pass
                for i, lbl in enumerate(getattr(self, 'value_labels', [])):
                    try:
                        if lbl is not None:
                            self.safe_config(lbl, text=str(511))
                    except Exception:
                        pass
            try:
                if getattr(self, 'length_vars', None):
                    for i, lv in enumerate(self.length_vars):
                        try:
                            if lv is not None:
                                lv.set(str(int(self.arm_lengths[i])))
                        except Exception:
                            pass
            except Exception:
                pass
            try:
                if getattr(self, 'max_angle_vars', None):
                    for i, mv in enumerate(self.max_angle_vars):
                        try:
                            if mv is not None:
                                mv.set(str(int(self.max_angles_deg[i])) + '°')
                        except Exception:
                            pass
            except Exception:
                pass
            try:
                if getattr(self, 'min_angle_vars', None):
                    for i, mv in enumerate(self.min_angle_vars):
                        try:
                            if mv is not None:
                                mv.set(str(int(self.min_angles_deg[i])) + '°')
                        except Exception:
                            pass
            except Exception:
                pass
            try:
                self._ensure_machine_dir()
                self.save_last_settings()
            except Exception as err:
                self._log_exception('_restore_factory_defaults.save', err)
        except Exception as err:
            self._log_exception('_restore_factory_defaults', err)

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def _init_translations(self) -> None:
        self.language = 'hr'
        self.translations = TRANSLATIONS

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def _init_variables(self) -> None:
        self.sequences = {i: [] for i in range(1, 10)}
        self.active_sequence = 1
        self.current_pose_index = 0
        self.loop_enabled = tk.BooleanVar()
        self.sequence_names = {i: self._default_sequence_name(i) for i in range(1, 10)}
        self.sliders = []
        self.value_labels = []
        self.servo_labels = []
        self.length_vars = []
        self.min_angle_vars = []
        self.max_angle_vars = []
        self.seq_copied_item = None
        self.seq_control_labels = []
        self.seq_listbox = None
        self.seq_pause_entry = None
        self.seq_pause_button = None
        self.seq_frame = None
        self.show_axis_planes = True
        self.lang_menu_var = tk.StringVar(value=self.language)

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def __init__(self, root: tk.Tk):
        self.root = root
        self.verbose = True
        self._init_translations()
        self._init_defaults()
        self._init_variables()
        self._init_ui()

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def _init_ui(self) -> None:
        try:
            self._apply_ui_scale()
        except Exception as err:
            self._log_exception(None, err)
        self._setup_styles()
        self._init_layout()
        self._setup_frames()
        self._post_init_setup()

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def _init_layout(self):
        self.root.title(self.t('window_title'))
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.configure(bg='lightgray')
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        self.main_frame.grid_rowconfigure(0, weight=2, minsize=330)
        self.main_frame.grid_rowconfigure(1, weight=1, minsize=280)
        self.main_frame.grid_columnconfigure(0, weight=3, minsize=680)
        self.main_frame.grid_columnconfigure(1, weight=1, minsize=260)
        self._setup_menubar()
        self._setup_serial_frame(self.main_frame)
        self._setup_sequence_frame()
        self._setup_control_frame()
        try:
            self.root.state('zoomed')
        except Exception as err:
            self._log_exception(None, err)
            try:
                self.root.attributes('-zoomed', True)
            except Exception as err:
                self._log_exception(None, err)
                try:
                    w = self.root.winfo_screenwidth()
                    h = self.root.winfo_screenheight()
                    self.root.geometry(f'{w}x{h}+0+0')
                except Exception as err:
                    self._log_exception(None, err)

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa i rad s pozicijama.
    def _post_init_setup(self) -> None:
        self.start_periodic_port_refresh()
        self._check_serial_connection()
        sys.stdout = sys.stderr = self
        try:
            self._ensure_machine_dir()
            self._ensure_work_dir()
            self._ensure_sequence_dir()
            try:
                self.load_last_settings()
            except Exception as err:
                self._log_exception('_post_init_setup.load_last_settings', err)
            self._setup_shortcuts()
            try:
                self.root.after(50, self._initial_redraw_once)
                self.root.after(500, self._auto_enable_fps)
            except Exception as err:
                self._log_exception('_initial_redraw_once schedule', err)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_styles(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use('default')
        except tk.TclError as err:
            self._log_exception(None, err)
        style.configure('Online.TButton', foreground='green')
        style.configure('Offline.TButton', foreground='red')
        style.configure('Selected.TButton', background='#4caf50', foreground='white')
        style.configure('Stop.TButton', foreground='white', background='red', font=('Helvetica', 12, 'bold'))
        style.configure('StopPlain.TButton', foreground='#D32F2F', font=('Helvetica', 10, 'bold'))
        style.map('StopPlain.TButton', foreground=[('active', '#B71C1C')])
        try:
            style.configure('ClearTiny.TButton', font=('TkDefaultFont', 8), padding=(2, 0))
        except Exception as err:
            self._log_exception('_setup_styles.cleartiny', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _apply_ui_scale(self) -> None:
        try:
            cur = float(self.root.tk.call('tk', 'scaling'))
            scale = float(getattr(self, 'ui_scale', 1.0))
            if scale and scale > 0 and (abs(scale - 1.0) > 0.001):
                self.root.tk.call('tk', 'scaling', cur * scale)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_shortcuts(self) -> None:
        try:
            def _cb(f):
                return lambda e: (f(), 'break')[1]
            try:
                self.root.bind_all('<Control-s>', _cb(self._save_workfile))
                self.root.bind_all('<Control-o>', _cb(self._load_workfile))
            except Exception as err:
                self._log_exception('_bind workfile', err)
            try:
                self.root.bind_all('<Control-Shift-S>', _cb(self.save_to_file))
                self.root.bind_all('<Control-Shift-O>', _cb(self.load_from_file))
                self.root.bind_all('<Control-S>', _cb(self.save_to_file))
                self.root.bind_all('<Control-O>', _cb(self.load_from_file))
            except Exception as err:
                self._log_exception('_bind seq', err)
            try:
                self.root.bind_all('<Control-m>', _cb(self.save_machine_to_file))
                self.root.bind_all('<Control-Shift-M>', _cb(self.load_machine_from_file))
            except Exception as err:
                self._log_exception('_bind machine', err)
            try:
                self.root.bind_all('<space>', _cb(self.toggle_pause))
                self.root.bind_all('<F5>', _cb(self.start_sequence_thread))
            except Exception as err:
                self._log_exception('_bind control', err)
        except Exception as err:
            self._log_exception('_setup_shortcuts', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _setup_sequence_pause_controls(self):
        pause_frame = ttk.Frame(self.seq_btns_frame)
        pause_frame.pack(side='top', fill='x', anchor='n', pady=(5, 0))
        self.seq_pause_button = ttk.Button(pause_frame, text=self.t('pause'), width=6, command=self.seq_insert_pause)
        self.seq_pause_button.grid(row=0, column=0, padx=(2, 5))
        self.seq_pause_entry = ttk.Entry(pause_frame, width=6)
        self.seq_pause_entry.insert(0, '1000')
        self.seq_pause_entry.grid(row=0, column=1, padx=(0, 5))
        self.seq_pause_label = ttk.Label(pause_frame, text=self.t('pause_label'))
        self.seq_pause_label.grid(row=0, column=2)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_frames(self):
        self.serial_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=5)
        self.control_frame.grid(row=0, column=1, sticky='nsew', padx=10, pady=5)
        self.sequence_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        self._setup_simulation_frame()

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _initial_redraw_once(self) -> None:
        try:
            if self._has_3d():
                self.ax3d.figure.canvas.draw()
        except Exception as err:
            self._log_exception('_initial_redraw_once', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_menubar(self) -> None:
        try:
            menubar = tk.Menu(self.root)
            display_names = {'hr': 'Hrvatski', 'en': 'English'}
            if not hasattr(self, 'lang_menu_var'):
                self.lang_menu_var = tk.StringVar(value=self.language)
            else:
                self.lang_menu_var.set(self.language)

            # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
            def _label(key: str, fallback: str) -> str:
                text = self.t(key)
                return text if text and text != key else fallback
            file_menu = tk.Menu(menubar, tearoff=0)
            file_menu.add_command(label=_label('menu.file.save_work', 'Save Workfile...'), accelerator='Ctrl+S', command=self._save_workfile)
            file_menu.add_command(label=_label('menu.file.load_work', 'Load Workfile...'), accelerator='Ctrl+O', command=self._load_workfile)
            file_menu.add_separator()
            file_menu.add_command(label=_label('save_all', 'Save sequences'), accelerator='Ctrl+Shift+S', command=self.save_to_file)
            file_menu.add_command(label=_label('load_sequences', 'Load sequences'), accelerator='Ctrl+Shift+O', command=self.load_from_file)
            file_menu.add_separator()
            file_menu.add_command(label=_label('save_machine_settings', 'Save settings'), accelerator='Ctrl+M', command=self.save_machine_to_file)
            file_menu.add_command(label=_label('load_machine_settings', 'Load settings'), accelerator='Ctrl+Shift+M', command=self.load_machine_from_file)
            file_menu.add_separator()
            file_menu.add_command(label=_label('menu.exit', 'Exit'), command=self.root.quit)
            menubar.add_cascade(label=_label('menu.file', 'File'), menu=file_menu, underline=0)
            view_menu = tk.Menu(menubar, tearoff=0)
            if not hasattr(self, 'show_fps_var'):
                self.show_fps_var = tk.BooleanVar(value=getattr(self, '_show_fps', False))
            view_menu.add_checkbutton(label=_label('menu.view.show_fps', 'Show FPS'), variable=self.show_fps_var, command=self._toggle_show_fps)
            if not hasattr(self, 'detail_mode_var'):
                self.detail_mode_var = tk.StringVar(value=getattr(self, 'detail_mode', 'auto'))
            view_menu.add_radiobutton(label=_label('menu.view.detail_high', 'Always High'), value='high', variable=self.detail_mode_var, command=self._on_detail_mode)
            view_menu.add_radiobutton(label=_label('menu.view.detail_auto', 'Auto'), value='auto', variable=self.detail_mode_var, command=self._on_detail_mode)
            view_menu.add_radiobutton(label=_label('menu.view.detail_low', 'Always Low'), value='low', variable=self.detail_mode_var, command=self._on_detail_mode)
            menubar.add_cascade(label=_label('menu.view', 'Animation'), menu=view_menu, underline=0)
            about_menu = tk.Menu(menubar, tearoff=0)
            about_menu.add_command(label=_label('menu.about_app', 'About'), command=self._show_about_dialog)
            menubar.add_cascade(label=_label('menu.about', 'About'), menu=about_menu, underline=0)
            lang_menu = tk.Menu(menubar, tearoff=0)
            for code in self.translations.keys():
                label = display_names.get(code, code.upper())
                lang_menu.add_radiobutton(label=label, value=code, variable=self.lang_menu_var, command=lambda c=code: self._set_language(c, persist=True))
            menubar.add_cascade(label=_label('language', 'Language'), menu=lang_menu, underline=0)
            self.root.config(menu=menubar)
            self.menubar = menubar
        except Exception as err:
            self._log_exception('_setup_menubar', err)

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def _setup_serial_frame(self, parent):
        self.serial_frame = ttk.LabelFrame(parent, text=self.t('serial_comm'))
        self.serial_frame.grid(row=0, column=0, sticky='nsew', padx=(10, 5), pady=5)
        for c in range(3):
            self.serial_frame.grid_columnconfigure(c, weight=1 if c == 1 else 0)
        self.serial_frame.grid_rowconfigure(2, weight=1)
        self.com_label = ttk.Label(self.serial_frame, text=self.t('select_com'))
        self.com_label.grid(row=0, column=0, sticky='w', padx=5, pady=1)
        self.combobox = ttk.Combobox(self.serial_frame, values=self.get_serial_ports(), state='readonly')
        self.combobox.grid(row=0, column=1, sticky='ew', padx=5, pady=1)
        self.combobox.bind('<<ComboboxSelected>>', self.connect_serial)
        self.combobox.bind('<<ComboboxDropdown>>', self.refresh_serial_ports)
        self.toggle_online_btn = ttk.Button(self.serial_frame, text='', style='Offline.TButton', command=self.toggle_online_mode)
        self.toggle_online_btn.grid(row=0, column=2, sticky='e', padx=5, pady=1)
        self._update_online_button()
        self.manual_label = ttk.Label(self.serial_frame, text=self.t('manual_input'))
        self.manual_label.grid(row=1, column=0, sticky='w', padx=5, pady=1)
        self.manual_entry = ttk.Entry(self.serial_frame)
        self.manual_entry.grid(row=1, column=1, sticky='ew', padx=5, pady=1)
        self.manual_send_btn = ttk.Button(self.serial_frame, text=self.t('send_command'), command=self.send_manual_command)
        self.manual_send_btn.grid(row=1, column=2, sticky='e', padx=5, pady=1)
        terminal_frame = ttk.Frame(self.serial_frame)
        terminal_frame.grid(row=2, column=0, columnspan=3, sticky='nsew', padx=5, pady=(2, 0))
        terminal_frame.grid_columnconfigure(0, weight=1)
        terminal_frame.grid_rowconfigure(1, weight=1)
        terminal_frame.grid_rowconfigure(4, weight=1)
        self.sent_label = ttk.Label(terminal_frame, text=self.t('sent'))
        self.sent_label.grid(row=0, column=0, sticky='w', pady=(0, 0))
        self.console_output = tk.Text(terminal_frame, height=5, wrap='word', state='disabled', bg='#111', fg='#0f0')
        self.console_output.grid(row=1, column=0, sticky='nsew', padx=2, pady=(0, 0))
        clear_txt = self.t('clear')
        if len(clear_txt) > 6:
            try:
                clear_txt = clear_txt[:6]
            except Exception:
                pass
        self.clear_sent_button = ttk.Button(terminal_frame, text=clear_txt, command=self.clear_console)
        try:
            self.clear_sent_button.configure(style='ClearTiny.TButton')
        except Exception:
            try:
                self.clear_sent_button.configure(font=('TkDefaultFont', 8))
            except Exception:
                pass
        self.clear_sent_button.grid(row=2, column=0, sticky='e', pady=(0, 0), padx=(0, 2))
        self.recv_label = ttk.Label(terminal_frame, text=self.t('received'))
        self.recv_label.grid(row=3, column=0, sticky='w', pady=(0, 0))
        self.arduino_output = tk.Text(terminal_frame, height=5, wrap='word', state='disabled', bg='#111', fg='#0ff')
        self.arduino_output.grid(row=4, column=0, sticky='nsew', padx=2, pady=(0, 0))
        self.clear_recv_button = ttk.Button(terminal_frame, text=clear_txt, command=self.clear_arduino_log)
        try:
            self.clear_recv_button.configure(style='ClearTiny.TButton')
        except Exception:
            try:
                self.clear_recv_button.configure(font=('TkDefaultFont', 8))
            except Exception:
                pass
        self.clear_recv_button.grid(row=5, column=0, sticky='e', pady=(0, 0), padx=(0, 2))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _setup_sequence_frame(self):
        self.sequence_frame = ttk.LabelFrame(self.main_frame, text=self.t('active_sequence'))
        self.sequence_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        self.sequence_frame.grid_rowconfigure(3, weight=1)
        self.sequence_frame.grid_columnconfigure(0, weight=1)
        name_frame = ttk.Frame(self.sequence_frame)
        name_frame.grid(row=0, column=0, sticky='ew', pady=(0, 5))
        self.name_entry_label = ttk.Label(name_frame, text=self.t('name'))
        self.name_entry_label.pack(side='left')
        self.sequence_name_var = tk.StringVar(value='')
        self.name_entry = ttk.Entry(name_frame, textvariable=self.sequence_name_var)
        self.name_entry.pack(side='left', fill='x', expand=True, padx=5)
        self.save_name_button = ttk.Button(name_frame, text=self.t('save_name'), command=self.save_sequence_name)
        self.save_name_button.pack(side='left', padx=5)
        mid_sequence_frame = ttk.Frame(self.sequence_frame)
        mid_sequence_frame.grid(row=1, column=0, sticky='ew', pady=5)
        mid_sequence_frame.grid_columnconfigure(0, weight=0)
        mid_sequence_frame.grid_columnconfigure(1, weight=0)
        for c in range(2, 30):
            mid_sequence_frame.grid_columnconfigure(c, weight=0)
        mid_sequence_frame.grid_columnconfigure(30, weight=1)
        self.seq_buttons_label = ttk.Label(mid_sequence_frame, text=self.t('active_sequence'))
        self.seq_buttons_label.grid(row=0, column=0, padx=4, pady=2, sticky='w')
        self.seq_buttons = []
        col = 1
        for i in range(1, 10):
            btn = ttk.Button(mid_sequence_frame, text=str(i), width=3, command=lambda idx=i: self.set_active_sequence(idx))
            btn.grid(row=0, column=col, padx=2, pady=2, sticky='w')
            self.seq_buttons.append(btn)
            col += 1
        self.run_button = ttk.Button(mid_sequence_frame, text=self.t('run'), command=self.start_sequence_thread, width=8)
        self.run_button.grid(row=0, column=col, padx=6, pady=2, sticky='w')
        col += 1
        self.run_canvas = tk.Canvas(mid_sequence_frame, width=18, height=18, highlightthickness=0, bg='lightgray')
        self.run_canvas.grid(row=0, column=col, padx=(0, 6), pady=0, sticky='w')
        self.run_canvas.create_polygon(4, 3, 14, 9, 4, 15, fill='lightgray', outline='black', tags='triangle')
        col += 1
        self.pause_button = ttk.Button(mid_sequence_frame, text=self.t('pause'), command=self.toggle_pause, width=8)
        self.pause_button.grid(row=0, column=col, padx=6, pady=2, sticky='w')
        col += 1
        self.pause_canvas = tk.Canvas(mid_sequence_frame, width=18, height=18, highlightthickness=0, bg='lightgray')
        self.pause_canvas.grid(row=0, column=col, padx=(0, 6), pady=0, sticky='w')
        self.pause_canvas.create_oval(4, 4, 14, 14, fill='lightgray', tags='circle')
        col += 1
        self.stop_button = ttk.Button(mid_sequence_frame, text=self.t('stop'), command=self.stop_all_actions, width=8, style='StopPlain.TButton')
        self.stop_button.grid(row=0, column=col, padx=6, pady=2, sticky='w')
        col += 1
        self.pose_label = ttk.Label(mid_sequence_frame, text=self.t('pose_label'), width=14)
        self.pose_label.grid(row=0, column=col, padx=6, pady=2, sticky='w')
        bottom_sequence_frame = ttk.Frame(self.sequence_frame)
        bottom_sequence_frame.grid(row=2, column=0, sticky='ew', pady=(5, 0))
        self.save_all_button = ttk.Button(bottom_sequence_frame, text=self.t('save_all'), command=self.save_to_file, width=18)
        self.save_all_button.pack(side='left', padx=5)
        self.load_button = ttk.Button(bottom_sequence_frame, text=self.t('load'), command=self.load_from_file, width=18)
        self.load_button.pack(side='left', padx=5)
        self.clear_seq_button = ttk.Button(bottom_sequence_frame, text=self.t('delete'), width=18, command=self.clear_active_sequence)
        self.clear_seq_button.pack(side='left', padx=5)
        self.loop_check = ttk.Checkbutton(bottom_sequence_frame, text=self.t('loop'), variable=self.loop_enabled)
        self.loop_check.pack(side='left', padx=5)
        self.sequence_editor_frame = ttk.Frame(self.sequence_frame)
        self.sequence_editor_frame.grid(row=3, column=0, sticky='nsew', pady=(5, 0))
        self._setup_sequence_editor_ui()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _auto_enable_fps(self) -> None:
        try:
            if hasattr(self, 'show_fps_var'):
                self.show_fps_var.set(True)
                try:
                    self._toggle_show_fps()
                except Exception as err:
                    self._log_exception('_auto_enable_fps.toggle', err)
                try:
                    self._schedule_robot_draw(delay=1)
                except Exception as err:
                    self._log_exception('_auto_enable_fps.schedule', err)
                if self._has_3d():
                    try:
                        self.ax3d.figure.canvas.draw()
                    except Exception as err:
                        self._log_exception('_auto_enable_fps.draw', err)
        except Exception as err:
            self._log_exception('_auto_enable_fps', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _setup_sequence_editor_ui(self):
        self.seq_frame = ttk.LabelFrame(self.sequence_editor_frame, text=self.t('sequence'))
        self.seq_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.seq_frame.grid_columnconfigure(0, weight=1)
        self.seq_frame.grid_rowconfigure(0, weight=1)
        list_frame = ttk.Frame(self.seq_frame)
        list_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(0, weight=1)
        self.seq_listbox = tk.Listbox(list_frame, height=10, width=60)
        self.seq_listbox.grid(row=0, column=0, sticky='nsew')
        self.seq_listbox.bind('<<ListboxSelect>>', self.seq_on_select)
        scrollbar = ttk.Scrollbar(list_frame, orient='vertical', command=self.seq_listbox.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        self.safe_config(self.seq_listbox, yscrollcommand=scrollbar.set)
        self.seq_btns_frame = ttk.Frame(self.seq_frame)
        self.seq_btns_frame.grid(row=0, column=1, sticky='n', padx=20, pady=5)
        self._create_sequence_control_buttons()
        self._setup_sequence_pause_controls()

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa.
    def create_slider(self, parent, idx):
        frame = ttk.Frame(parent)
        frame.pack(fill='x', pady=5)
        min_default = float(self.min_angles_deg[idx]) if idx < len(self.min_angles_deg) else -180.0
        min_var = tk.StringVar(value=str(int(min_default)) + '°')
        self.min_angle_vars.append(min_var)

        # U ovoj funkciji je pretvorbe i računanje kuteva.
        def _apply_min_angle(event=None, i=idx, v=min_var):
            try:
                raw = (v.get() or '').strip()
                if raw.endswith('°'):
                    raw = raw[:-1].strip()
                val = float(raw)
                max_val = float(self.max_angles_deg[i]) if i < len(self.max_angles_deg) else 180.0
                if not math.isfinite(val) or val >= max_val or val < -180.0:
                    raise ValueError
                self.min_angles_deg[i] = val
                try:
                    self.save_last_settings()
                except Exception as err:
                    self._log_exception(None, err)
                self._schedule_robot_draw(delay=1)
                v.set(str(int(self.min_angles_deg[i])) + '°')
            except Exception as err:
                self._log_exception(None, err)
                v.set(str(int(self.min_angles_deg[i]) if i < len(self.min_angles_deg) else -180) + '°')
        min_entry = ttk.Entry(frame, textvariable=min_var, width=6)
        min_entry.pack(side='left', padx=(0, 5))
        min_entry.bind('<Return>', _apply_min_angle)
        min_entry.bind('<FocusOut>', _apply_min_angle)
        slider = ttk.Scale(frame, from_=0, to=1023, orient='horizontal')
        slider.set(511)
        slider.pack(side='left', fill='x', expand=True, padx=5)
        self.sliders.append(slider)
        try:
            slider.bind('<ButtonRelease-1>', lambda e, i=idx: self._on_slider_release(i))
        except Exception:
            pass
        value_label = ttk.Label(frame, text='511', width=4, font=('TkFixedFont', 10))
        value_label.pack(side='left')
        self.value_labels.append(value_label)
        max_default = float(self.max_angles_deg[idx]) if idx < len(self.max_angles_deg) else 180.0
        max_var = tk.StringVar(value=str(int(max_default)) + '°')
        self.max_angle_vars.append(max_var)

        # U ovoj funkciji je pretvorbe i računanje kuteva.
        def _apply_max_angle(event=None, i=idx, v=max_var):
            try:
                raw = (v.get() or '').strip()
                if raw.endswith('°'):
                    raw = raw[:-1].strip()
                val = float(raw)
                min_val = float(self.min_angles_deg[i]) if i < len(self.min_angles_deg) else -180.0
                if not math.isfinite(val) or val <= min_val or val <= 0 or (val > 180.0):
                    raise ValueError
                self.max_angles_deg[i] = val
                try:
                    self.save_last_settings()
                except Exception as err:
                    self._log_exception(None, err)
                self._schedule_robot_draw(delay=1)
                v.set(str(int(self.max_angles_deg[i])) + '°')
            except Exception as err:
                self._log_exception(None, err)
                v.set(str(int(self.max_angles_deg[i]) if i < len(self.max_angles_deg) else 180) + '°')
        max_entry = ttk.Entry(frame, textvariable=max_var, width=6)
        max_entry.pack(side='left', padx=(2, 0))
        max_entry.bind('<Return>', _apply_max_angle)
        max_entry.bind('<FocusOut>', _apply_max_angle)
        if idx < 5:
            len_var = tk.StringVar(value=str(int(self.arm_lengths[idx] if idx < len(self.arm_lengths) else 0)))
            len_ent = ttk.Entry(frame, textvariable=len_var, width=5)
            len_ent.pack(side='left', padx=(8, 0))
            try:
                while len(self.length_vars) <= idx:
                    self.length_vars.append(None)
                self.length_vars[idx] = len_var
            except Exception as err:
                self._log_exception(None, err)

            # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
            def _apply_length(event=None, i=idx, v=len_var):
                try:
                    L = float(v.get())
                    if not math.isfinite(L) or L < 0:
                        raise ValueError
                    if i < len(self.arm_lengths):
                        self.arm_lengths[i] = L
                    self._schedule_robot_draw(delay=1)
                    self.save_last_settings()
                except Exception as err:
                    self._log_exception(None, err)
                    v.set(str(int(self.arm_lengths[i] if i < len(self.arm_lengths) else 0)))
            len_ent.bind('<Return>', _apply_length)
            len_ent.bind('<FocusOut>', _apply_length)
        if idx == 5:
            fl_var = tk.StringVar(value=str(int(self.gripper_length)))
            self.gripper_length_var = fl_var
            fl_ent = ttk.Entry(frame, textvariable=fl_var, width=5)
            fl_ent.pack(side='left', padx=(8, 0))

            # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
            def _apply_flen(event=None, v=fl_var):
                try:
                    Lg = float(v.get())
                    if not math.isfinite(Lg) or Lg <= 0:
                        raise ValueError
                    self.gripper_length = Lg
                    self._schedule_robot_draw(delay=1)
                    self.save_last_settings()
                except Exception as err:
                    self._log_exception(None, err)
                    v.set(str(int(self.gripper_length)))
            fl_ent.bind('<Return>', _apply_flen)
            fl_ent.bind('<FocusOut>', _apply_flen)
        slider.configure(command=lambda val, i=idx: self.slider_changed(i, val))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_3d_toolbar(self, toolbar_frame: ttk.Frame, center_cb) -> None:
        def _reset_view():
            self.ax3d.view_init(elev=25, azim=-60)
            try:
                self._update_axes_limits()
            except Exception as err:
                self._log_exception(None, err)
            self._draw_idle()

        # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
        def _zoom(factor):
            try:
                xlim = list(self.ax3d.get_xlim3d())
                ylim = list(self.ax3d.get_ylim3d())
                zlim = list(self.ax3d.get_zlim3d())

                # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
                def scale(lim):
                    mid = (lim[0] + lim[1]) / 2.0
                    return [mid + (lim[0] - mid) * factor, mid + (lim[1] - mid) * factor]
                self.ax3d.set_xlim3d(scale(xlim))
                self.ax3d.set_ylim3d(scale(ylim))
                self.ax3d.set_zlim3d(scale(zlim))
                self._draw_idle()
            except Exception as err:
                self._log_exception(None, err)
        self.tb_center_btn = ttk.Button(toolbar_frame, text=self.t('center'), command=center_cb)
        self.tb_center_btn.pack(side='left', padx=2)

        # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
        def _view_xz():
            self.ax3d.view_init(elev=0, azim=-90)
            try:
                self._update_axes_limits()
            except Exception as err:
                self._log_exception(None, err)
            self._draw_idle()

        # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
        def _view_yz():
            self.ax3d.view_init(elev=0, azim=0)
            try:
                self._update_axes_limits()
            except Exception as err:
                self._log_exception(None, err)
            self._draw_idle()
        self.tb_view_xz_btn = ttk.Button(toolbar_frame, text='XZ', command=_view_xz)
        self.tb_view_xz_btn.pack(side='left', padx=2)
        self.tb_view_yz_btn = ttk.Button(toolbar_frame, text='YZ', command=_view_yz)
        self.tb_view_yz_btn.pack(side='left', padx=2)
        self.tb_zoom_in_btn = ttk.Button(toolbar_frame, text=self.t('zoom_plus'), command=lambda: _zoom(0.8))
        self.tb_zoom_in_btn.pack(side='left', padx=2)
        self.tb_zoom_out_btn = ttk.Button(toolbar_frame, text=self.t('zoom_minus'), command=lambda: _zoom(1.25))
        self.tb_zoom_out_btn.pack(side='left', padx=2)
        self.tb_reset_btn = ttk.Button(toolbar_frame, text=self.t('reset'), command=_reset_view)
        self.tb_reset_btn.pack(side='left', padx=2)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_control_frame(self):
        self.control_frame = ttk.LabelFrame(self.main_frame, text=self.t('control_panel'))
        self.control_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
        top_row = ttk.Frame(self.control_frame)
        top_row.pack(fill='x', padx=5, pady=5)
        self.speed_label = ttk.LabelFrame(top_row, text=self.t('speed_label'))
        self.speed_label.pack(side='left', fill='x', expand=True)
        self.speed_var = tk.DoubleVar(value=100)
        self.accel_var = tk.DoubleVar(value=100)
        speed_row = ttk.Frame(self.speed_label)
        speed_row.pack(fill='x', padx=5, pady=5)
        self.speed_slider = ttk.Scale(speed_row, from_=1, to=100, orient='horizontal', command=self.update_speed_label, variable=self.speed_var)
        self.speed_slider.set(100)
        self.speed_slider.pack(side='left', fill='x', expand=True)
        accel_col = ttk.Frame(speed_row)
        accel_col.pack(side='left', padx=(8, 2))
        self.accel_entry = ttk.Entry(accel_col, textvariable=self.accel_var, width=6)
        self.accel_entry.pack(side='top')
        try:
            self.accel_entry.bind('<KeyRelease>', lambda e: self._on_accel_entry_change(e))
        except Exception:
            pass
        try:
            accel_caption = self.t('acceleration_label')
        except Exception:
            accel_caption = 'Accel %'
        bottom_row = ttk.Frame(self.speed_label)
        bottom_row.pack(fill='x', side='top', padx=4, pady=(0, 2))
        self.speed_value_label = ttk.Label(bottom_row, text='100%', font=('TkDefaultFont', 8))
        if getattr(self, 'language', 'hr') == 'hr':
            accel_caption_short = 'Akcel (%)'
        else:
            accel_caption_short = 'Accel (%)'
        self.accel_caption_label = ttk.Label(bottom_row, text=accel_caption_short, font=('TkDefaultFont', 8))
        self.accel_caption_label.pack(side='right', padx=(8, 2))

        # U ovoj funkciji je pomoćna logika specifična za aplikaciju i upravljanje brzinom.
        def _place_speed_val():
            try:
                self.speed_value_label.place(relx=0.5, rely=0.0, anchor='n')
            except Exception:
                pass
        self.root.after(10, _place_speed_val)
        try:
            self.safe_config(self.speed_label, text=self.t('speed_label'))
            self.update_speed_label(self.speed_var.get())
            self.safe_config(self.accel_caption_label, text=self.t('acceleration_label'))
        except Exception as err:
            self._log_exception('_setup_control_frame.init_labels', err)
        btns_side = ttk.Frame(top_row)
        btns_side.pack(side='left', padx=(8, 0))
        self.save_machine_btn = ttk.Button(btns_side, text=self.t('save_machine_settings'), command=self.save_machine_to_file)
        self.save_machine_btn.pack(side='top', pady=(0, 4))
        self.load_machine_btn = ttk.Button(btns_side, text=self.t('load_machine_settings'), command=self.load_machine_from_file)
        self.load_machine_btn.pack(side='top')
        header_frame = ttk.Frame(self.control_frame)
        header_frame.pack(fill='x', padx=5)
        self._motor_header_spacer = ttk.Label(header_frame, text='', width=0)
        self._motor_header_spacer.pack(side='left', padx=(0, 5))
        self.min_angle_header_label = ttk.Label(header_frame, text='Min', width=6)
        self.min_angle_header_label.pack(side='left')
        home_buttons_frame = ttk.Frame(header_frame)
        home_buttons_frame.pack(side='left', padx=(170, 25))
        self.update_home_btn = ttk.Button(home_buttons_frame, text=self.t('update_home'), command=self._update_home_position, width=15)
        self.update_home_btn.pack(side='left', padx=(0, 3))
        self.goto_home_btn = ttk.Button(home_buttons_frame, text=self.t('home_btn'), command=self._joystick_home, width=10)
        self.goto_home_btn.pack(side='left', padx=(0, 3))
        self.center_motors_btn = ttk.Button(home_buttons_frame, text=self.t('center_btn'), command=self._center_motors, width=10)
        self.center_motors_btn.pack(side='left')
        self._angle_header_center = ttk.Label(header_frame, text='', anchor='center')
        self._angle_header_center.pack(side='left', fill='x', expand=True)
        self._value_header = ttk.Label(header_frame, text='', width=4)
        self._value_header.pack(side='left')
        self.max_angle_header_label = ttk.Label(header_frame, text='Max', width=6)
        self.max_angle_header_label.pack(side='left', padx=(0, 0))
        length_hdr = self.t('length_short')
        try:
            length_hdr = length_hdr.capitalize()
        except Exception:
            pass
        self.length_header_label = ttk.Label(header_frame, text=length_hdr)
        self.length_header_label.pack(side='left', padx=(8, 2))
        sliders_frame = ttk.Frame(self.control_frame)
        sliders_frame.pack(fill='both', expand=True, padx=5, pady=5)
        self.min_angle_vars = []
        self.max_angle_vars = []
        for i in range(6):
            self.create_slider(sliders_frame, i)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_simulation_frame(self):
        self.simulation_frame = ttk.LabelFrame(self.main_frame, text=self.t('simulation'))
        self.simulation_frame.grid(row=1, column=1, columnspan=1, sticky='nsew', padx=(5, 10), pady=5)
        top_controls = ttk.Frame(self.simulation_frame)
        top_controls.pack(fill='x', padx=6, pady=(6, 0))
        try:
            if getattr(self, 'fps_label', None) is None:
                self.fps_label = ttk.Label(top_controls, text='')
                self.fps_label.pack(side='right')
        except Exception as err:
            self._log_exception(None, err)

        # U ovoj funkciji je pomoćna logika specifična za aplikaciju.
        def _center_view():
            self.ax3d.view_init(elev=25, azim=-60)
            self._draw_idle()
        if MATPLOTLIB_OK:
            self.fig3d = Figure(figsize=(10.0, 5.5), dpi=120)
            self.ax3d = self.fig3d.add_subplot(111, projection='3d')
            self.fig3d.subplots_adjust(left=0, right=1, bottom=0.0, top=1.0, wspace=0.0, hspace=0.0)
            try:
                if getattr(self, 'initial_view_dist', None) is not None:
                    self.ax3d.dist = self.initial_view_dist
            except Exception as err:
                self._log_exception(None, err)
            try:
                self.ax3d.set_box_aspect((getattr(self, 'aspect_ratio_xy', 16 / 9), 1.0, getattr(self, '_z_aspect_scale', 0.8)))
            except Exception as err:
                self._log_exception(None, err)
            self.ax3d.grid(self.show_axis_planes)
            if not getattr(self, 'minimal_view', False):
                self.ax3d.set_xlabel('X')
                self.ax3d.set_ylabel('Y')
                self.ax3d.set_zlabel('Z')
            L = sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300
            cell = float(getattr(self, 'grid_cell_size', 50.0))
            n_each = max(1, int(math.ceil(float(L) / cell)))
            g = np.arange(-n_each * cell, n_each * cell + 1e-06, cell)
            GX, GY = np.meshgrid(g, g)
            GZ = np.zeros_like(GX)
            self._ground = self.ax3d.plot_wireframe(GX, GY, GZ, linewidth=0.3, alpha=0.3)
            canvas_region = ttk.Frame(self.simulation_frame)
            canvas_region.pack(fill='both', expand=True, padx=0, pady=(0, 0))
            left_panel = ttk.Frame(canvas_region, width=250, height=500)
            left_panel.pack(side='left', fill='y')
            left_panel.pack_propagate(False)
            self._setup_joystick_panel(left_panel)
            self._setup_coordinate_panel(left_panel)
            center_holder = ttk.Frame(canvas_region)
            center_holder.pack(side='left', fill='both', expand=True)
            self.canvas3d = FigureCanvasTkAgg(self.fig3d, master=center_holder)
            cw = self.canvas3d.get_tk_widget()
            cw.pack(fill='both', expand=True, padx=0, pady=0)
            try:
                cw.configure(width=100)
            except Exception:
                pass
            self._canvas_container = cw
            self._canvas_container.bind('<Configure>', self._on_3d_container_resize)
            try:
                prev = getattr(self, 'preserve_view', True)
                self.preserve_view = False
                self._update_axes_limits()
                self._draw_idle()
                self.preserve_view = prev
            except Exception as err:
                self._log_exception(None, err)
            if getattr(self, 'minimal_view', False):
                self._apply_minimal_view()
            self._draw_ucs_gizmo()
            toolbar_frame = ttk.Frame(top_controls)
            toolbar_frame.pack(side='left', anchor='w')
            self._setup_3d_toolbar(toolbar_frame, _center_view)
            try:
                self.tcp_label = ttk.Label(top_controls, text=self.t('label.tcp', x=0, y=0, z=0), font=('TkFixedFont', 10), width=32, anchor='e')
                self.tcp_label.pack(side='right', padx=6)
            except Exception as err:
                self._log_exception(None, err)
        else:
            msg = ttk.Label(self.simulation_frame, text=self.t('msg.no_matplotlib'))
            msg.pack(fill='both', expand=True, padx=6, pady=(6, 6))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _ensure_machine_dir(self) -> None:
        try:
            if not os.path.isdir(self.machine_dir):
                os.makedirs(self.machine_dir, exist_ok=True)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _ensure_work_dir(self) -> None:
        try:
            if not os.path.isdir(self.work_dir):
                os.makedirs(self.work_dir, exist_ok=True)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i upravljanje ubrzanjem, rad s datotekama/sekvencama.
    def _sequences_with_accel(self) -> Dict[int, List[Dict[str, Any]]]:
        out = {}
        for k, seq in getattr(self, 'sequences', {}).items():
            newseq = []
            for item in seq or []:
                if not isinstance(item, dict):
                    newseq.append(item)
                    continue
                new_item = dict(item)
                if 'position' in new_item:
                    if 'acceleration' not in new_item:
                        try:
                            new_item['acceleration'] = float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get())
                        except Exception:
                            new_item['acceleration'] = 100.0
                newseq.append(new_item)
            out[k] = newseq
        return out

    # U ovoj metodi je spremanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def save_to_file(self) -> None:
        self._ensure_sequence_dir()
        now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        default_name = f'{now}'
        initialdir = self.sequence_dir if os.path.isdir(self.sequence_dir) else self.script_dir
        file_path = filedialog.asksaveasfilename(defaultextension='.seq', filetypes=[('Sequence files', '*.seq'), ('All files', '*.*')], initialfile=default_name, initialdir=initialdir, title=self.t('save_dialog_title'))
        if file_path:
            data = {'sequences': self._sequences_with_accel(), 'names': self.sequence_names}
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2)
                self._log(self.t('log.sequence.save_path', path=file_path))
            except Exception as err:
                self._log(self.t('log.sequence.save_error', error=err))
        else:
            self._log(self.t('log.sequence.save_cancel'))

    # U ovoj metodi je spremanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def save_machine_to_file(self) -> None:
        self._ensure_machine_dir()
        now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        default_name = f'{now}'
        file_path = filedialog.asksaveasfilename(defaultextension='.machine', filetypes=[('Machine files', '*.machine'), ('All files', '*.*')], initialfile=default_name, initialdir=self.machine_dir if os.path.isdir(self.machine_dir) else self.script_dir, title=self.t('save_machine_settings'))
        if file_path:
            data = {'settings': self._get_mechanical_settings()}
            try:
                with open(file_path, 'w') as f:
                    json.dump(data, f, indent=2)
                self._log(self.t('log.machine.save_success', path=file_path))
            except Exception as e:
                self._log(self.t('log.machine.save_error', error=e))
        else:
            self._log(self.t('log.machine.save_cancel'))

    # U ovoj metodi je spremanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def _save_workfile(self) -> None:
        self._ensure_work_dir()
        now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        default_name = f'{now}'
        initialdir = self.work_dir if os.path.isdir(self.work_dir) else self.script_dir
        file_path = filedialog.asksaveasfilename(defaultextension='.work', filetypes=[('Work files', '*.work'), ('All files', '*.*')], initialfile=default_name, initialdir=initialdir, title=self.t('menu.file.save_work_title'))
        if not file_path:
            self._log(self.t('log.work.save_cancel'))
            return
        data = {'machine': self._get_mechanical_settings(), 'sequences': {str(k): v for k, v in self._sequences_with_accel().items()}, 'names': {str(k): v for k, v in self.sequence_names.items()}, 'active_sequence': int(getattr(self, 'active_sequence', 1)), 'current_pose_index': int(getattr(self, 'current_pose_index', 0))}
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            self._log(self.t('log.work.save_success', path=file_path))
        except Exception as err:
            self._log(self.t('log.work.save_error', error=err))

    # U ovoj metodi je učitavanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def load_machine_from_file(self) -> None:
        self._ensure_machine_dir()
        file_path = filedialog.askopenfilename(defaultextension='.machine', filetypes=[('Machine files', '*.machine'), ('All files', '*.*')], initialdir=self.machine_dir if os.path.isdir(self.machine_dir) else self.script_dir, title=self.t('load_machine_settings'))
        if file_path and os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    loaded = json.load(f)
                self._apply_machine_settings(loaded.get('settings', {}))
                try:
                    self.save_last_settings()
                except Exception as err:
                    self._log_exception(None, err)
                self._log(self.t('log.machine.load_success', path=file_path))
            except Exception as e:
                self._log(self.t('log.machine.load_error', error=e))
        else:
            self._log(self.t('log.machine.load_invalid'))

    # U ovoj metodi je učitavanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def load_from_file(self) -> None:
        self._ensure_sequence_dir()
        initialdir = self.sequence_dir if os.path.isdir(self.sequence_dir) else self.script_dir
        file_path = filedialog.askopenfilename(defaultextension='.seq', filetypes=[('Sequence files', '*.seq'), ('All files', '*.*')], initialdir=initialdir)
        if file_path and os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    loaded = json.load(f)
                sequences_raw = loaded.get('sequences') or {}
                sequences = {i: [] for i in range(1, 10)}
                if isinstance(sequences_raw, dict):
                    for key, value in sequences_raw.items():
                        try:
                            idx = int(key)
                        except (TypeError, ValueError):
                            continue
                        if 1 <= idx <= 9 and isinstance(value, list):
                            sequences[idx] = value
                self.sequences = sequences
                names_raw = loaded.get('names') or {}
                names = {i: self.sequence_names.get(i, self._default_sequence_name(i)) for i in range(1, 10)}
                if isinstance(names_raw, dict):
                    for key, value in names_raw.items():
                        try:
                            idx = int(key)
                        except (TypeError, ValueError):
                            continue
                        if 1 <= idx <= 9 and isinstance(value, str):
                            names[idx] = value
                self.sequence_names = names
                self.current_pose_index = 0
                self.seq_refresh()
                self.update_sliders_from_pose()
                if hasattr(self, 'sequence_name_var'):
                    self.sequence_name_var.set(self.sequence_names.get(self.active_sequence, self._default_sequence_name(self.active_sequence)))
                self._log(self.t('log.sequence.load_path', path=file_path))
            except Exception as err:
                self._log(self.t('log.sequence.load_error', error=err))
        else:
            self._log(self.t('log.sequence.load_invalid'))

    # U ovoj metodi je učitavanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def _load_workfile(self) -> None:
        initialdir = self.work_dir if os.path.isdir(self.work_dir) else self.script_dir
        file_path = filedialog.askopenfilename(defaultextension='.work', filetypes=[('Work files', '*.work'), ('All files', '*.*')], initialdir=initialdir, title=self.t('menu.file.load_work_title'))
        if not file_path or not os.path.exists(file_path):
            self._log(self.t('log.work.load_invalid'))
            return
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as err:
            self._log(self.t('log.work.load_error', error=err))
            return
        try:
            machine_data = data.get('machine')
            if isinstance(machine_data, dict):
                self._apply_machine_settings(machine_data)
            sequences_raw = data.get('sequences') or {}
            sequences = {i: [] for i in range(1, 10)}
            if isinstance(sequences_raw, dict):
                for key, value in sequences_raw.items():
                    try:
                        idx = int(key)
                    except (TypeError, ValueError):
                        continue
                    if 1 <= idx <= 9 and isinstance(value, list):
                        sequences[idx] = value
            self.sequences = sequences
            names_raw = data.get('names') or {}
            names = {i: self.sequence_names.get(i, self._default_sequence_name(i)) for i in range(1, 10)}
            if isinstance(names_raw, dict):
                for key, value in names_raw.items():
                    try:
                        idx = int(key)
                    except (TypeError, ValueError):
                        continue
                    if 1 <= idx <= 9 and isinstance(value, str):
                        names[idx] = value
            self.sequence_names = names
            active_seq = int(data.get('active_sequence', self.active_sequence))
            if active_seq not in self.sequences:
                active_seq = 1
            self.active_sequence = active_seq
            current_pose = int(data.get('current_pose_index', 0))
            sequence_len = len(self.sequences.get(self.active_sequence, []))
            if sequence_len == 0:
                current_pose = 0
            else:
                current_pose = max(0, min(current_pose, sequence_len - 1))
            self.current_pose_index = current_pose
            self.update_sequence_buttons()
            self.seq_refresh()
            if hasattr(self, 'sequence_name_var'):
                self.sequence_name_var.set(self.sequence_names.get(self.active_sequence, self._default_sequence_name(self.active_sequence)))
            self.update_sliders_from_pose()
            self.update_pose_label()
            try:
                self.save_last_settings()
            except Exception as err:
                self._log_exception('save_last_settings after workfile load', err)
            self._log(self.t('log.work.load_success', path=file_path))
        except Exception as err:
            self._log(self.t('log.work.load_error', error=err))

    # U ovoj metodi je logiranje i ispis poruka.
    def log_arduino(self, msg: str) -> None:
        msg = self._tr_console(msg)
        self.arduino_output.configure(state='normal')
        self.arduino_output.insert('end', msg + '\n')
        self.arduino_output.see('end')
        self.arduino_output.configure(state='disabled')

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def clear_console(self) -> None:
        self.safe_config(self.console_output, state='normal')
        self.console_output.delete('1.0', tk.END)
        self.safe_config(self.console_output, state='disabled')

    # U ovoj metodi je logiranje i ispis poruka.
    def clear_arduino_log(self) -> None:
        self.safe_config(self.arduino_output, state='normal')
        self.arduino_output.delete('1.0', tk.END)
        self.safe_config(self.arduino_output, state='disabled')

    # U ovoj metodi je spremanje podataka ili konfiguracije.
    def write(self, msg: str) -> None:
        msg = self._tr_console(msg)
        if not msg.endswith('\n'):
            msg += '\n'
        self.safe_config(self.console_output, state='normal')
        try:
            self.console_output.insert('end', msg)
            self.console_output.see('end')
        except tk.TclError as err:
            self._log_exception(None, err)
        self.safe_config(self.console_output, state='disabled')

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def refresh_serial_ports(self, event: Optional[tk.Event]=None) -> None:
        ports = self.get_serial_ports()
        self.combobox['values'] = ports
        if ports:
            self.combobox.set(ports[0])
        else:
            self.combobox.set('')

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def _refresh_ports_periodically(self) -> None:
        current_ports = set(self.get_serial_ports())
        if current_ports != self.previous_ports:
            self.previous_ports = current_ports
            ports_list = list(current_ports)
            self.combobox['values'] = ports_list
            if ports_list:
                self.combobox.set(ports_list[0])
            else:
                self.combobox.set('')
        self.root.after(2000, self._refresh_ports_periodically)

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def log_serial(self, msg: str) -> None:
        msg = self._tr_console(msg)
        self.console_output.configure(state='normal')
        self.console_output.insert('end', msg + '\n')
        self.console_output.see('end')
        self.console_output.configure(state='disabled')

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def send_manual_command(self) -> None:
        cmd = self.manual_entry.get().strip()
        if self.ser and self.ser.is_open and cmd:
            try:
                self.ser.write((cmd + '\n').encode())
                self.log_serial(f'> {cmd}')
                self.manual_entry.delete(0, 'end')
            except Exception as e:
                self.log_serial(self.t('log.serial.send_error', error=e))
        else:
            self.log_serial(self.t('log.serial.command_not_sent'))

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def start_periodic_port_refresh(self) -> None:
        self.previous_ports = set(self.get_serial_ports())
        self._refresh_ports_periodically()

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja.
    def _update_online_button(self) -> None:
        try:
            if self.online_mode:
                self.safe_config(self.toggle_online_btn, text=self.t('button.online'), style='Online.TButton')
            else:
                self.safe_config(self.toggle_online_btn, text=self.t('button.offline'), style='Offline.TButton')
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def get_serial_ports(self) -> List[str]:
        try:
            import importlib, sys as _sys
            global serial
            if serial is None:
                try:
                    serial = importlib.import_module('serial')
                except Exception as err:
                    self._log_exception(None, err)
                    self._log(self.t('log.pyserial.missing'))
                    self._log(self.t('log.pyserial.python_path', path=_sys.executable))
                    return []
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception as e:
            self._log(self.t('log.serial.port_list_error', error=e))
            return []

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def read_serial(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.log_arduino(line)
                        if line == 'READY':
                            self._log(self.t('log.serial.device_ready'))
            except Exception as e:
                self.log_arduino(self.t('log.serial.read_error', error=e))
        self.root.after(100, self.read_serial)

    # U ovoj metodi je slanje pozicija motorima preko serijske veze i rad s pozicijama.
    def send_position(self, servo_id: int, position: int, speed: int, accel: Optional[float]=None) -> None:
        if self.online_mode and self.ser and self.ser.is_open:
            if accel is None:
                command = f'#{servo_id}P{position}S{speed}\n'
            else:
                try:
                    a = float(accel)
                    command = f'#{servo_id}P{position}S{speed}A{int(round(a))}\n'
                except Exception:
                    command = f'#{servo_id}P{position}S{speed}\n'
            try:
                self.ser.write(command.encode())
                # Log the sent command so user can see it in the Sent console
                self.log_serial(f'> {command.strip()}')
            except Exception as e:
                self.log_serial(self.t('log.serial.position_send_error', error=e))
            start = time.time()
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self.log_arduino(line)
                if time.time() - start > 0.1:
                    break
        elif not self.online_mode:
            # Log when offline so user knows why nothing is sent
            accel_str = f'A{int(accel)}' if accel else ''
            self.log_serial(f'[OFFLINE] Would send: #{servo_id}P{position}S{speed}{accel_str}')

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def _check_serial_connection(self) -> None:
        if self.ser and (not self.ser.is_open):
            self.ser = None
            self.safe_config(self.toggle_online_btn, text=self.t('button.offline'), style='Offline.TButton')
            self._log(self.t('log.serial.port_closed'))
        self.root.after(2000, self._check_serial_connection)

    # U ovoj metodi je upravljanje serijskom vezom i portovima.
    def connect_serial(self, event: Optional[tk.Event]=None) -> None:
        selected_port = self.combobox.get()
        if not selected_port:
            self._log(self.t('log.serial.no_port'))
            return
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(selected_port, BAUD_RATE, timeout=1)
            time.sleep(2)
            self._log(self.t('log.serial.connected', port=selected_port))
            # Start in offline mode for safety - user must click Online to enable sending
            self.safe_config(self.toggle_online_btn, text=self.t('button.offline'), style='Offline.TButton')
            self.online_mode = False
            self.root.after(100, self.read_serial)
        except Exception as e:
            self._log(self.t('log.serial.connect_error', error=e))
            self.ser = None
            self._update_online_button()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def toggle_online_mode(self) -> None:
        self.online_mode = not self.online_mode
        self._update_online_button()
        self._log(self.t('log.status.online') if self.online_mode else self.t('log.status.offline'))
        if self.ser and self.ser.is_open:
            try:
                if self.online_mode:
                    self.ser.write(b'HELLO\n')
                    self.log_serial(self.t('serial.hello'))
                else:
                    self.ser.write(b'BYE\n')
                    self.log_serial(self.t('serial.bye'))
            except Exception as e:
                self.log_serial(self.t('log.serial.hello_bye_error', error=e))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def get_active_sequence(self) -> List[Dict[str, Any]]:
        return self.sequences.get(self.active_sequence, [])
    def seq_get_selected_index(self) -> Optional[int]:
        selected = self.seq_listbox.curselection()
        return selected[0] if selected else None

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _ensure_sequence_dir(self) -> None:
        try:
            if not os.path.isdir(self.sequence_dir):
                os.makedirs(self.sequence_dir, exist_ok=True)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je inicijalizacija i priprema potrebnih resursa i rad s datotekama/sekvencama.
    def _create_sequence_control_buttons(self):
        controls_frame = ttk.Frame(self.seq_btns_frame)
        controls_frame.pack(side='top', fill='x', anchor='n')
        buttons_frame = ttk.Frame(controls_frame)
        buttons_frame.pack(side='top', anchor='n')
        button_actions = [('➕', self.seq_insert_pose, 'pose'), ('✏️', self.seq_edit_pose, 'edit_pose'), ('⬆️', self.seq_move_up, 'up'), ('⬇️', self.seq_move_down, 'down'), ('🗑', self.seq_delete_item, 'delete'), ('📋', self.seq_copy_item, 'copy'), ('📥', self.seq_paste_item, 'paste')]
        self.seq_control_labels = []
        for row, (emoji, cmd, key) in enumerate(button_actions):
            btn = ttk.Button(buttons_frame, text=emoji, command=cmd, width=7)
            btn.grid(row=row, column=0, padx=2, pady=2, sticky='w')
            label = ttk.Label(buttons_frame, text=self.t(key))
            label.grid(row=row, column=1, padx=2, pady=2, sticky='w')
            self.seq_control_labels.append((label, key))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_move_up(self) -> None:
        self._seq_move_item(-1)
    def seq_move_down(self) -> None:
        self._seq_move_item(1)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_insert_pause(self) -> None:
        try:
            duration = int(self.seq_pause_entry.get())
        except (ValueError, TypeError):
            duration = 1000
        self._seq_insert_item({'pause': duration})

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_paste_item(self) -> None:
        if self.seq_copied_item:
            self._seq_insert_item(self.seq_copied_item.copy())

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_retranslate_ui(self) -> None:
        self.seq_update_ui_texts()
    def _get_current_positions(self) -> List[int]:
        try:
            return [int(s.get()) for s in self.sliders]
        except (tk.TclError, ValueError) as err:
            self._log_exception('reading slider values', err)
            return [0] * min(6, len(getattr(self, 'sliders', [])))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _on_slider_release(self, idx: int, event: Optional[tk.Event]=None) -> None:
        try:
            if not self.sliders or idx >= len(self.sliders):
                return
            val = int(round(float(self.sliders[idx].get())))
            try:
                self.safe_config(self.value_labels[idx], text=str(val))
            except Exception:
                pass
            if getattr(self, '_suppress_slider_send', False):
                return
            try:
                accel_val = float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get())
            except Exception:
                accel_val = 100.0
            try:
                sp = getattr(self, 'speed_var', 100)
                try:
                    speed_val = int(sp.get()) if hasattr(sp, 'get') else int(sp)
                except Exception:
                    try:
                        speed_val = int(float(sp))
                    except Exception:
                        speed_val = 100
                self.send_position(idx + 1, val, speed_val, accel=accel_val)
            except Exception as err:
                self._log_exception('_on_slider_release.send', err)
            try:
                self.save_last_settings()
            except Exception as err:
                self._log_exception('_on_slider_release.save', err)
        except Exception as err:
            self._log_exception('_on_slider_release', err)

    # U ovoj metodi je planiranje odgođenih akcija/ponovnog crteža i rad s pozicijama.
    def _schedule_next_pose(self, delay_ms: int) -> None:
        if not self.sequence_thread_active:
            return
        self._cancel_after('_sequence_after_id')
        delay = max(0, int(delay_ms))
        self._sequence_after_id = self.root.after(delay, self._continue_sequence)

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i rad s datotekama/sekvencama.
    def update_sequence_buttons(self) -> None:
        for i, btn in enumerate(self.seq_buttons, start=1):
            if i == self.active_sequence:
                self.safe_config(btn, style='Selected.TButton')
            else:
                self.safe_config(btn, style='TButton')

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _lock_sequence_switching(self, lock: bool) -> None:
        state = 'disabled' if lock else 'normal'
        for btn in self.seq_buttons:
            self.safe_config(btn, state=state)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_refresh(self) -> None:
        self.seq_listbox.delete(0, tk.END)
        for idx, item in enumerate(self.get_active_sequence()):
            if 'pause' in item:
                self.seq_listbox.insert(tk.END, f"{idx + 1}. {self.t('pause_label')}: {item['pause']} ms")
            else:
                accel_part = ''
                if 'acceleration' in item:
                    accel_part = f" / {item.get('acceleration', 100)}% accel"
                self.seq_listbox.insert(tk.END, f"{idx + 1}. {self.t('pose')}: {item['position']} @ {item.get('speed', 100)}%{accel_part}")

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s pozicijama, rad s datotekama/sekvencama.
    def seq_insert_pose(self) -> None:
        pose = self.get_current_pose()
        if pose:
            self._seq_insert_item(pose.copy())

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_copy_item(self) -> None:
        idx = self.seq_get_selected_index()
        seq = self.get_active_sequence()
        if idx is not None:
            self.seq_copied_item = seq[idx].copy()

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i rad s datotekama/sekvencama.
    def seq_update_ui_texts(self) -> None:
        if self.seq_frame is not None:
            self.safe_config(self.seq_frame, text=self.t('sequence'))
        if self.seq_pause_button is not None:
            self.safe_config(self.seq_pause_button, text=self.t('pause'))
        if hasattr(self, 'seq_pause_label') and self.seq_pause_label is not None:
            self.safe_config(self.seq_pause_label, text=self.t('pause_label'))
        for label, key in self.seq_control_labels:
            self.safe_config(label, text=self.t(key))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s pozicijama.
    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        try:
            return {'position': [int(s.get()) for s in self.sliders], 'speed': int(self.speed_slider.get()), 'acceleration': int(self.accel_var.get())}
        except Exception as e:
            self._log(self.t('log.slider.error', error=e))
            return None

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def toggle_pause(self) -> None:
        self.paused = not self.paused
        if self.paused:
            self.pause_canvas.itemconfig('circle', fill='green')
            self._log(self.t('log.pause.enabled'))
        else:
            self.pause_canvas.itemconfig('circle', fill='lightgray')
            self._log(self.t('log.pause.resumed'))

    # U ovoj metodi je spremanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def save_sequence_name(self) -> None:
        naziv = self.sequence_name_var.get().strip()
        if naziv:
            self.sequence_names[self.active_sequence] = naziv
            self._log(self.t('log.sequence.name_saved', index=self.active_sequence, name=naziv))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_on_select(self, event: Optional[tk.Event]=None) -> None:
        idx = self.seq_get_selected_index()
        if idx is not None:
            self.set_current_index(idx)
            self.update_sliders_from_pose()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _continue_interpolation(self) -> None:
        state = getattr(self, '_interp_state', None)
        if state is None:
            return
        self._interp_after_id = None
        if not self.sequence_thread_active:
            self._complete_interpolation(aborted=True)
            return
        if self.paused:
            self._interp_after_id = self.root.after(100, self._continue_interpolation)
            return
        state['step'] += 1
        ratio = state['step'] / float(state['steps'])
        for i in range(state['n']):
            val = int(round(state['start'][i] + (state['target'][i] - state['start'][i]) * ratio))
            try:
                self.sliders[i].set(val)
                self.safe_config(self.value_labels[i], text=str(val))
            except tk.TclError as err:
                self._log_exception('updating slider during interpolation', err)
        
        # Send current interpolated positions to Arduino - send every step for smoothest motion
        if self.online_mode and self.ser and self.ser.is_open:
            pose = state['pose']
            speed = int(pose.get('speed', 100))
            accel = float(pose.get('acceleration', 100.0))
            for i in range(state['n']):
                val = int(round(state['start'][i] + (state['target'][i] - state['start'][i]) * ratio))
                try:
                    self.send_position(i + 1, val, speed, accel=accel)
                except Exception as err:
                    self._log_exception('send_position during interpolation step', err)
        
        try:
            self.draw_robot_arm()
        except Exception as err:
            self._log_exception('draw_robot_arm during interpolation', err)
        if state['step'] >= state['steps']:
            for i in range(state['n']):
                try:
                    self.sliders[i].set(state['target'][i])
                    self.safe_config(self.value_labels[i], text=str(state['target'][i]))
                except tk.TclError as err:
                    self._log_exception('finalising slider during interpolation', err)
            try:
                self.draw_robot_arm()
            except Exception as err:
                self._log_exception('draw_robot_arm finalising interpolation', err)
            self._complete_interpolation(aborted=False)
        else:
            self._interp_after_id = self.root.after(state['delay'], self._continue_interpolation)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _finish_sequence(self) -> None:
        self._cancel_after('_sequence_after_id')
        self._cancel_after('_interp_after_id')
        self._interp_state = None
        self._suppress_slider_send = False
        self.sequence_thread_active = False
        self.paused = False
        self._lock_sequence_switching(False)
        self.run_canvas.itemconfig('triangle', fill='lightgray')
        self._log(self.t('log.sequence.finished'))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def clear_active_sequence(self) -> None:
        self.sequences[self.active_sequence] = []
        self.current_pose_index = 0
        self.seq_refresh()
        self.update_sliders_from_pose()
        self._log(self.t('log.sequence.cleared', index=self.active_sequence))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def seq_delete_item(self) -> None:
        idx = self.seq_get_selected_index()
        seq = self.get_active_sequence()
        if idx is not None:
            del seq[idx]
            self.seq_refresh()
            if seq:
                new_idx = min(idx, len(seq) - 1)
                self.seq_listbox.selection_clear(0, tk.END)
                self.seq_listbox.select_set(new_idx)
                self.seq_listbox.see(new_idx)
                self.set_current_index(new_idx)
                self.update_sliders_from_pose()
            else:
                self.set_current_index(0)
                self.update_sliders_from_pose()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _complete_interpolation(self, aborted: bool) -> None:
        state = getattr(self, '_interp_state', None)
        if state is None:
            return
        self._cancel_after('_interp_after_id')
        self._interp_state = None
        self._suppress_slider_send = False
        if aborted:
            return
        idx = state['index']
        total = state['total']
        pose = state['pose']
        self._log(self.t('log.pose.detail', index=idx + 1, total=total, position=pose.get('position'), speed=pose.get('speed', 100)))
        self.current_pose_index = idx + 1
        self.update_pose_label()
        self._schedule_next_pose(0)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def set_active_sequence(self, seq_number: int) -> None:
        if self.sequence_thread_active:
            self._log(self.t('log.sequence.switch_blocked'))
            return
        self.active_sequence = seq_number
        self.current_pose_index = 0
        self.update_sequence_buttons()
        self.seq_refresh()
        self.update_sliders_from_pose()
        self.sequence_name_var.set(self.sequence_names[seq_number])
        self._log(self.t('log.sequence.active_set', index=seq_number))

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s pozicijama, rad s datotekama/sekvencama.
    def seq_edit_pose(self) -> None:
        idx = self.seq_get_selected_index()
        if idx is not None:
            pose = self.get_current_pose()
            if pose:
                self.get_active_sequence()[idx] = pose
                self.seq_refresh()
                self._log(self.t('log.pose.updated', index=idx + 1, pose=pose))
        else:
            self._log(self.t('log.pose.none_selected'))

    # U ovoj metodi je učitavanje podataka ili konfiguracije i rad s datotekama/sekvencama.
    def start_sequence_thread(self) -> None:
        if self.paused:
            self._log(self.t('log.pause.block_new'))
            return
        if self.sequence_thread_active:
            self._log(self.t('log.sequence.already_running'))
            return
        if not self.get_active_sequence():
            self._log(self.t('log.sequence.empty'))
            self.run_canvas.itemconfig('triangle', fill='lightgray')
            self.sequence_thread_active = False
            return
        self.sequence_thread_active = True
        self.paused = False
        self._lock_sequence_switching(True)
        self.run_canvas.itemconfig('triangle', fill='green')
        self._cancel_after('_sequence_after_id')
        self._cancel_after('_interp_after_id')
        self._interp_state = None
        self._suppress_slider_send = False
        self.play_sequence()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def play_sequence(self) -> None:
        self.paused = False
        sequence = self.get_active_sequence()
        if not sequence:
            self._log(self.t('log.sequence.empty'))
            self._finish_sequence()
            return
        self._log(self.t('log.sequence.start', index=self.active_sequence))
        self.current_pose_index = 0
        self.update_pose_label()
        self._schedule_next_pose(0)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s datotekama/sekvencama.
    def _continue_sequence(self) -> None:
        self._sequence_after_id = None
        if not self.sequence_thread_active:
            return
        if self.paused:
            self._schedule_next_pose(100)
            return
        sequence = self.get_active_sequence()
        total = len(sequence)
        if total == 0:
            self._log(self.t('log.sequence.empty'))
            self._finish_sequence()
            return
        if self.current_pose_index >= total:
            if self.loop_enabled.get() and self.sequence_thread_active:
                self._log(self.t('log.sequence.start', index=self.active_sequence))
                self.current_pose_index = 0
                self.update_pose_label()
                self._schedule_next_pose(0)
                return
            self._finish_sequence()
            return
        pose = sequence[self.current_pose_index]
        self.update_pose_label()
        if 'pause' in pose:
            pause_ms = int(pose.get('pause', 0))
            self._log(self.t('log.pause.wait', ms=pause_ms))
            self.current_pose_index += 1
            self.update_pose_label()
            self._schedule_next_pose(max(0, pause_ms))
            return
        self._start_pose_interpolation(pose, self.current_pose_index, total)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i rad s pozicijama.
    def _start_pose_interpolation(self, pose: Dict[str, Any], pose_index: int, total: int) -> None:
        target = list(pose.get('position', []))
        try:
            speed_val = float(pose.get('speed', 100))
            try:
                self.speed_var.set(speed_val)
            except Exception:
                pass
            try:
                self.speed_slider.set(speed_val)
            except Exception:
                pass
            self.update_speed_label(speed_val)
        except Exception:
            pass
        n = min(len(self.sliders), len(target))
        if n == 0:
            self._log(self.t('log.pose.detail', index=pose_index + 1, total=total, position=pose.get('position'), speed=pose.get('speed', 100)))
            self.current_pose_index = pose_index + 1
            self.update_pose_label()
            self._schedule_next_pose(0)
            return
        start = self._get_current_positions()[:n]
        target_vals = [int(target[i]) for i in range(n)]
        speed = int(pose.get('speed', 100))
        max_delta = max((abs(target_vals[i] - start[i]) for i in range(n))) or 1
        duration_base = float(getattr(self, 'time_per_unit', 0.01)) * max_delta
        accel = float(pose.get('acceleration', getattr(self, 'accel_var', tk.DoubleVar(value=100)).get()))
        try:
            accel = max(1.0, float(accel))
        except Exception:
            accel = 100.0
        duration = max(0.05, duration_base * (100.0 / max(1, speed)) * (100.0 / accel))
        fps = int(getattr(self, 'anim_fps', 60)) or 60
        steps = max(1, int(duration * fps))
        step_delay_ms = max(1, int(round(duration / steps * 1000.0)))
        if self.online_mode and self.ser and self.ser.is_open:
            for i in range(6):
                try:
                    self.min_angles_deg[i] = float(self.min_angles_deg[i])
                except Exception:
                    self.min_angles_deg[i] = -180.0
                if self.min_angles_deg[i] >= self.max_angles_deg[i]:
                    self.min_angles_deg[i] = self.max_angles_deg[i] - 1.0
                if self.min_angles_deg[i] < -180.0:
                    self.min_angles_deg[i] = -180.0
        self._suppress_slider_send = True
        self._interp_state = {'pose': pose, 'index': pose_index, 'total': total, 'start': start, 'target': target_vals, 'n': n, 'steps': steps, 'step': 0, 'delay': step_delay_ms}
        self._continue_interpolation()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def set_current_index(self, idx: int) -> None:
        self.current_pose_index = idx
    def _schedule_robot_draw(self, delay: int=75) -> None:
        try:
            self._draw_pending = True
            after_id = getattr(self, '_draw_after_id', None)
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception as err:
                    self._log_exception('_schedule_robot_draw.after_cancel', err)
            try:
                delay_ms = max(1, int(delay))
            except (TypeError, ValueError):
                delay_ms = 75
            self._draw_after_id = self.root.after(delay_ms, self._perform_scheduled_draw)
        except Exception as err:
            self._log_exception('_schedule_robot_draw', err)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _perform_scheduled_draw(self) -> None:
        self._draw_after_id = None
        if not getattr(self, '_draw_pending', False):
            return
        self._draw_pending = False
        try:
            self.draw_robot_arm()
        except Exception as err:
            self._log_exception('_perform_scheduled_draw', err)

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i rad s pozicijama.
    def update_pose_label(self) -> None:
        total = len(self.get_active_sequence())
        if total > 0:
            self.safe_config(self.pose_label, text=self.t('pose_label_dynamic', current=self.current_pose_index + 1, total=total))
        else:
            self.safe_config(self.pose_label, text=self.t('pose_label'))

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i rad s pozicijama.
    def update_sliders_from_pose(self) -> None:
        sequence = self.get_active_sequence()
        if sequence and 0 <= self.current_pose_index < len(sequence):
            pose = sequence[self.current_pose_index]
            if 'position' in pose:
                for idx, pos in enumerate(pose['position']):
                    if idx < len(self.sliders):
                        self.sliders[idx].set(pos)
                        self.safe_config(self.value_labels[idx], text=str(pos))
                        speed = pose.get('speed', 100)
                        accel_val = pose.get('acceleration', None)
                        if accel_val is None:
                            accel_val = getattr(self, 'accel_var', tk.DoubleVar(value=100)).get()
                        self.send_position(idx + 1, pos, speed, accel=accel_val)
                speed_val = float(pose.get('speed', 100))
                try:
                    self.speed_var.set(speed_val)
                except Exception:
                    pass
                try:
                    self.speed_slider.set(speed_val)
                except Exception:
                    pass
                self.update_speed_label(speed_val)
                try:
                    accel_val = float(pose.get('acceleration', self.accel_var.get()))
                    try:
                        self.accel_var.set(accel_val)
                    except Exception:
                        pass
                except Exception:
                    pass
        self.update_pose_label()
        self._schedule_robot_draw(delay=1)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def stop_all_actions(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'STOP\n')
                self.log_serial(self.t('serial.stop'))
            except Exception as e:
                self.log_serial(self.t('log.serial.stop_error', error=e))
        self.online_mode = False
        self.safe_config(self.toggle_online_btn, text=self.t('button.offline'), style='Offline.TButton')
        self._log(self.t('log.status.offline_stop'))
        self._cancel_after('_sequence_after_id')
        self._cancel_after('_interp_after_id')
        self._interp_state = None
        self._suppress_slider_send = False
        self.sequence_thread_active = False
        self._lock_sequence_switching(False)
        try:
            self.run_canvas.itemconfig('triangle', fill='lightgray')
        except Exception as err:
            self._log_exception(None, err)
        self.paused = False
        self.current_pose_index = 0
        self.update_sliders_from_pose()
        self.update_pose_label()
        try:
            self.pause_canvas.itemconfig('circle', fill='lightgray')
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _draw_or_update_base(self):
        if not self._has_3d():
            return
        if getattr(self, 'minimal_view', False):
            return
        if Poly3DCollection is None:
            return
        L = (sum(self.arm_lengths[:5]) if hasattr(self, 'arm_lengths') else 300) * 0.35
        verts = [[[-L, -L, 0], [-L, L, 0], [L, L, 0], [L, -L, 0]]]
        if self._base_patch is None:
            self._base_patch = Poly3DCollection(verts, alpha=0.15)
            self.ax3d.add_collection3d(self._base_patch)
        else:
            self._base_patch.set_verts(verts)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _plot_cylinder_between(self, p0, p1, radius=5.0, color='silver', resolution=16):
        if not hasattr(self, 'ax3d'):
            return None
        v = p1 - p0
        length = np.linalg.norm(v)
        if length < 1e-06:
            return None
        v_unit = v / length
        theta = np.linspace(0, 2 * np.pi, resolution)
        circle = np.vstack([radius * np.cos(theta), radius * np.sin(theta), np.zeros_like(theta)])
        z_axis = np.array([0.0, 0.0, 1.0])
        axis = np.cross(z_axis, v_unit)
        axis_len = np.linalg.norm(axis)
        if axis_len < 1e-08:
            if np.dot(z_axis, v_unit) > 0:
                R = np.eye(3)
            else:
                R = np.diag([1, 1, -1])
        else:
            axis = axis / axis_len
            angle = np.arccos(np.clip(np.dot(z_axis, v_unit), -1.0, 1.0))
            K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        pts = R @ circle
        n_z = max(2, int(max(2, length / max(1.0, radius * 0.5))))
        z_lin = np.linspace(0, length, n_z)
        X = np.zeros((resolution, z_lin.size))
        Y = np.zeros_like(X)
        Z = np.zeros_like(X)
        for i, z in enumerate(z_lin):
            layer = pts + np.outer(v_unit * z, np.ones(resolution))
            X[:, i] = layer[0, :]
            Y[:, i] = layer[1, :]
            Z[:, i] = layer[2, :]
        X += p0[0]
        Y += p0[1]
        Z += p0[2]
        surf = self.ax3d.plot_surface(X, Y, Z, color=color, linewidth=0, antialiased=False, shade=True)
        try:
            surf.set_label('robot_patch')
        except Exception as err:
            self._log_exception(None, err)
        return surf

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _plot_sphere_at(self, center, radius=5.0, color='k', resolution=16, edge=False, edge_color='k', edge_width=0.3):
        if not hasattr(self, 'ax3d'):
            return None
        u = np.linspace(0, np.pi, resolution)
        v = np.linspace(0, 2 * np.pi, resolution * 2)
        uu, vv = np.meshgrid(u, v)
        x = center[0] + radius * np.sin(uu) * np.cos(vv)
        y = center[1] + radius * np.sin(uu) * np.sin(vv)
        z = center[2] + radius * np.cos(uu)
        try:
            surf = self.ax3d.plot_surface(x, y, z, color=color, linewidth=0, antialiased=False, shade=True)
            if edge:
                try:
                    rstride = max(1, int(resolution / 6))
                    cstride = max(1, int(resolution * 2 / 6))
                    wf = self.ax3d.plot_wireframe(x, y, z, rstride=rstride, cstride=cstride, color=edge_color, linewidth=edge_width, alpha=0.9)
                    try:
                        self._transient_artists.append(wf)
                    except Exception as err:
                        self._log_exception(None, err)
                except Exception as err:
                    self._log_exception(None, err)
            return surf
        except Exception as err:
            self._log_exception(None, err)
            try:
                s = max(20, int(radius * 6))
                sc = self.ax3d.scatter([center[0]], [center[1]], [center[2]], s=s, c=color)
                if edge:
                    try:
                        outline = self.ax3d.scatter([center[0]], [center[1]], [center[2]], s=max(4, int(s * 0.12)), facecolors='none', edgecolors=edge_color)
                        try:
                            self._transient_artists.append(outline)
                        except Exception as err:
                            self._log_exception(None, err)
                    except Exception as err:
                        self._log_exception(None, err)
                return sc
            except Exception as err:
                self._log_exception(None, err)
                return None

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _apply_minimal_view(self) -> None:
        try:
            self.ax3d.grid(False)
            try:
                self.ax3d.set_xticks([])
                self.ax3d.set_yticks([])
                self.ax3d.set_zticks([])
                self.ax3d.set_xlabel('')
                self.ax3d.set_ylabel('')
                self.ax3d.set_zlabel('')
            except Exception as err:
                self._log_exception(None, err)
            try:
                self.ax3d.set_axis_off()
            except Exception as err:
                self._log_exception(None, err)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _apply_normal_view(self) -> None:
        try:
            try:
                self.ax3d.set_axis_on()
            except Exception as err:
                self._log_exception(None, err)
                try:
                    self.ax3d.set_frame_on(True)
                except Exception as err:
                    self._log_exception(None, err)
            self.ax3d.grid(self.show_axis_planes)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _ensure_ground(self) -> None:
        try:
            if getattr(self, '_ground', None) is None:
                L = sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300
                g = np.linspace(-L, L, 11)
                GX, GY = np.meshgrid(g, g)
                GZ = np.zeros_like(GX)
                self._ground = self.ax3d.plot_wireframe(GX, GY, GZ, linewidth=0.3, alpha=0.3)
            else:
                try:
                    self._ground.set_visible(True)
                except Exception as err:
                    self._log_exception(None, err)
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _draw_ucs_gizmo(self) -> None:
        try:
            if getattr(self, '_ucs_quivers', None):
                for q in self._ucs_quivers:
                    try:
                        q.remove()
                    except Exception as err:
                        self._log_exception(None, err)
                self._ucs_quivers = []
            Ltot = sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300
            giz = Ltot * 0.35
            ox, oy, oz = (0.0, 0.0, 0.0)
            qx = self.ax3d.quiver(ox, oy, oz, giz, 0, 0, arrow_length_ratio=0.3, color='r')
            qy = self.ax3d.quiver(ox, oy, oz, 0, giz, 0, arrow_length_ratio=0.3, color='g')
            dot = self.ax3d.scatter([ox], [oy], [oz], s=20, c='k')
            try:
                lx = ox + giz * 1.05
                ly = oy + giz * 1.05
                tx = self.ax3d.text(lx, oy, oz, 'X', color='r', fontsize=10, weight='bold')
                ty = self.ax3d.text(ox, ly, oz, 'Y', color='g', fontsize=10, weight='bold')
            except Exception:
                tx = ty = None
            self._ucs_quivers = [qx, qy, dot, tx, ty]
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def _draw_idle(self) -> None:
        try:
            if hasattr(self, 'canvas3d'):
                self.canvas3d.draw_idle()
            elif self._has_3d():
                self.ax3d.figure.canvas.draw_idle()
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja.
    def _update_axes_limits(self):
        total_len = (sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300) + 20
        zoom = getattr(self, 'initial_view_zoom', 0.5)
        try:
            total_len = total_len * float(zoom)
        except Exception as err:
            self._log_exception(None, err)
        hpad = 1.1
        vpad = 1.1
        ratio = float(getattr(self, 'aspect_ratio_xy', 16 / 9))
        self.ax3d.set_xlim(-total_len * hpad * ratio, total_len * hpad * ratio)
        self.ax3d.set_ylim(-total_len * hpad, total_len * hpad)
        self.ax3d.set_zlim(0, total_len * vpad)
        try:
            if getattr(self, 'initial_view_dist', None) is not None:
                self.ax3d.dist = self.initial_view_dist
        except Exception as err:
            self._log_exception(None, err)
        try:
            self._draw_ucs_gizmo()
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je crtanje/prikaz robota ili UI elemenata.
    def draw_robot_arm(self):
        detail_mode = self._current_detail_mode()
        try:
            if getattr(self, 'low_latency', False) and getattr(self, 'sequence_thread_active', False):
                now = time.time()
                throttle = 0.06 if detail_mode == 'low' else 0.04
                if now - getattr(self, '_last_draw_time', 0.0) < throttle:
                    return
                self._last_draw_time = now
        except Exception as err:
            self._log_exception(None, err)
        if hasattr(self, 'canvas3d'):
            try:
                now_f = time.time()
                last_f = getattr(self, '_fps_last_ts', None)
                if last_f is not None:
                    dt = max(1e-06, now_f - last_f)
                    fps = 1.0 / dt
                    a = 0.2
                    self._fps_ema = (1 - a) * float(getattr(self, '_fps_ema', 0.0)) + a * fps
                    if getattr(self, '_show_fps', False) and getattr(self, 'fps_label', None) is not None:
                        try:
                            self.safe_config(self.fps_label, text='FPS: ' + '%5.1f' % self._fps_ema)
                        except Exception as err:
                            self._log_exception(None, err)
                    elif getattr(self, 'fps_label', None) is not None:
                        try:
                            self.safe_config(self.fps_label, text='')
                        except Exception as err:
                            self._log_exception(None, err)
                self._fps_last_ts = now_f
            except Exception as err:
                self._log_exception(None, err)
            if getattr(self, '_transient_artists', None):
                for art in self._transient_artists:
                    try:
                        art.remove()
                    except Exception as err:
                        self._log_exception(None, err)
                self._transient_artists = []
            pose = self.get_current_pose()
            if not pose or 'position' not in pose:
                self._draw_idle()
                return
            xs, ys, zs, origin, R = self._fk(pose['position'])
            if detail_mode == 'low':
                cyl_resolution = 6
                joint_resolution = 6
                tcp_resolution = 8
            else:
                cyl_resolution = 12
                joint_resolution = 12
                tcp_resolution = 16
            if self._arm_patches:
                for p in self._arm_patches:
                    try:
                        p.remove()
                    except Exception as err:
                        self._log_exception(None, err)
            self._arm_patches = []
            total_len = sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300
            default_radius = max(10.0, total_len * 0.02)
            cyl_radius = default_radius * 0.5
            joint_radius = max(6.0, cyl_radius * 1.25)
            tcp_radius = max(9.0, cyl_radius * 1.6)
            for i in range(len(xs) - 1):
                p0 = np.array([xs[i], ys[i], zs[i]])
                p1 = np.array([xs[i + 1], ys[i + 1], zs[i + 1]])
                try:
                    surf = self._plot_cylinder_between(p0, p1, radius=cyl_radius, color='silver', resolution=cyl_resolution)
                    if surf is not None:
                        self._arm_patches.append(surf)
                    else:
                        self.ax3d.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color='silver', linewidth=max(2.0, cyl_radius * 0.3))
                except Exception as e:
                    self._log(self.t('log.plot.cylinder_fail', index=i, error=e))
                    try:
                        self.ax3d.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color='silver', linewidth=max(2.0, cyl_radius * 0.3))
                    except Exception as err:
                        self._log_exception(None, err)
                        if self._arm_line3d is None:
                            self._arm_line3d, = self.ax3d.plot(xs, ys, zs, marker='o', linewidth=2)
                        else:
                            self._arm_line3d.set_data(xs, ys)
                            self._arm_line3d.set_3d_properties(zs)
            joint_center = np.array([xs[-1], ys[-1], zs[-1]])
            if not getattr(self, 'preserve_view', True):
                self._update_axes_limits()
            ox, oy, oz = origin
            axis_len = 0.15 * (sum(self.arm_lengths) if hasattr(self, 'arm_lengths') else 300)
            ux = (R[0][0], R[1][0], R[2][0])
            uy = (R[0][1], R[1][1], R[2][1])
            uz = (R[0][2], R[1][2], R[2][2])
            if self._tcp_quivers:
                for q in self._tcp_quivers:
                    try:
                        if q:
                            q.remove()
                    except Exception as err:
                        self._log_exception(None, err)
            self._tcp_quivers = []
            if getattr(self, '_joint_spheres', None):
                for s in self._joint_spheres:
                    try:
                        s.remove()
                    except Exception as err:
                        self._log_exception(None, err)
            self._joint_spheres = []
            joint_radius = max(cyl_radius * 1.6, 8.0)
            for idx, (xj, yj, zj) in enumerate(zip(xs, ys, zs)):
                try:
                    surf = self._plot_sphere_at(np.array([xj, yj, zj]), radius=joint_radius, color='k', resolution=joint_resolution, edge=False, edge_color='black', edge_width=0.3)
                    if surf is not None:
                        self._joint_spheres.append(surf)
                except Exception as err:
                    self._log_exception(None, err)
            if getattr(self, '_tcp_sphere', None):
                try:
                    self._tcp_sphere.remove()
                except Exception as err:
                    self._log_exception(None, err)
            tcp_radius = max(10.0, cyl_radius * 1.8)
            try:
                ux_arr = np.array([R[0][0], R[1][0], R[2][0]])
                flen = float(getattr(self, 'gripper_length', 25.0))
                fingertip = joint_center + ux_arr * flen
                try:
                    self._last_tcp_pos = (float(fingertip[0]), float(fingertip[1]), float(fingertip[2]))
                except Exception:
                    try:
                        self._last_tcp_pos = (float(np.float64(fingertip[0])), float(np.float64(fingertip[1])), float(np.float64(fingertip[2])))
                    except Exception:
                        self._last_tcp_pos = (float(fingertip[0]), float(fingertip[1]), float(fingertip[2]))
                try:
                    if hasattr(self, 'tcp_label'):
                        self.safe_config(self.tcp_label, text=self.t('label.tcp', x=fingertip[0], y=fingertip[1], z=fingertip[2]))
                except Exception as err:
                    self._log_exception(None, err)
                try:
                    small_tcp_radius = max(4.0, tcp_radius * 0.45)
                    self._tcp_sphere = self._plot_sphere_at(np.array(fingertip), radius=small_tcp_radius, color='red', resolution=max(6, tcp_resolution // 2), edge=False, edge_color='black', edge_width=0.2)
                except Exception as err:
                    self._log_exception(None, err)
                    self._tcp_sphere = None
                try:
                    if getattr(self, '_fingertip_marker', None):
                        try:
                            self._fingertip_marker.remove()
                        except Exception:
                            pass
                    self._fingertip_marker = self.ax3d.scatter([fingertip[0]], [fingertip[1]], [fingertip[2]], s=12, c='red')
                    try:
                        self._transient_artists.append(self._fingertip_marker)
                    except Exception:
                        pass
                except Exception as err:
                    self._log_exception(None, err)
            except Exception as err:
                self._log_exception(None, err)
            if getattr(self, '_gripper_lines', None):
                for ln in self._gripper_lines:
                    try:
                        ln.remove()
                    except Exception as err:
                        self._log_exception(None, err)
                self._gripper_lines = []
            try:
                grip_raw = int(pose['position'][5]) if len(pose['position']) > 5 else 512
            except Exception as err:
                self._log_exception(None, err)
                grip_raw = 512
            flen = float(getattr(self, 'gripper_length', 25.0))
            ux = np.array([R[0][0], R[1][0], R[2][0]])
            uy = np.array([R[0][1], R[1][1], R[2][1]])
            o = joint_center
            try:
                gmin = float(self.min_angles_deg[5])
            except Exception as err:
                self._log_exception(None, err)
                gmin = 0.0
            try:
                gmax = float(self.max_angles_deg[5])
            except Exception as err:
                self._log_exception(None, err)
                gmax = 30.0
            span = max(gmax - gmin, 1e-06)
            ratio = max(0.0, min(1.0, float(grip_raw) / 1023.0))
            open_deg = gmin + ratio * span
            open_deg = max(0.0, min(180.0, open_deg))
            th = math.radians(open_deg * 0.5)
            c, s = (math.cos(th), math.sin(th))
            d1 = c * ux + s * uy
            d2 = c * ux - s * uy
            l0 = o
            r0 = o
            l1 = l0 + d1 * flen
            r1 = r0 + d2 * flen
            try:
                ln1 = self.ax3d.plot([l0[0], l1[0]], [l0[1], l1[1]], [l0[2], l1[2]], color='black', linewidth=max(2.0, cyl_radius * 0.35))
                ln2 = self.ax3d.plot([r0[0], r1[0]], [r0[1], r1[1]], [r0[2], r1[2]], color='black', linewidth=max(2.0, cyl_radius * 0.35))
                if ln1:
                    self._gripper_lines.append(ln1[0])
                if ln2:
                    self._gripper_lines.append(ln2[0])
            except Exception as err:
                self._log_exception(None, err)
            self._draw_or_update_base()
            self._draw_idle()
            return

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _current_detail_mode(self) -> str:
        mode = getattr(self, 'detail_mode', 'auto') or 'auto'
        if mode == 'auto':
            return 'low' if getattr(self, 'sequence_thread_active', False) else 'high'
        return 'low' if mode == 'low' else 'high'

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _toggle_show_fps(self) -> None:
        try:
            self._show_fps = bool(self.show_fps_var.get())
            if not self._show_fps and getattr(self, 'fps_label', None) is not None:
                self.safe_config(self.fps_label, text='')
        except Exception as err:
            self._log_exception('_toggle_show_fps', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _on_detail_mode(self) -> None:
        try:
            mode = self.detail_mode_var.get()
            self.detail_mode = mode
            rid = getattr(self, '_restore_detail_after_id', None)
            if rid is not None:
                try:
                    self.root.after_cancel(rid)
                except Exception as err:
                    self._log_exception('_on_detail_mode.cancel', err)
                self._restore_detail_after_id = None
            self._schedule_robot_draw(delay=1)
        except Exception as err:
            self._log_exception('_on_detail_mode', err)

    # U ovoj metodi je logiranje i ispis poruka.
    def _show_about_dialog(self) -> None:
        try:
            title = self.t('menu.about_title')
            if not title or title == 'menu.about_title':
                title = 'About'
            message = self.t('menu.about_message')
            if not message or message == 'menu.about_message':
                message = 'ArbotiX GUI'
            messagebox.showinfo(title, message)
        except Exception as err:
            self._log_exception('_show_about_dialog', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _on_3d_container_resize(self, event):
        try:
            w_in = max(1, (event.width - 2) / float(self.fig3d.dpi))
            h_in = max(1, (event.height - 2) / float(self.fig3d.dpi))
            self.fig3d.set_size_inches(w_in, h_in, forward=False)
            self._draw_idle()
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i upravljanje brzinom.
    def update_speed_label(self, val: float) -> None:
        if hasattr(self, 'speed_value_label'):
            try:
                self.safe_config(self.speed_value_label, text=f'{int(float(val))}%')
            except Exception as err:
                self._log_exception(None, err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i upravljanje ubrzanjem.
    def _on_accel_entry_change(self, event: Optional[tk.Event]=None) -> None:
        try:
            widget = event.widget if event is not None else getattr(self, 'accel_entry', None)
            if widget is None:
                return
            txt = (widget.get() or '').strip()
            if txt == '':
                return
            try:
                v = float(txt)
            except Exception:
                return
            if getattr(self, 'accel_var', None) is not None:
                try:
                    self.accel_var.set(v)
                except Exception:
                    pass
            try:
                prev = getattr(self, '_accel_save_after_id', None)
                if prev is not None:
                    try:
                        self.root.after_cancel(prev)
                    except Exception:
                        pass
                self._accel_save_after_id = self.root.after(600, lambda: self._persist_accel_change())
            except Exception as err:
                self._log_exception('_on_accel_entry_change.schedule', err)
        except Exception as err:
            self._log_exception('_on_accel_entry_change', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju i upravljanje ubrzanjem.
    def _persist_accel_change(self) -> None:
        try:
            try:
                self.save_last_settings()
            except Exception as err:
                self._log_exception('_persist_accel_change.save', err)
        except Exception as err:
            self._log_exception('_persist_accel_change', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def slider_changed(self, idx: int, val: str) -> None:
        if idx < len(self.value_labels):
            position = int(float(val))
            self.safe_config(self.value_labels[idx], text=str(position))
            if not getattr(self, '_suppress_slider_send', False):
                speed = int(float(self.speed_slider.get()))
                try:
                    accel_val = float(getattr(self, 'accel_var', tk.DoubleVar(value=100)).get())
                except Exception:
                    accel_val = 100.0
                self.send_position(idx + 1, position, speed, accel=accel_val)
            if not (getattr(self, 'low_latency', False) and getattr(self, 'sequence_thread_active', False)):
                self._schedule_robot_draw()
            try:
                self._update_tcp_position_display()
            except Exception:
                pass

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def retranslate_ui(self) -> None:
        self.root.title(self.t('window_title'))
        self.seq_retranslate_ui()
        elements = {self.serial_frame: 'serial_comm', self.simulation_frame: 'simulation', self.com_label: 'select_com', self.speed_label: 'speed_label', self.clear_sent_button: 'clear', self.clear_recv_button: 'clear', self.pause_button: 'pause', self.save_name_button: 'save_name', self.save_all_button: 'save_all', self.load_button: 'load', self.clear_seq_button: 'delete', self.run_button: 'run', self.name_entry_label: 'name', self.sequence_frame: 'active_sequence', self.manual_send_btn: 'send_command', self.manual_label: 'manual_input', self.sent_label: 'sent', self.recv_label: 'received', self.seq_buttons_label: 'active_sequence', self.control_frame: 'control_panel', self.stop_button: 'stop'}
        for element, key in elements.items():
            self.safe_config(element, text=self.t(key))
        self.safe_config(self.loop_check, text=self.t('loop'))
        if hasattr(self, 'save_machine_btn'):
            self.safe_config(self.save_machine_btn, text=self.t('save_machine_settings'))
        if hasattr(self, 'load_machine_btn'):
            self.safe_config(self.load_machine_btn, text=self.t('load_machine_settings'))
        for i, lbl in enumerate(self.servo_labels):
            self.safe_config(lbl, text=f"{self.t('motor')} {i + 1}")
        if hasattr(self, '_motor_header_spacer'):
            try:
                self.safe_config(self._motor_header_spacer, text='')
            except Exception:
                pass
        if hasattr(self, 'min_angle_header_label'):
            try:
                self.safe_config(self.min_angle_header_label, text='Min')
            except Exception:
                pass
        if hasattr(self, 'max_angle_header_label'):
            try:
                self.safe_config(self.max_angle_header_label, text='Max')
            except Exception:
                pass
        if hasattr(self, 'length_header_label'):
            try:
                self.safe_config(self.length_header_label, text=self.t('length_short').capitalize())
            except Exception:
                pass
        if hasattr(self, 'length_header_label'):
            try:
                self.safe_config(self.length_header_label, text=self.t('length_short'))
            except Exception:
                pass
        try:
            self._setup_menubar()
        except Exception as err:
            self._log_exception(None, err)
        if hasattr(self, 'tb_center_btn'):
            self.safe_config(self.tb_center_btn, text=self.t('center'))
        if hasattr(self, 'tb_zoom_in_btn'):
            self.safe_config(self.tb_zoom_in_btn, text=self.t('zoom_plus'))
        if hasattr(self, 'tb_zoom_out_btn'):
            self.safe_config(self.tb_zoom_out_btn, text=self.t('zoom_minus'))
        if hasattr(self, 'tb_reset_btn'):
            self.safe_config(self.tb_reset_btn, text=self.t('reset'))
        if hasattr(self, 'tb_toggle_ground_btn'):
            self.safe_config(self.tb_toggle_ground_btn, text=self.t('toggle_ground'))
        if hasattr(self, 'accel_caption_label') and self.accel_caption_label is not None:
            try:
                if getattr(self, 'language', 'hr') == 'hr':
                    short_lbl = 'Akcel (%)'
                else:
                    short_lbl = 'Accel (%)'
                self.safe_config(self.accel_caption_label, text=short_lbl)
            except Exception:
                pass
        if hasattr(self, 'update_home_btn'):
            self.safe_config(self.update_home_btn, text=self.t('update_home'))
        if hasattr(self, 'goto_home_btn'):
            self.safe_config(self.goto_home_btn, text=self.t('home_btn'))
        if hasattr(self, 'center_motors_btn'):
            self.safe_config(self.center_motors_btn, text=self.t('center_btn'))
        if hasattr(self, 'joystick_frame'):
            self.safe_config(self.joystick_frame, text=self.t('tcp_control'))
        if hasattr(self, 'step_label'):
            self.safe_config(self.step_label, text=f"{self.t('step')} ({self.t('step_unit')}):")
        if hasattr(self, 'tcp_home_btn'):
            self.safe_config(self.tcp_home_btn, text=self.t('home_btn'))
        if hasattr(self, 'tcp_center_btn'):
            self.safe_config(self.tcp_center_btn, text=self.t('center_btn'))
        if hasattr(self, 'z_plus_btn'):
            self.safe_config(self.z_plus_btn, text=self.t('z_plus'))
        if hasattr(self, 'z_minus_btn'):
            self.safe_config(self.z_minus_btn, text=self.t('z_minus'))
        if hasattr(self, 'tcp_pos_title_label'):
            self.safe_config(self.tcp_pos_title_label, text=f"{self.t('tcp_position')}:")
        if hasattr(self, 'coord_frame'):
            self.safe_config(self.coord_frame, text=self.t('coordinate_input'))
        self.seq_refresh()

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _toggle_minimal_view(self) -> None:
        try:
            self.minimal_view = not self.minimal_view
            if self.minimal_view:
                self._apply_minimal_view()
            else:
                self._apply_normal_view()
            self._draw_ucs_gizmo()
            self._ensure_ground()
            self._draw_idle()
        except Exception as err:
            self._log_exception(None, err)

    # U ovoj metodi je obrada ulaza joysticka.
    def _setup_joystick_panel(self, parent: ttk.Frame) -> None:
        try:
            self.joystick_frame = ttk.LabelFrame(parent, text=self.t('tcp_control'))
            self.joystick_frame.pack(fill='both', expand=True, padx=5, pady=5)
            step_frame = ttk.Frame(self.joystick_frame)
            step_frame.pack(fill='x', padx=5, pady=5)
            self.step_label = ttk.Label(step_frame, text=f"{self.t('step')} ({self.t('step_unit')}):")
            self.step_label.pack(side='left')
            self.joystick_step_var = tk.DoubleVar(value=self.joystick_step_mm)
            step_entry = ttk.Entry(step_frame, textvariable=self.joystick_step_var, width=8)
            step_entry.pack(side='right')
            btn_frame = ttk.Frame(self.joystick_frame)
            btn_frame.pack(padx=10, pady=5)
            btn_rotate_left = ttk.Button(btn_frame, text='⟲', width=5, command=lambda: self._motor_command('rotate_left'))
            btn_rotate_left.grid(row=1, column=0, padx=6, pady=3)
            btn_forward = ttk.Button(btn_frame, text='▲ TCP', width=8, command=lambda: self._motor_command('tcp_forward'))
            btn_forward.grid(row=0, column=1, padx=6, pady=3)
            btn_rotate_right = ttk.Button(btn_frame, text='⟳', width=5, command=lambda: self._motor_command('rotate_right'))
            btn_rotate_right.grid(row=1, column=2, padx=6, pady=3)
            self.tcp_home_btn = ttk.Button(btn_frame, text=self.t('home_btn') if hasattr(self, 't') else 'Home', width=8, command=self._joystick_home)
            self.tcp_home_btn.grid(row=1, column=1, padx=6, pady=3)
            btn_back = ttk.Button(btn_frame, text='▼ TCP', width=8, command=lambda: self._motor_command('tcp_back'))
            btn_back.grid(row=2, column=1, padx=6, pady=3)
            btn_z_up = ttk.Button(btn_frame, text='Z+', width=8, command=lambda: self._motor_command('z_up'))
            btn_z_up.grid(row=3, column=0, padx=6, pady=6)
            btn_z_down = ttk.Button(btn_frame, text='Z-', width=8, command=lambda: self._motor_command('z_down'))
            btn_z_down.grid(row=3, column=2, padx=6, pady=6)
            self.tcp_center_btn = ttk.Button(btn_frame, text=self.t('center_btn') if hasattr(self, 't') else 'Centriraj', width=10, command=self._center_motors)
            self.tcp_center_btn.grid(row=4, column=1, padx=6, pady=6)
            try:
                self.root.bind('<Left>', lambda e: self._motor_command('rotate_left'))
                self.root.bind('<Right>', lambda e: self._motor_command('rotate_right'))
                self.root.bind('<Up>', lambda e: self._motor_command('tcp_forward'))
                self.root.bind('<Down>', lambda e: self._motor_command('tcp_back'))
                self.root.bind('<Prior>', lambda e: self._motor_command('z_up'))
                self.root.bind('<Next>', lambda e: self._motor_command('z_down'))
                self.root.bind('q', lambda e: self._motor_command('rotate_left'))
                self.root.bind('e', lambda e: self._motor_command('rotate_right'))
                self.root.bind('w', lambda e: self._motor_command('z_up'))
                self.root.bind('s', lambda e: self._motor_command('z_down'))
            except Exception:
                pass
        except Exception as err:
            self._log_exception('_setup_joystick_panel', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _setup_coordinate_panel(self, parent: ttk.Frame) -> None:
        try:
            self.coord_frame = ttk.LabelFrame(parent, text=self.t('coordinate_input'))
            self.coord_frame.pack(fill='x', padx=5, pady=(5, 0))
        except Exception as err:
            self._log_exception('_setup_coordinate_panel', err)

    # U ovoj metodi je povratak ili postavljanje početnog položaja.
    def _joystick_home(self) -> None:
        try:
            home_positions = getattr(self, 'home_positions', [511] * 6)
            for i, slider in enumerate(self.sliders):
                if i < len(home_positions):
                    home_pos = int(home_positions[i])
                else:
                    home_pos = 511
                slider.set(home_pos)
                try:
                    self.safe_config(self.value_labels[i], text=str(home_pos))
                except Exception:
                    pass
            self._schedule_robot_draw(delay=1)
            self._update_tcp_position_display()
            try:
                self._log(self.t('log.tcp.home'))
            except Exception:
                pass
        except Exception as err:
            self._log_exception('_joystick_home', err)

    # U ovoj metodi je pomoćna logika specifična za aplikaciju.
    def _center_motors(self) -> None:
        try:
            for i, slider in enumerate(self.sliders):
                slider.set(511)
                try:
                    self.safe_config(self.value_labels[i], text='511')
                except Exception:
                    pass
            self._schedule_robot_draw(delay=1)
            self._update_tcp_position_display()
            try:
                self._log(self.t('log.motors.centered') if hasattr(self, 't') else 'Motori centrirani na poziciju 511')
            except Exception:
                pass
        except Exception as err:
            self._log_exception('_center_motors', err)

    # U ovoj metodi je ažuriranje stanja i korisničkog sučelja i rad s pozicijama.
    def _update_home_position(self) -> None:
        try:
            title = self.t('update_home_confirm_title') if hasattr(self, 't') else 'Potvrda'
            message = self.t('update_home_confirm_msg') if hasattr(self, 't') else 'Želite li postaviti trenutnu poziciju kao novu home poziciju?'
            result = messagebox.askyesno(title, message)
            if not result:
                return
            current_positions = self._get_current_positions()
            self.home_positions = current_positions[:6] + [511] * max(0, 6 - len(current_positions))
            try:
                self.save_last_settings()
                self._log(self.t('log.home.updated') if hasattr(self, 't') else f'Home pozicija ažurirana: {self.home_positions[:6]}')
            except Exception as err:
                self._log_exception('_update_home_position.save', err)
        except Exception as err:
            self._log_exception('_update_home_position', err)
        
    # pokreće odlazak TCP-a na zadane koordinate (trenutno prazno)
    def _goto_coordinates(self) -> None:
        pass

    # obrađuje smjerne tipke/joystick naredbe (trenutno prazno)
    def _joystick_move(self, direction: str) -> None:
        pass

    # centralni handler za akcije tipki u joystick okviru (trenutno prazno)
    def _motor_command(self, action: str) -> None:
        pass

    # pomiče TCP na zadanu XYZ poziciju (trenutno prazno)
    def _move_tcp_to_position(self, target_x: float, target_y: float, target_z: float) -> None:
        pass

    # ažurira prikaz trenutnog TCP položaja u GUI‑ju
    def _update_tcp_position_display(self) -> None:
        try:
            x, y, z = getattr(self, '_last_tcp_pos', (0.0, 0.0, 0.0))
            if hasattr(self, 'tcp_pos_label'):
                pos_text = f'X:{x:.1f} Y:{y:.1f} Z:{z:.1f}'
                self.safe_config(self.tcp_pos_label, text=pos_text)
        except Exception as err:
            self._log_exception('_update_tcp_position_display', err)
if __name__ == '__main__':
    root = tk.Tk()
    app = ArbotiXGUI(root)
    root.mainloop()