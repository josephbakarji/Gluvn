"""
GLUVN Real-Time Calibration Tool — BLE
M5StickC Plus 1.1 — 12-bit ADC
"""

import sys
import time
import os
import queue
import threading
import numpy as np
from core.port_read import Reader
from core.__init__ import BLE_NAME_R, BLE_NAME_L

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QGroupBox,
                             QComboBox, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

FINGER_COLORS = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12', '#9b59b6']
FINGER_NAMES  = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
BLE_NAMES     = {'r': BLE_NAME_R, 'l': BLE_NAME_L}


class CalibrationTool(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GLUVN Calibration Tool")
        self.setGeometry(100, 100, 1400, 800)

        # Active hand being calibrated (display/record focus)
        self.hand         = 'r'
        self.cal_step     = 'idle'
        self.recorded     = []
        self.rec_start    = None
        self.rec_duration = 3.0
        self.cur_finger   = 0
        self.current_mode = 'CALIBRATION'

        # Single Reader instance managing ALL connected gloves
        self.reader        = None
        self._pending_mode = None  # tuple set by bg thread, picked up by _update

        # Per-hand calibration state
        self.cal = {
            'r': {'flex_min': None, 'flex_max': None,
                'press_min': None, 'press_max': [None]*5},
            'l': {'flex_min': None, 'flex_max': None,
                'press_min': None, 'press_max': [None]*5},
        }

        self.current = {
            'r': {'flex': [0]*5, 'press': [0]*5},
            'l': {'flex': [0]*5, 'press': [0]*5},
        }

        self.max_pts = 500
        self.buf = {
            h: {
                'flex':  np.zeros((self.max_pts, 5), dtype=np.float32),
                'press': np.zeros((self.max_pts, 5), dtype=np.float32),
            } for h in ('r', 'l')
        }
        self.buf_idx = {'r': 0, 'l': 0}

        # Which hands are actually BLE-connected (updated by bg thread)
        self.connected_hands = set()

        self._setup_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(50)

        # Try to connect to both gloves at startup
        t = threading.Thread(target=self._connect_all_bg,
                             args=(self.current_mode,), daemon=True)
        t.start()

    # ------------------------------------------------------------------ UI --

    def _setup_ui(self):
        main = QWidget()
        self.setCentralWidget(main)
        splitter = QSplitter(Qt.Horizontal)
        QHBoxLayout(main).addWidget(splitter)
        self._setup_controls(splitter)
        self._setup_plots(splitter)
        splitter.setSizes([400, 1000])

    def _setup_controls(self, parent):
        w = QWidget(); parent.addWidget(w)
        layout = QVBoxLayout(w)

        title = QLabel("GLUVN Sensor Calibration Tool")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)

        # Connection status
        cg = QGroupBox("Connection Status"); cl = QVBoxLayout(cg)
        self.conn_r_label = QLabel("Right (Gluvn_R): Scanning...")
        self.conn_l_label = QLabel("Left  (Gluvn_L): Scanning...")
        self.conn_r_label.setStyleSheet("color:#e74c3c;font-weight:bold;")
        self.conn_l_label.setStyleSheet("color:#e74c3c;font-weight:bold;")
        cl.addWidget(self.conn_r_label)
        cl.addWidget(self.conn_l_label)
        self.reconnect_btn = QPushButton("Reconnect All")
        self.reconnect_btn.clicked.connect(self._reconnect_all)
        cl.addWidget(self.reconnect_btn)
        layout.addWidget(cg)

        # Hand selection (which hand to calibrate)
        hg = QGroupBox("Active Hand for Calibration"); hl = QVBoxLayout(hg)
        hand_row = QHBoxLayout()
        self.hand_combo = QComboBox()
        self.hand_combo.addItems(['Right Hand', 'Left Hand'])
        self.hand_combo.currentTextChanged.connect(self._change_hand)
        hand_row.addWidget(QLabel("Calibrate:")); hand_row.addWidget(self.hand_combo)
        hl.addLayout(hand_row)
        self.hand_label = QLabel("Active: R Hand")
        self.hand_label.setFont(QFont("Arial", 12, QFont.Bold))
        hl.addWidget(self.hand_label)
        layout.addWidget(hg)

        # Mode toggle
        mg = QGroupBox("Mode"); ml = QVBoxLayout(mg)
        self.mode_btn   = QPushButton("Switch to NORMAL Mode")
        self.mode_label = QLabel("Mode: CALIBRATION (Raw 12-bit)")
        self.mode_btn.setStyleSheet(
            "background-color:#e74c3c;color:white;font-weight:bold;padding:6px;")
        self.mode_btn.clicked.connect(self._toggle_mode)
        ml.addWidget(self.mode_btn); ml.addWidget(self.mode_label)
        layout.addWidget(mg)

        # Live values — color-coded per finger
        vg = QGroupBox("Live Sensor Values (Active Hand)"); vl = QVBoxLayout(vg)
        self.flex_labels, self.press_labels = [], []
        flex_row, press_row = QHBoxLayout(), QHBoxLayout()
        for i in range(5):
            for labels, row in ((self.flex_labels, flex_row),
                                (self.press_labels, press_row)):
                lbl = QLabel(f"{FINGER_NAMES[i]}\n0")
                lbl.setStyleSheet(
                    f"color:{FINGER_COLORS[i]};font-weight:bold;")
                lbl.setAlignment(Qt.AlignCenter)
                labels.append(lbl); row.addWidget(lbl)
        vl.addWidget(QLabel("Flex:"));  vl.addLayout(flex_row)
        vl.addWidget(QLabel("Press:")); vl.addLayout(press_row)
        layout.addWidget(vg)

        # Calibration steps
        sg = QGroupBox("Calibration Steps (active hand, CALIBRATION mode)")
        sl = QVBoxLayout(sg)
        self.step_btns = []
        for label, step in [
            ("Step 1: Flex MIN — fingers straight", 'flex_min'),
            ("Step 2: Flex MAX — fingers bent",     'flex_max'),
            ("Step 3: Pressure MIN — no pressure",  'press_min'),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, s=step: self._start_rec(s))
            self.step_btns.append(btn); sl.addWidget(btn)

        sl.addWidget(QLabel("Press each finger to its MAX:"))
        self.press_btns = []
        for i in range(5):
            btn = QPushButton(f"  {FINGER_NAMES[i]} MAX Pressure")
            btn.setStyleSheet(f"color:{FINGER_COLORS[i]};font-weight:bold;")
            btn.clicked.connect(lambda _, f=i: self._start_rec('press_max', f))
            self.press_btns.append(btn); sl.addWidget(btn)
        layout.addWidget(sg)

        self.status_label = QLabel("Scanning for devices ...")
        self.status_label.setFont(QFont("Arial", 10))
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self.save_btn = QPushButton("Save Active Hand Calibration to M5")
        self.save_btn.clicked.connect(self._save)
        self.save_btn.setEnabled(True)
        self.save_btn.setStyleSheet(
            "background-color:#2ecc71;color:white;font-weight:bold;padding:6px;")
        layout.addWidget(self.save_btn)

        self.reset_btn = QPushButton("Reset Active Hand Calibration")
        self.reset_btn.clicked.connect(self._reset_cal)
        self.reset_btn.setStyleSheet(
            "background-color:#f39c12;color:white;font-weight:bold;padding:6px;")
        layout.addWidget(self.reset_btn)

        layout.addStretch()

    def _setup_plots(self, parent):
        w = QWidget(); parent.addWidget(w)
        layout = QVBoxLayout(w)

        self.flex_plot = pg.PlotWidget(title='Flex Sensors — CH5-9 (active hand)')
        self.flex_plot.showGrid(x=True, y=True)
        self.flex_plot.setLabel('left', 'Value')
        self.flex_plot.setLabel('bottom', 'Sample')
        self.flex_plot.setYRange(0, 4095)
        self.flex_plot.addLegend(offset=(10, 10))
        self.flex_curves = [
            self.flex_plot.plot(pen=pg.mkPen(color=FINGER_COLORS[i], width=2),
                                name=FINGER_NAMES[i])
            for i in range(5)
        ]
        layout.addWidget(self.flex_plot)

        self.press_plot = pg.PlotWidget(title='FSR Pressure Sensors — CH0-4 (active hand)')
        self.press_plot.showGrid(x=True, y=True)
        self.press_plot.setLabel('left', 'Value')
        self.press_plot.setLabel('bottom', 'Sample')
        self.press_plot.setYRange(0, 4095)
        self.press_plot.addLegend(offset=(10, 10))
        self.press_curves = [
            self.press_plot.plot(pen=pg.mkPen(color=FINGER_COLORS[i], width=2),
                                 name=FINGER_NAMES[i])
            for i in range(5)
        ]
        layout.addWidget(self.press_plot)

    # ----------------------------------------------------------- BLE setup --

    def _connect_all_bg(self, mode):
        """
        Background thread: connect to both gloves if available.
        Uses a single Reader with both hands in sensor_config.
        Falls back gracefully if only one glove is found.
        """
        try:
            if self.reader:
                self.reader.stop_readers()
                self.reader = None
                time.sleep(1.0)

            # Always attempt both hands; Reader handles missing devices gracefully.
            self.reader = Reader(
                sensor_config={
                    'r': {'flex': True, 'press': True, 'imu': True},
                    'l': {'flex': True, 'press': True, 'imu': True},
                },
                use_ble=True
            )
            self.reader.start_readers()

            # Check which hands actually connected (10s timeout each, parallel)
            connected = set()
            def check_hand(h):
                is_ble = self.reader.threads[h].get('is_ble', False)
                if is_ble:
                    thread = self.reader.threads[h]['serial']
                    if thread.wait_connected(timeout=10.0):
                        connected.add(h)
                        self.reader.send_command(h, f'HAND_{h.upper()}')
                        time.sleep(0.05)
                        cmd = 'CAL_START' if mode == 'CALIBRATION' else 'CAL_STOP'
                        self.reader.send_command(h, cmd)
                else:
                    # USB: Reader.start_readers() already sent the handshake;
                    # thread is running, so treat presence in self.reader.threads as connected.
                    connected.add(h)
                    self.reader.send_command(h, f'HAND_{h.upper()}')
                    time.sleep(0.05)
                    cmd = 'CAL_START' if mode == 'CALIBRATION' else 'CAL_STOP'
                    self.reader.send_command(h, cmd)

            threads = [threading.Thread(target=check_hand, args=(h,), daemon=True)
                    for h in ('r', 'l')]
            for t in threads: t.start()
            for t in threads: t.join()

            self.connected_hands = connected
            self.current_mode    = mode
            self._pending_mode   = ('ok', mode)

        except Exception as e:
            err = str(e)
            self._pending_mode = ('err', err)

    def _send_to_hand(self, hand, *commands):
        """Send commands to a specific hand if connected."""
        if hand not in self.connected_hands or not self.reader:
            return
        for cmd in commands:
            try:
                self.reader.send_command(hand, cmd)
                time.sleep(0.05)
            except Exception as e:
                print(f"Send error ({hand}): {e}")

    @staticmethod
    def _parse_numeric_list(line):
        """Parse 'int NAME[] = {1, 2, , 4};' or 'float NAME[] = {1.0f, 2.0f};'
        into [1, 2, None, 4]. Blank slots (from a prior blank-write bug, or a
        field never calibrated) become None. Returns None if unparseable."""
        try:
            inside = line.split('{', 1)[1].split('}', 1)[0]
            out = []
            for tok in inside.split(','):
                tok = tok.strip().rstrip('fF')
                out.append(None if tok == '' else float(tok))
            return out
        except Exception:
            return None

    def _reconnect_all(self):
        self._lock_ui(True)
        self.status_label.setText("Reconnecting...")
        t = threading.Thread(target=self._connect_all_bg,
                             args=(self.current_mode,), daemon=True)
        t.start()

    # ----------------------------------------------------------- Handlers --

    def _change_hand(self, hand_text):
        """Switch active calibration hand — no BLE reconnect needed."""
        self.hand = 'r' if hand_text == 'Right Hand' else 'l'
        self.hand_label.setText(f"Active: {self.hand.upper()} Hand")
        is_connected = self.hand in self.connected_hands
        status = (f"Switched to {self.hand.upper()} hand"
                  if is_connected
                  else f"{self.hand.upper()} hand not connected — check BLE")
        self.status_label.setText(status)
        self._enable_cal_buttons(
            self.current_mode == 'CALIBRATION' and is_connected
        )

    def _toggle_mode(self):
        new_mode = 'NORMAL' if self.current_mode == 'CALIBRATION' else 'CALIBRATION'
        self._lock_ui(True)
        self.status_label.setText("Switching mode...")
        t = threading.Thread(target=self._switch_mode_bg,
                             args=(new_mode,), daemon=True)
        t.start()

    def _switch_mode_bg(self, mode):
        """
        Mode switch — reuses existing BLE connections, no rescan.
        Sends CAL_START/CAL_STOP to all connected hands then restarts
        Reader with new packet format.
        """
        try:
            cmd = 'CAL_START' if mode == 'CALIBRATION' else 'CAL_STOP'

            # Send to all connected hands before stopping reader
            for h in self.connected_hands:
                try:
                    self.reader.send_command(h, cmd)
                except Exception:
                    pass
            time.sleep(0.3)

            self.reader.stop_readers()
            self.reader = None
            time.sleep(0.8)

            self.reader = Reader(
                sensor_config={
                    'r': {'flex': True, 'press': True, 'imu': True},
                    'l': {'flex': True, 'press': True, 'imu': True},
                },
                use_ble=True
            )
            self.reader.start_readers()

            # Re-verify connections (should reconnect fast — devices still advertising)
            connected = set()
            def check_hand(h):
                is_ble = self.reader.threads[h].get('is_ble', False)
                if is_ble:
                    if self.reader.threads[h]['serial'].wait_connected(timeout=10.0):
                        connected.add(h)
                else:
                    connected.add(h)
            threads = [threading.Thread(target=check_hand, args=(h,), daemon=True)
                       for h in ('r', 'l')]
            for t in threads: t.start()
            for t in threads: t.join()

            self.connected_hands = connected
            self.current_mode    = mode
            self._pending_mode   = ('ok', mode)

        except Exception as e:
            err = str(e)
            self._pending_mode = ('err', err)

    def _finish_mode_switch(self, mode):
        """UI updates after background setup — runs on main thread via _update."""
        is_cal = (mode == 'CALIBRATION')
        y_max  = 4095 if is_cal else 255

        self.flex_plot.setYRange(0, y_max)
        self.press_plot.setYRange(0, y_max)
        self.flex_plot.setTitle(
            "Flex Sensors CH5-9 — RAW 12-bit" if is_cal
            else "Flex Sensors CH5-9 — Calibrated (0-255)"
        )
        self.press_plot.setTitle(
            "FSR Pressure Sensors CH0-4 — RAW 12-bit" if is_cal
            else "FSR Pressure Sensors CH0-4 — Calibrated (0-255)"
        )
        self.mode_label.setText(
            "Mode: CALIBRATION (Raw 12-bit)" if is_cal
            else "Mode: NORMAL (Calibrated 8-bit)"
        )
        self.mode_btn.setText(
            "Switch to NORMAL Mode" if is_cal else "Switch to CALIBRATION Mode"
        )
        self.mode_btn.setStyleSheet(
            "background-color:#e74c3c;color:white;font-weight:bold;padding:6px;" if is_cal
            else "background-color:#3498db;color:white;font-weight:bold;padding:6px;"
        )

        # Update connection status labels
        for h, lbl in (('r', self.conn_r_label), ('l', self.conn_l_label)):
            name = BLE_NAMES[h]
            side = 'Right' if h == 'r' else 'Left'
            if h in self.connected_hands:
                lbl.setText(f"{side} ({name}): Connected ✓")
                lbl.setStyleSheet("color:#2ecc71;font-weight:bold;")
            else:
                lbl.setText(f"{side} ({name}): Not found")
                lbl.setStyleSheet("color:#e74c3c;font-weight:bold;")

        is_active_connected = self.hand in self.connected_hands
        self._enable_cal_buttons(is_cal and is_active_connected)
        self._lock_ui(False)

        found = ', '.join(h.upper() for h in sorted(self.connected_hands)) or 'none'
        self.status_label.setText(
            f"Connected: {found} | Active: {self.hand.upper()} | "
            f"{'CALIBRATION' if is_cal else 'NORMAL'} mode"
        )

    def _lock_ui(self, locked):
        self.hand_combo.setEnabled(not locked)
        self.mode_btn.setEnabled(not locked)
        self.reconnect_btn.setEnabled(not locked)

    def _enable_cal_buttons(self, enabled):
        for btn in self.step_btns + self.press_btns:
            btn.setEnabled(enabled)

    # --------------------------------------------------------- Recording --

    def _start_rec(self, step, finger=None):
        self.cal_step    = step
        self.recorded    = []
        self.cur_finger  = finger if finger is not None else 0

        cal = self.cal[self.hand]

        self.rec_start    = time.time()
        self.rec_duration = 3.0

        label = (f"{FINGER_NAMES[finger]} MAX Press"
                if step == 'press_max' else step)
        self.status_label.setText(f"Recording {label}...")
        self._enable_cal_buttons(False)

    def _finish_rec(self):
        flex_avg  = np.mean([d['flex']  for d in self.recorded], axis=0)
        press_avg = np.mean([d['press'] for d in self.recorded], axis=0)
        cal       = self.cal[self.hand]

        if   self.cal_step == 'flex_min':  cal['flex_min']  = flex_avg.tolist()
        elif self.cal_step == 'flex_max':  cal['flex_max']  = flex_avg.tolist()
        elif self.cal_step == 'press_min': cal['press_min'] = press_avg.tolist()
        elif self.cal_step == 'press_max':
            cal['press_max'][self.cur_finger] = int(press_avg[self.cur_finger])

        step_label = (f"{FINGER_NAMES[self.cur_finger]} MAX Press"
                      if self.cal_step == 'press_max' else self.cal_step)
        self.cal_step  = 'idle'
        self.rec_start = None

        self._enable_cal_buttons(True)
        self.status_label.setText(
            f"✔ {step_label} recorded"
        )

    def _reset_cal(self):
        self.cal[self.hand] = {
            'flex_min': None, 'flex_max': None,
            'press_min': None, 'press_max': [None]*5,
        }
        self.status_label.setText(
            f"{self.hand.upper()} hand calibration cleared — redo all steps"
        )

    # -------------------------------------------------------------- Loop --

    def _update(self):
        # Pick up result from background thread
        if self._pending_mode is not None:
            kind, val = self._pending_mode
            self._pending_mode = None
            if kind == 'ok':
                self._finish_mode_switch(val)
            else:
                self.status_label.setText(f"BLE error: {val}")
                self._lock_ui(False)

        if not self.reader: return

        # Read data for ALL connected hands — buffer both
        updated_active = False
        for h in self.connected_hands:
            if h not in self.reader.threads: continue
            while True:
                try:
                    data = self.reader.threads[h]['parser'].getQ().get(block=False)
                    if data:
                        if 'flex'  in data: self.current[h]['flex']  = list(data['flex'])
                        if 'press' in data: self.current[h]['press'] = list(data['press'])
                        self.buf[h]['flex'][self.buf_idx[h]]  = self.current[h]['flex']
                        self.buf[h]['press'][self.buf_idx[h]] = self.current[h]['press']
                        self.buf_idx[h] = (self.buf_idx[h] + 1) % self.max_pts
                        if h == self.hand:
                            updated_active = True
                            if self.cal_step != 'idle' and self.rec_start:
                                self.recorded.append({
                                    'flex':  self.current[h]['flex'].copy(),
                                    'press': self.current[h]['press'].copy()
                                })
                except queue.Empty:
                    break

        # Update plots and labels only for active hand
        if updated_active:
            for i in range(5):
                self.flex_labels[i].setText(
                    f"{FINGER_NAMES[i]}\n{self.current[self.hand]['flex'][i]}")
                self.press_labels[i].setText(
                    f"{FINGER_NAMES[i]}\n{self.current[self.hand]['press'][i]}")

            x = np.arange(self.max_pts)
            for i in range(5):
                self.flex_curves[i].setData(
                    x, np.roll(self.buf[self.hand]['flex'][:,i], -self.buf_idx[self.hand]))
                self.press_curves[i].setData(
                    x, np.roll(self.buf[self.hand]['press'][:,i], -self.buf_idx[self.hand]))

            if self.cal_step != 'idle' and self.rec_start:
                elapsed = time.time() - self.rec_start
                if elapsed >= self.rec_duration:
                    self._finish_rec()
                else:
                    if self.cal_step == 'press_max':
                        label = f"{FINGER_NAMES[self.cur_finger]} MAX Press"
                    else:
                        label = self.cal_step

                    text = f"Recording {label}... {self.rec_duration - elapsed:.1f}s"
                    self.status_label.setText(text)

    # --------------------------------------------------------------- Save --

    def _save(self):
        if self.hand not in self.connected_hands:
            self.status_label.setText(f"{self.hand.upper()} hand not connected — cannot save")
            return
        try:
            h = self.hand.upper()
            cal = self.cal[self.hand]
            cmds = []
            if cal['flex_min'] is not None:
                cmds.append(
                    f"SET_CAL:{h}:MIN_FLEX:{','.join(map(str, map(int, cal['flex_min'])))}"
                )
            if cal['flex_max'] is not None:
                cmds.append(
                    f"SET_CAL:{h}:MAX_FLEX:{','.join(map(str, map(int, cal['flex_max'])))}"
                )
            if cal['press_min'] is not None:
                cmds.append(
                    f"SET_CAL:{h}:MIN_PRESS:{','.join(map(str, map(int, cal['press_min'])))}"
                )
            if any(x is not None for x in cal['press_max']):
                # Merge against calibration.h so a partial-finger recalibration
                # doesn't send blanks for untouched fingers — Arduino's
                # String::toInt() on "" returns 0, which would silently zero
                # those fingers' MAX_PRESS in firmware RAM/NVS once PREFS_SAVE
                # runs, with no parse error to catch it.
                _, _, existing_parsed = self._read_calibration_header()
                prior = existing_parsed.get(f"{h}_MAX_PRESS", [None] * 5)
                merged = [
                    cal['press_max'][i] if cal['press_max'][i] is not None
                    else (prior[i] if i < len(prior) else None)
                    for i in range(5)
                ]
                vals = ["" if v is None else str(int(v)) for v in merged]
                cmds.append(f"SET_CAL:{h}:MAX_PRESS:{','.join(vals)}")
            if not cmds: # nothing to save
                self.status_label.setText("Nothing new to save.")
                return
            cmds.append("PREFS_SAVE")
            self._send_to_hand(self.hand, *cmds)
            self._save_header_file()
            self.status_label.setText(f"Saved updated calibration values for {h} hand.")
        except Exception as e:
            self.status_label.setText(f"Save error: {e}")

    def _calibration_header_path(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
        target_dir = os.path.join(project_root, "firmware", "m5stick_firmware")
        return os.path.join(target_dir, "calibration.h")

    def _read_calibration_header(self):
        """Parse calibration.h -> (path, {varname: raw_line}, {varname: [parsed values]}).
        Shared by _save() (device-side merge) and _save_header_file() (text merge)
        so both agree on what's currently on disk."""
        path = self._calibration_header_path()
        existing, existing_parsed = {}, {}
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    for line in f:
                        stripped = line.strip()
                        if stripped.startswith(('int ', 'float ')):
                            varname = stripped.split()[1].split('[')[0]
                            existing[varname] = line
                            parsed = self._parse_numeric_list(line)
                            if parsed is not None:
                                existing_parsed[varname] = parsed
            except Exception as e:
                print(f"Error parsing existing header at {path}: {e}")
        return path, existing, existing_parsed

    def _save_header_file(self):
        h = self.hand.upper()
        other = 'L' if h == 'R' else 'R'
        cal = self.cal[self.hand]

        target_dir = os.path.join(os.path.dirname(self._calibration_header_path()))
        os.makedirs(target_dir, exist_ok=True)
        path, existing, existing_parsed = self._read_calibration_header()

        # MAX_PRESS is handled separately because it is merged per finger
        # with the existing header values.
        field_map = {
            f"{h}_MIN_FLEX":  ('flex_min',  lambda v: f"int {h}_MIN_FLEX[]  = {{{', '.join(map(str, map(int, v)))}}};\n"),
            f"{h}_MAX_FLEX":  ('flex_max',  lambda v: f"int {h}_MAX_FLEX[]  = {{{', '.join(map(str, map(int, v)))}}};\n"),
            f"{h}_MIN_PRESS": ('press_min', lambda v: f"int {h}_MIN_PRESS[] = {{{', '.join(map(str, map(int, v)))}}};\n"),
        }

        new_lines = []
        for varname, (key, fmt) in field_map.items():
            val = cal.get(key)
            if val is not None:
                new_lines.append(fmt(val))          # updated this session
            elif varname in existing:
                new_lines.append(existing[varname])  # untouched — preserve
            # else: never calibrated, no prior entry -> omit

        # MAX_PRESS: only touch it at all if at least one finger was
        # recorded this session; when it was, merge finger-by-finger against
        # calibration.h so fingers NOT touched this session keep their prior
        # value instead of being blanked to "".
        press_max_var = f"{h}_MAX_PRESS"
        if any(x is not None for x in cal['press_max']):
            prior = existing_parsed.get(press_max_var, [None] * 5)
            merged = [
                cal['press_max'][i] if cal['press_max'][i] is not None
                else (prior[i] if i < len(prior) else None)
                for i in range(5)
            ]
            vals = ["" if x is None else str(int(x)) for x in merged]
            new_lines.append(f"int {press_max_var}[] = {{{', '.join(vals)}}};\n")
        elif press_max_var in existing:
            new_lines.append(existing[press_max_var])  # untouched — preserve

        other_lines = [line for name, line in existing.items() if name.startswith(f'{other}_')]

        try:
            with open(path, 'w') as f:
                f.write("#ifndef CALIBRATION_H\n#define CALIBRATION_H\n\n")
                f.writelines(other_lines)
                f.write("\n")
                f.writelines(new_lines)
                f.write("\n#endif\n")
            print(f"Header successfully patched: {path}")
        except Exception as e:
            print(f"Header write failed for {path}: {e}")

    def closeEvent(self, event):
        if self.reader: self.reader.stop_readers()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CalibrationTool()
    window.show()
    sys.exit(app.exec_())