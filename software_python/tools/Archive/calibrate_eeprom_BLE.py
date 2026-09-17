#!/usr/bin/env python3
"""
GLUVN Real-Time Calibration Tool — BLE
M5StickC Plus 1.1 — 12-bit ADC
"""

import sys, time, os, threading
import numpy as np
from core.__init__ import BLE_NAME_L, BLE_NAME_R, mainDir
from core.port_read import Reader

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
        self.setWindowTitle("GLUVN Calibration Tool — BLE")
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

        title = QLabel("GLUVN Sensor Calibration — BLE")
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

        self.status_label = QLabel("Scanning for BLE devices...")
        self.status_label.setFont(QFont("Arial", 10))
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self.save_btn = QPushButton("Save Active Hand Calibration to M5")
        self.save_btn.clicked.connect(self._save)
        self.save_btn.setEnabled(False)
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

            fmt    = '>sBHHHHHHHHHHHHHHHH' if mode == 'CALIBRATION' else '>sBHHHHHHBBBBBBBBBB'
            length = 34                      if mode == 'CALIBRATION' else 24

            # Always attempt both hands — Reader handles missing devices gracefully
            self.reader = Reader(
                sensor_config={
                    'r': {'flex': True, 'press': True, 'imu': True},
                    'l': {'flex': True, 'press': True, 'imu': True},
                },
                save=False, message_format=fmt, length_checksum=length,
                use_ble=True
            )
            self.reader.start_readers()

            # Check which hands actually connected (10s timeout each, parallel)
            connected = set()
            def check_hand(h):
                thread = self.reader.threads[h]['serial']
                if thread.wait_connected(timeout=10.0):
                    connected.add(h)
                    # Send init commands
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

            fmt    = '>sBHHHHHHHHHHHHHHHH' if mode == 'CALIBRATION' else '>sBHHHHHHBBBBBBBBBB'
            length = 34                      if mode == 'CALIBRATION' else 24

            self.reader = Reader(
                sensor_config={
                    'r': {'flex': True, 'press': True, 'imu': True},
                    'l': {'flex': True, 'press': True, 'imu': True},
                },
                save=False, message_format=fmt, length_checksum=length,
                use_ble=True
            )
            self.reader.start_readers()

            # Re-verify connections (should reconnect fast — devices still advertising)
            connected = set()
            def check_hand(h):
                if self.reader.threads[h]['serial'].wait_connected(timeout=10.0):
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
        if self.current_mode != 'CALIBRATION':
            self.status_label.setText("Switch to CALIBRATION mode first")
            return
        if self.hand not in self.connected_hands:
            self.status_label.setText(f"{self.hand.upper()} hand not connected")
            return
        self.cal_step   = step
        self.recorded   = []
        self.rec_start  = time.time()
        self.cur_finger = finger if finger is not None else 0
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

        done = bool(cal['flex_min'] and cal['flex_max'] and cal['press_min']
                    and all(x is not None for x in cal['press_max']))
        self.save_btn.setEnabled(done)
        self._enable_cal_buttons(True)
        self.status_label.setText(
            f"✔ {step_label} recorded — ready to save!" if done
            else f"✔ {step_label} recorded"
        )

    def _reset_cal(self):
        self.cal[self.hand] = {
            'flex_min': None, 'flex_max': None,
            'press_min': None, 'press_max': [None]*5
        }
        self.save_btn.setEnabled(False)
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
                        # Buffer both hands
                        self.buf[h]['flex'][self.buf_idx[h]]  = self.current[h]['flex']
                        self.buf[h]['press'][self.buf_idx[h]] = self.current[h]['press']
                        self.buf_idx[h] = (self.buf_idx[h] + 1) % self.max_pts
                        if h == self.hand:
                            updated_active = True
                            # Record for calibration
                            if self.cal_step != 'idle' and self.rec_start:
                                self.recorded.append({
                                    'flex':  self.current[h]['flex'].copy(),
                                    'press': self.current[h]['press'].copy()
                                })
                except: break

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
                    label = (f"{FINGER_NAMES[self.cur_finger]} MAX Press"
                             if self.cal_step == 'press_max' else self.cal_step)
                    self.status_label.setText(
                        f"Recording {label}... {self.rec_duration - elapsed:.1f}s"
                    )

    # --------------------------------------------------------------- Save --

    def _save(self):
        if self.hand not in self.connected_hands:
            self.status_label.setText(f"{self.hand.upper()} hand not connected — cannot save")
            return
        try:
            h   = self.hand.upper()
            cal = self.cal[self.hand]
            cmds = [
                f"SET_CAL:{h}:MIN_FLEX:{','.join(map(str, map(int, cal['flex_min'])))}",
                f"SET_CAL:{h}:MAX_FLEX:{','.join(map(str, map(int, cal['flex_max'])))}",
                f"SET_CAL:{h}:MIN_PRESS:{','.join(map(str, map(int, cal['press_min'])))}",
                f"SET_CAL:{h}:MAX_PRESS:{','.join(map(str, map(int, cal['press_max'])))}",
                "PREFS_SAVE"
            ]
            self._send_to_hand(self.hand, *cmds)
            self._save_header_file()
            self.status_label.setText(
                f"Saved {h} hand calibration to M5 NVS and calibration.h"
            )
        except Exception as e:
            self.status_label.setText(f"Save error: {e}")

    def _save_header_file(self):
        h     = self.hand.upper()
        other = 'L' if h == 'R' else 'R'
        cal   = self.cal[self.hand]

        new_lines = [
            f"int {h}_MIN_FLEX[]  = {{{', '.join(map(str, map(int, cal['flex_min'])))}}};\n",
            f"int {h}_MAX_FLEX[]  = {{{', '.join(map(str, map(int, cal['flex_max'])))}}};\n",
            f"int {h}_MIN_PRESS[] = {{{', '.join(map(str, map(int, cal['press_min'])))}}};\n",
            f"int {h}_MAX_PRESS[] = {{{', '.join(map(str, map(int, cal['press_max'])))}}};\n",
        ]

        # 1. Get the absolute path to the 'gluvn_python' folder
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 2. Step up one level to the 'GLUVN-M5' root folder
        project_root = os.path.dirname(script_dir) 

        # 3. Construct the exact target path based on your image
        target_dir = os.path.join(project_root, "arduino", "sendsens")
        path = os.path.join(target_dir, "calibration.h")

        # Create the directory if it somehow gets deleted
        os.makedirs(target_dir, exist_ok=True)

        other_lines = []
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    other_lines = [l for l in f if l.startswith(f'int {other}_')]
            except Exception as e:
                print(f"Error reading existing file: {e}")

        # Write the combined calibration data
        try:
            with open(path, 'w') as f:
                f.write("#ifndef CALIBRATION_H\n#define CALIBRATION_H\n\n")
                f.writelines(other_lines)
                f.write("\n")
                f.writelines(new_lines)
                f.write("\n#endif\n")
            print(f"Successfully wrote calibration to: {path}")
        except Exception as e:
            print(f"Failed to write to {path}: {e}")

    def closeEvent(self, event):
        if self.reader: self.reader.stop_readers()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CalibrationTool()
    window.show()
    sys.exit(app.exec_())