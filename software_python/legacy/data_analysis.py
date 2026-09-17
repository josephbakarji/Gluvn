"""
GLUVN Data Analysis Module — rewritten for BLE/USB pipeline (M5StickC Plus 1.1)

I/O layer: ReadWrite (CSV persistence, header-driven parsing)
Diagnostics: Analyze / Analyze2Hands
Legacy: Stats (finger-transition feature extraction — MPU6050-era, path-fixed only)
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from core.__init__ import learnDir, figDir, EXPDIR, mainDir

FLEX_RE = None
PRESS_RE = None
import re
FLEX_RE = re.compile(r'^f\d+$')
PRESS_RE = re.compile(r'^p\d+$')


# ============================================================
# I/O layer
# ============================================================

class ReadWrite:
    def __init__(self, directory, filename):
        self.directory = directory
        self.filename = filename
        self.direct = self.getFullPath()

    def get_filename(self):
        return self.filename

    def getFullPath(self):
        return os.path.join(self.directory, self.filename)

    def saveData(self, sens_dataq, hand='r', key_dataq=None, saveoption='addnew'):
        self.makeDir(saveoption=saveoption)
        self.saveSensorsFromDict(sens_dataq, hand=hand)
        if key_dataq is not None:
            self.saveKeyboard(key_dataq)

    def makeDir(self, saveoption='addnew'):
        if saveoption == 'addnew':
            i = 0
            fm = self.filename
            while os.path.isdir(os.path.join(self.directory, fm)):
                fm = f'{self.filename}_{i}'
                i += 1
            self.filename = fm
            self.direct = self.getFullPath()
            print('Creating directory:', self.direct)
            os.makedirs(self.direct, exist_ok=True)

        elif saveoption == 'manual':
            if os.path.isdir(self.direct):
                prompt = input(f"Overwriting {self.direct}\nPress Enter to proceed, 'n' to abort, "
                                "or type a new filename: ")
                if prompt == 'n':
                    raise SystemExit
                elif prompt:
                    self.filename = prompt
                    self.direct = self.getFullPath()
            os.makedirs(self.direct, exist_ok=True)
        else:
            raise ValueError(f"Invalid saveoption '{saveoption}'")

    # --------------------------------------------------------
    # Write
    # --------------------------------------------------------

    def saveSensorsFromDict(self, dataq, hand, imu_mode='accel'):
        """
        Drains a parser queue and writes one CSV per hand.
        imu_mode: 'accel' or 'gyro' — must match the firmware's use_gyro state
        at capture time (frame layout is mode-agnostic on the wire; label only).
        """
        import queue as queue_mod
        records = []
        while True:
            try:
                records.append(dataq.get(block=False))
            except queue_mod.Empty:
                break

        if not records:
            print(f'No data queued for hand {hand} — nothing saved')
            return

        keys0 = set(records[0].keys())
        included = [k for k in ('time', 'seq', 'imu', 'flex', 'press') if k in keys0]

        imu_labels = (['yaw', 'pitch', 'roll', 'gx', 'gy', 'gz'] if imu_mode == 'gyro'
                      else ['yaw', 'pitch', 'roll', 'ax', 'ay', 'az'])
        label_map = {
            'time': ['time'], 'seq': ['seq'], 'imu': imu_labels,
            'flex': [f'f{i+1}' for i in range(5)],
            'press': [f'p{i+1}' for i in range(5)],
        }
        header = [lbl for k in included for lbl in label_map[k]]

        path = os.path.join(self.direct, f'{self.filename}_sensors_{hand}.csv')
        with open(path, 'w', newline='') as ff:
            ff.write(', '.join(header) + '\n')
            for rec in records:
                row = []
                for k in included:
                    v = rec[k]
                    row += list(v) if isinstance(v, (tuple, list, np.ndarray)) else [v]
                ff.write(', '.join(f'{x:.5f}' if isinstance(x, float) else str(x) for x in row) + '\n')
        print(f'Saved {len(records)} frames -> {path}')

    def saveKeyboard(self, dataq):
        path = os.path.join(self.direct, f'{self.filename}_keyboard.csv')
        with open(path, 'w', newline='') as ff:
            ff.write('time, note, velocity\n')
            while not dataq.empty():
                t, msg = dataq.get(block=True)
                ff.write(f'{t:.6f}, {msg.note}, {msg.velocity}\n')

    # --------------------------------------------------------
    # Read
    # --------------------------------------------------------

    def readSensors(self, hand='', Filter=True, spike_thresh=200):
        """
        Header-driven parse — robust to any sensor_config subset
        (e.g. flex-only, single-hand). Returns (time, press, flex, imu, seq).
        """
        suffix = f'_{hand}' if hand else ''
        path = os.path.join(self.direct, f'{self.filename}_sensors{suffix}.csv')
        with open(path, 'r') as g:
            reader = csv.reader(g)
            rows = list(reader)
        header = [h.strip() for h in rows.pop(0)]
        data = np.array(rows, dtype=float)

        col = {name: i for i, name in enumerate(header)}
        time_idx = col.get('time')
        seq_idx = col.get('seq')
        flex_idx = sorted(i for name, i in col.items() if FLEX_RE.match(name))
        press_idx = sorted(i for name, i in col.items() if PRESS_RE.match(name))
        imu_idx = sorted(i for name, i in col.items()
                          if name not in ('time', 'seq') and i not in flex_idx and i not in press_idx)

        timeSens = data[:, time_idx].tolist() if time_idx is not None else list(range(len(data)))
        seq = data[:, seq_idx].astype(int).tolist() if seq_idx is not None else None
        imuData = data[:, imu_idx].astype(int).tolist() if imu_idx else []
        flexData = data[:, flex_idx].astype(int).tolist() if flex_idx else []
        pressData = data[:, press_idx].astype(int).tolist() if press_idx else []

        if Filter:
            flexData = self.naiveFilter(flexData, spike_thresh)
            pressData = self.naiveFilter(pressData, spike_thresh)

        return timeSens, pressData, flexData, imuData, seq

    def readKeyboard(self):
        path = os.path.join(self.direct, f'{self.filename}_keyboard.csv')
        if not os.path.isfile(path):
            print('No keyboard file')
            return [], [], []
        with open(path, 'r') as g:
            reader = csv.reader(g)
            rows = list(reader)
        rows.pop(0)
        data = np.array(rows, dtype=float)
        return data[:, 0].tolist(), data[:, 1].astype(int).tolist(), data[:, 2].astype(int).tolist()

    def getSaveLocation(self):
        return self.directory, self.filename

    @staticmethod
    def naiveFilter(rows, spike_thresh=200):
        """De-spike: reject single-sample jumps > spike_thresh (sensor glitch, not physical motion)."""
        rows = [list(r) for r in rows]
        for i in range(1, len(rows)):
            for j in range(len(rows[i])):
                if abs(rows[i][j] - rows[i - 1][j]) > spike_thresh:
                    rows[i][j] = rows[i - 1][j]
        return rows


# ============================================================
# Diagnostics — single hand
# ============================================================

class Analyze(ReadWrite):
    def __init__(self, directory=learnDir, filename='test0', hand=''):
        super().__init__(directory, filename)
        self.hand = hand
        self.timeSens, self.pressData, self.flexData, self.imuData, self.seq = [], [], [], [], None
        self.timeKeyboard, self.notes, self.velocity = [], [], []

    def readFullData(self):
        if not os.path.isdir(self.direct):
            raise FileNotFoundError(self.direct)
        self.timeSens, self.pressData, self.flexData, self.imuData, self.seq = self.readSensors(hand=self.hand)
        self.timeKeyboard, self.notes, self.velocity = self.readKeyboard()

    def _cols(self, data):
        """(n_samples, n_channels) list-of-rows -> list of per-channel arrays."""
        return [] if not data else [np.array(c) for c in zip(*data)]

    def getPressData(self): return self._cols(self.pressData)
    def getFlexData(self):  return self._cols(self.flexData)
    def getImuData(self):   return self._cols(self.imuData)

    # --------------------------------------------------------
    # Packet integrity
    # --------------------------------------------------------

    def packet_loss_report(self):
        """
        Detects gaps in the wrapping uint8 seq counter (Delta seq != 1 mod 256).
        Direct integrity check on the BLE/USB frame stream — no seq column
        means the file predates the frame_seq addition to the firmware.
        """
        if self.seq is None:
            print('No seq column in this file — cannot assess packet loss')
            return None
        seq = np.asarray(self.seq)
        d = np.diff(seq.astype(int)) % 256
        lost = np.where(d != 1)[0]
        loss_rate = len(lost) / len(d) if len(d) else 0.0
        print(f'Packet loss: {len(lost)}/{len(d)} gaps ({loss_rate*100:.2f}%)')
        return {'n_gaps': len(lost), 'loss_rate': loss_rate, 'gap_indices': lost}

    # --------------------------------------------------------
    # Export for SINDy / PINN pipelines
    # --------------------------------------------------------

    def to_dataframe(self):
        import pandas as pd
        cols = {'time': self.timeSens}
        if self.seq is not None:
            cols['seq'] = self.seq
        for i, ch in enumerate(self.getFlexData()):
            cols[f'flex{i+1}'] = ch
        for i, ch in enumerate(self.getPressData()):
            cols[f'press{i+1}'] = ch
        for i, ch in enumerate(self.getImuData()):
            cols[f'imu{i+1}'] = ch
        return pd.DataFrame(cols)

    # --------------------------------------------------------
    # Plotting
    # --------------------------------------------------------

    def _multi_plot(self, ax, t, channels, labels, title):
        for ch, lbl in zip(channels, labels):
            ax.plot(t, ch, lw=1.5, label=lbl)
        ax.set_title(title)
        ax.legend(fontsize=9)

    def plotSensors(self, savedir=figDir, savename='', option='default'):
        self.readFullData()
        t = self.timeSens
        flex, press, imu = self.getFlexData(), self.getPressData(), self.getImuData()

        fig, axs = plt.subplots(2, 2, figsize=(11, 7))
        plt.subplots_adjust(bottom=0.2)
        ax1, ax2, ax3, ax4 = axs.flat

        if option == 'default':
            self._multi_plot(ax1, t, press, [f'p{i+1}' for i in range(len(press))], 'Pressure')
            self._multi_plot(ax2, t, flex, [f'f{i+1}' for i in range(len(flex))], 'Flex')
            self._multi_plot(ax3, t, imu[:3] if len(imu) >= 3 else imu,
                              ['yaw', 'pitch', 'roll'][:len(imu)], 'Orientation')
            self._multi_plot(ax4, t, imu[3:6] if len(imu) >= 6 else [],
                              ['x', 'y', 'z'], 'Gyro/Accel (mode-dependent)')

        elif option == 'imu':
            self._multi_plot(ax1, t, imu[:3], ['yaw', 'pitch', 'roll'], 'Orientation')
            self._multi_plot(ax2, t, imu[3:6], ['x', 'y', 'z'], 'Gyro/Accel')
            ax3.axis('off'); ax4.axis('off')

        elif option == 'imumag':
            # ||a|| = sqrt(ax^2 + ay^2 + az^2) — matches accel_mag volume controller
            self._multi_plot(ax1, t, imu[:3], ['yaw', 'pitch', 'roll'], 'Orientation')
            if len(imu) >= 6:
                mag = np.sqrt(imu[3] ** 2 + imu[4] ** 2 + imu[5] ** 2)
                ax2.plot(t, mag, lw=1.5)
                ax2.set_title('||a|| (accel_mag)')
            ax3.axis('off'); ax4.axis('off')
        else:
            raise ValueError(f"Unknown plotting option '{option}'")

        axcolor = 'lightgoldenrodyellow'
        axpos = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor=axcolor)
        spos = Slider(axpos, 'Time', t[0], t[-1])

        def update(val):
            for a in axs.flat:
                a.set_xlim(spos.val, spos.val + 5)
            fig.canvas.draw_idle()
        spos.on_changed(update)

        plt.show()
        out = savename or input("Filename (Enter for 'default'): ") or 'default'
        fig.savefig(os.path.join(savedir, out + '.pdf'))


# ============================================================
# Two-hand wrapper — thin, delegates channel logic to Analyze
# ============================================================

class Analyze2Hands:
    def __init__(self, directory=learnDir, filename='test0'):
        self.R = Analyze(directory, filename, hand='r')
        self.L = Analyze(directory, filename, hand='l')

    def readAllSens(self):
        self.R.readFullData()
        self.L.readFullData()

    def compare_hands(self, sensor='flex'):
        """Overlay right/left channels of the same sensor type for symmetry checks."""
        self.readAllSens()
        fR = self.R.getFlexData() if sensor == 'flex' else self.R.getPressData()
        fL = self.L.getFlexData() if sensor == 'flex' else self.L.getPressData()
        n = min(len(fR), len(fL))
        fig, axes = plt.subplots(1, n, figsize=(3 * n, 3), squeeze=False)
        for i in range(n):
            axes[0, i].plot(self.R.timeSens, fR[i], label='R')
            axes[0, i].plot(self.L.timeSens, fL[i], label='L')
            axes[0, i].set_title(f'{sensor}{i+1}')
            axes[0, i].legend(fontsize=8)
        plt.tight_layout()
        return fig


# ============================================================
# Legacy — finger-transition feature extraction (MPU6050-era)
# Path/casing fixed only; validate frame-layout assumptions before use
# with the new BLE pipeline (imu tuple ordering, hand casing already
# aligned to lowercase 'r'/'l' throughout).
# ============================================================

class Stats:
    def __init__(self, includefile='', directory=learnDir,
                 note_table_path=None):
        self.directory = directory
        self.includefile = includefile
        self.filelist = []
        self.note_table_path = note_table_path or os.path.join(
            mainDir, 'data', 'tables', 'note2num.csv')

    def _load_cnotes_idx(self):
        Cnotes = []
        with open(self.note_table_path) as f:
            for key, val in csv.reader(f):
                if len(key) == 2:
                    Cnotes.append(val)
        return {int(v): i for i, v in enumerate(sorted(Cnotes))}

    def collect_data(self):
        if not os.path.isdir(self.directory):
            raise FileNotFoundError(self.directory)
        self.filelist = self.get_filelist()
        out = []
        for fn in self.filelist:
            a = Analyze(self.directory, fn, hand='r')
            a.readFullData()
            out.append((a.timeSens, a.pressData, a.flexData, a.imuData,
                        a.timeKeyboard, a.notes, a.velocity))
        return out

    def get_filelist(self):
        if not self.includefile:
            return next(os.walk(self.directory))[1]
        with open(self.includefile) as f:
            option = f.readline().strip()
            include_set = {line.strip() for line in f}
        all_files = next(os.walk(self.directory))[1]
        matched = [f for f in all_files if '_'.join(f.split('_')[:-1]) in include_set]
        return matched if option == 'include' else [f for f in all_files if f not in matched]

    def collect_trigger_events(self, idxminus=10, idxplus=3):
        data = self.collect_data()
        trigevent = {'on': [], 'off': [], 'filename': []}
        for runidx, run in enumerate(data):
            timeSens, pressData, flexData, imuData, timeKeyboard, notes, velocity = run
            tnotes = self._get_notes(timeKeyboard, notes, velocity)
            press = np.asarray(list(map(list, zip(*pressData))))
            flex = np.asarray(list(map(list, zip(*flexData))))
            imu = np.asarray(list(map(list, zip(*imuData))))
            senstime = np.asarray(timeSens)

            trigon, trigoff = [], []
            for note in tnotes:
                i0 = int(np.abs(senstime - note[2]).argmin())
                i1 = int(np.abs(senstime - note[3]).argmin())
                sr = np.array(range(i0 - idxminus, i0 + idxplus))
                er = np.array(range(i1 - idxminus, i1 + idxplus))
                press_idx = int(press[:, i0:i1].sum(axis=1).argmax())
                trigon.append({'press': press[:, sr], 'pressidx': press_idx, 'flex': flex[:, sr],
                                'imu': imu[:, sr], 'senstime': senstime[sr], 'velocity': note[1],
                                'note': note[0], 'trigtime': note[2]})
                trigoff.append({'press': press[:, er], 'pressidx': press_idx, 'flex': flex[:, er],
                                 'imu': imu[:, er], 'senstime': senstime[er],
                                 'note': note[0], 'trigtime': note[2]})
            trigevent['on'].append(trigon)
            trigevent['off'].append(trigoff)
            trigevent['filename'].append(self.filelist[runidx])
        return trigevent

    @staticmethod
    def _get_notes(timeKeyboard, notes, velocity):
        timed = []
        for i, note in enumerate(notes):
            if velocity[i] != 0:
                start = timeKeyboard[i]
                for j in range(i + 1, len(notes)):
                    if velocity[j] == 0 and notes[j] == note:
                        timed.append((note, velocity[i], start, timeKeyboard[j]))
                        break
        return timed