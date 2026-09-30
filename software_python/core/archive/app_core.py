"""
GLUVN Sensor-to-Note Processing Module (Modern Version)

Unified sensor processing architecture for converting sensor data into MIDI
note events.

Key Components:
- SensorProcess: thread that processes sensor data and applies trigger logic
- BaseApp: base class for all sensor-to-note applications
- MovingWindow: window-based note mapping (double_flex / ten_finger)
- Harmonizer: chord-based harmonization application

Author: Joseph Bakarji
Created: May 20, 2016
Last Updated: July 2026 by Helene Jabbour
"""

import numpy as np
from core.port_read import Reader
from core.__init__ import BLE_NAME_L, BLE_NAME_R
from core.note_mapper import NoteMapper
from core.midi_writer import MidiWriter
from threading import Thread
from collections import deque

import queue
import time
from itertools import islice, repeat
import sys, asyncio
from bleak import BleakScanner

from base_config import TWO_BYTE, BYTE, ZERO_ACCEL


class SensorProcess(Thread):
    def __init__(self,
                 hand,
                 sensor_q,
                 collect_q,
                 trigger_sensor='flex',
                 trigger_thresh=180,
                 trigger_hysteresis=5,
                 mod_sensors=None,
                 mod_idx=None,
                 send_all_data=False):

        super().__init__()
        self.hand = hand
        self.sensor_q = sensor_q
        self.trigger_sensor = trigger_sensor
        self.mod_sensors = mod_sensors
        self.mod_idx = mod_idx
        self.send_all_data = send_all_data
        self.collect_q = collect_q
        self.trigger_thresh = trigger_thresh
        self.trigger_hysteresis = trigger_hysteresis
        self.turn_state = np.zeros(5, dtype=bool)
        self.trig_on = np.zeros(5, dtype=bool)
        self.trig_off = np.zeros(5, dtype=bool)

    def trigger_logic(self, sensor_values):
        """Hysteresis-based triggering: turn on/off events only fire on the
        rising/falling edge crossing trigger_thresh +/- trigger_hysteresis."""
        trigon_prev = self.trig_on
        trigoff_prev = self.trig_off
        sensarr = np.asarray(sensor_values)
        sdiff = sensarr - self.trigger_thresh
        trigon = sdiff - self.trigger_hysteresis > 0
        trigoff = sdiff + self.trigger_hysteresis < 0
        turnon = np.logical_and(np.logical_and(trigon, np.logical_not(trigon_prev)), np.logical_not(self.turn_state))
        turnoff = np.logical_and(np.logical_and(trigoff, np.logical_not(trigoff_prev)), self.turn_state)
        n_switch = turnon.astype(int) - turnoff.astype(int)   # +1 on, -1 off, 0 no action
        self.turn_state = n_switch + self.turn_state
        self.trig_on, self.trig_off = trigon, trigoff

        return n_switch

    def run(self):
        """Process sensor data and apply trigger logic."""
        send_dict = {'hand': self.hand}
        while True:
            sensor_dict = self.sensor_q.get(block=True)

            if self.send_all_data:
                sensor_dict['hand'] = self.hand
                self.collect_q.put(sensor_dict)

            else:
                n_switch = self.trigger_logic(sensor_dict[self.trigger_sensor])

                mod_sensors_h = self.mod_sensors
                if mod_sensors_h and mod_sensors_h[0] is not None:
                    for idx, sensor in enumerate(mod_sensors_h):
                        mod_sens_data = sensor_dict[sensor]
                        if self.mod_idx is not None and self.mod_idx[idx] is not None:
                            mod_sens_data = mod_sens_data[self.mod_idx[idx]]
                            sensor = sensor + str(self.mod_idx[idx])
                        send_dict[sensor] = mod_sens_data

                if (np.any(n_switch) or (mod_sensors_h is not None and np.any(self.turn_state))):
                    send_dict['switch'] = n_switch
                    send_dict['time'] = time.time()
                    self.collect_q.put(send_dict.copy())


class BaseApp(Thread):
    def __init__(self,
                 root_note='D',
                 scale='minor',
                 sensor_config=None,
                 thresholds=None,
                 trigger_sensors=None,
                 mod_sensors=None,
                 mod_idx=None,
                 send_all_data=False,
                 hands=None,
                 hysteresis=None,
                 use_ble=True):
        super().__init__()
        self.daemon = True

        if isinstance(hysteresis, (int, float)):
            self.hysteresis = {'flex': hysteresis, 'press': hysteresis}
        else:
            self.hysteresis = hysteresis or {'flex': 5, 'press': 5}

        self.thresholds     = thresholds or {'flex': 200, 'press': 15}
        self.trigger_sensors = trigger_sensors or {'r': 'flex'}
        self.mod_sensors    = mod_sensors or {h: [None] for h in (hands or ['r'])}
        self.mod_idx        = mod_idx or {h: [None] for h in (hands or ['r'])}
        self.send_all_data  = send_all_data
        self.use_ble        = use_ble

        self.hands = hands if hands is not None else ['r']

        self.sensor_config  = self._get_sensor_config(sensor_config)
        self.collect_q      = queue.Queue(maxsize=20)

        # Reader (and the hardware connection it opens) is constructed
        # lazily in run() via _init_reader(), not here -- opening a serial
        # port / starting a BLE scan should happen when the app actually
        # starts, not when it's merely instantiated. self.reader is None
        # until then; self.start_error holds a hardware-connection failure
        # so callers can check it after start() without needing an
        # uncaught-exception hook, since a Thread's run() failing raises in
        # the background thread, not to whoever called start().
        self.reader = None
        self.start_error = None
        self.mapper = NoteMapper(root_note=root_note, scale=scale)
        self.midi_writer = MidiWriter()
        self._attitude_rate_state = {}

    def _init_reader(self):
        """Construct self.reader (opens the hardware connection). Call at
        the top of run(), never from __init__. Stores any failure on
        self.start_error and re-raises, so the failure surfaces both to a
        caller polling start_error and to Python's default thread-exception
        reporting rather than dying silently."""
        try:
            self.reader = Reader(sensor_config=self.sensor_config, use_ble=self.use_ble)
        except Exception as e:
            self.start_error = e
            print(f"[Error] Reader failed to start: {e}")
            raise

    def _get_sensor_config(self, sensor_config):
        """Generate sensor configuration based on active hands only."""
        if self.send_all_data:
            return {h: {'flex': True, 'press': True, 'imu': True}
                    for h in self.hands}

        if sensor_config is not None:
            return sensor_config

        config = {h: {'flex': False, 'press': False, 'imu': False}
                  for h in self.hands}

        for hand in self.hands:
            if hand in self.mod_sensors and self.mod_sensors[hand]:
                for sensor in self.mod_sensors[hand]:
                    if sensor and sensor in config[hand]:
                        config[hand][sensor] = True

            if hand in self.trigger_sensors:
                trigger = self.trigger_sensors[hand]
                if trigger and trigger in config[hand]:
                    config[hand][trigger] = True

        return config

    def initialize_triggers(self):
        """Create and start sensor processing threads for each active hand."""
        triggers = {}
        for hand in self.hands:
            if hand not in self.trigger_sensors:
                continue
            trigger_sensor = self.trigger_sensors[hand]
            if not trigger_sensor:
                continue
            if hand not in self.reader.threads:
                print(f"Warning: {hand} hand not in reader threads — skipping trigger init")
                continue
            triggers[hand] = SensorProcess(
                hand,
                self.reader.threads[hand]['parser'].getQ(),
                self.collect_q,
                trigger_sensor=trigger_sensor,
                trigger_thresh=self.thresholds[trigger_sensor],
                trigger_hysteresis=self.hysteresis[trigger_sensor],
                mod_sensors=self.mod_sensors[hand],
                mod_idx=self.mod_idx[hand],
                send_all_data=self.send_all_data
            )
            triggers[hand].start()
        self.triggers = triggers

    def run(self):
        """Main application loop."""
        self._init_reader()
        self.reader.start_readers()
        notemaps = self.mapper.basic_map_2hands()
        self.initialize_triggers()

        while True:
            reading_dict = self.collect_q.get(block=True)
            hand = reading_dict.get('hand')
            if 'switch' in reading_dict and hand in notemaps:
                self.midi_writer.trig_note_array(reading_dict['switch'], notemaps[hand])


class MovingWindow(BaseApp):
    def __init__(self, *args,
                 volume_controller=None,
                 pitch_bender=None,
                 num_lh_fingers=5,
                 num_rh_fingers=5,
                 averaging_window_size=10,
                 base_volume=20,
                 instrument='double_flex',
                 averaging_window_controller=None,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.num_lh_fingers = num_lh_fingers
        self.num_rh_fingers = num_rh_fingers
        self.playing_notes = [None]*num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        self.averaging_window_size_max = averaging_window_size
        self.averaging_window_controller = averaging_window_controller
        self.averaging_queue = deque(repeat(0, self.averaging_window_size_max), maxlen=self.averaging_window_size_max)
        self.base_volume = base_volume
        self.instrument = instrument

        self.pitch_bend_limit = 8192
        self.global_volume = base_volume
        self.averaging_window_size = self.averaging_window_size_max
        self.note_array = None

        if self.instrument == 'double_flex':
            # Left hand fingers select the active note window.
            self.window_trigger, self.note_windows = self.mapper.moving_window(
                num_lhf=num_lh_fingers,
                num_rhf=num_rh_fingers
            )
            self.note_array, note_array_idx = self.mapper.window_map(
                [0]*self.num_lh_fingers,
                self.window_trigger,
                self.note_windows
            )
        elif self.instrument == 'ten_finger':
            self.note_windows = self.mapper.basic_map_2hands()
        else:
            raise ValueError('Instrument not recognized. Please choose from double_flex or ten_finger')

    # Clamped both bounds — MIDI range is hard-constrained [0,127]
    def input_scaling(self, input, min_output=0, max_output=127, shift=0,
                       min_input=0, max_input=BYTE):
        output = int(min_output + (max_output - min_output) *
                     ((input + shift) - min_input) / (max_input - min_input))
        return min(max(output, min_output), max_output)

    def scaling_modulo(self, input, min_output=0, max_output=127,
                        min_input=0, max_input=BYTE):
        return int(((max_output + (max_output - min_output) *
                     (input - min_input) / (max_input - min_input))
                    % (max_output - min_output)) - max_output)

    def pitch_bend(self, reading_dict):
        if self.pitch_bender == 'imu2' and self.pitch_bender in reading_dict:
            return self.scaling_modulo(reading_dict[self.pitch_bender],
                                        min_output=-self.pitch_bend_limit,
                                        max_output=self.pitch_bend_limit,
                                        min_input=0, max_input=TWO_BYTE)
        return 0

    def volume_control(self, reading_dict):
        if self.volume_controller == 'imu1' and self.volume_controller in reading_dict:
            return self.input_scaling(reading_dict['imu1'], max_output=127, min_input=-90, max_input=90)

        if (self.volume_controller == 'accel_mag'
                and all(k in reading_dict for k in ('imu3', 'imu4', 'imu5'))):
            norm = np.sqrt(sum((reading_dict[f'imu{i}'] - TWO_BYTE / 2.0) ** 2
                                for i in (3, 4, 5)))
            volume = max(norm - ZERO_ACCEL, 0)
            self.averaging_queue.append(volume)
            mean = sum(islice(self.averaging_queue,
                            self.averaging_window_size_max - self.averaging_window_size,
                            self.averaging_window_size_max)) / self.averaging_window_size
            return self.base_volume + self.input_scaling(
                mean, max_output=127 - self.base_volume, min_input=0, max_input=15000)

        if (self.volume_controller == 'attitude_rate'
                and all(k in reading_dict for k in ('imu0', 'imu1', 'imu2', 'time'))):
            hand = reading_dict.get('hand')
            yaw, pitch, roll = reading_dict['imu0'], reading_dict['imu1'], reading_dict['imu2']
            now = reading_dict['time']

            prev = self._attitude_rate_state.get(hand)
            self._attitude_rate_state[hand] = (yaw, pitch, roll, now)
            if prev is None:
                return self.base_volume   # no rate available on first sample for this hand

            _, prev_pitch, prev_roll, prev_time = prev
            dt = now - prev_time
            if dt <= 0:
                return self.base_volume   # guard against stale/duplicate timestamps

            def wrapped_delta(a, b):
                d = a - b
                return (d + 180.0) % 360.0 - 180.0   # handles yaw/roll wraparound at +-180deg

            rate = (abs(wrapped_delta(pitch, prev_pitch)) +
                    abs(wrapped_delta(roll,  prev_roll))) / dt   # deg/s

            self.averaging_queue.append(rate)
            mean = sum(islice(self.averaging_queue,
                            self.averaging_window_size_max - self.averaging_window_size,
                            self.averaging_window_size_max)) / self.averaging_window_size
            return self.base_volume + self.input_scaling(
                mean, max_output=127 - self.base_volume, min_input=0, max_input=300)  # 300 deg/s ceiling, tune empirically

        return self.base_volume

    def window_averaging_control(self, reading_dict):
        if self.averaging_window_controller not in reading_dict:
            return self.averaging_window_size_max
        return self.input_scaling(
            reading_dict[self.averaging_window_controller],
            min_output=1,
            max_output=self.averaging_window_size_max,
            min_input=0, max_input=TWO_BYTE)

    def ten_finger_instrument(self, reading_dict):
        """Each finger triggers its own note."""
        hand = reading_dict.get('hand')
        if 'switch' in reading_dict and hand in self.note_windows:
            self.midi_writer.trig_note_array(
                reading_dict['switch'],
                self.note_windows[hand],
                vel=self.global_volume
            )

        if self.averaging_window_controller is not None and hand == 'l':
            new_size = self.window_averaging_control(reading_dict)
            if new_size != self.averaging_window_size:
                self.averaging_window_size = new_size

        if self.volume_controller is not None:
            new_volume = self.volume_control(reading_dict)
            if new_volume != self.global_volume:
                self.global_volume = new_volume
                self.midi_writer.aftertouch(self.global_volume)

        if self.pitch_bender is not None:
            bend_value = self.pitch_bend(reading_dict)
            if bend_value != 0:
                self.midi_writer.pitch_bend(bend_value)

    def double_flex_instrument(self, reading_dict):
        """Left hand selects the note window; right hand plays notes within it."""
        hand = reading_dict.get('hand')

        if hand == 'l' and 'switch' in reading_dict:
            idx_array = []
            for i, sw in enumerate(reading_dict['switch']):
                if sw == 1:   # finger down
                    idx_array.append(i)
                elif sw == -1:   # finger up
                    if i in idx_array:
                        idx_array.remove(i)

            if idx_array:
                self.note_array, note_array_idx = self.mapper.window_map(
                    idx_array,
                    self.window_trigger,
                    self.note_windows
                )

        elif hand == 'r' and 'switch' in reading_dict and self.note_array is not None:
            self.midi_writer.trig_note_array(
                reading_dict['switch'],
                self.note_array,
                vel=self.global_volume
            )

        if self.volume_controller is not None:
            self.global_volume = self.volume_control(reading_dict)
            self.midi_writer.aftertouch(self.global_volume)

        if self.averaging_window_controller is not None and hand == 'l':
            self.averaging_window_size = self.window_averaging_control(reading_dict)

        if self.pitch_bender is not None:
            bend_value = self.pitch_bend(reading_dict)
            if bend_value != 0:
                self.midi_writer.pitch_bend(bend_value)

    def run(self):
        """Main application loop for MovingWindow."""
        self._init_reader()
        self.reader.start_readers()
        self.initialize_triggers()

        while True:
            reading_dict = self.collect_q.get(block=True)

            if self.instrument == 'ten_finger':
                self.ten_finger_instrument(reading_dict)
            elif self.instrument == 'double_flex':
                self.double_flex_instrument(reading_dict)


class Harmonizer(MovingWindow):
    """Chord-based harmonization application. NOT YET IMPLEMENTED -- inherits
    MovingWindow's single-note triggering; no chord-stacking logic exists yet."""
    def __init__(self, *args, chord_type='major', **kwargs):
        raise NotImplementedError(
            "Harmonizer has no chord-triggering logic yet -- it currently "
            "behaves identically to MovingWindow. Implement chord stacking "
            "(see NoteMapper.get_chord/get_chord_from_note) before using this class."
        )


def scan_available_ble(timeout=5.0):
    async def _scan():
        found = set()
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name == BLE_NAME_R: found.add('r')
            elif d.name == BLE_NAME_L: found.add('l')
        return found
    return asyncio.run(_scan())


def launch_app(config, AppClass=MovingWindow, trigger_sensor='flex', mod_idx=None):
    """
    Shared entrypoint for all ten_finger-family presets.
    config: dict of MovingWindow/BaseApp kwargs EXCEPT hands/trigger_sensors/
            mod_sensors/mod_idx/sensor_config/use_ble — those are derived here
            from whichever gloves are actually connected.
    """
    active_hands = scan_available_ble(timeout=5.0)
    if not active_hands:
        print("[Error] No active Gluvn hardware found.")
        sys.exit(1)
    print(f"Discovered gloves: {[h.upper() for h in sorted(active_hands)]}")

    mod_idx = mod_idx or []
    mods = ['imu'] * len(mod_idx)

    app = AppClass(
        **config,
        trigger_sensors={h: trigger_sensor for h in active_hands},
        mod_sensors={h: mods for h in active_hands},
        mod_idx={h: mod_idx for h in active_hands},
        sensor_config={h: {'flex': True, 'press': True, 'imu': True} for h in active_hands},
        hands=list(active_hands),
        use_ble=True,
    )
    app.start()

    input('Press any key to finish\n')
    print('Shutting down...')
    if app.start_error is not None:
        print(f"[Error] App failed to start: {app.start_error}")
    elif app.reader is not None:
        app.reader.stop_readers()
    for hand in app.hands:
        if hand in getattr(app, 'triggers', {}):
            app.triggers[hand].join(timeout=0.5)
    app.join(timeout=0.5)
