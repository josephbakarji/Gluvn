"""
core/music_app.py -- the base class for every glove music app.

WHAT THIS FILE IS FOR
---------------------
Everything that is the same no matter what the instrument sounds like lives in
core: connecting the gloves, turning raw sensor frames into calibrated motion
(core.pose_provider), the MIDI port, the scale/note tables, the effect and
visualisation queues the digital twin reads, and the run loop. A music app
subclasses MusicApp and writes ONLY the musical decisions:

    class MyInstrument(MusicApp):
        def on_start(self):                       # once, after calibration
            ...
        def on_frame(self, frame, motion):        # every valid frame: continuous
            ...                                   #   controls (e.g. roll_depth -> CC)
        def on_event(self, event, motion):        # every edge: discrete triggers
            ...                                   #   (burst, pitch_step, press/flex)

``motion`` is a core.pose_provider.MotionFrame and ``event`` a MotionEvent (read
their docstrings for every field). An app should NOT compute angles, thresholds,
latches or calibration, and must NOT read a parser queue: doing so bypasses the
single source of truth and steals frames from the provider. If a new instrument
needs a signal the core does not deliver yet, add it to pose_provider.py's
HandMotion so every app and the twin get it, instead of computing it locally.

LIFECYCLE (owned here, in this order -- see PoseProvider.bring_up)
    __init__   Reader + PoseProvider + MIDI + mapper + queues; nothing streams yet
    start()    run(): bring_up -> calibrate -> on_start -> loop -> on_stop
    shutdown() stop the provider/reader and join the thread

ATTRIBUTES THE DIGITAL TWIN READS (keep them if you override __init__)
    reader, hands, viz_q, effect_q, live_controls
"""

import asyncio
import queue
import time
from dataclasses import replace
from threading import Event, Thread

from bleak import BleakScanner

from core.__init__ import BLE_NAME_L, BLE_NAME_R
from core.midi_writer import MidiWriter
from core.note_mapper import NoteMapper
from core.port_read import Reader
from core.pose_provider import LiveControls, MotionConfig, PoseProvider


NOTE_NAMES_SHARP = ('C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B')
NOTE_NAMES_FLAT  = ('C', 'Db', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B')


def midi_note_name(midi_note, use_flats=False):
    """MIDI note number -> name string, e.g. 60 -> 'C4', 61 -> 'C#4' (or
    'Db4' with use_flats=True). Standard MIDI octave numbering: note 60 is
    C4 (middle C), i.e. octave = midi_note // 12 - 1. Returns 'N/A' for
    None (idle finger / no note assigned) rather than raising, since this is
    called directly from live per-finger status construction where an idle
    slot is the common case, not an error."""
    if midi_note is None:
        return 'N/A'
    names = NOTE_NAMES_FLAT if use_flats else NOTE_NAMES_SHARP
    pitch_class = names[midi_note % 12]
    octave = midi_note // 12 - 1
    return f'{pitch_class}{octave}'


def scan_available_ble(timeout=5.0):
    async def _scan():
        found = set()
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name == BLE_NAME_R: found.add('r')
            elif d.name == BLE_NAME_L: found.add('l')
        return found
    return asyncio.run(_scan())


class MusicApp(Thread):
    """Base class for glove music apps. See the module docstring."""

    MAIN_LOOP_POLL_SEC = 0.1

    def __init__(self, root_note='D', scale='minor',
                 sensor_config=None, thresholds=None, trigger_sensors=None,
                 mod_sensors=None, mod_idx=None, send_all_data=False,
                 hands=None, hysteresis=None, use_ble=True,
                 enable_motion_stream=True, *,
                 motion_config=None, live_controls=None, diagnostics=False,
                 midi_writer=None):
        super().__init__()
        self.daemon = True
        self.hands = list(hands) if hands is not None else ['r']
        self.use_ble = use_ble
        self.enable_motion_stream = enable_motion_stream
        self.send_all_data = send_all_data
        # mod_sensors / mod_idx belonged to the pre-pose-provider raw-sensor
        # pitch-bend/volume routing that nothing calls any more. Accepted so
        # existing launch code keeps working; ignored.
        self.mod_sensors, self.mod_idx = mod_sensors, mod_idx

        # Finger-latch parameters live in MotionConfig (the single default);
        # whatever the caller passes overrides it.
        cfg = motion_config if motion_config is not None else MotionConfig()
        overrides = {}
        if thresholds is not None:
            overrides['finger_thresholds'] = dict(thresholds)
        if hysteresis is not None:
            overrides['finger_hysteresis'] = dict(hysteresis)
        if trigger_sensors is not None:
            overrides['trigger_sensors'] = {h: list(v) for h, v in trigger_sensors.items()}
        self.motion_config = replace(cfg, **overrides) if overrides else cfg
        self.thresholds = self.motion_config.finger_thresholds
        self.hysteresis = self.motion_config.finger_hysteresis
        self.trigger_sensors = {h: self.motion_config.sensors_for(h) for h in self.hands}

        self.sensor_config = self._get_sensor_config(sensor_config)
        # Orientation IS the imu bytes (Mahony's Euler output), so every hand
        # needs them whatever the caller's sensor_config says.
        for _h in self.hands:
            self.sensor_config.setdefault(_h, {})['imu'] = True
        self.reader = Reader(sensor_config=self.sensor_config, use_ble=self.use_ble)
        # The ONLY reader of every parser queue, and the one place pose,
        # calibration, thresholds and gesture edges are computed.
        self.pose = PoseProvider(reader=self.reader, config=self.motion_config,
                                 tuning=live_controls, diagnostics=diagnostics)
        self.live_controls = self.pose.tuning          # what the twin's sliders drive
        self.effect_q = queue.Queue(maxsize=5)         # app -> twin: human-readable effects
        # Twin's raw+motion frames: private per-hand subscriptions, created now
        # so they exist before the first frame.
        self.viz_q = self.pose.subscribe_all(maxsize=5, name='viz')

        self.mapper = NoteMapper(root_note=root_note, scale=scale)
        self.midi_writer = midi_writer if midi_writer is not None else MidiWriter()
        self.main_loop_live = Event()                  # set once the loop is consuming frames
        self._stop_evt = Event()
        self._frames = None

    # -- construction helpers ---------------------------------------------

    def _get_sensor_config(self, sensor_config):
        if self.send_all_data:
            return {h: {'flex': True, 'press': True, 'imu': True} for h in self.hands}
        if sensor_config is not None:
            return sensor_config
        config = {h: {'flex': False, 'press': False, 'imu': False} for h in self.hands}
        for hand in self.hands:
            for sensor in config[hand]:
                if sensor in self.trigger_sensors.get(hand, []):
                    config[hand][sensor] = True
        return config

    # -- hooks (override these) -------------------------------------------

    def on_start(self):
        """Once, after bring-up and calibration, before the first frame."""

    def on_frame(self, frame, motion):
        """Every frame with valid motion, from either hand (``motion.hand``).
        For continuous controls. ``frame`` is the enriched dict (raw sensor
        keys too); prefer ``motion``."""

    def on_event(self, event, motion):
        """Every edge, one call each, in order. ``event.hand`` is the hand that
        produced it -- not necessarily ``motion.hand`` (see subscribe_merged)."""

    def on_stop(self):
        """Once, when the loop exits."""

    # -- helpers for subclasses -------------------------------------------

    def emit(self, hand, text, notes=None, use_yaw=None):
        """Tell the twin what just happened. Never blocks."""
        msg = {'hand': hand, 'text': text}
        if notes is not None:
            msg['notes'] = notes
        if use_yaw is not None:
            msg['use_yaw'] = bool(use_yaw)
        try:
            self.effect_q.put_nowait(msg)
        except queue.Full:
            pass

    # -- lifecycle ---------------------------------------------------------

    def run(self):
        # Subscribe BEFORE bring_up so the first frame is seen.
        self._frames = self.pose.subscribe_merged(maxsize=64, name=type(self).__name__)
        self.pose.bring_up(motion=self.enable_motion_stream,
                   nav_quat=False)
        self.pose.calibrate()
        try:
            self.on_start()
            self.main_loop_live.set()
            while not self._stop_evt.is_set():
                try:
                    frame = self._frames.get(timeout=self.MAIN_LOOP_POLL_SEC)
                except queue.Empty:
                    continue
                motion = frame.get('motion')
                if motion is None:
                    continue
                if motion.valid:
                    self.on_frame(frame, motion)
                for event in motion.events:
                    self.on_event(event, motion)
        finally:
            self.on_stop()

    def shutdown(self):
        """Stop the loop, the provider and the reader."""
        self._stop_evt.set()
        self.pose.stop()
        self.join(timeout=0.5)
