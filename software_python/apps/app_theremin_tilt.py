"""
example_app.py -- a complete second instrument, as a template for new apps.

"Tilt theremin": hold finger 1 (press 1) to sound a note; the forearm's pitch
(tilt) when you press picks the scale degree; rolling the wrist sweeps the mod
wheel (CC1). Twelve lines of musical logic -- no angles, thresholds, latches,
calibration, queues or threads, because core delivers all of that.

Run it exactly like the choir (see app_jacob_choir.py's __main__): construct,
app.start(), optionally build_app_and_window(reader=app) for the twin.
"""

from core.music_app import MusicApp


class TiltTheremin(MusicApp):
    STEPS = 8                                     # scale degrees across the tilt range

    def on_start(self):
        self.notes = self.mapper.basic_map(first_note=self.mapper.root_note + '3',
                                           num_notes=self.STEPS)
        self.sounding = {}                        # hand -> note currently held

    def on_frame(self, frame, motion):            # continuous: roll -> mod wheel
        self.midi_writer.control_change(int(motion.roll_depth * 127), 1)

    def on_event(self, event, motion):            # discrete: press 1 on/off
        if event.kind != 'press' or event.index != 0:
            return
        if event.value == 1:
            tilt = self.pose.latest_motion(event.hand).scaled_pitch        # 0-127
            note = self.notes[min(self.STEPS - 1, tilt * self.STEPS // 128)]
            self.midi_writer.trig_note(note, 90)
            self.sounding[event.hand] = note
            self.emit(event.hand, f'THEREMIN ON [{event.hand}] note {note}')
        elif event.hand in self.sounding:
            self.midi_writer.trig_note(self.sounding.pop(event.hand), 0)
