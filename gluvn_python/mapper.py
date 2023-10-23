import numpy as np



class NoteMapper:
    def __init__(self, root_note='C', scale='major'):
        self.scale = scale 
        self.root_note = root_note
        self.note2midi, self.midi2note = self.generate_notemidi_dict()
        self.notes_in_scale = self.get_all_notes(root_note, scale)

    def generate_notemidi_dict(self):
        midi_numbers = list(range(24, 128))  # MIDI numbers range from 0 to 127
        octaves = list(range(0, 10))
        notes_sharp = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
        notes_flat = ['C', 'Db', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B']
        # Generate the note names for all octaves and associate them with their corresponding MIDI numbers

        Note2Midi = {}
        Midi2Note = {}
        for octave in octaves:
            for note_sharp, note_flat, midi in zip(notes_sharp, notes_flat, midi_numbers[octave*12:(octave+1)*12]):
                Note2Midi[f"{note_sharp}{octave}"] = midi
                Note2Midi[f"{note_flat}{octave}"] = midi
                Midi2Note[midi] = f"{note_sharp}{octave}"

        return Note2Midi, Midi2Note 


    def get_all_notes(self, root_note='C', scale='major'):
        # Map scale types to their interval sequences.
        if scale == 'minor': scale = 'natural_minor'
        scales = {
            'major': [2, 2, 1, 2, 2, 2, 1],
            'natural_minor': [2, 1, 2, 2, 1, 2, 2],
            'harmonic_minor': [2, 1, 2, 2, 1, 3, 1],
            'melodic_minor_asc': [2, 1, 2, 2, 2, 2, 1],
            'melodic_minor_desc': [2, 1, 2, 2, 1, 2, 2], # same as natural minor
            'pentatonic' : [3, 2, 2, 3, 2]
            # Add other scales as needed.
        }
        # Get the MIDI number of the root note.
        root_midi = self.note2midi[root_note+'0']
        
        # Get the interval sequence for the scale type.
        scale_intervals = scales[scale]
        
        # Calculate the MIDI numbers of the notes in the scale.
        midi_notes = [root_midi]
        i = 0
        interval = scale_intervals[i]
        while (midi_notes[-1] + interval) < 127:
            midi_notes.append(midi_notes[-1] + interval)
            i += 1
            interval = scale_intervals[i%len(scale_intervals)]
        
        return midi_notes
    
    def basic_map_2hands(self, first_note='C3'):
        
        first_note_midi = self.note2midi[first_note]

        # get index of idx_root_note in self.notes_in_scale
        idx_root_note = self.notes_in_scale.index(first_note_midi)
        if idx_root_note == -1:
            raise ValueError(f"Note {first_note} not in scale {self.scale}")

        note_dict = {}
        note_dict['r'] = self.notes_in_scale[idx_root_note:idx_root_note+5]
        note_dict['l'] = self.notes_in_scale[idx_root_note-5:idx_root_note][::-1]

        return note_dict 

    def basic_map(self, first_note='C3', num_notes=5):
        "Returns the the first num_notes notes after the first_note in midi format"
        first_note_midi = self.note2midi[first_note]

        # get index of idx_root_note in self.notes_in_scale
        idx_root_note = self.notes_in_scale.index(first_note_midi)
        if idx_root_note == -1:
            raise ValueError(f"Note {first_note} not in scale {self.scale}")
        return self.notes_in_scale[idx_root_note:idx_root_note+num_notes]


    def moving_window(self, num_rhf=5, num_lhf=5):
        window_trigger = self.window_setter(mode='standard', num_fingers=num_lhf)
        note_windows = self.generate_windows(window_trigger, num_fingers=num_rhf)
        return window_trigger, note_windows


    def generate_windows(self, window_trigger, num_fingers=5):
        first_note = self.root_note+'3'
        first_note_midi = self.note2midi[first_note]
        idx_root_note = self.notes_in_scale.index(first_note_midi)
        window_trigger_array = np.array(window_trigger)
        window_list = []

        # add windows above first note
        for i in range(10):
            idx0 = idx_root_note + num_fingers*i
            window_list.append(self.notes_in_scale[idx0:idx0+num_fingers])
            if window_list[-1][-1] >= self.notes_in_scale[-num_fingers]:
                break

        # add windows below first note
        for i in range(10):
            idx0 = idx_root_note - num_fingers*(i+1)
            window_list.append(self.notes_in_scale[idx0:idx0+num_fingers])
            if window_list[-1][-1] <= self.notes_in_scale[num_fingers]:
                break
        
        # Sort windows according to first note, and turn to numpy array
        window_list = np.array(window_list)
        window_list = window_list[np.argsort(window_list[:, 0])]

        # Return as many rows as there are in window_trigger
        fingers_bent = np.sum(window_trigger, axis=1)
        num_windows = window_trigger.shape[0]
        idx_mid_window_list = np.where(window_list[:, 0] == first_note_midi)[0][0] 
        open_palm_idx = np.where(fingers_bent == 0)[0][0]

        while (open_palm_idx > idx_mid_window_list):
            # add row to the beginning of window_list (same as existing first row) 
            window_list = np.vstack((window_list[0, :], window_list)) 
            idx_mid_window_list = np.where(window_list[:, 0] == first_note_midi)[0][0] 

        idx0 = max(0, idx_mid_window_list - open_palm_idx)
        idxend = min(idx0+num_windows, window_list.shape[0])
        window_list = window_list[idx0:idxend, :]

        return window_list

    def window_setter(self, mode='standard', num_fingers=5):
        # midnote and scale not used
        if mode == 'standard':
            if num_fingers == 5:
                WArr = np.array(\
                [[1, 1, 1, 1, 1], 
                [0, 1, 1, 1, 1], 
                [0, 0, 1, 1, 1], 
                [0, 0, 0, 1, 1], 
                [0, 0, 0, 0, 1], 
                [0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0],
                [1, 1, 0, 0, 0],
                [1, 1, 1, 0, 0],
                [1, 1, 1, 1, 0]])
            elif num_fingers == 3:
                WArr = np.array(\
                [[1, 1, 1], 
                [0, 1, 1], 
                [0, 0, 1], 
                [0, 0, 0],
                [1, 0, 0],
                [1, 1, 0]])
        return WArr

    def window_map(self, nswitch, window_trigger, note_windows):
        print(nswitch, window_trigger)
        idx_list = np.where(np.all(window_trigger == nswitch, axis=1))[0]
        if len(idx_list) > 0:
            idx = idx_list[0]
            return note_windows[idx], idx 
        else:
            return None


if __name__=="__main__":
    num_rhf = 5
    mapper = NoteMapper(root_note='C', scale='major')
    window_trigger = mapper.window_setter(mode='standard')
    note_windows = mapper.generate_windows(window_trigger, num_rhf=num_rhf)
    for i in range(window_trigger.shape[0]):
        print(window_trigger[i, :])
        names = [mapper.midi2note[note] for note in note_windows[i, :]]
        print(names)
        print("---")