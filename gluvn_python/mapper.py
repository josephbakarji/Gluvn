import numpy as np



class NoteMapper:
    def __init__(self, mapping_option, scale='major'):
        self.mapping_option = mapping_option
        self.scale = scale 
        self.note2midi, self.midi2note = self.generate_notemidi_dict()
        self.notes_in_scale = self.get_all_notes(scale)

    def generate_notemidi_dict():
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
        scales = {
            'major': [2, 2, 1, 2, 2, 2, 1],
            'natural_minor': [2, 1, 2, 2, 1, 2, 2],
            'harmonic_minor': [2, 1, 2, 2, 1, 3, 1],
            'melodic_minor_asc': [2, 1, 2, 2, 2, 2, 1],
            'melodic_minor_desc': [2, 1, 2, 2, 1, 2, 2] # same as natural minor
            # Add other scales as needed.
        }
        # Get the MIDI number of the root note.
        root_midi = self.note2midi[root_note+'0']
        
        # Get the interval sequence for the scale type.
        scale_intervals = scales[scale]
        
        # Calculate the MIDI numbers of the notes in the scale.
        notes = [root_midi]
        for interval in scale_intervals:
            if notes[-1] + interval > 127:
                break
            notes.append(notes[-1] + interval)
        
        return notes
    

    def basicMap(self, first_note='C3', num_notes=5):
        idx_root_note = Note2Num[first_note]
        return self.notes_in_scale[idx_root_note:idx_root_note+num_notes]



    def windowMap(state, WArr, NArr):
        for i, win in enumerate(WArr):
            if (win == state).all():
                return NArr[i]
        return NArr[5]


    def generateNoteMap(midnote, scale='major', mode='standard'):
        # midnote and scale not used
        if mode == 'standard':

            WArr = [ [1, 1, 1, 1, 1], 
            [0, 1, 1, 1, 1], 
            [0, 0, 1, 1, 1], 
            [0, 0, 0, 1, 1], 
            [0, 0, 0, 0, 1], 
            [0, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
            [1, 1, 0, 0, 0],
            [1, 1, 1, 0, 0],
            [1, 1, 1, 1, 0] ]

            NArr = [ ['B0', 'C1', 'D1'],
            ['E1', 'F1', 'G1'],
            ['A1', 'B1', 'C2'],
            ['D2', 'E2', 'F2'],
            ['G2', 'A2', 'B2'],
            ['C3', 'D3', 'E3'],
            ['F3', 'G3', 'A3'],
            ['B3', 'C4', 'D4'],
            ['E4', 'F4', 'G4'],
            ['A4', 'B4', 'C5']]

        return WArr, NArr
