from mido import MidiFile, MetaMessage

mid = MidiFile('./data/run1R.mid')
for i, track in enumerate(mid.tracks):
	print('Track {}: {}'.format(i, track.name))
	for msg in track:
		if(msg.type == 'note_on'):
			print(msg)