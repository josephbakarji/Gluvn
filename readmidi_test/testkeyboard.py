import mido

names = mido.get_input_names()
keyboard = names[0]
print(keyboard)
inport = mido.open_input('Digital Piano')
while True:
	msg = inport.receive()
	print(msg)

# with mido.open_input('Digital Piano') as inport:
# 	for msg in inport:
# 		print(msg)