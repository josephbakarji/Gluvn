from senstonote_new import BaseApp


root_note = 'D'
scale = 'minor'
thresholds = {'flex': 150, 'press': 20}
trigger_sensors = {'l': 'flex', 'r': 'flex'}
hysteresis = 5


app = BaseApp(root_note=root_note,
              scale=scale,
              thresholds=thresholds,
              trigger_sensors=trigger_sensors,
              hysteresis=hysteresis)
app.start()

key = input('press any key to finish \n')
print('Shutting down...')

app.reader.stop_readers()
for hand in app.hands:
    app.triggers[hand].join(timeout=.1)
app.join(timeout=.1)  