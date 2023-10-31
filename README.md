
# Introduction
We use our hands to play most musical instruments. Why not make a hand-instrument? A combination of sensors mounted on a glove is used to generate sounds on a personal computer. 

5 flex sensors, 5 force sensitive resistors, and an IMU are hooked up to the glove and send analog data via a micro-controller (e.g. Arduino) to the serial port on a computer.
The data then gets processed in real-time to create music.
For a start, Python and Arduino are used to make it accessible to everyone and encourage developers to contribute (or not). 
The basic functionality also requires a Digital Audio Workstation (DAW) like GarageBand, Logic or Ableton.

# Hardware
## Sensors
The glove has 
- 5 [flex sensors](https://www.adafruit.com/product/1070) that measure the bending angle of each finger.
- 5 [force sensitive resistors](https://www.adafruit.com/product/166) that measure the intensity of the pressure on the tip of each finger. 
- An [inertial measurement unit](https://www.amazon.com/MPU-6050-MPU6050-Accelerometer-Gyroscope-Converter/dp/B008BOPN40) (MPU-6050) consisting of a 3-axis gyroscope and a 3-axis accelerometer, used in various ways depending on the application.


#Setup
## Packages
This package uses Python 3.5. The following packages are required 
- [mido](https://pypi.python.org/pypi/mido/1.1.3) is used to send midi messages to a virtual instrument. Install with `pip install mido`.
- [rtmidi](https://pypi.python.org/pypi/python-rtmidi/0.3.1a) is used as a backend for mido. Install with `pip install python-rtmidi`.
- [PyQt5](http://pyqt.sourceforge.net/Docs/PyQt5/) is used for an optional GUI. Install with `pip install pyqt5` and by following instructions [here](http://pyqt.sourceforge.net/Docs/PyQt5/installation.html).

## Virtual instrument
MIDI messages are sent from the python code to a DAW (e.g. GarageBand) on the computer. For this you need to setup a virtual midi port. For mac, use the following [instructions](http://feelyoursound.com/setup-midi-os-x/). Then, replace the `IACDriver` variable `__init__.py` with your virtual midi port name. Once this is done, the DAW will discover the app as a midi instrument.

- In my experience Aftertouch is best supported by Ableton's Expression Control, which offers the most flexibility in assigning different modes of expression to velocity, Pitchbend etc.

# Usage
For compiling a non-GUI version, run `python Main.py`. In `Main.py`:
- For `PressureNotes = 1`, the glove triggers a set of notes with the pressure sensors specified by `n1`, with threshold pressure `pressThresh`
- For `FlexNotes = 1`, the glove triggers a set of notes with the flex sensors specified by `n2`, with threshold flex `pressThresh`
- For `FlexPitchChords = 1` uses the IMU pitch data to trigger a chord when below a certain angle, and the flex sensors to specify the chord. The mapping is specified in the `trig2chord` function in `notemidi.py`.
- For a table of all possible notes, see the csv file `data/tables/note2midi.csv`.
- Pressure and Flex sensors can be used in combination as they run in two seperate threads but this is not recommended. For now, only have one of the above set to 1.

- Simulation: if you don't have a glove, it is most convenient to use the simulation functionality. For `sim = 1` and a specified simulation file `simfile`, the program will read sensor data recorded from the glove in the `data/sim/` folder. Otherwise, if you are using the glove, set it to 0.
- Recording: if you have the glove and want to record a file for future simulation, run `python sensor_read.py`. Your sensor data will be recorded in the `filename` you specify in the script.

## GUI
For a more intuitive usage of the glove, there is a GUI application using PyQt5 (not yet complete). This can be used by running `python gluvUI.py`.


## Potential Applications
- String instrument (e.g. violin) with pressure sensors for notes and accelerometer for vibrato
- pressure sensors with accelerometer data triggering percussion instruments by shaking the hand (audio using PyAudio)
- Accelerometer conductor baton for controlling playback speed in realtime (requires addition of playback capabilities)
- General language (gesture sequences) for triggering specific musical events
- Incorporating VMO for improvising with the gluvn!
- Simulating percussion

## Hardware improvements
- High bandwidth RF channel for data transfer
- Op-amp based sensor data conditioning to make pressure and bend sensors more sensitive
- Cortex-M4, with DSP
- 6 accelerometers


# TO DO:
### To do:
- Play notes with vibrato
    - Calibrate flexes (especially finger 2) and choose thresholds
    - For testing, try playing and saving at the same time (duplicate data might be needed)
    - Save previous n (=3) values of all sensors in queue
    - Plot magnitude of gyration and map to desired output)
    - Plot Acceleration (check how to send it instead of pitch-roll-yaw)

- ISSUES:
    - Why do gyros not get negative values?

- Animate simulated data. (https://pythonprogramming.net/live-graphs-matplotlib-tutorial/)
- Calibrate sensors on computer (not Arduino) 


### Data Analysis:

### Machine Learning Ideas:
- READUNDANCY: learn from flex sensors in file 
- Learning trigger threshold and velocity 
- Learning transition matrix for next note
- Use RNN
- Set up HMM for learning notes fingers.
- Problem with learning trigger is need for false labels: can I find a law (without learning)? Learn a constant across velcities.
- Use scales to label thumb-under for note and finger (when finger goes down but note up then its a thumb-under).
- Add sensor bending and IMU as conditions in a Bayesian Network
- Learn trigger on/off and velocity
- make transition matrix with features (previous_finger, present_finger, thumb_under (0, 1))

## Hardware Improvements:
- Test new resistors
- Make new board for both gloves
- Make wireless


