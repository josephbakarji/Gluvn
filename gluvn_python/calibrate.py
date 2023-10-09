from port_read import ReadSerial, ParseSerial
from __init__ import portL, portR, baud, mainDir
import time
import numpy as np
import os
import shutil
import re
import pdb

caltime = 5
save_h = True

## Which hand?
hand = input("Which hand are you calibrating? Default R, (L/R) \n").upper()
if hand == "R":
    port = portR
elif hand == "L":
    port = portL
else:
    print("Invalid input. Defaulting to R.")
    hand = "R"
    port = portR

time0 = time.time()
sensThread = ReadSerial(port, baud)
parseThread = ParseSerial(sensThread.sensq, time0, format='>sHHHHHHHHHHHHHHHH', length_checksum=33)
parseThread.readFullData = True
sensThread.start()
parseThread.start()



## Flatten hand:
key = input("Open hand, straighten fingers, and press any key to continue \n")
timerec0 = time.time()

openhand_list = []
with parseThread.dataq.mutex:
    parseThread.dataq.queue.clear()

while time.time() - timerec0 < caltime:
    openhand_list.append(parseThread.dataq.get(block=True)[8:18])

openhand = np.array(openhand_list)
openhand_mean = np.mean(openhand, axis=0)
flex_openhand_mean = openhand_mean[:5]
press_openhand_mean = openhand_mean[5:]

print("Open hand mean, MIN_FLEX: ", flex_openhand_mean)
print("No press mean, MIN_PRESS: ", press_openhand_mean)


## Fist hand:
key = input("Close hand, Make fist, and press any key to continue \n")
timerec0 = time.time()

fisthand_list = []
with parseThread.dataq.mutex:
    parseThread.dataq.queue.clear()

while time.time() - timerec0 < caltime:
    fisthand_list.append(parseThread.dataq.get(block=True)[8:13])

fisthand = np.array(fisthand_list)
flex_fisthand_mean = np.mean(fisthand, axis=0)

print("Fist hand mean, MAX_FLEX: ", flex_fisthand_mean)


## Pressure per finger:
press_finger_max = []
for i in range(5):
    key = input(f"Press finger {i+1}, (where i=1 is the thumb, i=2 the index etc.) as strongly as you can, and press any key to continue \n")
    timerec0 = time.time()

    press_list = []
    with parseThread.dataq.mutex:
        parseThread.dataq.queue.clear()

    while time.time() - timerec0 < caltime:
        press_list.append(parseThread.dataq.get(block=True)[13:18])

    press_list = np.array(press_list)[:, i]
    press_finger_max.append( np.min(press_list, axis=0) ) # pressure sensor value decreases when pressed

    print(f"Press finger {i+1} max, MAX_PRESS: ", press_finger_max)

press_finger_max = np.array(press_finger_max)


# Save calibration data in txt file

if save_h:
    filename = "calibration.h"
    filepath = os.path.join(mainDir, "data", "calibration", filename)

    # Check if the file exists
    file_exists = os.path.exists(filepath)

    # Use a prefix based on the hand
    prefix = "R_" if hand == "R" else "L_"

    # Generate the new calibration values
    new_content = []

    new_content.append(f"const int {prefix}MIN_FLEX[] = {{")
    for i in range(4):
        new_content.append(f"{int(flex_openhand_mean[i])}, ")
    new_content.append(f"{int(flex_openhand_mean[4])}}};\n")

    new_content.append(f"const int {prefix}MAX_FLEX[] = {{")
    for i in range(4):
        new_content.append(f"{int(flex_fisthand_mean[i])}, ")
    new_content.append(f"{int(flex_fisthand_mean[4])}}};\n")

    new_content.append(f"const int {prefix}MIN_PRESS[] = {{")
    for i in range(4):
        new_content.append(f"{int(press_openhand_mean[i])}, ")
    new_content.append(f"{int(press_openhand_mean[4])}}};\n")

    new_content.append(f"const int {prefix}MAX_PRESS[] = {{")
    for i in range(4):
        new_content.append(f"{int(press_finger_max[i])}, ")
    new_content.append(f"{int(press_finger_max[4])}}};\n")

    new_content = ''.join(new_content)

    # If the file exists, read its content and update the values
    if file_exists:
        with open(filepath, "r") as f:
            content = f.read()

        # Check if the specific hand's values already exist
        if prefix + "MIN_FLEX" in content:
            # Update the values
            content = re.sub(f"const int {prefix}MIN_FLEX\[.*\] = \{{.*\}};", new_content.split('\n')[0], content)
            content = re.sub(f"const int {prefix}MAX_FLEX\[.*\] = \{{.*\}};", new_content.split('\n')[1], content)
            content = re.sub(f"const int {prefix}MIN_PRESS\[.*\] = \{{.*\}};", new_content.split('\n')[2], content)
            content = re.sub(f"const int {prefix}MAX_PRESS\[.*\] = \{{.*\}}", new_content.split('\n')[3], content)
        else:
            # Append the new values
            content = content.replace("#endif", new_content + "\n#endif")

        # Write the updated content back to the file
        with open(filepath, "w") as f:
            f.write(content)
    else:
        # If the file doesn't exist, create it with the new values
        with open(filepath, "w") as f:
            f.write("#ifndef CALIBRATION_H\n")
            f.write("#define CALIBRATION_H\n")
            f.write("\n")
            f.write(new_content)
            f.write("\n")
            f.write("#endif\n")

    # Copy the file to the desired location
    shutil.copy(filepath, os.path.join(mainDir, "..", "arduino", "sendsens_Right", "calibration.h"))

else:
    calibration_data = np.vstack((flex_openhand_mean, flex_fisthand_mean, press_openhand_mean, press_finger_max))
    np.savetxt(os.path.join(mainDir, "data", "calibration/calibration_pressflex.txt"), calibration_data, fmt="%10.3f", delimiter=",")
    print("Calibration data saved in calibration_data.txt")

