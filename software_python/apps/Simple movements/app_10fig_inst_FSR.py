import sys
import asyncio
import numpy as np
from bleak import BleakScanner

from software_python.core.app_core import BaseApp
from core.__init__ import BLE_NAME_L, BLE_NAME_R


def scan_available_ble(timeout=5.0):
    """
    Scans for active Gluvn BLE peripherals.
    Returns a set containing 'l', 'r', or both.
    """
    async def _scan():
        found = set()
        print(f"Scanning for active Gluvn gloves ({timeout}s timeout)...")
        try:
            devices = await BleakScanner.discover(timeout=timeout)
            for d in devices:
                if d.name == BLE_NAME_R:
                    found.add('r')
                elif d.name == BLE_NAME_L:
                    found.add('l')
        except Exception as e:
            print(f"BLE scan warning: {e}")
        return found

    return asyncio.run(_scan())


# 1. Auto-detect online gloves
active_hands = scan_available_ble(timeout=5.0)

if not active_hands:
    print("\n[Error] No active Gluvn hardware found. Power on M5StickC and retry.")
    sys.exit(1)

print(f"Discovered gloves: {[h.upper() for h in sorted(active_hands)]}")

# 2. Operational parameters
root_note  = 'D'
scale      = 'minor'
thresholds = {'flex': 130, 'press': 20}
hysteresis = 5   

# 3. Build configs strictly from discovered hands
# ROUTED TO PRESSURE INSTEAD OF FLEX FOR NOTE ON/OFF TRIGGERING
trigger_sensors = {hand: 'press' for hand in active_hands}

sensor_config   = {hand: {'flex': True, 'press': True, 'imu': True}
                   for hand in active_hands}
mod_sensors     = {hand: [None] for hand in active_hands}
mod_idx         = {hand: [None] for hand in active_hands}

# 4. Instantiate app
app = BaseApp(
    root_note=root_note,
    scale=scale,
    thresholds=thresholds,
    trigger_sensors=trigger_sensors,
    sensor_config=sensor_config,    
    mod_sensors=mod_sensors,
    mod_idx=mod_idx,
    hands=list(active_hands),       
    hysteresis=hysteresis,
    use_ble=True                    
)

app.start()
print("App started successfully.")

input('Press any key to finish\n')
print('Shutting down...')

# 5. Clean teardown
app.reader.stop_readers()
for hand in app.hands:
    if hand in app.triggers:
        app.triggers[hand].join(timeout=0.5)
app.join(timeout=0.5)