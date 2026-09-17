import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import rfft, rfftfreq

# =====================================================
# CONFIG
# =====================================================

# update files names

FILES = [
    "session_1784194074.csv",
    "session_1784194234.csv",
    "session_1784194277.csv",
]

MAX_FREQ = 10

# =====================================================
# LOAD SENSOR LIST
# =====================================================

df0 = pd.read_csv(FILES[0])

sensor_cols = [
    c for c in df0.columns
    if c != "time"
]

# =====================================================
# ANALYZE EACH SENSOR
# =====================================================

for sensor in sensor_cols:

    print(f"Processing {sensor}")

    spectra = []
    freq_ref = None

    for file in FILES:

        df = pd.read_csv(file)

        t = df["time"].values
        x = df[sensor].values

        # remove DC
        x = x - np.mean(x)

        dt = np.mean(np.diff(t))
        fs = 1.0 / dt

        freqs = rfftfreq(len(x), dt)

        fft_mag = np.abs(rfft(x))

        # normalize energy
        if np.max(fft_mag) > 0:
            fft_mag = fft_mag / np.max(fft_mag)

        if freq_ref is None:
            freq_ref = freqs
        else:
            L = min(len(freq_ref), len(freqs))
            freq_ref = freq_ref[:L]
            fft_mag = fft_mag[:L]

        spectra.append(fft_mag)

    spectra = np.vstack(spectra)

    mean_fft = np.mean(spectra, axis=0)
    std_fft = np.std(spectra, axis=0)

    plt.figure(figsize=(8,4))

    plt.plot(freq_ref, mean_fft)

    plt.fill_between(
        freq_ref,
        mean_fft - std_fft,
        mean_fft + std_fft,
        alpha=0.3
    )

    plt.title(sensor)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Normalized Magnitude")
    plt.xlim(0, MAX_FREQ)
    plt.grid(True)

plt.show()