import serial
import numpy as np
import sounddevice as sd

# --- Serial Configuration ---
port = 'COM5'
baudrate = 115200
timeout = 1
ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Audio Parameters ---
sample_rate = 44100      # Standard audio sampling frequency
buffer_size = 1024       # Audio buffer size (not directly used here)

# Initial frequency (A4 tone)
freq = 440

def callback(outdata, frames, time, status):
    """
    Audio callback function. This is called continuously by the audio stream.
    Generates a sine wave at the current global frequency 'freq'.
    """
    global freq
    # Time axis for the buffer
    t = (np.arange(frames) + callback.frame) / sample_rate
    # Generate sine wave and write to output buffer (mono channel)
    outdata[:] = (0.2 * np.sin(2 * np.pi * freq * t)).reshape(-1,1)
    callback.frame += frames

callback.frame = 0  # Keeps track of sample index across callback calls

# Open the output audio stream using the callback
stream = sd.OutputStream(channels=1, callback=callback, samplerate=sample_rate)
stream.start()

try:
    while True:
        # Read one line from serial (blocking until newline or timeout)
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:
            continue

        # Handle special messages from MCU
        if "ERROR" in raw:
            print("!!! ERROR !!!")
            freq = 100   # Set low frequency beep on SYSTEM ERROR
            continue
        elif "WARNING" in raw:
            print("WARNING")
            freq = 880   # High pitch tone on WARNING state
            continue

        # Parse voltage value (mV) from string like: "Mode=RAW | ADC=1234 -> 987 mV"
        try:
            parts = raw.split("->")
            val_part = parts[1].replace("mV", "").strip()
            value = float(val_part)
        except Exception:
            # Skip malformed data
            continue

        # Convert voltage (0-3300mV) to frequency (200 - 2000 Hz)
        freq = 200 + (value / 3300) * 1800

except KeyboardInterrupt:
    print("Stopped")

finally:
    # Clean shutdown of audio and serial interfaces
    stream.stop()
    stream.close()
    ser.close()
