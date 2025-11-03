import serial
import numpy as np
import sounddevice as sd

# Parametri seriale
port = 'COM5'
baudrate = 115200
timeout = 1
ser = serial.Serial(port, baudrate, timeout=timeout)

# Parametri audio
sample_rate = 44100
buffer_size = 1024

# Frequenza iniziale
freq = 440

def callback(outdata, frames, time, status):
    global freq
    t = (np.arange(frames) + callback.frame) / sample_rate
    outdata[:] = (0.2 * np.sin(2 * np.pi * freq * t)).reshape(-1,1)
    callback.frame += frames
callback.frame = 0

stream = sd.OutputStream(channels=1, callback=callback, samplerate=sample_rate)
stream.start()

try:
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:
            continue
        try:
            parts = raw.split("mV")
            val_part = parts[0].split("->")[-1].strip()
            value = float(val_part)
        except Exception:
            continue

        # Mappa valore ADC a frequenza (ad libitum)
        freq = 200 + (value / 3300) * 1800

except KeyboardInterrupt:
    print("Stopped")
finally:
    stream.stop()
    stream.close()
    ser.close()
