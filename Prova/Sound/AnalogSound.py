import serial
import numpy as np
import sounddevice as sd

# --- Parametri seriale ---
port = 'COM5'
baudrate = 115200
timeout = 1
ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Parametri audio ---
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

        # Gestione ERROR / WARNING
        if "ERROR" in raw:
            print("!!! ERROR !!!")
            freq = 100  # suono basso fisso in caso di errore
            continue
        elif "WARNING" in raw:
            print("WARNING")
            freq = 880  # suono acuto in caso di warning
            continue

        # Parse valore mV dal messaggio
        try:
            parts = raw.split("->")
            val_part = parts[1].replace("mV", "").strip()
            value = float(val_part)
        except Exception:
            continue

        # Mappa valore ADC a frequenza (ad libitum)
        freq = 200 + (value / 3300) * 1800  # range 200-2000 Hz

except KeyboardInterrupt:
    print("Stopped")
finally:
    stream.stop()
    stream.close()
    ser.close()
