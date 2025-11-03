import serial
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# Parametri seriale
port = 'COM5'         # cambia con la tua porta
baudrate = 115200     # deve corrispondere a quella del micro
timeout = 1           # in secondi

ser = serial.Serial(port, baudrate, timeout=timeout)

# Finestra dati per grafico
window_size = 200
buffer = deque([0]*window_size, maxlen=window_size)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(buffer)
ax.set_ylim(0, 3300)             # se stai usando mV con scala 0‑3300
ax.set_xlabel('Campione')
ax.set_ylabel('Tensione (mV)')
ax.set_title('ADC realtime plot')

try:
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        # Es: raw = "ADC=1234 -> 1000 mV"
        if not raw:
            continue
        # Parse il valore in mV (adatta se formato diverso)
        try:
            parts = raw.split("mV")
            # la parte prima: "... -> 1000 "
            val_part = parts[0].split("->")[-1].strip()
            value = float(val_part)
        except Exception as e:
            # parsing fallito
            #print("Parse error:", raw)
            continue

        buffer.append(value)
        line.set_ydata(buffer)
        ax.set_xlim(0, len(buffer)-1)
        fig.canvas.draw()
        fig.canvas.flush_events()
except KeyboardInterrupt:
    print("Stopped by user")
finally:
    ser.close()
    plt.ioff()
    plt.show()
