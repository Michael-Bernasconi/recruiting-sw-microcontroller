import serial
import matplotlib.pyplot as plt
from collections import deque

# --- Parametri seriale ---
port = 'COM5'         # cambia con la tua porta
baudrate = 115200
timeout = 1           # secondi

ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Finestra dati per grafico ---
window_size = 200
buffer = deque([0]*window_size, maxlen=window_size)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(buffer, color='green')
ax.set_ylim(0, 3300)  # in mV
ax.set_xlabel('Campione')
ax.set_ylabel('Tensione (mV)')
ax.set_title('ADC realtime plot')

try:
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:
            continue

        # Gestione ERROR / WARNING
        if "ERROR" in raw:
            print("!!! ERROR !!!")
            line.set_color('red')   # linea rossa in caso di errore
            continue
        elif "WARNING" in raw:
            print("WARNING")
            line.set_color('orange')  # linea arancione in caso di warning
            continue
        else:
            line.set_color('green')  # normale

        # Parse valore mV dal messaggio: "Mode=XXX | ADC=YYYY -> ZZZZ mV"
        try:
            parts = raw.split("->")
            val_part = parts[1].replace("mV", "").strip()
            value = float(val_part)
        except Exception as e:
            # parsing fallito, ignora
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
