import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from collections import deque

# --- Serial Port Parameters ---
port = 'COM5'         # Cambia con la tua porta reale
baudrate = 115200
timeout = 1           # Secondi di attesa

ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Data Buffers for Realtime Plot ---
window_size = 200
raw_buffer = deque([0]*window_size, maxlen=window_size)
ma_buffer = deque([0]*window_size, maxlen=window_size)
noise_buffer = deque([0]*window_size, maxlen=window_size)

# Segnale corrente da visualizzare
current_signal = 'RAW'

# --- Funzioni per i pulsanti ---
def show_raw(event):
    global current_signal
    current_signal = 'RAW'

def show_ma(event):
    global current_signal
    current_signal = 'MA'

def show_noise(event):
    global current_signal
    current_signal = 'NOISE'

# --- Setup plot ---
plt.ion()
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)  # spazio per pulsanti

line_raw, = ax.plot(raw_buffer, color='green', label='RAW')
line_ma, = ax.plot(ma_buffer, color='blue', label='MA')
line_noise, = ax.plot(noise_buffer, color='orange', label='NOISE')

ax.set_ylim(0, 3300)
ax.set_xlabel('Sample')
ax.set_ylabel('Voltage (mV)')
ax.set_title('ADC Realtime Plot')
ax.legend()

# --- Pulsanti ---
ax_raw = plt.axes([0.1, 0.05, 0.2, 0.075])
ax_ma = plt.axes([0.4, 0.05, 0.2, 0.075])
ax_noise = plt.axes([0.7, 0.05, 0.2, 0.075])

btn_raw = Button(ax_raw, 'RAW')
btn_raw.on_clicked(show_raw)

btn_ma = Button(ax_ma, 'MA')
btn_ma.on_clicked(show_ma)

btn_noise = Button(ax_noise, 'NOISE')
btn_noise.on_clicked(show_noise)

try:
    while True:
        raw_line = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw_line:
            continue

        if "ERROR" in raw_line:
            print("!!! ERROR !!!")
            continue
        elif "WARNING" in raw_line:
            print("WARNING")
            continue

        # Parse serial data
        try:
            parts = raw_line.split('|')
            mode_part = parts[0].split('=')[1].strip()
            value_part = parts[1].split('->')[1].replace('mV','').strip()
            value = float(value_part)
        except Exception:
            continue

        # Aggiorna buffer
        if mode_part == "RAW":
            raw_buffer.append(value)
        elif mode_part in ("MA", "MOVING_AVERAGE"):
            ma_buffer.append(value)
        elif mode_part in ("RANDOM_NOISE", "NOISE"):
            noise_buffer.append(value)

        # Mostra solo il segnale selezionato
        if current_signal == 'RAW':
            line_raw.set_ydata(raw_buffer)
            line_ma.set_ydata([0]*window_size)
            line_noise.set_ydata([0]*window_size)
        elif current_signal == 'MA':
            line_raw.set_ydata([0]*window_size)
            line_ma.set_ydata(ma_buffer)
            line_noise.set_ydata([0]*window_size)
        elif current_signal == 'NOISE':
            line_raw.set_ydata([0]*window_size)
            line_ma.set_ydata([0]*window_size)
            line_noise.set_ydata(noise_buffer)

        ax.set_xlim(0, window_size-1)
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    ser.close()
    plt.ioff()
    plt.show()
