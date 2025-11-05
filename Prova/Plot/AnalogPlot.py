import serial
import matplotlib.pyplot as plt
from collections import deque

# --- Serial Port Parameters ---
port = 'COM5'         # Change this to your actual serial port
baudrate = 115200
timeout = 1           # Seconds to wait for serial data

ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Data Buffers for Realtime Plot ---
# These act as rolling windows that keep the last N samples
window_size = 200
raw_buffer = deque([0]*window_size, maxlen=window_size)     # RAW signal
ma_buffer = deque([0]*window_size, maxlen=window_size)      # Moving Average signal
noise_buffer = deque([0]*window_size, maxlen=window_size)   # Noise signal

# Enable interactive plotting
plt.ion()
fig, ax = plt.subplots()

# Create line objects (they will be updated dynamically)
line_raw, = ax.plot(raw_buffer, color='green', label='RAW')
line_ma, = ax.plot(ma_buffer, color='blue', label='Moving Average')
line_noise, = ax.plot(noise_buffer, color='orange', label='Random Noise')

# Configure plot appearance
ax.set_ylim(0, 3300)                       # ADC range converted to mV
ax.set_xlabel('Sample')
ax.set_ylabel('Voltage (mV)')
ax.set_title('ADC Realtime Plot - RAW / MA / NOISE')
ax.legend()

try:
    while True:
        # Read one line from serial
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:       # Skip empty reads
            continue

        # Handle special state messages from MCU
        if "ERROR" in raw:
            print("!!! ERROR !!!")
            continue
        elif "WARNING" in raw:
            print("WARNING")
            continue

        # Parse mode and voltage value
        # Expected format example:
        #   Mode=RAW | ADC=1082 -> 871 mV
        try:
            parts = raw.split('|')
            mode_part = parts[0].split('=')[1].strip()
            value_part = parts[1].split('->')[1].replace('mV', '').strip()
            value = float(value_part)
        except Exception:
            # Skip malformed lines
            continue

        # Update the correct signal buffer based on mode
        if mode_part == "RAW":
            raw_buffer.append(value)
        elif mode_part in ("MA", "MOVING_AVERAGE"):
            ma_buffer.append(value)
        elif mode_part in ("RANDOM_NOISE", "NOISE"):
            noise_buffer.append(value)
        else:
            continue

        # Update plotted line data
        line_raw.set_ydata(raw_buffer)
        line_ma.set_ydata(ma_buffer)
        line_noise.set_ydata(noise_buffer)

        # Redraw plot
        ax.set_xlim(0, window_size-1)
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    # Clean plot and serial close on exit
    ser.close()
    plt.ioff()
    plt.show()
