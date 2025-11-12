import serial
import matplotlib.pyplot as plt
from collections import deque

# --- Serial Port Parameters ---
port = 'COM5'       
baudrate = 115200
timeout = 1

ser = serial.Serial(port, baudrate, timeout=timeout)

# --- Data Buffers  --- 
window_size = 50        # number of pressur
button_buffer = deque([0]*window_size, maxlen=window_size)

# --- Setup plot ---
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(button_buffer, color='red', label='Button PA1')

ax.set_ylim(-0.1, 1.1)
ax.set_xlabel('Sample')
ax.set_ylabel('Pressed (1) / Released (0)')
ax.set_title('PA1 Interrupt Realtime')
ax.legend()

try:
    while True:
        raw_line = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw_line:
            continue

        # interrupt
        if "Button pressed" in raw_line:
            button_buffer.append(1)
        else:
            button_buffer.append(0)

        # Update chart
        line.set_ydata(button_buffer)
        ax.set_xlim(0, window_size-1)
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    ser.close()
    plt.ioff()
    plt.show()
