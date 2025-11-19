import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import threading
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
MAX_DISPLAY_POINTS = 200 # Number of data points to show on the plot

# Global variables for data storage and synchronization
data_values = []
current_signal = 'RAW'
data_lock = threading.Lock() # Lock to ensure thread-safe access to data_values
is_running = True

# --- SERIAL SETUP ---
try:
    # Initialize the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    # Exit if we can't open the port
    exit()

# --- SERIAL READER THREAD FUNCTION ---
def serial_reader():
    """Reads data from the serial port continuously and safely updates data_values."""
    global is_running
    while is_running:
        try:
            # Read a line and decode it
            raw_line = ser.readline().decode('utf-8').strip()
        except serial.SerialTimeoutException:
            continue
        except Exception as e:
            # Handle decoding or communication errors
            print(f"Serial read error: {e}")
            time.sleep(0.1)
            continue

        if not raw_line or "|" not in raw_line:
            continue

        try:
            # Expected format: "TYPE|VALUEmV" e.g., "RAW|1234.5mV"
            parts = raw_line.split('|')
            # Extract value, remove 'mV' if present, and convert to float
            value = float(parts[1].replace('mV', '').strip())
            
            # Use the lock before modifying the shared list
            with data_lock:
                data_values.append(value)
                # Keep the list size bounded to prevent memory issues over time
                if len(data_values) > 1000:
                    data_values.pop(0) 

        except (ValueError, IndexError):
            # Ignore lines that don't parse correctly (e.g., corrupted data)
            # print(f"Skipping unparseable line: {raw_line}")
            pass

# Start the serial reading thread
threading.Thread(target=serial_reader, daemon=True).start()

# --- MATPLOTLIB SETUP ---

# Create figure and axes
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([], [], lw=2, color='#1f77b4') # Nice default blue color
ax.set_ylim(0, 3500) # Assuming ADC max value is around 3500mV
ax.set_xlim(0, MAX_DISPLAY_POINTS)
ax.set_xlabel("Sample Index")
ax.set_ylabel("Voltage (mV)")
ax.grid(True, linestyle='--', alpha=0.6)
plt.subplots_adjust(bottom=0.2) # Make space for the buttons

# --- ANIMATION UPDATE FUNCTION ---
def update(frame):
    """Updates the plot data for each animation frame."""
    global current_signal
    # Use the lock before reading the shared list
    with data_lock:
        # Get the last N points to plot (N = MAX_DISPLAY_POINTS)
        display_data = data_values[-MAX_DISPLAY_POINTS:]
    
    # Update the line data
    line.set_data(range(len(display_data)), display_data)
    
    # Update the plot title dynamically
    ax.set_title(f"Misure STM32 - Segnale: {current_signal}", fontsize=16)

    # Re-adjust X-limits if the data is shorter than MAX_DISPLAY_POINTS
    # ax.set_xlim(0, max(MAX_DISPLAY_POINTS, len(display_data)))
    
    # Return the updated artist (line)
    return line,

# --- BUTTON HANDLERS ---

def show_raw(event):
    """Sends 'R' command for RAW signal and updates the current_signal state."""
    global current_signal
    ser.write(b'R')
    current_signal = 'RAW'
    print("Command sent: R (RAW)")

def show_ma(event):
    """Sends 'M' command for Moving Average signal and updates the current_signal state."""
    global current_signal
    ser.write(b'M')
    current_signal = 'MOVING AVERAGE'
    print("Command sent: M (Moving Average)")

def show_noise(event):
    """Sends 'N' command for NOISE signal and updates the current_signal state."""
    global current_signal
    ser.write(b'N')
    current_signal = 'NOISE'
    print("Command sent: N (Noise)")

# --- BUTTON LAYOUT ---

# Define positions for the buttons (left, bottom, width, height)
ax_raw = plt.axes([0.1, 0.05, 0.2, 0.07])
ax_ma = plt.axes([0.4, 0.05, 0.2, 0.07])
ax_noise = plt.axes([0.7, 0.05, 0.2, 0.07])

btn_raw = Button(ax_raw, "RAW", color='#4CAF50', hovercolor='#66BB6A')
btn_ma = Button(ax_ma, "MA", color='#FFC107', hovercolor='#FFD54F')
btn_noise = Button(ax_noise, "NOISE", color='#2196F3', hovercolor='#42A5F5')

# Assign the handlers to button clicks
btn_raw.on_clicked(show_raw)
btn_ma.on_clicked(show_ma)
btn_noise.on_clicked(show_noise)

# Initialize animation (interval in milliseconds)
ani = FuncAnimation(fig, update, interval=100, blit=False)

# --- CLEANUP AND EXECUTION ---

# Add a function to handle window close event for cleanup
def on_close(event):
    global is_running
    print("Closing application and serial port...")
    is_running = False
    if ser.is_open:
        ser.close()

fig.canvas.mpl_connect('close_event', on_close)

# Display the plot
plt.show()

# Final safety close in case plt.show() blocks execution
if ser.is_open:
    ser.close()