import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from collections import deque
import threading
from functools import partial

# Serial configuration
COM_PORT = 'COM10'
BAUD_RATE = 460800 * 2
DISPLAY_WINDOW = 36000

# Thread-safe data storage
data_lock = threading.Lock()
adc_buffer = deque(maxlen=DISPLAY_WINDOW)  # Stores (timestamp, value) pairs
serial_reader = None
timestamp = 0

# Plot configuration
PLOT_REFRESH_INTERVAL = 50  # ms 

def serial_read_thread():
    """Continuous thread for reading and parsing serial data"""
    global serial_reader, adc_buffer, timestamp
    buffer = b""
    data_points_per_second = 0
    last_time = time.time()
    
    while True:

        try:
            if not serial_reader:
                serial_reader = serial.Serial(COM_PORT, BAUD_RATE, timeout=0)
                print("Connected to serial port")
                
            # Read all available bytes
            while serial_reader.in_waiting > 0:
                buffer += serial_reader.read(serial_reader.in_waiting) #.decode('utf-8', errors='replace')
                
                # Process complete lines
                while b'stat' in buffer:
                    line, buffer = buffer.split(b'stat', 1)
                    # print(line)
                    # line = line.strip()
                    if b'ende' in line:
                        try:
                            data = line.split(b'ende')[0]
                            values = [[val for val in data[i:i+4]] for i in range(0, len(data), 4)]
                            for value in values:
                                if len(value) != 4:
                                    print(f"oh no! {value} {len(value)}")
                                    continue
                                with data_lock:
                                    adc_buffer.append((timestamp, value))
                                    timestamp += 1
                                    data_points_per_second += 1
                                    while adc_buffer[0][0] < timestamp - DISPLAY_WINDOW:
                                        adc_buffer.popleft()
                        except (ValueError, IndexError):
                            pass

                if time.time() - last_time > 1:
                    print(f"Data points per second: {data_points_per_second}")
                    last_time = time.time()
                    data_points_per_second = 0
            if time.time() - last_time > 1:
                print(f"Data points per second: {data_points_per_second}")
                last_time = time.time()
                data_points_per_second = 0
                        
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            if serial_reader:
                serial_reader.close()
                serial_reader = None
            time.sleep(1)
        except Exception as e:
            # print(f"Unexpected error: {e}")
            raise e
            break

offset = 0
def update_plot(frame):
    global lines
    """Update function for matplotlib animation"""
    with data_lock:
        if not adc_buffer:
            return lines
        current_time = timestamp
        # Convert to relative time and values
        # print(adc_buffer)
        
        times = [current_time - ts for ts, _ in adc_buffer]
        for offset in range(4):
            values = [val[offset] for _, val in adc_buffer]
            lines[offset].set_data(times, values)
    
    # Adjust x-axis limits to show sliding window
    # plt.xlim(max(0, times[-1]-0.1), DISPLAY_WINDOW+0.1) if times else (0, 1)
    # plt.xlim(max(0, current_time - DISPLAY_WINDOW), current_time)
    plt.xlim(0, DISPLAY_WINDOW)
    return lines

# Setup plot
plt.figure(figsize=(10, 5))
lines = []
for offset in range(4):
    line, = plt.plot([], [], lw=2)
    lines.append(line)
plt.xlim(0, DISPLAY_WINDOW)
plt.ylim(0, 256)
plt.xlabel('Time (s)')
plt.ylabel('ADC Value')
plt.title('Real-time ADC Monitoring')
plt.subplots_adjust(bottom=0.2)

# Start serial thread
threading.Thread(target=serial_read_thread, daemon=True).start()
from matplotlib.widgets import Button
button_axes = [plt.axes([0.125 + i * 0.2, 0.015, 0.15, 0.075]) for i in range(4)]
buttons = {label: Button(ax, label) for ax, label in zip(button_axes, ["LK", "LD", "RD", "RK"])}

def button_click(label, event):
    global offset
    offset = {
        'LK': 0,
        'LD': 1, 
        'RD': 2,
        'RK': 3
    }[label]
    # if serial_reader:
    #     serial_reader.write({
    #         'LK': b'\x00',
    #         'LD': b'\x01', 
    #         'RD': b'\x02',
    #         'RK': b'\x03'
    #     }[label])
    #     print(f"Sent command to change to {label}")

for label, button in buttons.items():
    button.on_clicked(partial(button_click, label))

# Start animation
ani = FuncAnimation(plt.gcf(), update_plot, interval=PLOT_REFRESH_INTERVAL, blit=True)
plt.show()