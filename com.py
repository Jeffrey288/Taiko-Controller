import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

# Set up serial port
com_port = 'COM10'  # Change this to your COM port
baud_rate = 115200
ser = serial.Serial(com_port, baud_rate)

# Initialize lists to store time and ADC values
adc_values = []
buffer = ""

while True:
    try:
        if(ser == None):
            ser = serial.Serial(com_port, baud_rate)
            print("Reconnecting")

        # Read data from serial port
        if ser.in_waiting > 0:
            buffer = ser.read(ser.in_waiting) #.decode('utf-8')
            print(buffer, end="")
            
    except serial.serialutil.SerialException as e:
        if (not(ser == None)):
            ser.close()
            ser = None

# Animation
ani = FuncAnimation(plt.gcf(), update, interval=1) 
plt.show()