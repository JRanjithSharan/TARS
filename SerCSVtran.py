import serial
import time
import os

print("Starting GPS CSV sender...")

# Validate file
if not os.path.exists('trajectory.csv'):
    print("Error: 'trajectory.csv' not found in:", os.getcwd())
    exit()

# Try connecting to serial
try:
    ser = serial.Serial('COM5', 115200)
    time.sleep(2)
except serial.SerialException as e:
    print("Could not open serial port: ", e)
    exit()

# Downsampling factor
n = 3

# Read file
with open('trajectory.csv', 'r') as file:
    lines = file.readlines()

downsampled_lines = lines[::n]

for line in downsampled_lines:
    line = line.strip()
    if line:
        print(f"Sending: {line}")
        ser.write((line + '\n').encode())
        time.sleep(1)

ser.close()
print("Done.")
