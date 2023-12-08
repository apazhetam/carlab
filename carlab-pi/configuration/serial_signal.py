# serial_signal.py
# Use this to test the serial signal sent to the robot
import serial
import time

ser = serial.Serial("/dev/ttyAMA0", 115200)

# Modify the sequence of actions sent to the robot 
actions = "FBRL"

actions += "X"
print(actions)
ser.write(bytes(actions,'utf-8'))
for i in range(len(actions)):
    ser.write(bytes(actions[i],'utf-8'))
    time.sleep(0.1)