import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(3)

class Servo(Enum):
	ARM = 0
	RIGHT = 1
	LEFT = 2
	COMPART = 3

# print(Servo.ARM.value)
# while True:
def claw_pickup():
    ser.write(b'm' + (Servo.RIGHT.value).to_bytes(1, 'big') + struct.pack('f', 300-95) + b'\n')
    ser.write(b'm' + (Servo.LEFT.value).to_bytes(1, 'big') + struct.pack('f', 95) + b'\n')
    time.sleep(0.5)
    ser.write(b'm' + (Servo.ARM.value).to_bytes(1, 'big') + struct.pack('f', 180) + b'\n')
    time.sleep(0.5)

    
    # time.sleep(0.5)

def claw_drop_open():
    ser.write(b'm' + (Servo.RIGHT.value).to_bytes(1, 'big') + struct.pack('f', 300) + b'\n')
    ser.write(b'm' + (Servo.LEFT.value).to_bytes(1, 'big') + struct.pack('f', 0) + b'\n')
    time.sleep(0.5)
    ser.write(b'm' + (Servo.ARM.value).to_bytes(1, 'big') + struct.pack('f', 0) + b'\n')

claw_drop_open()
time.sleep(3)
claw_pickup()
time.sleep(2)
claw_drop_open()
    