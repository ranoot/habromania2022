import time
import struct
from enum import Enum

class Servo(Enum):
	ARM = 0
	RIGHT = 1
	LEFT = 2
	COMPART = 3

class Claw:
    def __init__(self, ser):
        self.ser = ser
    
    def write_claw(self, servo: Servo, angle):
        self.ser.write(b'm' + (servo.value).to_bytes(1, 'big') + struct.pack('f', angle) + b'\n')
        return self

    #* RIGHT servo's resting state is angle of 300
    def open(self):
        self.write_claw(Servo.RIGHT, 300)
        self.write_claw(Servo.LEFT, 0)
        return self

    def close(self, angle):
        if angle < 0 or angle > 150:
            return self
        self.write_claw(Servo.RIGHT, 300 - angle)
        self.write_claw(Servo.LEFT, angle)
        return self

    def raisin(self):
        self.write_claw(Servo.ARM, 180)
        return self

    def lower(self):
        self.write_claw(Servo.ARM, 0)
        return self

    def delay(self, seconds):
        time.sleep(seconds)
        return self

