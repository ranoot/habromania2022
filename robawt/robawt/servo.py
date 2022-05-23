import time
import struct
from enum import Enum

class Servo(Enum):
	ARM = 0
	LEFT = 1
	RIGHT = 2
	COMPART = 3

class Claw:
    def __init__(self, ser):
        self.ser = ser
    
    def write_claw(self, servo: Servo, angle):
        self.ser.write(b'm' + (servo.value).to_bytes(1, 'big') + struct.pack('f', angle) + b'\n')
        return self

    #* RIGHT servo's resting state is angle of 300
    def open(self):
        self.write_claw(Servo.RIGHT, 300-25)
        self.write_claw(Servo.LEFT, 25)
        return self

    def close(self, angle):
        if angle < 0 or angle > 150:
            return self
        self.write_claw(Servo.RIGHT, 300 - angle)
        self.write_claw(Servo.LEFT, angle)
        return self

    def raisin(self):
        self.write_claw(Servo.ARM, 155)
        return self

    def lower(self):
        self.write_claw(Servo.ARM, 0)
        return self

    def delay(self, seconds):
        time.sleep(seconds)
        return self

    def tilt_compartment(self):
        self.write_claw(Servo.COMPART, 180-10)
        time.sleep(1)
        self.write_claw(Servo.COMPART, 180-55)
        time.sleep(1)
        return self

    def reset_compartment(self):
        self.write_claw(Servo.COMPART, 180)
        return self
    
    def left(self, angle):
        self.write_claw(Servo.LEFT, 20)
        self.write_claw(Servo.RIGHT, 205-angle)
        return self

    def right(self, angle):
        self.write_claw(Servo.RIGHT, 300-20)
        self.write_claw(Servo.LEFT, 95+angle)
        return self
        
    def dead(self):
        self.close(95).delay(0.5).raisin().delay(0.5).left(35).delay(0.5).open().delay(0.5)
        return self

    def alive(self):
        self.close(95).delay(0.5).raisin().delay(0.5).right(35).delay(0.5).open().delay(0.5)
        return self

    def rescuekit(self):
        self.close(105).delay(0.5).raisin().delay(0.5).left(45).delay(0.5).open().delay(0.5)
        return self