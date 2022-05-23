from ast import While
import serial
import struct
import time
from servo import Servo, Claw

ser = serial.Serial('COM6', 9600, timeout=1)
time.sleep(3)
claw = Claw(ser)
deviation = 0
# ser.open()
while True:
    deviation += 0.001
    ser.write(b's' + struct.pack('f', deviation) + b'\n')
    ser.write(b'r' + struct.pack('f', deviation) + b'\n')
    print(deviation)
    # claw.lower().delay(1)
    # claw.write_claw(Servo.COMPART, (180-20)).delay(1).write_claw(Servo.COMPART, 180-55)
    if deviation >= 1:    
        break
# while True:
#     # claw.lower().close(95).delay(1).open()
#     ser.write(b's' + struct.pack('f', 1) + b'\n')
#     break
# while True:
#     deviation += 0.001
#     ser.write(b'r' + struct.pack('f', deviation) + b'\n')
# #     # print(b'l' + (1).to_bytes(1, 'big') + b'\n')
#     line = ser.readline().decode('utf-8').rstrip()
#     print(line)
# #     # print(deviation)
# #     if deviation >= 1:
# #         break
#     time.sleep(1)