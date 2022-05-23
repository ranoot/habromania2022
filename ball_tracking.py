import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from servo import Claw, Servo

dim = {"w": 320, "h": 240}

cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(3)

claw = Claw(ser)
see_line = True

while True:
	ser.write(b'l' + see_line + b'\n')
	count = 0
	frame_org = cam.read()
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 170 , param2 = 20, maxRadius = 120)

	balls = []
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2):
				count += 1
				mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
				# print(type(x), type(y))
				mask = cv.circle(mask, (int(x),int(y)), int(r), 255, -1)
				cv.imwrite("out.png", mask)
				mean, std = cv.meanStdDev(frame_org, mask=mask)
				balls.append({
					"x": x,
					"y": y,
					"r": r,
					"dev": np.sum(std)
				})

	print(count)
	claw.lower().delay(0.5).open().delay(0.5)

	if len(balls) > 0:
		biggest_ball = max(balls, key = lambda a: a["r"])
		rotation = math.atan((biggest_ball["x"] - frame_org.shape[1]/2)/ (frame_org.shape[0] - biggest_ball["y"]))
		print(biggest_ball["r"])
		if biggest_ball["r"] >= 70:
			print(biggest_ball["r"])
			ser.write(b's' + struct.pack('f', 0) + b'\n')
			claw.lower().delay(0.5).close(95).delay(0.5)
			claw.raisin().delay(0.5).open().delay(0.5)
		else:
			print(balls)
			ser.write(b's' + struct.pack('f', 1) + b'\n')
			ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	
			
				


	