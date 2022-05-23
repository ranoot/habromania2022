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
see_line = (1).to_bytes(1, "big")

claw.lower().open()


while True:
	ser.write(b'l' + see_line + b'\n')
	count = 0
	frame_org = cam.read()
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 170 , param2 = 30, maxRadius = 95)
	
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
					"dev": np.sum(std),
					"type": "Alive" if np.sum(std) >= 100 else "Dead"
				})

	print(count)

	if len(balls) > 0:
		biggest_ball = max(balls, key = lambda a: a["r"])
		rotation = math.atan((biggest_ball["x"] - frame_org.shape[1]/2)/ (frame_org.shape[0] - biggest_ball["y"]))
		# print("biggest ball:", biggest_ball["r"], "x:", biggest_ball["x"], "y:", biggest_ball["y"], "type:", biggest_ball["type"])
		print(balls)
		if biggest_ball["r"] >= 68 and biggest_ball["type"] == "Dead":
			claw.dead()
		elif biggest_ball["r"] >= 40 and biggest_ball["type"] == "Alive":
			claw.alive()
		# elif np.sum(gray_org) <= 12000000:
		# 	# pickup_ball()
		# 	print("pickup")
		elif biggest_ball["r"] >= 20:
			print(balls)
			ser.write(b's' + struct.pack('f', 0.5) + b'\n') #TODO: change values
			ser.write(b'r' + struct.pack('f', rotation) + b'\n')
		else:
			print(balls)
			ser.write(b's' + struct.pack('f', 1) + b'\n')
			ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	
			
				


	