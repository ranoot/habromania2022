import cv2 as cv
import numpy as np
import math
from matrix_ops import normalized_var, constrain
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
# from servo import Claw, Servo

ser = serial.Serial('/dev/serial/by-id/usb-DFRobot_www.dfrobot.com__0042_75932313838351201101-if00', 9600, timeout=1, write_timeout=5)
dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((50,50,50), (80,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 10
b_crop_h = 0
gs_crop_h =90
b_gs_crop_h = 10

b_w_thresh = 45 #
#* the region of image to consider whether robot is "done" with the particular green square
gs_lt_roi = 50

rk_crop_h = 30
b_rk_crop_h = 0

obs_crop_h = 80 # height of image also affected by "crop_h" and "b_crop_h"
line_min = 3000

epsilon = 30 * math.pi/180
delta = 10 * math.pi/180

d_green_square_min = 1000000 #TODO: Determine Actual Min, this is just a guess (I was very incorrect)
s_green_square_min = 5000

right_turn_rotation = 0.75
min_area = 100

# s_tape_minmax = {"w": (120, 160), "h": (0, 30)}
time.sleep(3)
# claw = Claw(ser)
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()

while True:
	frame_org = cam.read()
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)

	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27)
	
	balls = []
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2 + 10): # Circle is probably bogus if it's centre is below half the frame
				# cv.imwrite("out.png", mask)
				# mean, std = cv.meanStdDev(frame_org, mask=mask)
				balls.append({
					"x": x,
					"y": y,
					"r": r,
					# "dev": np.sum(std)
				})

	#* Giving the robot a bias towards the ball
	ball_mask = np.full(gray_org.shape[:2], 255, dtype=np.uint8)
	if len(balls) > 0:
		biggest_ball = max(balls, key=lambda b: b['r'])
		#TODO Adjust this constant
		ball_dev = (biggest_ball['x'] - dim["w"]/2) * 0.01
		# print(ball_dev)
		ball_dev = constrain(ball_dev, -0.8, 0.8)
		ball_mask = cv.circle(ball_mask, (int(biggest_ball['x']),int(biggest_ball['y'])), int(biggest_ball['r']), 0, -1)
		ser.write(b'b' + (1).to_bytes(1, "big") + b'\n')
		ser.write(b'r' + struct.pack('f', ball_dev) + b'\n') 
	else: #TODO: Cover the cases that the bot supposed to keep turning
		ser.write(b'b' + (0).to_bytes(1, "big") + b'\n')
		ser.write(b'r' + struct.pack('f', 0) + b'\n')

	#* Detecting for silver tape
	silver_mask = cv.bitwise_and(ball_mask, cv.bitwise_not(thresh))
	st = ((n := normalized_var(gray_org, 30, 15, mask = silver_mask)) > 10)
	# print(n)
	if st:
		ser.write(b't' + (1).to_bytes(1, "big") + b'\n')

	#* Detecting (a) if you see the deposit (b) if a, then find the deviation of black centroid
	if (s := (np.sum(thresh)/255)) >= 2_300: #?
		ser.write(b'd' + (1).to_bytes(1, "big") + b'\n')

		black_M = cv.moments(thresh)
		black_cX = int(black_M["m10"] / black_M["m00"])

		dep_dev = (black_cX - dim['w']/2) * 0.007
		dep_dev = constrain(dep_dev, -0.6, 0.6)
		# print(s, dep_dev)

		ser.write(b'D' + struct.pack('f', dep_dev) + b'\n')
	else: 
		# print(s)
		ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')
		