import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from servo import Claw, Servo

dim = {"w": 160, "h": 120}
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


cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
# ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(3)

# claw = Claw(ser)
see_line = (1).to_bytes(1, "big")

def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop):
	"""Ensure that max(thresh) = 255!"""
	not_green = cv.bitwise_not(green)
	not_black = cv.bitwise_not(thresh)
	st_mask = cv.bitwise_and(not_black, not_green)

	st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
	gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]
	_, std = cv.meanStdDev(gray_org ,mask = st_mask)
	
	return (std[0][0] > min_std_dev and np.sum(st_mask) > min_mask_sum)

def see_exit(green, min_green, t_crop, b_crop):
	green_c = green[t_crop:(green.shape[0] - b_crop), :]
	# print(np.sum(green_c))
	return (np.sum(green_c)) >= min_green*255

# claw.lower().open()

# def pickup_ball():
# 	ser.write(b's' + struct.pack('f', 0) + b'\n')
# 	claw.lower().delay(0.5).close(95).delay(0.5)
# 	claw.raisin().delay(0.5).open().delay(0.5)
# 	claw.lower()

while True:
	# ser.write(b'l' + see_line + b'\n')
	count = 0
	frame_org = cam.read()
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	st = see_entry(gray_org, thresh, green, 42, 700000, 90, 10)
	gt = see_exit(green, 120, 90, 10)
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =20)
	cv.imwrite("out.png", cv.Canny(gray_org, 75, 150))
	balls = []
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2):
				count += 1
				mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
				# print(type(x), type(y))
				mask = cv.circle(mask, (int(x),int(y)), int(r), 255, -1)
				# cv.imwrite("out.png", mask)
				mean, std = cv.meanStdDev(frame_org, mask=mask)
				balls.append({
					"x": x,
					"y": y,
					"r": r,
					"dev": np.sum(std)
				})

	# print(count)
	print(len(balls))
	print(list(map(lambda b: b["dev"], balls)))

	if len(balls) > 0:
		biggest_ball = max(balls, key = lambda a: a["r"])
		sped = math.atan((biggest_ball["x"] - frame_org.shape[1]/2)/ (frame_org.shape[0] - biggest_ball["y"]))
		# print("biggest ball:", biggest_ball["r"], "x:", biggest_ball["x"], "y:", biggest_ball["y"], "type:", biggest_ball["type"])
		ser.write(b's' + struct.pack('f', sped) + b'\n')
		ser.write(b'r' + struct.pack('f', 1) + b'\n')
	
			
				


	