import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from collections import defaultdict
from servo import Claw, Servo

dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((41,50,50), (69,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 10
b_crop_h = 0
gs_crop_h =40
b_gs_crop_h = 10

b_w_thresh = 40 #
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

b_w_thresh = 40 
k_p = 1

right_turn_rotation = 0.75
min_area = 100

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

claw = Claw(ser)

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

claw.lower().open()
time.sleep(3)
see_line = (1).to_bytes(1, "big")
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()

while True:
	# continue
	frame_org = cam.read()
	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	st = see_entry(gray_org, thresh, green, 43, 15000000, 180, 20)
	gt = see_exit(green, 120, 90, 10)
	print(np.sum(green))
	cv.imwrite("green.png", green)
	if np.sum(green) > 10000:
		
		print("green")
		Moments = cv.moments(green)
		xGreen = int(Moments["m10"] / Moments["m00"])
		yGreen = int(Moments["m01"] / Moments["m00"])
		finalx = xGreen - green.shape[1]/2
		finaly = green.shape[0] - yGreen
		if xGreen!=0:
			deviation = math.atan(finalx/finaly)
			#sus?
			deviation *= k_p
			deviation = deviation if deviation <= 1 else 1
			deviation = deviation if deviation >= -1 else -1
			rotation = deviation
	else:
		print("no green")
		rotation = 1
	if st:
		ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
		print("hi1")
	elif gt:
		ser.write(b's' + struct.pack('f', 1) + b'\n')
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
		break

	ser.write(b'l' + see_line + b'\n')
	ser.write(b'r' + struct.pack('f', rotation) + b'\n')

print("done")
# while True:
#     frame_org = cam.read()
#     claw.write_claw(Servo.COMPART, 180-20)
	# time.sleep(1)
	# claw.write_claw(Servo.COMPART, 180-55)
	# time.sleep(1)
	# ser.write(b'l' + see_line + b'\n')
	# ser.write(b's' + struct.pack('f', 0) + b'\n')
	# mean, std = cv.meanStdDev(frame_org)
	# print("done")
	# claw.lower().close(95)

	# print(std)

	# claw.dead()
	# claw.lower().open().delay(0.5)
	# claw.alive()
	# claw.lower().open().delay(0.5)

	# time.sleep(0.5)
	# claw.rescuekit()
	# claw.lower().open().delay(0.5)
	# time.sleep(0.5)
	# claw.tilt_compartment()
	# time.sleep(1)