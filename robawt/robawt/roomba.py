import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from servo import Claw, Servo

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, write_timeout=10)
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
claw = Claw(ser)
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()

def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop):
	"""Ensure that max(thresh) = 255!"""
	not_green = cv.bitwise_not(green)
	not_black = cv.bitwise_not(thresh)
	st_mask = cv.bitwise_and(not_black, not_green)

	st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
	gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]
	_, std = cv.meanStdDev(gray_org ,mask = st_mask)
	print(std, np.sum(st_mask))
	return (std[0][0] > min_std_dev and np.sum(st_mask) > min_mask_sum)

def see_exit(green, min_green, t_crop, b_crop):
	green_c = green[t_crop:(green.shape[0] - b_crop), :]
	# print(np.sum(green_c))
	return (np.sum(green_c)) >= min_green*255

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
	if st:
		ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
		print("hi1")
	if gt:
		ser.write(b'g' + (1).to_bytes(1, "big") + b'\n')
		print("hi2")
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27)
	balls = []
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2):
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
	t, thresh_b = cv.threshold(gray_org, 120, 255 ,cv.THRESH_BINARY_INV)
	majik = 30
	thresh_b = thresh_b[0:(thresh_b.shape[0]-90), majik:(thresh_b.shape[1]-majik)]
	cv.imwrite("pain.png",thresh_b)
	if len(balls) > 0:
		biggest_ball = max(balls, key=lambda b: b['r'])
		print(biggest_ball['x'] - dim["w"]/2, biggest_ball['r'])
		ser.write(b'r' + struct.pack('f', (-0.30 if (biggest_ball['x'] - dim["w"]/2) < 0 else 0.30)) + b'\n')
	else:
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
	print(np.sum(thresh_b))
	if np.sum(thresh_b) > 5000000:
		ser.write(b's' + struct.pack('f', 0.7) + b'\n')
		ser.write(b'p' + (1).to_bytes(1, "big") + b'\n')
		claw.close(95).delay(0.5)
		ser.write(b's' + struct.pack('f', 0) + b'\n')
		if np.sum(thresh) > 3000000:
			claw.dead()
		else:
			claw.alive()
		claw.lower().open()
		ser.write(b'p' + (0).to_bytes(1, "big") + b'\n')
		ser.write(b's' + struct.pack('f', 0.7) + b'\n')
	# print(st, gt)
	# if gt:
	# 	break
	# print(np.max(thresh))
	# cv.imwrite("out.png", cv.bitwise_and(cv.bitwise_and(cv.bitwise_not(thresh), not_green), gray_org))
	# cv.imwrite("out.png", thresh)
	# cv.imwrite("org.png", st_mask)
	# blur = cv.GaussianBlur(gray_org,(5,5),0)
	# circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 = 20)
	# plt.imshow(gray_org, cmap='gray')
	# edges = cv.Canny(blur,100, 200)
	# contours,hierarchy = cv.findContours(edges, 1, 2)
	# s_tape = []
	# for cnt in contours:
	# 	rect = cv.boundingRect(cnt)
	# 	x, y, w, h = rect
	# 	w_good = (w >= s_tape_minmax["w"][0] and w <= s_tape_minmax["w"][1])
	# 	h_good = (h >= s_tape_minmax["h"][0] and h <= s_tape_minmax["h"][1])
	# 	if (h_good and w_good):
	# 		s_tape.append(rect)
	# 		mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
	# 		cv.rectangle(mask, (int(x), int(y)), (int(x+w), int(y+h)), 255, -1)
	# 		# cv.imwrite("out.png", )
	# 		frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	# 		mean, std = cv.meanStdDev(frame_org, mask=mask)
	# 		print(std)
	# print(s_tape)
	# cv.imwrite("canny_out.png", edges)