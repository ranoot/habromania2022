import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from servo import Claw, Servo

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

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

dz_min = 10000

claw = Claw(ser)
# s_tape_minmax = {"w": (120, 160), "h": (0, 30)}
time.sleep(3)
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()

def scaled_vector(image_h, degree):
	xs = np.array([])
	# shape => (height, width)
	# print(dim["h"] - crop_h)
	for i in range(0, image_h):
		xs = np.append(xs, ((i/image_h) ** degree))
	return xs[:, np.newaxis]

dep_scale = scaled_vector(dim["h"], 0)

def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop):
	"""Ensure that max(thresh) = 255!"""
	not_green = cv.bitwise_not(green)
	not_black = cv.bitwise_not(thresh)
	st_mask = cv.bitwise_and(not_black, not_green)

	st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
	gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]
	_, std = cv.meanStdDev(gray_org ,mask = st_mask)
	# print(std, np.sum(st_mask))
	return (std[0][0] > min_std_dev and np.sum(st_mask) > min_mask_sum)

def see_exit(green, min_green, t_crop, b_crop):
	green_c = green[t_crop:(green.shape[0] - b_crop), :]
	# print(np.sum(green_c))
	return (np.sum(green_c)) >= min_green*255

def d_vector(masked_img, h_scaled_v = None, w_scaled_v = None, scaled_m = None):
	masked_img = masked_img.astype(float)
	if scaled_m is None:
		if h_scaled_v is None:
			raise ValueError()
		masked_img *= h_scaled_v
		if w_scaled_v:
			masked_img *= w_scaled_v
	else: 
		masked_img *= scaled_m

	xs = np.sum(masked_img, axis = 0)
	ys = np.sum(masked_img, axis = 1)
	ys = ys[::-1]
	x, y = 0, 0
	for index, i in enumerate(xs):
			index -= len(xs)/2
			x += index*i
			# print(index)
	for index, j in enumerate(ys):
			y += index*j
	return (x, y)

def see_deposit(thresh):
	contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	# Go through each contour...
	for contour in contours:
		rect = cv.boundingRect(contour)
		x, y, w, h = rect
		print(w)
		claw.raisin().delay(1).reset_compartment()
		if w > 150 and h > 90:
			print("commence deposit")
			ser.write(b's' + struct.pack('f', 0) + b'\n')
			ser.write(b'r' + struct.pack('f', 0) + b'\n')
			claw.write_claw(Servo.ARM, 90).close(95).delay(0.5)
			claw.tilt_compartment()
			claw.open().delay(0.5)
			claw.raisin().delay(0.5)
			claw.reset_compartment()
			time.sleep(1)
			claw.lower().open()
		elif w > 100:
			ser.write(b's' + struct.pack('f', 0.8) + b'\n')
			ser.write(b'r' + struct.pack('f', 0) + b'\n')
			print("straight")
		else:
			ser.write(b's' + struct.pack('f', 1) + b'\n')
			ser.write(b'r' + struct.pack('f', 0) + b'\n')
			print("roomba")
		print("h:", h)

ser.write(b'e' + (1).to_bytes(1, "big") + b'\n')
claw.close(95).raisin()
while True:
	# continue
	
	frame_org = cam.read()
	# cv.imwrite("org.png", frame_org)
	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	dv = d_vector(thresh, dep_scale)
	dev = math.atan(dv[0]/dv[1])
	# cv.imwrite("pppoopoo.png", thresh)
	ser.write(b's' + struct.pack('f', 0.8) + b'\n')
	ser.write(b'r' + struct.pack('f', dev) + b'\n')
	# see_deposit(thresh)
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
	
	# break
