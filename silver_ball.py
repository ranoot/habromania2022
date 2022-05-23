import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from matplotlib import pyplot as plt

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

right_turn_rotation = 0.75
min_area = 100

cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
while True:
	# continue
	frame_org = cam.read()

	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 = 13, maxRadius = 100)
	# plt.imshow(gray_org, cmap='gray')
	edges = cv.Canny(gray_org,100, 200)
	balls = []
	count = 0
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2):
				count += 1
				mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
				# print(type(x), type(y))
				mask = cv.circle(mask, (int(x),int(y)), int(r), 255, -1)
				cv.imwrite(f"out{count}.png", mask)
				# print(mask)
				mean, std = cv.meanStdDev(frame_org, mask=mask)
				balls.append(std)
				c = plt.Circle((x, y), r, fill=False, lw=3, ec='C1')
				plt.gca().add_patch(c)
	plt.gcf().set_size_inches((12, 8))
	plt.imshow(edges,cmap = 'gray')
	# plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
	plt.savefig("out.png")
	print("done")
	print(balls)
	print(count)
	break