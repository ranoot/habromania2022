import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream

#* Constants

class Servo(Enum):
	ARM = 0
	RIGHT = 1
	LEFT = 2
	COMPART = 3


class GreenSquare(Enum):
	RIGHT = 1
	LEFT = 2
	DOUBLE = 3

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
dim = {"w": 160, "h": 120}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((50,50,50), (80,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 30
b_crop_h = 10
gs_crop_h = 40
b_gs_crop_h = 10

b_w_thresh = 40 #
k_p = 1.25

#* the region of image to consider whether robot is "done" with the particular green square
gs_lt_roi = 30

rk_crop_h = 30
b_rk_crop_h = 0

obs_crop_h = 80 # height of image also affected by "crop_h" and "b_crop_h"
line_min = 3000

epsilon = 30 * math.pi/180
delta = 10 * math.pi/180

d_green_square_min = 1000000 #TODO: Determine Actual Min, this is just a guess (I was very incorrect)
s_green_square_min = 5000

right_turn_rotation = 0.75
min_area = 25

st_crop_h = 80
st_b_crop_h = 10

#* Functions
def scaled_2d_matrix(height, width, degree1, degree2, degree3):
	zs = np.zeros((height, width), dtype = float)
	max_v_len = math.sqrt((height ** 2 + (width/2) ** 2))
	with np.nditer(zs, op_flags=['readwrite'], flags=['multi_index']) as it:
		for a in it:
			j, i = it.multi_index
			i -= width/2
			j = height - j
			v_len = math.sqrt((j ** 2 + i ** 2))
			len_scale = (1 - (v_len/max_v_len)) ** degree1
			y_scale = (1 - (j/height)) ** degree2
			x_scale = (2 * abs(i)/width) ** degree3

			a[...] = len_scale * y_scale * x_scale
			# a[...] = (1 - (j/height)) ** degree
	return zs/np.max(zs)

def scaled_vector(image_h, degree):
	xs = np.array([])
	# shape => (height, width)
	# print(dim["h"] - crop_h)
	for i in range(0, image_h):
		xs = np.append(xs, ((i/image_h) ** degree))
	return xs[:, np.newaxis]

def w_scaled_vector(image_w, degree):
	xs = np.array([])
	for i in range(-int(image_w/2), int(image_w/2)):
		xs = np.append(xs, (2 * i / image_w) ** degree)
	pass

scaled_v = scaled_vector(dim["h"] - b_crop_h - crop_h, 1.3)
w_scaled_v = w_scaled_vector(dim["w"], 10)
gs_scaled_v = scaled_vector(dim["h"] - b_gs_crop_h - gs_crop_h, 0)
scaled_m = scaled_2d_matrix(dim["h"] - crop_h - b_crop_h, dim["w"], 2.5, 0.4, 0.2)
cv.imwrite("out.png", scaled_m * 255)

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

def h_line_filter(line):
	rho, theta = line[0]
	if theta <= math.pi/2 + epsilon and theta >= math.pi/2 - epsilon:
		return True
	else:
		return False

def y_value(line, x_value):
	rho, theta = line[0]
	return (rho - x_value*math.cos(theta))/(math.sin(theta))

l = 1
def green_square(lines, g_frame):
	global l
	if lines is None:
		return
	f_lines = list(filter(h_line_filter, lines))
	width = g_frame.shape[1]
	# print(True if len(f_lines) > 0 else False)
	if len(f_lines) > 0:
		y_intercepts = [int(y_value(line, 0)) for line in f_lines]
		width_intercepts = [int(y_value(line, width)) for line in f_lines]

		mask = np.ones(g_frame.shape[:2], dtype=np.uint8)
		useless_pixels = np.array([
			[0, 0], 
			[width, 0], 
			[width, max(width_intercepts)],
			[0, max(y_intercepts)]
		])
		
		cv.fillPoly(mask, pts =[useless_pixels], color=0)
		# print(mask.shape)
		# print(frame.shape)
		roi = cv.bitwise_and(g_frame, g_frame, mask=mask)	
		# cv.imwrite("out.png", roi)
		# roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
		# green = cv.inRange(roi, boundaries["green"][0], boundaries["green"][1])
		contours,hierarchy = cv.findContours(roi, 1, 2)
		
		green_vector = d_vector(roi, h_scaled_v=gs_scaled_v)
		# print(green_vector)
		# cv.imwrite("out.png", green)
		g_sum = np.sum(roi)
		# print(g_sum)
		# print("hello")
		# if g_sum > s_green_square_min:
		# print(len(contours))
		f_contours = list(filter(lambda cnt: cv.contourArea(cnt) > min_area, contours))
		# print(len(contours))
		if len(f_contours) == 2:
			return GreenSquare.DOUBLE
		# else:
		elif len(f_contours) == 1:
			# Cropping isn't perfect and some green still remains on the screen
			# this is to prevent the "specks" of green to count as a green square

			# cv.imwrite(f"out{l}.png", green)
			# print(f"dv{l} =", green_vector)
			l += 1
			if green_vector[0] < 0:
				return GreenSquare.LEFT
			elif green_vector[0] > 0:
				return GreenSquare.RIGHT
	# return (None, 0)

def turn(rotation, timeout):
	timeout_start = time.time()
	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	while time.time() < timeout_start + timeout:
		pass
	ser.write(b'r' + struct.pack('f', 0) + b'\n')

cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
# cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
# cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

# out = cv.VideoWriter('outpy.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (dim["w"],dim["h"]))
# record_start = time.time()
# time.sleep(3)

ser.write(b's' + struct.pack('f', 1.0) + b'\n')
curr = None
start_time = 0
journey = []
retreat = False
while True:
	# continue
	frame_org = cam.read()
	gs_frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green_org = cv.inRange(gs_frame_org, boundaries["green"][0], boundaries["green"][1])
	# print(ret)
	# frame1 = frame[:, crop_h:(frame.shape[0])]
	# frame_org = cv.rotate(frame_org, cv.ROTATE_90_CLOCKWISE)
	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	green = green_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	blue_frame = frame_org[rk_crop_h:(frame_org.shape[0] - b_rk_crop_h), :]
	# print("gjiorjgi0owjri0o")

	gs_frame_gray = cv.cvtColor(gs_frame, cv.COLOR_BGR2GRAY)


	

	# not_green = cv.bitwise_not(green)
	# gs_frame_gray = cv.bitwise_and(gs_frame_gray, gs_frame_gray, mask=not_green)

	edges = cv.Canny(gs_frame_gray,100, 200)
	lines = cv.HoughLines(edges,1,np.pi/180,40)
	
	rotation = 0

	# print(curr)
	if curr:
		green = green[gs_lt_roi:(green.shape[0]) ,:]
		g_sum = np.sum(green)
		if curr == GreenSquare.DOUBLE:
			if (time.time() - start_time) < 2.5:
				rotation = 1
			else:
				curr = None
		else:
			if g_sum > 50:
				ser.write(b's' + struct.pack('f', 0.65) + b'\n')
				if curr == GreenSquare.RIGHT:
					rotation = right_turn_rotation
				elif curr == GreenSquare.LEFT:
					rotation = -right_turn_rotation
			else: 
				ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				curr = None
	else:
		curr = green_square(lines, green)
		start_time = time.time()

		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		
		#* to tell the robot when to stop when it sees an obstacle
		obs_frame = thresh[obs_crop_h:thresh.shape[0], :]
		see_line = np.sum(obs_frame) > line_min
		ser.write(b'l' + int(see_line).to_bytes(1, "big") + b'\n')
		
		#* detecting the silver tape
		st_green_mask = cv.bitwise_not(green_org[crop_h:(frame_org.shape[0] - b_crop_h), :])
		st_green_mask = st_green_mask[st_crop_h:(st_green_mask.shape[0] - st_b_crop_h), :]
		# mean, std = cv.meanStdDev(gray ,mask = cv.bitwise_and(cv.bitwise_not(thresh), st_green_mask))
		# t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
		
		#* Scaling based on the amount of white in the image (abit hacky)
		white_scale = (((np.sum(cv.bitwise_not(thresh * 255))/(dim["w"] * dim["h"] * 255))) ** 1.5 ) * 1.2 + 0.2 #the magical number
		white_scale = white_scale if white_scale >= 1 else 1
		white_scale = 2 if white_scale >= 2 else white_scale

		#* Main vector line-tracking logic
		dv = d_vector(thresh, scaled_m = scaled_m)
		# dv = d_vector(thresh, h_scaled_v = scaled_v, w_scaled_v = w_scaled_v)
		deviation = math.atan(dv[0]/dv[1])

		deviation *= k_p * white_scale #** 10
		deviation = deviation if deviation <= 1 else 1
		deviation = deviation if deviation >= -1 else -1
		print(deviation)
		rotation = deviation

		
		# print(b'l' + see_line + b'\n')

	# blue_frame = cv.GaussianBlur(blue_frame, (5, 5), 0)

	# hsv = cv.cvtColor(blue_frame, cv.COLOR_BGR2HSV)
	# maskBlue = cv.inRange(hsv, boundaries["blue"][0], boundaries["blue"][1])
	# # maskBlue = cv.erode(maskBlue, None, iterations=2)
	# # maskBlue = cv.dilate(maskBlue, None, iterations=2)

	# if np.sum(maskBlue) >= 2000000:
	# 	ser.write(b's' + struct.pack('f', 0) + b'\n')
	# 	time.sleep(2)
	# 	print(journey)
	# 	retreat = True
	# elif np.sum(maskBlue) > 10000:
	# 	if not retreat:
	# 		print("blue")
	# 		Moments = cv.moments(maskBlue)
	# 		xBlue = int(Moments["m10"] / Moments["m00"])
	# 		yBlue = int(Moments["m01"] / Moments["m00"])
	# 		finalx = xBlue - frame.shape[1]/2
	# 		finaly = frame.shape[0] - yBlue
	# 		if xBlue!=0:
	# 			deviation = math.atan(finalx/finaly)
	# 			#sus?
	# 			deviation *= k_p
	# 			deviation = deviation if deviation <= 1 else 1
	# 			deviation = deviation if deviation >= -1 else -1
	# 			rotation = deviation
	# 			journey.append(deviation)

	
	# if retreat:
	# 	if len(journey)>0:
	# 		ser.write(b's' + struct.pack('f', -1) + b'\n')
	# 		print(len(journey))
	# 		rotation = journey[len(journey) - 1]
	# 		journey.pop()
	# 	else:
	# 		ser.write(b's' + struct.pack('f', 1) + b'\n')
	# 		retreat = False

	# print("r", rotation)

	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	# if (time.time() - record_start > 10):
	# 	break

out.release()
cam.release()
cv.destroyAllWindows()
