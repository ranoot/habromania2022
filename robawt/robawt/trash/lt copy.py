import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from displacement_est import *

#* Constants
x, y, theta = 0, 0, 0

class GreenSquare(Enum):
	RIGHT = 1
	LEFT = 2
	DOUBLE = 3

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
dim = {"w": 160, "h": 120}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((41,70,70), (69,190,190)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 20
b_crop_h = 0
gs_crop_h = 80
b_gs_crop_h = 0

b_w_thresh = 70 #
#* the region of image to consider whether robot is "done" with the particular green square
gs_lt_roi = 50

epsilon = 30 * math.pi/180
delta = 10 * math.pi/180

d_green_square_min = 1000000 #TODO: Determine Actual Min, this is just a guess (I was very incorrect)
s_green_square_min = 5000

right_turn_rotation = 0.75
min_area = 200
#* Functions


def scaled_vector(image_h, degree):
	xs = np.array([])
	image_h = image_h # shape => (height, width)
	# print(dim["h"] - crop_h)
	for i in range(0, image_h):
		xs = np.append(xs, ((i/image_h) ** degree))
	return xs[:, np.newaxis]

scaled_v = scaled_vector(dim["h"] - b_crop_h - crop_h, 1.3)
gs_scaled_v = scaled_vector(dim["h"] - b_gs_crop_h - gs_crop_h, 0)

def d_vector(masked_img, scaled_v):
		masked_img = masked_img.astype(float)
		masked_img *= scaled_v

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
		green_vector = d_vector(roi, gs_scaled_v)
		# print(green_vector)
		# cv.imwrite("out.png", green)
		g_sum = np.sum(roi)
		# print(g_sum)
		# print("hello")
		# if g_sum > s_green_square_min:
		f_contours = list(filter(lambda cnt: cv.contourArea(cnt) > min_area, contours))
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

k_p = 1.1
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
# cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
# cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

begin = time.time()

# time.sleep(3)

ser.write(b's' + struct.pack('f', 1.0) + b'\n')
curr = None
start_time = 0
loop_time_start = 0
while True:
	# continue
	frame_org = cam.read()
	green_org = cv.inRange(cv.cvtColor(frame_org, cv.COLOR_BGR2HSV), boundaries["green"][0], boundaries["green"][1])
	# print(ret)
	# frame1 = frame[:, crop_h:(frame.shape[0])]
	# frame_org = cv.rotate(frame_org, cv.ROTATE_90_CLOCKWISE)
	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	green = green_org[gs_crop_h:(green_org.shape[0] - b_gs_crop_h), :]
	gs_frame_gray = cv.cvtColor(gs_frame, cv.COLOR_BGR2GRAY)

	edges = cv.Canny(gs_frame_gray,100, 200)
	lines = cv.HoughLines(edges,1,np.pi/180,80)
	
	rotation = 0
	speed = 1

	print(curr)
	if curr:
		g_sum = np.sum(green)
		if curr == GreenSquare.DOUBLE:
			if (time.time() - start_time) < 2:
				rotation = 1
				speed = 1
			else:
				curr = None
		else:
			green = green[gs_lt_roi:(green.shape[0]) ,:]
			
			# print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaaa")
			if g_sum > 0:
				speed = 0.65
				ser.write(b's' + struct.pack('f', 0.65) + b'\n')
				if curr == GreenSquare.RIGHT:
					rotation = right_turn_rotation
				elif curr == GreenSquare.LEFT:
					rotation = -right_turn_rotation
			else: 
				speed = 1
				ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				curr = None
	else:
		curr = green_square(lines, green)
		start_time = time.time()

		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		# t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
		dv = d_vector(thresh, scaled_v)
		deviation = math.atan(dv[0]/dv[1])

		# cv.imwrite("out.png", thresh*255)

		deviation *= k_p
		deviation = deviation if deviation <= 1 else 1
		deviation = deviation if deviation >= -1 else -1
	
		rotation = deviation

	frameBlue = cv.GaussianBlur(frame, (5, 5), 0)

	hsv = cv.cvtColor(frameBlue, cv.COLOR_BGR2HSV)
	maskBlue = cv.inRange(hsv, boundaries["blue"][0], boundaries["blue"][1])
	maskBlue = cv.erode(maskBlue, None, iterations=2)
	maskBlue = cv.dilate(maskBlue, None, iterations=2)

	if np.sum(maskBlue) > 15000:
		Moments = cv.moments(maskBlue)
		xBlue = int(Moments["m10"] / Moments["m00"])
		yBlue = int(Moments["m01"] / Moments["m00"])
		finalx = xBlue - frame.shape[1]/2
		finaly = frame.shape[0] - yBlue
		if xBlue!=0:
			deviation = math.atan(finalx/finaly)
			#sus?
			deviation *= k_p
			deviation = deviation if deviation <= 1 else 1
			deviation = deviation if deviation >= -1 else -1
			rotation = deviation

	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	delta_t = time.time() - loop_time_start
	x_p, y_p, theta_p = displacement_vector(speed, rotation, delta)
	x += x_p
	y += y_p
	theta += theta_p
	print(x_p, y_p, theta_p)
	loop_time_start = time.time() 

	if (time.time() - begin >= 5):
		ser.write(b's' + struct.pack('f', 0) + b'\n')
		print(x, y, theta)
		break
	# cv.imshow('frame', frame_org)
	# if cv.waitKey(1) == ord('q'):
	# 	break

	# line = ser.readline().decode('utf-8').rstrip()
	# time.sleep(0.5)
	# print(line)
	# ser.flush()
	# ser.flush()
	# line = ser.readline().decode('utf-8').rstrip()
	# print(f"back: {line}")
	#TODO: Test if ser.flush() works, print and read output thru Serial from another arduino, 

	# print(bytearray(struct.pack("f", (deviation * k_p) )) + b"$")
#        print(deviation*180/math.pi)
		
# ser.write(b'w')
# cv.imshow("bruh", thresh)
# cam.release()
cv.destroyAllWindows()
