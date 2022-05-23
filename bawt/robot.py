from ctypes.wintypes import DOUBLE
import serial
import time
import numpy as np
import cv2 as cv
from enum import Enum
import struct
from constants import *

def scaled_vector(index, crop_h):
	xs = np.array([])
	# print(dim["h"] - crop_h)
	for i in range(0, dim["h"] - crop_h):
		xs = np.append(xs, ((i/(dim["h"] - crop_h)) ** index))
	return xs[:, np.newaxis]

scaled_v = scaled_vector(1, crop_h)
scaled_v_g = scaled_vector(1, crop_h + green_square_crop_h)

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

def green_square(lines, gray, frame):
	if lines is not None:
		f_lines = list(filter(h_line_filter, lines))
		width = gray.shape[1]
		# print(True if len(f_lines) > 0 else False)
		if len(f_lines) > 0:
			y_intercepts = [int(y_value(line, 0)) for line in f_lines]
			width_intercepts = [int(y_value(line, width)) for line in f_lines]

			mask = np.ones(gray.shape, dtype=np.uint8)
			useless_pixels = np.array([
				[0, 0], 
				[width, 0], 
				[width, max(width_intercepts)],
				[0, max(y_intercepts)]
			])
			# print(gray.shape)
			# print(useless_pixels)
			cv.fillPoly(mask, pts =[useless_pixels], color=0)

			gs_frame = frame[green_square_crop_h:(gray.shape[1]), :]
			roi = cv.bitwise_and(gs_frame, gs_frame, mask=mask)	

			roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
			green = cv.inRange(roi, boundaries["green"][0], boundaries["green"][1])

			green_vector = d_vector(green, scaled_v_g)
			print(green_vector)
			
			if np.sum(green) > d_green_square_min:
				return GreenSquare.DOUBLE
			else:
				if green_vector[0] < 0:
					return GreenSquare.LEFT
				else:
					return GreenSquare.RIGHT

class Robot:
	__speed = 0
	__rotation = 0
	__curry_state = State.LINETRACK

	def __init__(self, port, baud_rate, cam):
		self.ser = serial.Serial(port, baud_rate, timeout=1)
		self.cam = cam

	def write_packet(self, start_char, val):
		if start_char == b'r' or start_char == b's':
			if not isinstance(val, float):
				return
			self.ser.write(start_char + struct.pack('f', val) + b"\n")

	def d_steer(self, speed, rotation):
		if not __speed == speed:
			self.write_packet(b's', speed)
			__speed = speed
		if not __rotation == rotation:
			self.write_packet(b'r', rotation)
			__rotation = rotation

	def turn(self, green_square):
		timeout = 0
		if green_square == GreenSquare.RIGHT or green_square == GreenSquare.LEFT:
			timeout = 2
			# right, left
		elif green_square == GreenSquare.DOUBLE:
			timeout = 5

		timeout_start = time.time()

		while time.time() < timeout_start + timeout:
			test = 0
			self.ser.write(b'r' + struct.pack('f', 1.0) + b'\n')
			if test == 5:
				break
			test -= 1

		while True:
			ret, frame = self.cam.read()

			frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
			gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
			t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
			thresh = thresh[crop_h:(thresh.shape[0]), :]

			dv = d_vector(thresh, scaled_v)
			deviation = math.atan(dv[0]/dv[1])
			
			if abs(deviation) <= delta:
				self.d_steer(0, 0)
				break