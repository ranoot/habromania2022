import cv2 as cv
import numpy as np
import math
import serial
import struct
import time

def scaled_vector(index):
	global dim, boundaries, crop_h
	xs = np.array([])
	# print(dim["h"] - crop_h)
	for i in range(0, dim["h"] - crop_h):
		xs = np.append(xs, ((i/(dim["h"] - crop_h)) ** index))
	return xs[:, np.newaxis]

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

# shape => (height, width)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

time.sleep(3)

dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((0,0,0), (0,0,0))
}
crop_h = 30
green_square_crop_h = 50
delta = 12 * math.pi/180

k_p = 1.15
cam = cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

# time.sleep(3)
ret, frame = cam.read()

scaled_v = scaled_vector(1)

timeout = 2   # [seconds]



while True:
	ret, frame = cam.read()

	frame = frame[:, crop_h:(frame.shape[0])]
	frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
	gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)

	gray = gray[green_square_crop_h:(gray.shape[1]), :]
	edges = cv.Canny(gray,100, 200)
	lines = cv.HoughLines(edges,1,np.pi/180,70)

	dv = d_vector(thresh, scaled_v)
	deviation = math.atan(dv[0]/dv[1])

	print(deviation)

	if abs(deviation) <= delta:
		ser.write(b's' + struct.pack('f', 0.0000000000) + b'\n')
		ser.write(b'r' + struct.pack('f', 0.0000000000) + b'\n')
		break
		# ser.write(b's' + struct.pack('f', 0.0) + b'\n')
		# ser.write(b'r' + struct.pack('f', 0.0) + b'\n')
		# ser.write(b's' + struct.pack('f', 0.0) + b'\n')
		# ser.write(b'r' + struct.pack('f', 0.0) + b'\n')
		# break
