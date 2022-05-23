from constants import *
import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from robot import *

#hsv(106, 77%, 72%)
# shape => (height, width)
# ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

cam = cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

# time.sleep(3)
ret, frame = cam.read()

while True:
	ret, frame = cam.read()

	frame = frame[:, crop_h:(frame.shape[0])]
	frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
	gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)

	gray = gray[green_square_crop_h:(gray.shape[1]), :] 
	# crops image since horizon seems to produce a lot of horizontal Hough Lines
	edges = cv.Canny(gray,100, 200)
	lines = cv.HoughLines(edges,1,np.pi/180,70)

	dv = d_vector(thresh, scaled_v)
	deviation = math.atan(dv[0]/dv[1])

	deviation *= k_p
	deviation = deviation if deviation <= 1 else 1
	deviation = deviation if deviation >= -1 else -1

	gs = green_square(lines, gray, frame)

	print(gs)
	break
	# ser.write(b'r' + struct.pack('f', deviation) + b'\n')
	# line = ser.readline().decode('utf-8').rstrip()
	#TODO: Test if ser.flush() works, print and read output thru Serial from another arduino, 
		
# ser.write(b'w')
# cv.imshow("bruh", thresh)
# cam.release()
cv.destroyAllWindows()
