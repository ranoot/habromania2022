from threading import Thread
import cv2 as cv
import math
import time
import numpy as np

class VideoStream:
	def __init__(self, resolution=(240, 320)):
		# initialize the camera and stream
		self.camera = cv.VideoCapture(0)
		self.camera.set(cv.CAP_PROP_FRAME_WIDTH, resolution[1])
		self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[0])

		self.frame = None

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		time.sleep(5)
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			ret, frame = self.camera.read()
			if ret:
				self.frame = frame

	def read(self):
		# return the frame most recently read
		return self.frame