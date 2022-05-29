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
		self.__change_res = False
		self.__res = resolution
		self.frame = None
		self.stop = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		time.sleep(5)
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			if not self.stop:
				if self.__change_res:
					self.__change_res = False
					self.camera.set(cv.CAP_PROP_FRAME_WIDTH, self.__res[1])
					self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, self.__res[0])
					time.sleep(3)
				else:
					ret, frame = self.camera.read()
					if ret:
						self.frame = frame
			else:
				self.camera.release()
				time.sleep(2.5)
				return

	def change_res(self, resolution=(240, 320)):
		self.__change_res = True
		self.__res = resolution
		return

	def read(self):
		# return the frame most recently read
		return self.frame

	def kill(self):
		self.stop = True