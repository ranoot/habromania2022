import cv2 as cv
import numpy as np

class Image:
	def __init__(self, image):
		self.image = image
		self.scaled_vector_store = {}

	def crop_top(self, crop_factor):
		return self.image[crop_factor:(self.image.shape[0]), :]

	def portrait(self):
		return cv.rotate(self.image, cv.ROTATE_90_CLOCKWISE)

	def grayscale(self):
		return cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

	def threshold(self):
		return (cv.threshold(self.image, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU))[1]

	def lines(self):
		edges = cv.Canny(self.image,100, 200)
		lines = cv.HoughLines(edges,1,np.pi/180,70)
		return lines
	
	def scaled_vector(self, degree):
		xs = np.array([])
		image_h = self.image.shape[0] # shape => (height, width)
		# print(dim["h"] - crop_h)
		for i in range(0, image_h):
			xs = np.append(xs, ((i/image_h) ** degree))
		return xs[:, np.newaxis]

	def d_vector(self, degree):
		self.image = self.image.astype(float) #TODO: move this checking logic to self.scaled_vector please
		if k := (*self.image.shape, degree) in self.scaled_vector_store:
			self.image *= self.scaled_vector_store[k]
		else: 
			self.image *= (s_v := self.scaled_vector(degree))
			self.scaled_vector_store[k] = s_v

		xs = np.sum(self.image, axis = 0)
		ys = np.sum(self.image, axis = 1)
		ys = ys[::-1]
		x, y = 0, 0
		for index, i in enumerate(xs):
				index -= len(xs)/2
				x += index*i
				# print(index)
		for index, j in enumerate(ys):
				y += index*j
		return (x, y)
