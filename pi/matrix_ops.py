import math
import numpy as np
import cv2 as cv

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
	# print(l_dim["h"] - crop_h)
	for i in range(0, image_h):
		xs = np.append(xs, ((i/image_h) ** degree))
	return xs[:, np.newaxis]

def w_scaled_vector(image_w, degree):
	xs = np.array([])
	for i in range(-int(image_w/2), int(image_w/2)):
		xs = np.append(xs, (2 * i / image_w) ** degree)
	pass

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
	# ys = ys[::-1]
	x, y = 0, 0
	for index, i in enumerate(xs):
		x += (index - len(xs)/2) * i/255
	for index, j in enumerate(ys):
		y += (len(ys) - index) * j/255
	return (x, y)

# def d_vector(masked_img, h_scaled_v = None, w_scaled_v = None, scaled_m = None):
# 	masked_img = masked_img.astype(float)
# 	if scaled_m is None:
# 		if h_scaled_v is None:
# 			raise ValueError()
# 		masked_img *= h_scaled_v
# 		if w_scaled_v:
# 			masked_img *= w_scaled_v
# 	else: 
# 		masked_img *= scaled_m

# 	xs = np.sum(masked_img, axis = 0)
# 	ys = np.sum(masked_img, axis = 1)
# 	ys = ys[::-1]
# 	x, y = 0, 0
# 	for index, i in enumerate(xs):
# 			index -= len(xs)/2
# 			x += index*i
# 			# print(index)
# 	for index, j in enumerate(ys):
# 			y += index*j
# 	return (x, y)

def segment_by_angle_kmeans(lines, k=2, **kwargs):
	"""
	Group lines by their angle using k-means clustering.

	Code from here:
	https://stackoverflow.com/a/46572063/1755401
	"""

	# Define criteria = (type, max_iter, epsilon)
	default_criteria_type = cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER
	criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))

	flags = kwargs.get('flags', cv.KMEANS_RANDOM_CENTERS)
	attempts = kwargs.get('attempts', 10)

	# Get angles in [0, pi] radians
	angles = np.array([line[0][1] for line in lines])

	# Multiply the angles by two and find coordinates of that angle on the Unit Circle
	pts = np.array([[np.cos(2*angle), np.sin(2*angle)] for angle in angles], dtype=np.float32)

	# Run k-means
	# if sys.version_info[0] == 2:
	# 	# python 2.x
	# 	ret, labels, centers = cv.kmeans(pts, k, criteria, attempts, flags)
	# else: 
	# 	# python 3.x, syntax has changed.
	labels, centers = cv.kmeans(pts, k, None, criteria, attempts, flags)[1:]

	labels = labels.reshape(-1) # Transpose to row vector

	# Segment lines based on their label of 0 or 1
	segmented = defaultdict(list)
	for i, line in zip(range(len(lines)), lines):
		segmented[labels[i]].append(line)

	segmented = list(segmented.values())
	# print("Segmented lines into two groups: %d, %d" % (len(segmented[0]), len(segmented[1])))

	return segmented


def intersection(line1, line2):
	"""
	Find the intersection of two lines 
	specified in Hesse normal form.

	Returns closest integer pixel locations.

	See here:
	https://stackoverflow.com/a/383527/5087436
	"""

	rho1, theta1 = line1[0]
	rho2, theta2 = line2[0]
	A = np.array([[np.cos(theta1), np.sin(theta1)],
				  [np.cos(theta2), np.sin(theta2)]])
	b = np.array([[rho1], [rho2]])
	x0, y0 = np.linalg.solve(A, b)
	x0, y0 = int(np.round(x0)), int(np.round(y0))

	return [[x0, y0]]


def segmented_intersections(lines):
	"""
	Find the intersection between groups of lines.
	"""

	intersections = []
	for i, group in enumerate(lines[:-1]):
		for next_group in lines[i+1:]:
			for line1 in group:
				for line2 in next_group:
					intersections.append(intersection(line1, line2)) 

	return intersections

def get_black_endpoint(thresh):
	"""
		returns max and min, 
		min is more of a loose enclose use max for finding end of line
	"""
	# 0 => white, 1 => black
	empty_index,  = np.where(np.sum(thresh, axis=1) == 0)
	# This tries to find columns that are all white
	bl_index_max = max(empty_index)	if empty_index.size > 0 else 0
	bl_index_min = min(empty_index)	if empty_index.size > 0 else 0
	return bl_index_max, bl_index_min

def normalized_var(gray_org, bl_index:int, epsilon:int, mask=None):
	roi = gray_org[(bl_index - epsilon):(bl_index + epsilon), :]
	if roi.size > 0:
		if mask is None:	
			mean, std = cv.meanStdDev(roi)
			return ((std[0][0]) ** 2)/mean[0][0]
		else:
			roi_mask = mask[(bl_index - epsilon):(bl_index + epsilon), :]
			if roi_mask.size > 0:
				mean, std = cv.meanStdDev(roi, mask=roi_mask)
				return ((std[0][0]) ** 2)/mean[0][0]
	
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))