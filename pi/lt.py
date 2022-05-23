import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from collections import defaultdict
from const import *
from ctypes import Union
from matrix_ops import get_black_endpoint

#* Constants
# PRINT_DEVIATION = True
# PRINT_STATE = True
# PRINT_KIT = True 
# PRINT_FPS = True

class Servo(Enum):
	ARM = 0
	RIGHT = 1
	LEFT = 2
	COMPART = 3

class GreenSquare(Enum):
	RIGHT = 1
	LEFT = 2
	DOUBLE = 3

class MissionStage(Enum):
	LINE = 0
	EVAC = 1

ser = serial.Serial('/dev/serial/by-id/usb-DFRobot_www.dfrobot.com__0042_75932313838351201101-if00', 9600, timeout=1, write_timeout=5)

# element = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))

#* Functions
def write_speed(s: float):
	"""Write speed to arduino"""
	ser.write(b's' + struct.pack('f', s) + b'\n')

def write_rotation(r: float):
	"""Write rotation to arduino"""
	ser.write(b'r' + struct.pack('f', r) + b'\n')

def see_stop(red, t_crop, b_crop):
	red_c = red[t_crop:(red.shape[0] - b_crop), :]
	# lower mask (0-10)
	lower_red = np.array(boundaries["red_lower"][0])
	upper_red = np.array(boundaries["red_lower"][1])
	mask0 = cv.inRange(red_c, lower_red, upper_red)

	# upper mask (170-180)
	lower_red = np.array(boundaries["red_upper"][0])
	upper_red = np.array(boundaries["red_upper"][0])
	mask1 = cv.inRange(red_c, lower_red, upper_red)

	# join my masks
	mask = mask0+mask1
	return (np.sum(mask)) >= min_red

def h_line_filter(line):
	rho, theta = line[0]
	return theta <= math.pi/2 + epsilon and theta >= math.pi/2 - epsilon
	

def y_value(line, x_value):
	rho, theta = line[0]
	return (rho - x_value*math.cos(theta))/(math.sin(theta))

def x_value(line, y_value):
	rho, theta = line[0]
	return (rho - y_value*math.sin(theta))/(math.cos(theta))

def green_square(lines, g_frame, thresh):
	if g_frame.shape[1] != thresh.shape[1]:
		raise ValueError()
	if lines is None:
		return
	f_lines = list(filter(h_line_filter, lines))
	width = g_frame.shape[1]
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
		# get mask for region beneath black line

		roi = cv.bitwise_and(g_frame, g_frame, mask=mask) # get region below black line
		contours,hierarchy = cv.findContours(roi, 1, 2)

		# green_vector = d_vector(roi, h_scaled_v=gs_scaled_v) # which side of the frame the green is on

		black_M = cv.moments(thresh)
		black_cX = int(black_M["m10"] / black_M["m00"])

		# Cropping isn't perfect and some green still remains on the screen
		# this is to prevent the "specks" of green to count as a green square
		f_contours = list(filter(lambda cnt: cv.contourArea(cnt) > min_area, contours))
		# print(black_cX)
		if len(f_contours) == 2:
			return GreenSquare.DOUBLE
		elif len(f_contours) == 1:
			green_M = cv.moments(f_contours[0])
			green_cX = int(green_M["m10"] / green_M["m00"])
			if green_cX > black_cX:
				return GreenSquare.RIGHT
			elif green_cX < black_cX:
				return GreenSquare.LEFT
	# return (None, 0)

def n_green_square(g_frame, g_thresh):
	max_black = np.where(g_thresh.any(axis=0), g_thresh.argmax(axis=0), -1)
	valid_cols = []
	prev_i = -1
	for i, g_col in enumerate(g_frame.transpose()):
		if g_col.any():
			if g_col[max_black[i]:].any(): # there is green beneath black
				if not ((i - prev_i) <= 2 and (i - prev_i) >= 0 and prev_i >= 0):
					valid_cols.append([i])
				prev_i = i
				continue
		if ((i - prev_i) <= 2 and (i - prev_i) >= 0 and prev_i >= 0):
			valid_cols[len(valid_cols)-1].append(i)
	# if (g_frame.shape[1] - prev_i == 1 and prev_i >= 0):
	# 	valid_cols[len(valid_cols)-1].append(g_frame.shape[1] - 1)
	if (g_frame.shape[1] - prev_i == 1 and prev_i >= 0):
		valid_cols[len(valid_cols)-1].append(prev_i)
	# print(valid_cols)
	gs_list = list(filter(lambda xs: xs[-1] - xs[0] > 30, valid_cols))

	if len(gs_list) == 0 and len(gs_list) >= 3:
		return None 

	black_M = cv.moments(thresh)
	if black_M["m00"] == 0:
		return None
	black_cX = int(black_M["m10"] / black_M["m00"])

	gs_mp_list = list(map(lambda maxmin: (maxmin[0] + maxmin[1])/2, gs_list))
	if len(gs_list) == 1:
		if gs_mp_list[0] > black_cX:
			return GreenSquare.RIGHT
		elif gs_mp_list[0] < black_cX:
			return GreenSquare.LEFT
	elif len(gs_list) == 2:
		if black_cX > min(gs_mp_list) and black_cX < max(gs_mp_list):
			return GreenSquare.DOUBLE

prev_rotation = 0
freeze_rotation = False
def rotation_scale(thresh:np.ndarray, deviation: float) -> float:
	global prev_rotation, freeze_rotation
	# cv.imwrite("pp.png", thresh * 255)
	# print(np.argmax(thresh, axis=0))
	maxes = list(filter(lambda i: i != 0, np.argmax(thresh, axis=0)))
	if len(maxes) > 0:
		max_y = (thresh.shape[0] - min(maxes))/(thresh.shape[0])
		print("v:", max_y , "t:", max_y < 0.5 and abs(deviation) > 0.70)
		rotation = math.pow(abs(deviation), (max_y)) * (1 if deviation > 0 else -1)
		if freeze_rotation:
			print("F")
			if max_y > 0.5:
				freeze_rotation = False
			return prev_rotation
		else:
			if max_y < 0.5 and abs(rotation) > 0.80:
				prev_rotation = rotation
				freeze_rotation = True
			return rotation
		# print(max_y/thresh.shape[0], deviation, rotation* (1 if deviation > 0 else -1))
	else:
		return prev_rotation

journey = []
retreat = False
def rescue_kit(blue_frame, deviation):
	global journey, retreat
	if np.sum(blue_frame) >= 2000000:
		ser.write(b's' + struct.pack('f', 0) + b'\n')
		time.sleep(2)
		if PRINT_KIT:
			print(journey)
		retreat = True
	elif np.sum(blue_frame) > 200000 and not retreat:
		if PRINT_KIT:
			print("retrieving kit")
			print(np.sum(blue_frame))
		Moments = cv.moments(blue_frame)
		xBlue = int(Moments["m10"] / Moments["m00"])
		yBlue = int(Moments["m01"] / Moments["m00"])
		finalx = xBlue - frame.shape[1]/2
		finaly = frame.shape[0] - yBlue
		if xBlue!=0:
			deviation = math.atan(finalx/finaly)
			format_deviation(deviation)
			journey.append(deviation)
	if retreat:
		if len(journey)>0:
			ser.write(b's' + struct.pack('f', -1) + b'\n')
			if PRINT_KIT:
				print(len(journey), journey)
			deviation = journey[len(journey) - 1]
			if PRINT_KIT:
				print("retreating from kit")
			journey.pop()
		else:
			ser.write(b's' + struct.pack('f', 1) + b'\n')
			retreat = False
	return deviation

def format_deviation(deviation):
	deviation *= k_p
	deviation = deviation if deviation <= 1 else 1
	deviation = deviation if deviation >= -1 else -1

cam = VideoStream(resolution=(l_dim["h"], l_dim["w"])).start()
curr_stage = MissionStage.LINE
def transition_state():
	global curr_stage, cam
	cam.kill()
	cam = VideoStream()

# out = cv.VideoWriter('outpy.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (l_dim["w"],l_dim["h"]))
# record_start = time.time()
# time.sleep(3)
write_speed(1)
curr = None
start_time = 0
start1_time = time.time()
prev_see_line = False
stop_movement = False

start2_time = time.time()
x1 = 1 # displays the frame rate every 1 second
counter = 0
while True:
	# continue
	frame_org = cam.read()
	hsv_frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green_org = cv.inRange(hsv_frame_org, boundaries["green"][0], boundaries["green"][1])
	blue_org = cv.inRange(hsv_frame_org, boundaries["blue"][0], boundaries["blue"][1])
	
	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	green = green_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	blue_frame = blue_org[rk_crop_h:(frame_org.shape[0] - b_rk_crop_h), :]

	frame_org_gray = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(frame_org_gray, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	g_thresh = thresh[gs_crop_h:(thresh.shape[0] - b_gs_crop_h), :]
	w_thresh = cv.bitwise_not(thresh)
	
	#* filter out the floor outside the mat
	# contours, _ = cv.findContours(w_thresh, 1, 2)
	# # filter out all the really small contours
	# contours = list(filter(lambda cnt: cv.contourArea(cnt) > lt_min_area, contours))
	# if len(contours) > 0:
	# 	uni_hull = []
	# 	for cnt in contours:
	# 		convs = cv.convexHull(cnt)
	# 		for conv in convs:
	# 			uni_hull.append(conv)
	# 	uni_hull.append([[l_dim['h'], 0]])
	# 	uni_hull.append([[l_dim['h'], l_dim['w']]])
	# 	uni_hull = np.array(uni_hull)
	# 	uni_hull = cv.convexHull(uni_hull)
	# 	white_chunk_mask = np.zeros(thresh.shape[:2], dtype=np.int8)
	# 	cv.fillPoly(white_chunk_mask, pts =[uni_hull], color=255)
	# 	thresh = cv.bitwise_and(thresh, thresh, mask=white_chunk_mask)

	#* removing the vectors that are not the line in consideration
	# empty_index,  = np.where(np.sum(thresh, axis=1) == 0)
	# bl_index = max(empty_index)	if empty_index.size > 0 else 0
	bl_index = get_black_endpoint(thresh)
	thresh[0:bl_index, :] = 0

	edges = cv.Canny(thresh, 100, 200)

	gs_frame_edges = edges[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	rotation = 0
	green = green[gs_lt_roi:(green.shape[0]) ,:]
	g_sum = np.sum(green)
	if PRINT_STATE:
		print(curr)
	if curr:
		if curr == GreenSquare.DOUBLE:
			if (time.time() - start_time) < 2.5:
				rotation = 1
			else:
				curr = None
		else:
			if g_sum > 0:
				curr = x if (x := n_green_square(green, g_thresh)) else curr
				write_speed(0.8)
				if curr == GreenSquare.RIGHT:
					rotation = right_turn_rotation
				elif curr == GreenSquare.LEFT:
					rotation = -right_turn_rotation
				else:
					write_speed(1)
					continue
			else: 
				write_speed(1)
				curr = None
	else:
		# lines = cv.HoughLines(gs_frame_edges,1,np.pi/180,gs_voting)
		# curr = green_square(lines, green, thresh)
		curr = n_green_square(green, g_thresh)
		start_time = time.time()

		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		# cv.imwrite("gray.png", gray)
		lt_thresh = thresh[crop_h:(thresh.shape[0] - b_crop_h), :]

		if see_stop(hsv_frame_org, 90, 10):
			if not stop_movement:
				ser.write(b's' + struct.pack('f', 0) + b'\n')
				stop_movement = True
		else: 
			if stop_movement:
				ser.write(b's' + struct.pack('f', 1) + b'\n')
				stop_movement = False

		#* to tell the robot when to stop when it sees an obstacle
		# obs_frame = thresh[obs_crop_h:thresh.shape[0], :]
		# see_line = np.sum(obs_frame) > line_min
		# print(np.sum(obs_frame))
		# if see_line:#!= prev_see_line:
		# 	print("hi")
		# 	ser.write(b'l' + (1).to_bytes(1, "big") + b'\n')
		# prev_see_line = see_line

		#* Main vector line-tracking logic
		# dv = list(d_vector(thresh, scaled_m = scaled_m))
		dv = list(d_vector(lt_thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
		# print(dv)

		deviation = 0
		if dv[1] != 0:
			# xy_ratio = dv[0]/dv[1]
			vec_mag = math.sqrt(dv[0] ** 2 + dv[1] ** 2)
			dv[0] /= vec_mag
			dv[1] /= vec_mag 
			# print(dv[0], dv[1])
			deviation = math.atan(dv[0]/dv[1])
		deviation /= (np.pi/2) 
		format_deviation(deviation)
		
		# deviation = rotation_scale(thresh, deviation)

		if PRINT_DEVIATION:
			print(deviation)

		# deviation = rescue_kit(blue_frame, deviation)

		#* Halt if deivation == 1
		# if abs(deviation) > 0.999999:
		# 	speed(0)
		# 	while True:
		# 		pass
		rotation = deviation
		
		#* Just go forward if a line gap is observed 
		# white_roi = thresh[bl_index:(thresh.shape[0]), :]
		# if not freeze_rotation:
		# 	white_density = np.sum(white_roi)/(white_roi.shape[0] * white_roi.shape[1])
		# 	rotation = rotation if white_density > 0.005 else 0

		#* Detecting silver tape
		norm_var = normalized_var(frame_org_gray, bl_index, 10)
		print(norm_var)

		if not freeze_rotation:
			white_density = np.sum(w_thresh)/(w_thresh.shape[0] * w_thresh.shape[1])
			# print(white_density)
			rotation = rotation if white_density > 0.9 else 0
		

	write_rotation(rotation)
	# ser.flush()

	#* recording thingy for kenneth
	# if (time.time() - start1_time) > 17:
		# cam.kill()
		# print("broke")
	# 	break
	# if (time.time() - record_start > 10):
	# 	break

	#* FPS Counter
	if PRINT_FPS:
		counter+=1
		if (time.time() - start2_time) > x1 :
			print("FPS: ", counter / (time.time() - start2_time))
			counter = 0
			start2_time = time.time()
