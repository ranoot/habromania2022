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
from matrix_ops import get_black_endpoint

#* Constants
# PRINT_DEVIATION = True
# PRINT_STATE = True
# PRINT_KIT = True 
# PRINT_FPS = True
# FREEZE_ROTATION = True

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
			roi = g_col[max_black[i]:constrain(max_black[i]+50, max_black[i], g_col.shape[0])]
			valid_green = np.sum(roi)/255
			# print(roi)
			if valid_green > 3: # there is green beneath black
				if not ((i - prev_i) <= 3 and (i - prev_i) >= 0 and prev_i >= 0):
					valid_cols.append([i])
				prev_i = i
				continue
		if ((i - prev_i) >= 3 and (i - prev_i) >= 0 and prev_i >= 0 and len(valid_cols[len(valid_cols)-1]) == 1):
			valid_cols[len(valid_cols)-1].append(i)
	# if (g_frame.shape[1] - prev_i == 1 and prev_i >= 0):
	# 	valid_cols[len(valid_cols)-1].append(g_frame.shape[1] - 1)
	if (g_frame.shape[1] - prev_i == 1 and prev_i >= 0):
		valid_cols[len(valid_cols)-1].append(prev_i)
	# print(valid_cols)
	gs_list = list(filter(lambda xs: xs[-1] - xs[0] > 8, valid_cols))

	if len(gs_list) == 0 and len(gs_list) >= 3:
		return None 

	black_M = cv.moments(thresh)
	if black_M["m00"] == 0:
		return None
	black_cX = int(black_M["m10"] / black_M["m00"])

	gs_mp_list = list(map(lambda maxmin: (maxmin[0] + maxmin[1])/2, gs_list))
	# print(gs_mp_list)
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
		# print("v:", max_y , "t:", max_y < 0.5 and abs(deviation) > 0.70)
		rotation = math.pow(abs(deviation), (max_y)) * (1 if deviation > 0 else -1)
		if not FREEZE_ROTATION:
			return rotation
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
		return prev_rotation if FREEZE_ROTATION else deviation

# journey = []
# retreat = False
prev_see_blue = False
def rescue_kit(blue_frame, deviation):
	global prev_see_blue
	# if np.sum(blue_frame) >= 2000000:
		
	# 	#claw.write() #TO-DO
	# 	if PRINT_KIT:
	# 		print(journey)
	# 	retreat = True
	
	if np.sum(blue_frame) > 200000:
		see_blue = True
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
			# journey.append(deviation)
	# if retreat:
	# 	if len(journey)>0:
	# 		ser.write(b's' + struct.pack('f', -1) + b'\n')
	# 		if PRINT_KIT:
	# 			print(len(journey), journey)
	# 		deviation = journey[len(journey) - 1]
	# 		if PRINT_KIT:
	# 			print("retreating from kit")
	# 		journey.pop()
	# 	else:
	# 		ser.write(b's' + struct.pack('f', 1) + b'\n')
	# 		retreat = False
	else:
		see_blue = False
	
	if prev_see_blue != see_blue:
		ser.write(b'k' + (int(see_blue)).to_bytes(1, "big") + b'\n')
		print(see_blue)
	
	prev_see_blue = see_blue

	return deviation, see_blue

def format_deviation(deviation):
	deviation *= k_p
	deviation = deviation if deviation <= 1 else 1
	deviation = deviation if deviation >= -1 else -1

# cam = VideoStream(resolution=(e_dim["h"], e_dim["w"])).start()
cam = VideoStream(resolution=(l_dim["h"], l_dim["w"])).start()
curr_stage = MissionStage.LINE
def transition_state(stage): # Just don't do anything if stage unknown
	global curr_stage, cam
	if stage == MissionStage.EVAC:
		cam.change_res()
		curr_stage = stage
	elif stage == MissionStage.LINE:
		cam.change_res(resolution=(l_dim["h"], l_dim["w"]))
		curr_stage = stage

# out = cv.VideoWriter('outpy.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (l_dim["w"],l_dim["h"]))
# record_start = time.time()
# time.sleep(3)
write_speed(1)
curr = None
start_time = 0
start1_time = time.time()
prev_see_line = False
prev_st = False
stop_movement = False

erosion_size = 11
element = cv.getStructuringElement(cv.MORPH_RECT, (2 * erosion_size + 1, 2 * erosion_size + 1), (erosion_size, erosion_size))

start2_time = time.time()
x1 = 1 # displays the frame rate every 1 second
counter = 0
while True:
	if curr_stage == MissionStage.LINE:
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
		thresh = cv.bitwise_and(cv.bitwise_not(green_org), thresh)
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
		bl_index_max, bl_index_min = get_black_endpoint(thresh)
		# print(bl_index_max, bl_index_min)
		# thresh[0:bl_index_min, :] = 0
		# gray_line = thresh.copy()
		# cv.line(gray_line,(0, bl_index_max),(l_dim['w'] - 1,bl_index_max),255,2)
		# cv.imwrite("bruh.png", gray_line)
		# print(bl_index_max)
		is_top_line = bl_index_max > (l_dim['h'] - 10)
		if not is_top_line:
			thresh[0:int(bl_index_max), :] = 0
		# else:
		# 	thresh[0:int(bl_index_min), :] = 0
		# thresh[0:int((bl_index_min + bl_index_max)/2), :] = 0

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
					# write_speed(0.8)
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
			obs_frame = thresh[obs_crop_h:thresh.shape[0], :]
			see_line = np.sum(obs_frame) > line_min
			# print(np.sum(obs_frame))
			if see_line != prev_see_line:
				ser.write(b'l' + (1).to_bytes(1, "big") + b'\n')
			prev_see_line = see_line

			#* Main vector line-tracking logic
			# dv = list(d_vector(thresh, scaled_m = scaled_m))
			if not is_top_line:
				dv = list(d_vector(lt_thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
			else: 
				dv = list(d_vector(lt_thresh, h_scaled_v = scaled_v_grad))
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
			
			deviation = rotation_scale(thresh, deviation)

			if PRINT_DEVIATION:
				print(deviation)

			#*Rescue kit
			deviation, see_blue = rescue_kit(blue_frame, deviation)

			#* Halt if deviation == 1
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
			if bl_index_max > (l_dim['h'] - 40) and bl_index_max < l_dim['h']-5 and not see_blue:
				norm_var = normalized_var(frame_org_gray, bl_index_max, 6)
				if norm_var and np.sum(thresh[(l_dim['h'] - 90) :(l_dim['h'] - 1)])/255 < 200:
					if norm_var > 3.8:
						pass
						# write_speed(0)
						# while True:
						# 	print(norm_var, bl_index_max, np.sum(thresh))
						# 	cv.imwrite(thresh)
							# cv.imwrite("gay.png", roi)
							# cv.imwrite("henlo.png", frame_org_gray[(bl_index_max-6):(bl_index_max), :])
						transition_state(MissionStage.EVAC)
						ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
				print(norm_var)
			# if not freeze_rotation:
			# white_density = np.sum(w_thresh)/(w_thresh.shape[0] * w_thresh.shape[1] * 255)
			# print(white_density)
			# rotation = rotation if white_density < 0.90 else 0
		
		write_rotation(rotation)
		
	elif curr_stage == MissionStage.EVAC:
		frame_org = cam.read()
		frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
		green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
		gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)

		circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 = 23)
		
		balls = []
		if circles is not None:
			for x, y, r in circles[0]:
				if y <= (e_dim["h"]/2 + 10): # Circle is probably bogus if it's centre is below half the frame
					# cv.imwrite("out.png", mask)
					# mean, std = cv.meanStdDev(frame_org, mask=mask)
					balls.append({
						"x": x,
						"y": y,
						"r": r,
						# "dev": np.sum(std)
					})

		#* Giving the robot a bias towards the ball
		ball_mask = np.full(gray_org.shape[:2], 255, dtype=np.uint8)
		if len(balls) > 0:
			biggest_ball = max(balls, key=lambda b: b['r'])
			#TODO Adjust this constant
			ball_dev = (biggest_ball['x'] - e_dim["w"]/2) * 0.011
			print("bruh")
			ball_dev = constrain(ball_dev, -0.9, 0.9)
			ball_mask = cv.circle(ball_mask, (int(biggest_ball['x']),int(biggest_ball['y'])), int(biggest_ball['r']), 0, -1)
			ser.write(b'b' + (1).to_bytes(1, "big") + b'\n')
			ser.write(b'r' + struct.pack('f', ball_dev) + b'\n') 
		else: #TODO: Cover the cases that the bot supposed to keep turning
			ser.write(b'b' + (0).to_bytes(1, "big") + b'\n')
			ser.write(b'r' + struct.pack('f', 0) + b'\n')

		#* Detecting for silver tape
		# silver_mask = cv.bitwise_and(ball_mask, cv.bitwise_not(thresh))
		st = ((n := normalized_var(gray_org, e_dim['h'] - 200, 20)) > 40)
		# print("s ", n)
		if st != prev_st:
			ser.write(b't' + (int(st)).to_bytes(1, "big") + b'\n')
			# print('s')
		prev_st = st
		#* Detecting (a) if you see the deposit (b) if a, then find the deviation of black centroid
		# ser.write(b'd' + (1).to_bytes(1, "big") + b'\n') #!!!!
		# ser.write(b'D' + struct.pack('f', 0) + b'\n')
		eroded = cv.erode(thresh, element, iterations=2)
		contours,hierarchy = cv.findContours(eroded, 1, 2)
		f_contours = list(filter(lambda cnt: cv.contourArea(cnt) > 1500, contours))
		print(list(map(lambda cnt: cv.contourArea(cnt), f_contours)))
		if (s := len(f_contours)) == 1: #?
			ser.write(b'd' + (1).to_bytes(1, "big") + b'\n')

			black_M = cv.moments(max(f_contours, key=lambda k: cv.contourArea(k)))
			black_cX = int(black_M["m10"] / black_M["m00"])

			dep_dev = (black_cX - e_dim['w']/2) * 0.007
			dep_dev = constrain(dep_dev, -0.6, 0.6)
			print("D ", s)

			ser.write(b'D' + struct.pack('f', dep_dev) + b'\n')
		else: 
			ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')
	# ser.flush()

		#*Stopping outside of evac
		if see_stop(frame_org_hsv, 90, 10):
			if not stop_movement:
				ser.write(b'x' + (1).to_bytes(1, "big") + b'\n')
				stop_movement = True
		else: 
			if stop_movement:
				ser.write(b'x' + (0).to_bytes(1, "big") + b'\n')
				stop_movement = False

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
