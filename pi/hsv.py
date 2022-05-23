import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream

b_w_thresh = 0

dim = {"w": 320, "h": 240}
max_bound = [0, 0, 0]
min_bound = [0, 0, 0]
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
def on_change(var, index):
    def A(val):
        var[index] = val
    return A
h_min = on_change(min_bound, 0)
s_min = on_change(min_bound, 1)
v_min = on_change(min_bound, 2)

h_max = on_change(max_bound, 0)
s_max = on_change(max_bound, 1)
v_max = on_change(max_bound, 2)

frame_org = cam.read()
frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
green_org = cv.inRange(frame_org, np.array(min_bound), np.array(max_bound))
cv.imshow('frame', green_org)
cv.createTrackbar('h min', 'frame', 0, 255, h_min)
cv.createTrackbar('s min', 'frame', 0, 255, s_min)
cv.createTrackbar('v min', 'frame', 0, 255, v_min)
cv.createTrackbar('h max', 'frame', 0, 255, h_max)
cv.createTrackbar('s max', 'frame', 0, 255, s_max)
cv.createTrackbar('v max', 'frame', 0, 255, v_max)
while True:
    frame_org = cam.read()
    frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
    green_org = cv.inRange(frame_org, np.array(min_bound), np.array(max_bound))
    
    cv.imshow('frame', green_org)
    if cv.waitKey(1) == ord('q'):
        break