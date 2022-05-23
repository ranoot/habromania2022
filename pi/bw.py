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
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
def on_change(val):
    global b_w_thresh
    b_w_thresh = val

frame_org = cam.read()
gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
cv.imshow('frame', thresh)
cv.createTrackbar('threshold', 'frame', 0, 255, on_change)
while True:
    frame_org = cam.read()
    gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
    t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
    
    cv.imshow('frame', thresh)
    if cv.waitKey(1) == ord('q'):
        break