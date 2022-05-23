from matrix_ops import *
import math

l_dim = {"w": 160, "h": 120}
e_dim = {"w": 320, "h": 240}
boundaries = {
	"red_upper": ((170,50,50), (0,0,0)),
	"red_lower": ((0,50,50), (10,255,255)),
	"green": ((50,50,50), (95,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 0
b_crop_h = 0
gs_crop_h = 76
b_gs_crop_h = 0

b_w_thresh = 42 #
k_p =8

lt_min_area = 50

#* the region of image to consider whether robot is "done" with the particular green square
gs_lt_roi = 0
gs_voting = 12

rk_crop_h = 30
b_rk_crop_h = 0

obs_crop_h = 60 # height of image also affected by "crop_h" and "b_crop_h"
line_min = 2000

epsilon = 30 * math.pi/180
delta = 10 * math.pi/180

right_turn_rotation = 0.8
min_area = 50

st_crop_h = 80
st_b_crop_h = 10

min_red = 40000

scaled_v = scaled_vector(l_dim["h"] - b_crop_h - crop_h, 6)
w_scaled_v = w_scaled_vector(l_dim["w"], 10)
gs_scaled_v = scaled_vector(l_dim["h"] - b_gs_crop_h - gs_crop_h, 0)
scaled_m = scaled_2d_matrix(l_dim["h"] - crop_h - b_crop_h, l_dim["w"], 2.5, 0.4, 0.2)

# Debugging constants
PRINT_DEVIATION = False
PRINT_STATE = False
PRINT_KIT = False #False
PRINT_FPS = False
FREEZE_ROTATION = False