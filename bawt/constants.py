import math
from enum import Enum

k_p = 1.15

class State(Enum):
    LINETRACK = 1
    OBSTACLE_TRACK = 2

dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((40,20,20), (70,225,225))
}
crop_h = 30
green_square_crop_h = 50
epsilon = 30 * math.pi/180

delta = 10 * math.pi/180

d_green_square_min = 500 #TODO: Determine Actual Min, this is just a guess
class GreenSquare(Enum):
    RIGHT = 1
    LEFT = 2
    DOUBLE = 3