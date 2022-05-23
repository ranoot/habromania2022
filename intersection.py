import cv2 as cv

dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0))
}
crop_h = 30

k_p = 1.15
cam = cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

ret, frame = cam.read()

while True:
    ret, frame = cam.read()
	frame = frame[:, crop_h:(frame.shape[0])]
	frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
	gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)