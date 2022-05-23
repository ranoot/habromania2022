import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # t, gray = cv.threshold(gray, 0, 255 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
    lwr = np.array([0, 0, 143])
    upr = np.array([179, 61, 252])
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    msk = cv.inRange(hsv, lwr, upr)

    krn = cv.getStructuringElement(cv.MORPH_RECT, (50, 30))
    dlt = cv.dilate(msk, krn, iterations=5)
    res = 255 - cv.bitwise_and(dlt, msk)

    # cv.imshow('bruh', res)
    # cv.waitKey(0)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (4,4), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        
        corners2 = cv.cornerSubPix(gray,corners, (5,5), (-1,-1), criteria)
        if objp.any() and corners.any():
            objpoints.append(objp)
            imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (4,4), corners2, ret)
        # cv.imshow('img', img)
        # cv.waitKey(500)
cv.destroyAllWindows()
# objpoints = objpoints[:16]
# print(objpoints.shape)
print(np.array(imgpoints).shape)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
img = cv.imread('WIN_20220308_15_22_18_Pro.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

fovx, fovy, focalLength, principalPoint, aspectRatio = 	cv.calibrationMatrixValues(mtx, (w, h), 2.176, 1.632) 
# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)