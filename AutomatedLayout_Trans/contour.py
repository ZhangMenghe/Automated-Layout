import numpy as np
import cv2

area_threshold = 20
im = cv2.imread('contour_test2.png')
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
# ret, thresh = cv2.threshold(imgray, 127, 255,3)
thresh = cv2.Canny(imgray, 100, 200)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
clusterTooSmall = []
hullList = []
for i, cnt in enumerate(contours):
	# moments
	M = cv2.moments(cnt)
	area = cv2.contourArea(cnt)
	if(area<area_threshold):
		clusterTooSmall.append(i)
	else:
		hull = cv2.convexHull(cnt)
		hullList.append(hull)
contours = np.delete(np.array(contours), clusterTooSmall)

im3 = cv2.drawContours(im, hullList, -1, (0,255,0), 1)
# cnt = contours[4]
# im3 = cv2.drawContours(im, [cnt], 0, (0,255,0), 1)
cv2.imshow('image', im3)
cv2.waitKey(0)