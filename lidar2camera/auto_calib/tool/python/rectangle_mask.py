import numpy as np
import cv2 as cv

img = cv.imread("python/mask.jpg")
pts = np.array([[1197,209],[1203,210],[1196,439],[1188,440]], np.int32)
pts = pts.reshape((-1,1,2))
cv.fillPoly(img,[pts],(255,255,255))
cv.imwrite('mask_rec.jpg',img)