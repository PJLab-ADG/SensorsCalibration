import numpy as np
import cv2 
import math

def get_lines(lines_in):
    if cv2.__version__ < '3.0':
        return lines_in[0]
    return [l[0] for l in lines_in]

img = cv2.imread("python/test.png")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  
lines = cv2.HoughLines(gray, rho=1, theta=np.pi/180, threshold=100)

# l[0] - line; l[1] - angle
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    

cv2.imwrite('test_result.png',img)
