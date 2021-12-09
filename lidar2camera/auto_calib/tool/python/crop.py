import cv2
img = cv2.imread("kitti_q2.jpg")
crop_img = img[200:300,450:550] 

cv2.imwrite("crop_q2.jpg",crop_img) 
