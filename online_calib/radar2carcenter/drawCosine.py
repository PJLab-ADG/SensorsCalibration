import numpy as np
import cv2
import os
import json
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline


np.set_printoptions(suppress=True)

# filename = "./fitline/111_-1.848388.txt"
# x, y = [], []
# A = 1
# yaw_deg = 0
# cnt = 0
# with open(filename, 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         val = [float(s) for s in line.split()]
#         if (cnt == 0):
#             A = val[0]
#             yaw_deg = val[1]
#         else:
#             x.append(val[0])
#             y.append(val[1])
#         cnt += 1

# print "A =",A
# print "cnt:", cnt
# x2 = np.linspace(-50.0, 50.0, num = 360)
# y2 = []
# for d in x2:
#     y2.append(A*np.cos((d+yaw_deg) / 180 * np.pi))

# mid_pointx = [-yaw_deg]
# mid_pointy = [A]

# output_img = "111_-1.848388.jpg"
# plt.plot(x, y,'.', color='gold', label = 'object')
# plt.plot(x2, y2,'-',color='green', label = 'cosine')
# plt.plot(mid_pointx, mid_pointy,'.',color='red', label = 'mid point')
# plt.grid()
# plt.legend()
# plt.savefig(output_img)
# plt.show()


txt_dir = './fitline'
txt_list = os.listdir(txt_dir)
img_path = './fitline_pic'

for txt_name in txt_list:
    txt_path = os.path.join(txt_dir, txt_name)
    x, y = [], []
    xm, ym = [], []
    A = 1
    yaw_deg = 0
    cnt = 0
    with open(txt_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            val = [float(s) for s in line.split()]
            if (cnt == 0):
                A = val[0]
                yaw_deg = val[1]
            elif (len(val) == 2):
                x.append(val[0])
                y.append(val[1])
            elif (len(val) == 3):
                xm.append(val[1])
                ym.append(val[2])
            cnt += 1

    print "A =",A
    print "cnt:", cnt
    x2 = np.linspace(-50.0, 50.0, num = 360)
    y2 = []
    for d in x2:
        y2.append(A*np.cos((d+yaw_deg) / 180 * np.pi))

    mid_pointx = [-yaw_deg]
    mid_pointy = [A]

    name = txt_name.split('.')[0] + '.jpg'
    img_name = os.path.join(img_path, name)
    plt.plot(xm, ym,'.', color='gold', label = 'object')
    plt.plot(x, y,'.', color='blue', label = 'stationary')
    plt.plot(x2, y2,'-',color='green', label = 'cosine')
    plt.plot(mid_pointx, mid_pointy,'.',color='red', label = 'mid point')
    plt.grid()
    plt.legend()
    plt.savefig(img_name)
    plt.show()