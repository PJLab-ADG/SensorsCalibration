import numpy as np
import cv2
import os
import json
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline


np.set_printoptions(suppress=True)

txt_dir = './stationary'
txt_list = os.listdir(txt_dir)
img_path = './stationary_pic'

for txt_name in txt_list:
    txt_path = os.path.join(txt_dir, txt_name)
    # filename = "./stationary/0_280.txt"
    x, y = [], []
    x2, y2 = [], []
    yaw_deg = 0
    cnt = 0
    with open(txt_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            val = [float(s) for s in line.split()]
            if (len(val) == 2):
                x.append(val[0])
                y.append(val[1])
            if (len(val) == 3):
                x2.append(val[1])
                y2.append(val[2])
            cnt += 1

    print "cnt:", cnt

    # output_img = "0_280.jpg"
    name = txt_name.split('.')[0] + '.jpg'
    img_name = os.path.join(img_path, name)
    plt.plot(x2, y2,'.',color='blue', label = 'moving')
    plt.plot(x, y,'.', color='red', label = 'stationary object')
    plt.grid()
    plt.legend()
    plt.savefig(img_name)
    plt.clf()
    # plt.show()
