import numpy as np
import scipy.linalg as linalg
import math

def vphl2R(vp, angle, K):
    h = math.atan(angle)
    roll = angle
    print("roll: {} rad".format(roll))
    Rz = linalg.expm(np.cross(np.eye(3), [0, 0, 1] / linalg.norm([0, 0, 1]) * h))

    K_ = np.matrix(K)
    K_inv = K_.I
    r3 = (K_inv * vp) / np.linalg.norm(K_inv * vp)
    pitch = math.asin(r3[1])
    yaw = math.atan(r3[0] / r3[2])
    print("pitch: {} rad".format(pitch))
    print("yaw: {} rad".format(yaw))
    Rx = linalg.expm(np.cross(np.eye(3), [1, 0, 0] / linalg.norm([1, 0, 0]) * pitch))
    Ry = linalg.expm(np.cross(np.eye(3), [0, 1, 0] / linalg.norm([0, 1, 0]) * yaw))
    R = Rz * Ry * Rx
    return R


if __name__ == "__main__":
    hl_angle = 0.01224   # horizon line angle (rad)
    K = np.array([[2117.31, 0, 924.681],[0, 2113.29, 656.457],[0, 0, 1]]) # camera instrinsic
    vp = np.array([1020.7, 618.227, 1]).reshape(3, 1) # vanishing point [vp_x, vp_y, 1]

    R = vphl2R(vp, hl_angle, K)
    print("R:\n {}".format(R))