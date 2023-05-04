'''
Visualize the extracted line features in an image
'''
import sys
import cv2
import numpy as np
from pylsd import lsd
import numpy.linalg as LA

def sample_segs_np(segs, num_sample, use_prob=True):    
    num_segs = len(segs)
    sampled_segs = np.zeros([num_sample, 4], dtype=np.float32)
    mask = np.zeros([num_sample, 1], dtype=np.float32)
    if num_sample > num_segs:
        sampled_segs[:num_segs] = segs
        mask[:num_segs] = np.ones([num_segs, 1], dtype=np.float32)
    else:    
        lengths = LA.norm(segs[:,2:] - segs[:,:2], axis=-1)
        prob = lengths/np.sum(lengths)        
        idxs = np.random.choice(segs.shape[0], num_sample, replace=True, p=prob)
        sampled_segs = segs[idxs]
        mask = np.ones([num_sample, 1], dtype=np.float32)
    return sampled_segs, mask

def filter_length(segs, min_line_length=10):
    lengths = LA.norm(segs[:,2:4] - segs[:,:2], axis=1)
    segs = segs[lengths > min_line_length]
    return segs[:,:4]

def normalize_segs(segs, pp, rho):    
    pp = np.array([pp[0], pp[1], pp[0], pp[1]], dtype=np.float32)    
    return rho*(segs - pp)

def normalize_safe_np(v, axis=-1, eps=1e-6):
    de = LA.norm(v, axis=axis, keepdims=True)
    de = np.maximum(de, eps)
    return v/de

def segs2lines_np(segs):
    ones = np.ones(len(segs))
    ones = np.expand_dims(ones, axis=-1)
    p1 = np.concatenate([segs[:,:2], ones], axis=-1)
    p2 = np.concatenate([segs[:,2:], ones], axis=-1)
    lines = np.cross(p1, p2)
    return normalize_safe_np(lines)

def sample_vert_segs_np(segs, thresh_theta=22.5):    
    lines = segs2lines_np(segs)
    (a,b) = lines[:,0],lines[:,1]
    theta = np.arctan2(np.abs(b),np.abs(a))
    thresh_theta = np.radians(thresh_theta)
    return segs[theta < thresh_theta]

def draw_seg(img, segs, name):
    for seg in segs:
        start_point = np.round(seg[:2]).astype(int)
        end_point = np.round(seg[2:4]).astype(int)
        color = (0, 0, 255)
        thickness = 1
        cv2.line(img, start_point, end_point, color, thickness)
    cv2.imwrite(name, img)

def main():
    output_path = "./output/"
    img = cv2.imread("./pic/000000.png") # image_name
    assert img is not None

    org_h, org_w = img.shape[0], img.shape[1]

    sz = img.shape[0:2]
    side_length = np.min(sz)
    if sz[0] > sz[1]:
        ul_x = 0  
        ul_y = int(np.floor((sz[0]/2) - (side_length/2)))
        x_inds = [ul_x, sz[1]-1]
        y_inds = [ul_y, ul_y + side_length - 1]
    else:
        ul_x = int(np.floor((sz[1]/2) - (side_length/2)))
        ul_y = 0
        x_inds = [ul_x, ul_x + side_length - 1]
        y_inds = [ul_y, sz[0]-1]

    c_img = img[y_inds[0]:y_inds[1]+1, x_inds[0]:x_inds[1]+1, :]

    # cv2.imwrite("000000_1.png",c_img)
    
    pp = (org_w/2, org_h/2)
    rho = 2.0/np.minimum(org_w,org_h)
    min_line_length = 10
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    org_segs = lsd(gray, scale=1)
    draw_seg(img.copy(), org_segs, output_path + "/org_segs.png")
    org_segs = filter_length(org_segs, min_line_length)
    draw_seg(img.copy(),org_segs, output_path + "/segs_filtered.png")
    num_segs = len(org_segs)
    assert num_segs > 10, print(num_segs)

    num_input_lines = 512
    segs = normalize_segs(org_segs, pp=pp, rho=rho)
    sampled_segs, line_mask = sample_segs_np(
            segs, num_input_lines)
    sampled_lines = segs2lines_np(sampled_segs)
    
    vert_line_angle = 22.5
    vert_segs = sample_vert_segs_np(segs, thresh_theta=vert_line_angle)
    vert_segs_1 = vert_segs / rho + [pp[0],pp[1],pp[0],pp[1]]
    draw_seg(img.copy(),vert_segs_1, output_path + "/vert_segs.png")

if __name__ == '__main__':
    main()