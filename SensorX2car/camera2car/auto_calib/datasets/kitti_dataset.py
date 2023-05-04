import os
import os.path as osp

import torch
from torch.utils.data import Dataset
from torch.utils.data.dataloader import default_collate
from torchvision.transforms import functional as F
import glob as gb
import numpy as np
import numpy.linalg as LA
import cv2
import json
import csv
import matplotlib.pyplot as plt
from pylsd import lsd

import datasets.transforms as T
import pickle

def get_transform(center, scale, res, rot=0):
    # Generate transformation matrix
    h = 200 * scale
    t = np.zeros((3, 3))
    t[0, 0] = float(res[1]) / h
    t[1, 1] = float(res[0]) / h
    t[0, 2] = res[1] * (-float(center[0]) / h + .5)
    t[1, 2] = res[0] * (-float(center[1]) / h + .5)
    t[2, 2] = 1
    if not rot == 0:
        rot = -rot # To match direction of rotation from cropping
        rot_mat = np.zeros((3,3))
        rot_rad = rot * np.pi / 180
        sn,cs = np.sin(rot_rad), np.cos(rot_rad)
        rot_mat[0,:2] = [cs, -sn]
        rot_mat[1,:2] = [sn, cs]
        rot_mat[2,2] = 1
        # Need to rotate around center
        t_mat = np.eye(3)
        t_mat[0,2] = -res[1]/2
        t_mat[1,2] = -res[0]/2
        t_inv = t_mat.copy()
        t_inv[:2,2] *= -1
        t = np.dot(t_inv,np.dot(rot_mat,np.dot(t_mat,t)))
    return t

def transform(pt, center, scale, res, invert=0, rot=0):
    # Transform pixel location to different reference
    t = get_transform(center, scale, res, rot=rot)
    if invert:
        t = np.linalg.inv(t)
    new_pt = np.array([pt[0], pt[1], 1.]).T
    new_pt = np.dot(t, new_pt)
    return new_pt[:2].astype(int)

def crop(img, center, scale, res, rot=0):
    # Upper left point
    ul = np.array(transform([0, 0], center, scale, res, invert=1))
    # Bottom right point
    br = np.array(transform(res, center, scale, res, invert=1))
    new_shape = [br[1] - ul[1], br[0] - ul[0]]
    if len(img.shape) > 2:
        new_shape += [img.shape[2]]
    new_img = np.zeros(new_shape)

    # Range to fill new array
    new_x = max(0, -ul[0]), min(br[0], len(img[0])) - ul[0]
    new_y = max(0, -ul[1]), min(br[1], len(img)) - ul[1]
    # Range to sample from original image
    old_x = max(0, ul[0]), min(len(img[0]), br[0])
    old_y = max(0, ul[1]), min(len(img), br[1])
    new_img[new_y[0]:new_y[1], new_x[0]:new_x[1]] = img[old_y[0]:old_y[1], old_x[0]:old_x[1]]
    return cv2.resize(new_img, res)

def create_masks(image):
    height = image.shape[0]
    width = image.shape[1]
    masks = torch.zeros((1, height, width), dtype=torch.uint8)
    return masks

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

def sample_vert_segs_np(segs, thresh_theta=22.5):    
    lines = segs2lines_np(segs)
    (a,b) = lines[:,0],lines[:,1]
    theta = np.arctan2(np.abs(b),np.abs(a))
    thresh_theta = np.radians(thresh_theta)
    return segs[theta < thresh_theta]

def augment(image, vp, angle, division, w):
    if division == 1:  # left-right flip
        vp[0] = w - vp[0]
        return image[:, ::-1].copy(), vp, -angle
    return image, vp, angle

def kpt_affine(kpt, mat):
    kpt = np.array(kpt)
    shape = kpt.shape
    kpt = kpt.reshape(-1, 2)
    return np.dot( np.concatenate((kpt, kpt[:, 0:1]*0+1), axis = 1), mat.T ).reshape(shape)

def draw_segs(img, segs):
    for seg in segs:
        start_point = (int(seg[0]), int(seg[1]))
        end_point = (int(seg[2]), int(seg[3]))
        cv2.line(img, start_point, end_point, (0, 255, 0), 1)

class KittiDataset(Dataset):
    def __init__(self, cfg, listpath, basepath, return_masks=False, transform=None):
        self.listpath = listpath # csv_path:'kitti_split/train.csv'
        self.basepath = basepath # pkl_path:'.../kitti_horizon_vp'
        self.input_width = cfg.DATASETS.INPUT_WIDTH
        self.input_height = cfg.DATASETS.INPUT_HEIGHT
        self.min_line_length = cfg.DATASETS.MIN_LINE_LENGTH
        self.num_input_lines = cfg.DATASETS.NUM_INPUT_LINES
        self.num_input_vert_lines = cfg.DATASETS.NUM_INPUT_VERT_LINE
        self.vert_line_angle = cfg.DATASETS.VERT_LINE_ANGLE
        self.return_vert_lines = cfg.DATASETS.RETURN_VERT_LINES
        self.return_masks = return_masks
        self.transform = transform
        self.mode = cfg.MODE
        self.list_filename = []

        with open(self.listpath, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=' ')
            for row in reader:
                date = row[0] # '2011_09_26 '
                drive = row[1] # '0001'
                
                # # if use kitti_horizon_vp dataset
                # total_length = int(row[2]) # 108
                # for index in range(total_length):
                #     self.list_filename.append(self.basepath + '/' + date + '/' + drive + '/%06d.pkl' % index)
                
                # if use kitti_horizon_vp_straight dataset
                path = os.path.join(basepath, date, drive ,'*.pkl')
                pkl_path = gb.glob(path)
                self.list_filename.extend(pkl_path)
                
        
        if self.mode == "train" and cfg.DATASETS.AUGMENTATION:
            self.size = len(self.list_filename) * 2
        else:
            self.size = len(self.list_filename)
        print('dataset size: {} images'.format(self.size))
        
    def __getitem__(self, idx):
        target = {}
        extra = {}
        
        filename = self.list_filename[idx % len(self.list_filename)]
        
        with open(filename, 'rb') as fp:
            data = pickle.load(fp)

        image = np.transpose(data['image'], [1, 2, 0]) # (c, h, w) -> (h, w, c)
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        org_h, org_w = image.shape[0], image.shape[1]
        org_sz = np.array([org_h, org_w])
        
        center = (org_w / 2, org_h / 2)
        pp = (self.input_width / 2, self.input_height / 2)
        rho = 2.0 / np.minimum(self.input_width, self.input_height)
        s = max(image.shape[0], image.shape[1]) / 200 # scale
        shape = (self.input_width, self.input_height)
        
        
        gt_angle = data['angle']
        if np.isnan(gt_angle):
            gt_angle = 0.
            print("warning:gt_angle is Nan")
        gt_vp = data['vp']

        image, gt_vp, gt_hl = augment(image, gt_vp, gt_angle, idx // len(self.list_filename), org_w)
        
        org_image = image.copy()
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        org_segs = lsd(gray, scale=0.8)
        org_segs = filter_length(org_segs, self.min_line_length)
        num_segs = len(org_segs)
        assert num_segs > 10, print(num_segs)

        # ---crop---
        image_processed = crop(image, center, s, shape)
        gt_vp = transform(gt_vp, center, s, shape)
        
        for i in range(num_segs):
            org_segs[i][0:2] = transform(org_segs[i][0:2], center, s, shape)
            org_segs[i][2:4] = transform(org_segs[i][2:4], center, s, shape)
        
        # radius = 3
        # color = (0, 0, 255)
        # thickness = -1
        # cropped_visual = image_processed.copy()
        # cv2.circle(cropped_visual, (int(gt_vp[0]), int(gt_vp[1])), radius, color, thickness)
        # draw_segs(cropped_visual, org_segs)    
        # cv2.imwrite('cropped.png', cropped_visual)
        
        # ---rot and scale---
        if self.mode == 'train':
            new_center = np.array((shape[0] / 2, shape[1] / 2))
            scale = max(shape[0], shape[1]) / 200
            aug_rot = (np.random.random() * 2 - 1) * 20. 
            aug_scale = 0.75 + np.random.random() * 0.5 
            scale *= aug_scale
            mat_mask = get_transform(new_center, scale, shape, aug_rot)[:2]
            mat = get_transform(new_center, scale, shape, aug_rot)[:2]
            image_processed= cv2.warpAffine(image_processed, mat, shape).astype(np.float32)
            gt_vp = kpt_affine(gt_vp, mat_mask)
            org_segs[:, 0:2] = kpt_affine(org_segs[:, 0:2], mat_mask)
            org_segs[:, 2:4] = kpt_affine(org_segs[:, 2:4], mat_mask)
        else:
            aug_rot = 0
            image_processed = image_processed.astype(np.float32)

        # rot_scale_visual = image_processed.copy()
        # cv2.circle(rot_scale_visual, (int(gt_vp[0]), int(gt_vp[1])), radius, color, thickness)
        # draw_segs(rot_scale_visual, org_segs)
        # cv2.imwrite('rotscale.png',rot_scale_visual)

        segs = normalize_segs(org_segs, pp=pp, rho=rho)
        
        # whole segs
        sampled_segs, line_mask = sample_segs_np(
            segs, self.num_input_lines)
        sampled_lines = segs2lines_np(sampled_segs)
        
        # vertical directional segs
        vert_segs = sample_vert_segs_np(segs, thresh_theta=self.vert_line_angle)
        if len(vert_segs) < 2:
            vert_segs = segs
            
        sampled_vert_segs, vert_line_mask = sample_segs_np(
            vert_segs, self.num_input_vert_lines)
        sampled_vert_lines = segs2lines_np(sampled_vert_segs)
        
        if self.return_masks:
            masks = create_masks(image_processed)
        
        final_vp = np.ones(3)
        final_vp[0] = rho * (gt_vp[0] - pp[0])
        final_vp[1] = rho * (gt_vp[1] - pp[1])
        
        final_hl = gt_hl + aug_rot / 180 * np.pi
        
        final_image = np.ascontiguousarray(image_processed)
        target['hl'] = torch.from_numpy(np.ascontiguousarray(final_hl)).contiguous().float()
        target['vp'] = torch.from_numpy(np.ascontiguousarray(final_vp)).contiguous().float()
        
        if self.return_vert_lines:
            target['segs'] = torch.from_numpy(np.ascontiguousarray(sampled_vert_segs)).contiguous().float()
            target['lines'] = torch.from_numpy(np.ascontiguousarray(sampled_vert_lines)).contiguous().float()
            target['line_mask'] = torch.from_numpy(np.ascontiguousarray(vert_line_mask)).contiguous().float()
        else:
            target['segs'] = torch.from_numpy(np.ascontiguousarray(sampled_segs)).contiguous().float()
            target['lines'] = torch.from_numpy(np.ascontiguousarray(sampled_lines)).contiguous().float()
            target['line_mask'] = torch.from_numpy(np.ascontiguousarray(line_mask)).contiguous().float()
                    
        if self.return_masks:
            target['masks'] = masks
        target['org_img'] = org_image
        target['org_sz'] = org_sz
        target['input_sz'] = shape
        target['img_path'] = filename
        target['filename'] = filename
        
        extra['lines'] = target['lines'].clone()
        extra['line_mask'] = target['line_mask'].clone()

        return self.transform(final_image, extra, target)
    
    def __len__(self):
        return self.size  

def make_transform():
    return T.Compose([
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]) 

def build_kitti(image_set, cfg):
    root = cfg.DATASET_DIR # dataset path
    if not os.path.exists(root):
        print('Dataset path: {} does not exist'.format(root))
        exit()
        
    PATHS = {
        "train": 'kitti_split/train.csv',
        "val":   'kitti_split/val.csv',
        "test":  'kitti_split/test.csv',
    }

    ann_file = PATHS[image_set]

    dataset = KittiDataset(cfg, ann_file, root, return_masks=cfg.MODELS.MASKS, transform=make_transform())
    return dataset

