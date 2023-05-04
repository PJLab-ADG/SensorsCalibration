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

def create_masks(image):
    masks = torch.zeros((1, image.size()[0], image.size()[1]), dtype=torch.uint8)
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

class ImageDataset(Dataset):
    def __init__(self, cfg, return_masks=False, transform=None):
        self.input_width = cfg.DATASETS.INPUT_WIDTH
        self.input_height = cfg.DATASETS.INPUT_HEIGHT
        self.min_line_length = cfg.DATASETS.MIN_LINE_LENGTH
        self.num_input_lines = cfg.DATASETS.NUM_INPUT_LINES
        self.num_input_vert_lines = cfg.DATASETS.NUM_INPUT_VERT_LINE
        self.vert_line_angle = cfg.DATASETS.VERT_LINE_ANGLE
        self.return_vert_lines = cfg.DATASETS.RETURN_VERT_LINES
        self.return_masks = return_masks
        self.transform = transform
        self.root = cfg.DATASET_DIR
        self.list_filename = os.listdir(self.root)
        
    def __getitem__(self, idx):
        target = {}
        extra = {}
        
        filename = os.path.join(self.root, self.list_filename[idx])
                
        image = cv2.imread(filename)
        assert image is not None, print("Can' t open image file ", filename)
        org_image = image.copy()

        image = image[:,:,::-1] # convert to rgb
        
        org_h, org_w = image.shape[0], image.shape[1]
        org_sz = np.array([org_h, org_w]) 
        
        center = (org_w / 2, org_h / 2)
        pp = (self.input_width / 2, self.input_height / 2)
        rho = 2.0/np.minimum(self.input_width, self.input_height)
        s = max(image.shape[0], image.shape[1]) / 200 # scale
        shape = (self.input_width, self.input_height)
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        org_segs = lsd(gray, scale=0.8)
        org_segs = filter_length(org_segs, self.min_line_length)
        num_segs = len(org_segs)
        assert num_segs > 10, print(num_segs)

        # ---crop---
        cropped = crop(image, center, s, shape).astype(np.float32)
        for i in range(num_segs):
            org_segs[i][0:2] = transform(org_segs[i][0:2], center, s, shape)
            org_segs[i][2:4] = transform(org_segs[i][2:4], center, s, shape)

        # cropped_visual = cropped.copy()
        # draw_segs(cropped_visual, org_segs)
        # cropped_visual = cropped_visual[:,:,::-1]  
        # cv2.imwrite('cropped.png', cropped_visual)    
            
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
            masks = create_masks(cropped)
            
        cropped = np.ascontiguousarray(cropped)
        
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
        target['filename'] = self.list_filename[idx]
        
        extra['lines'] = target['lines'].clone()
        extra['line_mask'] = target['line_mask'].clone()

        return self.transform(cropped, extra, target)
    
    def __len__(self):
        return len(self.list_filename)   

def make_transform():
    return T.Compose([
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]) 

def build_image(cfg):
    dataset = ImageDataset(cfg, return_masks=cfg.MODELS.MASKS, transform=make_transform())
    return dataset

