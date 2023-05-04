'''
usage:
python test_img.py --config-file config-files/ctrlc.yaml --opts MODE test DATASET_DIR ./pic/ OUTPUT_DIR ./output/
'''
import os
import os.path as osp
import argparse
from datetime import date
import json
import random
import time
from pathlib import Path
import numpy as np
import numpy.linalg as LA
from tqdm import tqdm
import matplotlib as mpl
import matplotlib.pyplot as plt
import cv2
import csv
import math
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
import util.misc as utils
from datasets import build_image_dataset
from models import build_model
from config import cfg

cmap = plt.get_cmap("jet")
norm = mpl.colors.Normalize(vmin=0.0, vmax=1.0)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])

def c(x):
    return sm.to_rgba(x)

def get_args_parser():
    parser = argparse.ArgumentParser('Set gptran', add_help=False)
    parser.add_argument('--config-file', 
                        metavar="FILE",
                        help="path to config file",
                        type=str,
                        default='config-files/ctrlc.yaml')
    parser.add_argument("--opts",
                        help="Modify config options using the command-line",
                        default=None,
                        nargs=argparse.REMAINDER
                        )
    return parser

def compute_horizon(angle, vp, img_sz):
    hl_left = vp[1] + (vp[0] + img_sz[1] / 2) * math.tan(angle)
    hl_right = vp[1] + (vp[0] - img_sz[1] / 2) * math.tan(angle)
    return hl_left, hl_right
    

def to_device(data, device):
    if type(data) == dict:
        return {k: v.to(device) for k, v in data.items()}
    return [{k: v.to(device) if isinstance(v, torch.Tensor) else v
             for k, v in t.items()} for t in data]


def main(cfg):
    device = torch.device(cfg.DEVICE)
    
    model, _ = build_model(cfg)
    model.to(device)
    
    dataset_test = build_image_dataset(cfg)
    sampler_test = torch.utils.data.SequentialSampler(dataset_test)
    data_loader_test = DataLoader(dataset_test, 1, sampler=sampler_test,
                                 drop_last=False, 
                                 collate_fn=utils.collate_fn, 
                                 num_workers=1)
    
    checkpoint = torch.load(cfg.LOAD, map_location='cpu') # change model
    model.load_state_dict(checkpoint['model'])
    model = model.eval()
    
    start = time.time()
    image_num = 0

    for i, (samples, extra_samples, targets) in enumerate(tqdm(data_loader_test)):
        with torch.no_grad():
            samples = samples.to(device)
            extra_samples = to_device(extra_samples, device)
            outputs = model(samples, extra_samples)

            filename = targets[0]['filename']
            filename = osp.splitext(filename)[0]

            pred_vp = outputs['pred_vp'].to('cpu')[0].numpy()
            pred_hl = outputs['pred_hl'].to('cpu')[0].numpy()

            input_sz = targets[0]['input_sz']
            rho = 2.0/np.minimum(input_sz[0],input_sz[1])

            img = targets[0]['org_img']
            origin_sz = (img.shape[1], img.shape[0])
            
            pred_vp /= pred_vp[2]
            crop_vp = np.array((pred_vp[0] / rho, pred_vp[1] / rho))
            
            crop_left, crop_right = compute_horizon(pred_hl, crop_vp, input_sz)
            
            vp = np.ones(3)
            vp[0] = crop_vp[0] / input_sz[0] * max(origin_sz[0], origin_sz[1]) + origin_sz[0] / 2
            vp[1] = crop_vp[1] / input_sz[0] * max(origin_sz[0], origin_sz[1]) + origin_sz[1] / 2
            
            hl_left = crop_left / input_sz[0] * max(origin_sz[0], origin_sz[1]) + origin_sz[1] / 2
            hl_right = crop_right / input_sz[0] * max(origin_sz[0], origin_sz[1]) + origin_sz[1] / 2


            cv2.line(img, (0, int(hl_left)), (origin_sz[0] - 1, int(hl_right)), (255, 0, 0), 2)
            cv2.circle(img, (int(vp[0]), int(vp[1])), 6, (0, 0, 255), -1)
            
            img_name = cfg.OUTPUT_DIR +'/' + filename + '.png'
            cv2.imwrite(img_name, img)
            image_num += 1
            
            print("filename: ", filename)
            print("VP: ({}, {})".format(vp[0], vp[1]))
            print("HL angle: {} rad".format(pred_hl))

    end = time.time()
    print("total inferece time:", end - start)
    print("FPS:", image_num / (end - start))                

            
if __name__ == '__main__':
    parser = argparse.ArgumentParser('GPANet training and evaluation script', 
                                     parents=[get_args_parser()])
    args = parser.parse_args()
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    
    if cfg.OUTPUT_DIR:
        Path(cfg.OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
    main(cfg)
