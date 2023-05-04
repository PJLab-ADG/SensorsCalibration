'''
usage:
python test_kitti.py --config-file config-files/ctrlc.yaml --opts MODE test
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
import math
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
import sklearn.metrics
import util.misc as utils
from datasets import build_kitti_dataset
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

def compute_vp_error2(pred_vp, target_vp):
    error_vp = LA.norm(pred_vp - target_vp)
    return error_vp
                    
def compute_hl(hl, img_sz):
    y_left = - hl[2] / hl[1]
    y_right = y_left - hl[0] / hl[1] * img_sz[1]
    return y_left,y_right

def normalize_safe_np(v, eps=1e-7):
    return v/np.maximum(LA.norm(v), eps)

def compute_horizon_error(pred_hl, target_hl, img_sz):
    target_hl_pts = compute_hl(target_hl, img_sz)
    pred_hl_pts = compute_hl(pred_hl, img_sz)
    err_hl = np.maximum(np.abs(target_hl_pts[0] - pred_hl_pts[0]),
                        np.abs(target_hl_pts[1] - pred_hl_pts[1]))
    err_hl /= img_sz[0] # height
    return err_hl

def compute_vp_error(pred_vp, target_vp):
    cos_sim = abs(np.dot(pred_vp, target_vp)/(LA.norm(pred_vp) * LA.norm(target_vp)))      
    error_vp = (1.0 - cos_sim).mean()
    
    return error_vp

def to_device(data, device):
    if type(data) == dict:
        return {k: v.to(device) for k, v in data.items()}
    return [{k: v.to(device) if isinstance(v, torch.Tensor) else v
             for k, v in t.items()} for t in data]

def calc_auc(error_array, cutoff=0.25):

    error_array = error_array.squeeze()
    error_array = np.sort(error_array)
    num_values = error_array.shape[0]

    plot_points = np.zeros((num_values, 2))

    midfraction = 1.

    for i in range(num_values):
        fraction = (i + 1) * 1.0 / num_values
        value = error_array[i]
        plot_points[i, 1] = fraction
        plot_points[i, 0] = value
        if i > 0:
            lastvalue = error_array[i - 1]
            if lastvalue < cutoff < value:
                midfraction = (lastvalue * plot_points[i - 1, 1] + value * fraction) / (value + lastvalue)

    if plot_points[-1, 0] < cutoff:
        plot_points = np.vstack([plot_points, np.array([cutoff, 1])])
    else:
        plot_points = np.vstack([plot_points, np.array([cutoff, midfraction])])

    sorting = np.argsort(plot_points[:, 0])
    plot_points = plot_points[sorting, :]

    auc = sklearn.metrics.auc(plot_points[plot_points[:, 0] <= cutoff, 0],
                              plot_points[plot_points[:, 0] <= cutoff, 1])
    auc = auc / cutoff

    return auc, plot_points

def main(cfg):
    device = torch.device(cfg.DEVICE)
    
    model, _ = build_model(cfg)
    model.to(device)
    
    build_dataset = build_kitti_dataset

    dataset_test = build_dataset(image_set='test', cfg=cfg)
    sampler_test = torch.utils.data.SequentialSampler(dataset_test)
    data_loader_test = DataLoader(dataset_test, 1, sampler=sampler_test,
                                 drop_last=False, 
                                 collate_fn=utils.collate_fn, 
                                 num_workers=2)
    
    checkpoint = torch.load(cfg.LOAD, map_location='cpu') # change model
    model.load_state_dict(checkpoint['model'])
    model = model.eval()
    
    output_folder = './output'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        
    start_time = time.time()
    
    vp_errors_cos = []
    vp_errors_l2 = []
    hl_errors = []
    for i, (samples, extra_samples, targets) in enumerate(tqdm(data_loader_test)):
        with torch.no_grad():
            samples = samples.to(device)
            extra_samples = to_device(extra_samples, device)
            outputs = model(samples, extra_samples)        
            pred_vp = outputs['pred_vp'].to('cpu')[0].numpy()
            pred_hl = outputs['pred_hl'].to('cpu')[0].numpy()

            img_sz = targets[0]['input_sz'] #(512,512)
            pp = (img_sz[1]/2, img_sz[0]/2) #(256,256)
            rho = 2.0 / np.minimum(img_sz[0],img_sz[1])
            
            filename = targets[0]['filename']
            filename = osp.splitext(filename)[0]


            target_hl = targets[0]['hl'].numpy()
            target_vp = targets[0]['vp'].numpy()
                            

            # vp error
            err_vp = compute_vp_error(pred_vp, target_vp)
            vp_errors_cos.append(err_vp)
            
            target_vp = target_vp / target_vp[2]
            target_vp[0] = target_vp[0] / rho + pp[0]
            target_vp[1] = target_vp[1] / rho + pp[1]
            pred_vp = pred_vp / pred_vp[2]
            pred_vp[0] = pred_vp[0] / rho + pp[0]
            pred_vp[1] = pred_vp[1] / rho + pp[1]
            err_vp = compute_vp_error2(pred_vp, target_vp)
            vp_errors_l2.append(err_vp)
            
            # horizon line error
            err_hl = abs(math.tan(target_hl)-math.tan(pred_hl))
            hl_errors.append(err_hl)           
            

    end_time = time.time()
    vp_error_cos_avg = np.array(vp_errors_cos).mean()
    vp_error_l2_avg = np.array(vp_errors_l2).mean()
    
    hl_errors = np.array(hl_errors)
    hl_error_avg = hl_errors.mean()
    auc_angle, plot_points = calc_auc(hl_errors, cutoff=0.05)
    MSE = np.square(hl_errors).mean()
    
    print("average_vp_error(cosine similarity) = {}".format(vp_error_cos_avg))
    print("average_vp_error(L2) = {}".format(vp_error_l2_avg))
    print("average_hl_error(angle) = {}".format(hl_error_avg))
    print("MSE(angle) = {}".format(MSE))
    print("AUC(angle):{}".format(auc_angle))
    print("total_time of testing {} images = {}".format(i + 1, end_time - start_time))
    print("FPS = {}".format((i + 1) / (end_time - start_time)))
            
    plt.figure()
    plt.plot(plot_points[:,0], plot_points[:,1], 'b-')
    plt.xlim(0, 0.05)
    plt.ylim(0, 1.0)
    plt.text(0.175, 0.05, "AUC: %.2f" % (auc_angle * 100), fontsize=12)
    plt.suptitle("AUC %.3f %%, MSE %.6f" %
                (auc_angle*100, MSE,), fontsize=10)
    plt.savefig(os.path.join(output_folder, "error_angle_histogram.png"), dpi=300)
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser('GPANet training and evaluation script', 
                                     parents=[get_args_parser()])
    args = parser.parse_args()
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    
    if cfg.OUTPUT_DIR:
        Path(cfg.OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
    main(cfg)