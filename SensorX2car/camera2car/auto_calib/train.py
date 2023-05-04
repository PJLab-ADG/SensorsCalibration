# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
import argparse
import datetime
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, DistributedSampler
from torch.utils.tensorboard import SummaryWriter

import datasets
import util.misc as utils
from datasets import build_kitti_dataset
from engine import evaluate, train_one_epoch
from models import build_model
from config import cfg

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
                        nargs=argparse.REMAINDER)
    
    return parser


def main(cfg):
    utils.init_distributed_mode(cfg)
    #print("git:\n  {}\n".format(utils.get_sha()))

    if utils.is_main_process():
        writer = SummaryWriter()

    if cfg.MODELS.FROZEN_WEIGHT is not None:
        assert cfg.MODELS.MASKS, "Frozen training is meant for segmentation only"
    print(cfg)

    device = torch.device(cfg.DEVICE)

    # fix the seed for reproducibility
    seed = cfg.SEED + utils.get_rank()
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    model, criterion = build_model(cfg)
    model.to(device)

    model_without_ddp = model
    if cfg.DISTRIBUTED:
        model = torch.nn.parallel.DistributedDataParallel(model, device_ids=[cfg.GPU], find_unused_parameters=True)
        model_without_ddp = model.module
    n_parameters = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print('number of params:', n_parameters)

    param_dicts = [
        {"params": [p for n, p in model_without_ddp.named_parameters() if "backbone" not in n and p.requires_grad]},
        {
            "params": [p for n, p in model_without_ddp.named_parameters() if "backbone" in n and p.requires_grad],
            "lr": cfg.SOLVER.LR_BACKBONE,
        },
    ]
    optimizer = torch.optim.AdamW(param_dicts, lr=cfg.SOLVER.LR,
                                  weight_decay=cfg.SOLVER.WEIGHT_DECAY)
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, cfg.SOLVER.LR_DROP)
    
    build_dataset = build_kitti_dataset

    dataset_train = build_dataset(image_set='train', cfg=cfg)
    dataset_val = build_dataset(image_set='val', cfg=cfg)

    if cfg.DISTRIBUTED:
        sampler_train = DistributedSampler(dataset_train)
        sampler_val = DistributedSampler(dataset_val, shuffle=False)
    else:
        sampler_train = torch.utils.data.RandomSampler(dataset_train)
        sampler_val = torch.utils.data.SequentialSampler(dataset_val)

    batch_sampler_train = torch.utils.data.BatchSampler(
        sampler_train, cfg.SOLVER.BATCH_SIZE, drop_last=True)

    data_loader_train = DataLoader(dataset_train, 
                                   batch_sampler=batch_sampler_train,
                                   collate_fn=utils.collate_fn, 
                                   num_workers=cfg.NUM_WORKERS)
                                   
    data_loader_val = DataLoader(dataset_val, 
                                 cfg.SOLVER.BATCH_SIZE, 
                                 sampler=sampler_val,
                                 drop_last=False, 
                                 collate_fn=utils.collate_fn, 
                                 num_workers=cfg.NUM_WORKERS)

    if cfg.MODELS.FROZEN_WEIGHT is not None:
        checkpoint = torch.load(cfg.MODELS.FROZEN_WEIGHT, map_location='cpu')
        model_without_ddp.detr.load_state_dict(checkpoint['model'])

    output_dir = Path(cfg.OUTPUT_DIR)
    if cfg.RESUME:
        checkpoint = torch.load(cfg.RESUME, map_location='cpu')
        model_without_ddp.load_state_dict(checkpoint['model'])
        if (not cfg.VAL and 
           'optimizer' in checkpoint and 
           'lr_scheduler' in checkpoint and 
           'epoch' in checkpoint):
            optimizer.load_state_dict(checkpoint['optimizer'])
            lr_scheduler.load_state_dict(checkpoint['lr_scheduler'])
            cfg.START_EPOCH = checkpoint['epoch'] + 1

    print("Start training")
    start_time = time.time()
    for epoch in range(cfg.START_EPOCH, cfg.SOLVER.EPOCHS):
        if cfg.DISTRIBUTED:
            sampler_train.set_epoch(epoch)
        train_stats = train_one_epoch(
            model, criterion, data_loader_train, optimizer, device, epoch,
            cfg.SOLVER.CLIP_MAX_NORM)
        lr_scheduler.step()
        if cfg.OUTPUT_DIR:
            checkpoint_paths = [output_dir / 'checkpoint.pth']
            # extra checkpoint before LR drop and every 100 epochs
            if ((epoch + 1) % cfg.SOLVER.LR_DROP == 0 or 
                (epoch + 1) % cfg.CHECKPOINT_PERIOD == 0):
                checkpoint_paths.append(output_dir / f'checkpoint{epoch:04}.pth')
            for checkpoint_path in checkpoint_paths:
                utils.save_on_master({
                    'model': model_without_ddp.state_dict(),
                    'optimizer': optimizer.state_dict(),
                    'lr_scheduler': lr_scheduler.state_dict(),
                    'epoch': epoch,
                    'cfg': cfg,
                }, checkpoint_path)
        
        val_stats = evaluate(
            model, criterion, data_loader_val, device, cfg.OUTPUT_DIR
        )

        log_stats = {**{f'train_{k}': v for k, v in train_stats.items()},
                     **{f'val_{k}': v for k, v in val_stats.items()},
                     'epoch': epoch,
                     'n_parameters': n_parameters}
  
        if cfg.OUTPUT_DIR and utils.is_main_process():
            with (output_dir / "log.txt").open("a") as f:
                f.write(json.dumps(log_stats) + "\n")
            
        if utils.is_main_process():
            loss_names = ['loss'] + list(dict(cfg.LOSS.WEIGHTS).keys())
            for loss_name in loss_names:
                writer.add_scalar(f'train/{loss_name}', train_stats[loss_name], epoch)
                writer.add_scalar(f'val/{loss_name}', val_stats[loss_name], epoch)
            
    total_time = time.time() - start_time
    total_time_str = str(datetime.timedelta(seconds=int(total_time)))
    print('Training time {}'.format(total_time_str))


if __name__ == '__main__':
    parser = argparse.ArgumentParser('GPANet training script', 
                                     parents=[get_args_parser()])
    args = parser.parse_args()
    
    cfg.merge_from_file(args.config_file)
    # cfg.merge_from_list(args.opts)
    
    if cfg.OUTPUT_DIR:
        Path(cfg.OUTPUT_DIR).mkdir(parents=True, exist_ok=True)
    main(cfg)
