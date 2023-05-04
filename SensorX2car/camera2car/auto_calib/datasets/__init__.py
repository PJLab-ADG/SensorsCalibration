from datasets.image_dataset import build_image
from datasets.kitti_dataset import build_kitti

def build_kitti_dataset(image_set, cfg):
    return build_kitti(image_set, cfg)

def build_image_dataset(cfg):
    return build_image(cfg)