import os 
import sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
from io import BytesIO
import tarfile
import tempfile
from six.moves import urllib

from matplotlib import gridspec
from matplotlib import pyplot as plt
import numpy as np
import time
from PIL import Image
 
import tensorflow as tf 

physical_devices = tf.config.experimental.list_physical_devices('GPU')
assert len(physical_devices) > 0, "Not enough GPU hardware devices available"
tf.config.experimental.set_memory_growth(physical_devices[0], True)

class DeepLabModel(object):
  """Class to load deeplab model and run inference."""

  INPUT_TENSOR_NAME = 'ImageTensor:0'
  OUTPUT_TENSOR_NAME = 'ResizeBilinear_3:0'
  INPUT_SIZE = 769
  FROZEN_GRAPH_NAME = 'frozen_inference_graph'

  def __init__(self, tarball_path):
    """Creates and loads pretrained deeplab model."""
    self.graph = tf.Graph()

    graph_def = None
    # Extract frozen graph from tar archive.
    tar_file = tarfile.open(tarball_path)
    for tar_info in tar_file.getmembers():
      if self.FROZEN_GRAPH_NAME in os.path.basename(tar_info.name):
        file_handle = tar_file.extractfile(tar_info)
        graph_def = tf.GraphDef.FromString(file_handle.read())
        break

    tar_file.close()

    if graph_def is None:
      raise RuntimeError('Cannot find inference graph in tar archive.')

    with self.graph.as_default():
      tf.import_graph_def(graph_def, name='')

    self.sess = tf.Session(graph=self.graph)

  def run(self, image):
    """Runs inference on a single image.

    Args:
      image: A PIL.Image object, raw input image.

    Returns:
      resized_image: RGB image resized from original input image.
      seg_map: Segmentation map of `resized_image`.
    """
    width, height = image.size
    resize_ratio = 1.0 * self.INPUT_SIZE / max(width, height)
    target_size = (int(resize_ratio * width), int(resize_ratio * height))
    resized_image = image.convert('RGB').resize(target_size, Image.ANTIALIAS)
 
   
    batch_seg_map = self.sess.run(
        self.OUTPUT_TENSOR_NAME,
        feed_dict={self.INPUT_TENSOR_NAME: [np.asarray(resized_image)]})   
 

    # all_names = [op.name for op in self.graph.get_operations()]
    # print (all_names)
    sess = tf.Session() 
    with sess.as_default():
        print()
        seg_map = batch_seg_map[:,:int(769*height/width),:769,:]
        seg_map = tf.image.resize_bilinear(seg_map, (height,width), align_corners=True)
        seg_map = tf.nn.softmax(seg_map)
        seg_map = seg_map.eval()
    return resized_image, seg_map


def create_cityscapes_label_colormap():
  """Creates a label colormap used in PASCAL VOC segmentation benchmark.

  Returns:
    A Colormap for visualizing segmentation results.
  """
  colormap = np.zeros((256, 3), dtype=int)
  ind = np.arange(256, dtype=int)

  for shift in reversed(range(8)):
    for channel in range(3):
      colormap[:, channel] |= ((ind >> channel) & 1) << shift
    ind >>= 3

  return colormap

def vis_segmentation(seg_map): 
  seg_image = label_to_color_image(seg_map).astype(np.uint8) 
  pil_img = Image.fromarray(seg_image) 
      


def label_to_color_image(label):
  """Adds color defined by the dataset colormap to the label.

  Args:
    label: A 2D array with integer type, storing the segmentation label.

  Returns:
    result: A 2D array with floating type. The element of the array
      is the color indexed by the corresponding element in the input label
      to the PASCAL color map.

  Raises:
    ValueError: If label is not of rank 2 or its value is larger than color
      map maximum entry.
  """
  if label.ndim != 2:
    raise ValueError('Expect 2-D input label')

  colormap = create_cityscapes_label_colormap()

  if np.max(label) >= len(colormap):
    raise ValueError('label value too large.')

  return colormap[label]


LABEL_NAMES = np.asarray([
    'unlabeled', 'ego vehicle', 'rectification border', 'out of roi', 'static', 'dynamic', 'ground', 'road', 'sidewalk',
    'parking', 'rail track', 'building', 'wall', 'fence', 'guard rail', 'bridge', 'tunnel', 'pole', 'polegroup',          
    'traffic light', 'traffic sign', 'vegetation', 'terrain', 'sky', 'rider', 'car', 'truck', 'bus', 'caravan',             
    'trailer', 'train', 'motorcycle', 'bicycle', 'license plate' 
])

FULL_LABEL_MAP = np.arange(len(LABEL_NAMES)).reshape(len(LABEL_NAMES), 1)
FULL_COLOR_MAP = label_to_color_image(FULL_LABEL_MAP)

MODEL = DeepLabModel("../deeplabv3_cityscapes_train_2018_02_06.tar.gz") 

def run_model(image_name):
  original_im = Image.open(image_name) 
  resized_im, seg_map = MODEL.run(original_im)
  seg_map = np.squeeze(seg_map)
  seg_map_result = np.squeeze(np.argmax(seg_map,2)) 
  vis_segmentation(seg_map_result)
  return (seg_map,seg_map_result)

if __name__ == "__main__":
  if len(sys.argv) != 2:
        print('Usage: ./main img_path')
  else:
    run_model(sys.argv[1])