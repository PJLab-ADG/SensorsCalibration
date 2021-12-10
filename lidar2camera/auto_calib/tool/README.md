# Introduction
This project is used to extract features of lane lines and pillars in the picture.

# Requirements
It is recommended to use the conda environment, created by the following command:
```
conda env create -f lidar2camera.yaml
```
Pull the model by the following command: (pretrained network for semantic segmentation)
```
wget http://download.tensorflow.org/models/deeplabv3_cityscapes_train_2018_02_06.tar.gz
```
# Usage
```
python merge_mask.py calib.png mask.png
```


 
