import numpy as np

data = np.fromfile('python/kitti_319.bin',dtype=np.float32)
data = data.reshape(-1,4)
print (np.max(data[:,3]))
print (np.min(data[:,3]))
print (data[0,:])