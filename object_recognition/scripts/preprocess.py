
import cv2
import numpy as np

height = 64
width = 64
nClasses = 5
def preprocess(image):
  print(image.shape)
  image = cv2.resize(image, (height, width))
  image = image.reshape((-1, height, width, 3))
  image = image.astype('uint8')
  #image = image.astype('float') / 255
  # This is now done in train.py and test.py when a batch is needed. Saves RAM..
  return image

def oneHot(size, idx):
  vec = np.zeros((size), dtype = 'byte')
  vec[idx] = 1
  vec = vec.reshape((-1, nClasses))
  return vec
