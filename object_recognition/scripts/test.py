import numpy as np
import cv2
import os
from network import Network
import tensorflow as tf


NPY_STORAGE = os.environ['HOME'] + "/numpy/"

trainData = np.load(NPY_STORAGE + "trainData.npy")
trainLabels = np.load(NPY_STORAGE + "trainLabels.npy")

validationData = np.load(NPY_STORAGE + "testData.npy")
validationLabels = np.load(NPY_STORAGE + "testLabels.npy")

N_BATCHES = 4
CHECKPOINT_DIR = "./checkpoint/network.ckpt"
validationDataBatches = np.array_split(validationData, N_BATCHES)
validationLabelBatches = np.array_split(validationLabels, N_BATCHES)


nData = len(validationData)
print nData
print "nBatches: ", len(validationDataBatches)
remainder_test = nData % N_BATCHES
batchsize = int(nData / N_BATCHES)
nBatchesValidation = int(nData / batchsize)


config = tf.ConfigProto()
config.gpu_options.allow_growth=True
session_recog = tf.InteractiveSession(config = config)
network = Network(session_recog)
network.load_checkpoint(CHECKPOINT_DIR)

def test():

  #nBatches = len(validationDataBatches)

  runningAvg = 0.0
  nBatches = len(validationDataBatches)
  ##print "nData = ", nData
  #print "vallabshape ", validationLabelBatches.shape
  counter = 0
  for batchIdx in range(nBatches):
    data = validationDataBatches[batchIdx]
    data = np.array([cv2.resize(img, (64,64)) for img in data])
    data = data.astype('float')
    data /= 255
    labels = validationLabelBatches[batchIdx]
    labels = labels.astype('float')
    if batchIdx == nBatches - 1 and remainder_test != 0:
        acc = network.test_batch(data[:remainder_test], labels[:remainder_test])
        print acc
        acc *= remainder_test
        counter += remainder_test
    else:
        acc = network.test_batch(data, labels)
        acc *= batchsize
        counter += batchsize
    runningAvg += acc# * len(validationDataBatches)
  runningAvg /= counter
  return runningAvg

  def test_demo_rois():
      folder_loc = "/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/final_demo/rois"
      folder_out = "/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/final_demo/rois_new"

      folder = []
      for img_name in folder:
          img = cv2.imread(folder_loc + 'img', img_name)
          cv2.imshow('please check folderimg - shown img correspondence', img)
          cv2.waitKey()

          results = network.feed_batch([img])
          print(results)

          cv2.imwrite(folder_out + img, img_name)

if __name__ == "__main__":
  print "validation score: ", test()
