import os
import numpy as np
import cv2
from preprocess import preprocess, oneHot

DATADIR = os.environ['HOME'] + "/Desktop/augmented/"
NPY_STORAGE = os.environ['HOME'] + "/numpy/augmented/"
imHeight = 64
imWidht  = 64

nLabels = len(os.listdir(DATADIR))
init = False
trainData = np.asarray([])
trainLabels = np.asarray([]) ### todo, initcheck
testLabels = np.asarray([])
testData = np.asarray([])

labels = os.listdir(DATADIR)
nLabels = len(labels)
labelFile = open('labels.txt', 'w+')
oneHotLabels = {}
labelIdx = 0
for label in labels:
  labelFile.write(label + '\n')
  oneHotLabels[label] = oneHot(nLabels, labelIdx)
  labelIdx += 1

labelFile.close()
for label in labels:
  print "Processing " + label

  currentData = []
  currentLabels = []
  dataInitialized = False
  print(DATADIR + '/' + label)
  for x in os.walk(DATADIR + label): #1 iter
    nImgs = len(x[2])
    currentData = np.zeros((nImgs, imHeight, imWidht, 3), dtype = 'uint8')
    currentLabels = np.zeros((nImgs, len(labels)), dtype = 'uint8')
    imIdx = 0
    for im in x[2]:

      print "reading ", (DATADIR + label + '/' + im)
      image = cv2.imread(DATADIR + label + '/' + im)
      currentData[imIdx] = preprocess(image)
      currentLabels[imIdx] = np.asarray(oneHotLabels[label])
      imIdx += 1
      continue
      tgt = preprocess(image)
      if not dataInitialized:
        currentData = np.asarray(tgt)
        currentLabels = np.asarray(oneHotLabels[label])
        dataInitialized = True
      else:
        currentData = np.concatenate([currentData, tgt])
        currentLabels = np.concatenate([currentLabels, oneHotLabels[label]])


  currentData = np.asarray(currentData)
  print "nData in class ", currentData.shape
  print "check1: ", currentData.shape
  splitIdx = int(len(currentData) * 0.8)
  train = np.asarray(currentData[:splitIdx])
  print "appending shape ", train.shape
  trainL = np.asarray(currentLabels[:splitIdx])
  test = np.asarray(currentData[splitIdx:])
  testL = np.asarray(currentLabels[splitIdx:])
  if init == False:
    init = True
    trainData = np.asarray(train)
    trainLabels = np.asarray(trainL)
    testData = np.asarray(test)
    testLabels = np.asarray(testL)
  else:
    trainData = np.concatenate([trainData,train])
    trainLabels = np.concatenate([trainLabels, trainL])
    testLabels = np.concatenate([testLabels, testL])
    testData = np.concatenate([testData, test])
print "shuffeling data..."
idx = np.random.permutation(len(trainData))
trainData, trainLabels = trainData[idx], trainLabels[idx]
idx = np.random.permutation(len(testData))
testData, testLabels = testData[idx], testLabels[idx]
print "traindatashape: ", trainData.shape
print "storing data to folder: ", NPY_STORAGE
np.save(NPY_STORAGE + "trainData", trainData)
np.save(NPY_STORAGE + "trainLabels", trainLabels)
np.save(NPY_STORAGE + "testData", testData)
np.save(NPY_STORAGE + "testLabels", testLabels)
