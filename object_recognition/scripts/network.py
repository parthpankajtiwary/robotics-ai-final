import tensorflow as tf
import numpy as np
import cv2
import re

from tensorflow.keras.layers import Conv2D, Dense, MaxPool2D, Input, Flatten

# FLAGS = tf.app.flags.FLAGS
# TOWER_NAME = 'tower'

# class Network:
#   def __init__(self, sess):
#     self.inputWidth, self.inputHeight = (128, 128)
#     self.inputChannels = 3
#     self.nLabels = 5
#     self.learningRate = 0.002
#     self.sess = sess
#     self.build()
#
#     variables_to_restore_full = tf.contrib.framework.get_variables_to_restore()
#     variables_to_restore = []
#
#     for var in variables_to_restore_full:
#         if 'Adam' in var.name:
#             continue
#         print(var)
#         variables_to_restore.append(var)
#
#     self.saver = tf.train.Saver(variables_to_restore)#
#
#   def weight_variable(self, shape):
#     initial = tf.truncated_normal(shape, stddev=0.1)
#     return tf.Variable(initial)
#
#   def bias_variable(self, shape):
#     initial = tf.constant(0.1, shape=shape)
#     return tf.Variable(initial)
#
#   def conv2d(self, x, W, padding = 'SAME'):
#     return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')
#
#   def max_pool_2x2(self, x, padding = 'SAME'):
#     return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding=padding)
#
#   def store_checkpoint(self, ckpt):
#       self.saver.save(self.sess, ckpt)
#
#   def load_checkpoint(self, ckpt):
#       self.saver.restore(self.sess, ckpt)
#
#   def build(self):
#
#     #with tf.device('/gpu:0'):
#     self.keep_prob = tf.placeholder(tf.float32)
#     self.x = tf.placeholder(tf.float32, shape=[None, self.inputHeight, self.inputWidth, self.inputChannels])
#     self.target_output = tf.placeholder(tf.float32, shape=[None, self.nLabels])
#
#     image = tf.reshape(self.x, [-1, self.inputWidth, self.inputHeight, self.inputChannels])
#
#     with tf.name_scope('reshape'):
#         x_image = tf.reshape(image, [-1, 128, 128, 3])
#
#     # First convolutional layer - maps one grayscale image to 32 feature maps.
#     with tf.name_scope('conv1'):
#         W_conv1 = self.weight_variable([3, 3, 3, 32])
#         b_conv1 = self.bias_variable([32])
#         h_conv1 = tf.nn.relu(self.conv2d(x_image, W_conv1) + b_conv1)
#
#     # Pooling layer - downsamples by 2X.
#     with tf.name_scope('pool1'):
#         h_pool1 = self.max_pool_2x2(h_conv1)
#
#     # Second convolutional layer -- maps 32 feature maps to 64.
#     with tf.name_scope('conv2'):
#         W_conv2 = self.weight_variable([3, 3, 32, 64])
#         b_conv2 = self.bias_variable([64])
#         h_conv2 = tf.nn.relu(self.conv2d(h_pool1, W_conv2) + b_conv2)
#
#     # Second pooling layer.
#     with tf.name_scope('pool2'):
#         h_pool2 = self.max_pool_2x2(h_conv2)
#
#     with tf.name_scope('conv3'):
#         W_conv3 = self.weight_variable([3, 3, 64, 128])
#         b_conv3 = self.bias_variable([128])
#         h_conv3 = tf.nn.relu(self.conv2d(h_pool2, W_conv3) + b_conv3)
#
#     # Second pooling layer.
#     with tf.name_scope('pool3'):
#         h_pool3 = self.max_pool_2x2(h_conv3)
#
#     with tf.name_scope('conv4'):
#         W_conv4 = self.weight_variable([3, 3, 128, 256])
#         b_conv4 = self.bias_variable([256])
#         h_conv4 = tf.nn.relu(self.conv2d(h_pool3, W_conv4) + b_conv4)
#
#     # Second pooling layer.
#     with tf.name_scope('pool4'):
#         h_pool4 = self.max_pool_2x2(h_conv4)
#
#     with tf.name_scope('conv5'):
#         W_conv5 = self.weight_variable([3, 3, 256, 256])
#         b_conv5 = self.bias_variable([256])
#         h_conv5 = tf.nn.relu(self.conv2d(h_pool4, W_conv5) + b_conv5)
#
#     # Second pooling layer.
#     with tf.name_scope('pool5'):
#         h_pool5 = self.max_pool_2x2(h_conv5)
#
#     # Fully connected layer 1 -- after 2 round of downsampling, our 28x28 image
#     # is down to 7x7x64 feature maps -- maps this to 1024 features.
#     with tf.name_scope('fc1'):
#         W_fc1 = self.weight_variable([4 * 4 * 256, 512])
#         b_fc1 = self.bias_variable([512])
#
#         h_pool5_flat = tf.reshape(h_pool5, [-1, 4 * 4 * 256])
#         h_fc1 = tf.nn.relu(tf.matmul(h_pool5_flat, W_fc1) + b_fc1)
#
#     # Dropout - controls the complexity of the model, prevents co-adaptation of
#     # features.
#     with tf.name_scope('dropout'):
#         h_fc1_drop = tf.nn.dropout(h_fc1, self.keep_prob)
#
#     # Map the 1024 features to 5 classes, one for each digit
#     with tf.name_scope('fc2'):
#         W_fc2 = self.weight_variable([512, 5])
#         b_fc2 = self.bias_variable([5])
#
#         y = tf.matmul(h_fc1_drop, W_fc2) + b_fc2
#         self.softmax_out = tf.nn.softmax(y)
#     self.cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels = self.target_output, logits = y), name="mean_cross_entropy")
#     self.train_step = tf.train.AdamOptimizer(self.learningRate).minimize(self.cross_entropy)
#     # self.train_step = tf.train.GradientDescentOptimizer(self.learningRate).minimize(self.cross_entropy)
#
#
#     self.correct_prediction = tf.equal(tf.argmax(y ,1), tf.argmax(self.target_output,1), name = "correct")
#     self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32), name = "accuracy")
#
#     # self.saver = tf.train.Saver()
#
#     self.summary = tf.summary.merge_all()
#     self.variables = tf.global_variables_initializer()
#     print(self.sess.run(self.variables))
#
#     self.training_summary = tf.summary.scalar("training accuracy", self.accuracy)
#     self.loss_summary = tf.summary.scalar("loss", self.cross_entropy)
#     self.merged = tf.summary.merge_all()
#     self.summary_writer = tf.summary.FileWriter("./tensorboard/", self.sess.graph)
#
#   def train_batch(self, data, labels):
#     #print "input shape@train: ", data.shape
#     self.train_step.run(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: .5})
#
#   def test_batch(self, data, labels):
#     #print "input shape@test: ", data.shape
#     return self.accuracy.eval(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: 1.0})
#
#
#   def feed_batch(self, data):
#     out = self.softmax_out.eval(session = self.sess, feed_dict = {self.x: data, self.keep_prob: 1.0})
#     print("FEED OUTPUT: ", out, "_shape: ", out.shape)
#     return out


class Network():
    def __init__(self, sess):
      self.inputWidth, self.inputHeight, self.inputChannels = (128, 128, 3)
      self.nLabels = 5
      self.sess = sess
      self.logdir = './checkpoint'
      self.learningRate = 0.002
      self.build()

    def store_checkpoint(self, name):
        self.saver.save(self.sess, self.logdir + '/' + name + '.ckpt')

    def load_checkpoint(self, name, logdir = None):
        dir = logdir if logdir is not None else self.logdir

        self.saver.restore(self.sess, dir + '/' + name + '.ckpt')

    def build(self):
        self.keep_prob = tf.placeholder(tf.float32)
        self.x = tf.placeholder(tf.float32, shape=[None, self.inputHeight, self.inputWidth, self.inputChannels])
        self.target_output = tf.placeholder(tf.float32, shape=[None, self.nLabels])

        image = tf.reshape(self.x, [-1, self.inputWidth, self.inputHeight, self.inputChannels])
        default = {'kernel_size' : 3, 'activation' : 'relu', 'padding' : 'same', 'use_bias' : True}

        img_inputs = tf.keras.Input(tensor=image)
        conv1 = Conv2D(filters=32, **default)(img_inputs)
        pool1 = MaxPool2D(pool_size=2)(conv1)
        conv2 = Conv2D(filters=64, **default)(pool1)
        pool2 = MaxPool2D(pool_size=2)(conv2)
        conv3 = Conv2D(filters=128, **default)(pool2)
        pool3 = MaxPool2D(pool_size=2)(conv3)
        conv4 = Conv2D(filters=256, **default)(pool3)
        pool4 = MaxPool2D(pool_size=2)(conv4)
        conv5 = Conv2D(filters=256, **default)(pool4)
        pool5 = MaxPool2D(pool_size=2)(conv5)
        flatt = Flatten()(pool5)
        print(flatt)
        dens1 = Dense(512, use_bias=True, activation='relu')(flatt)
        # drop1 = Dropout()
        drop1 = tf.nn.dropout(dens1, self.keep_prob)
        y = Dense(5, use_bias=True)(drop1)
        self.softmax_out = tf.nn.softmax(y)

        print('y', y)
        print('self.target_output', self.target_output)
        # exit()

        self.cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels = self.target_output, logits = y), name="mean_cross_entropy")
        self.correct_prediction = tf.equal(tf.argmax(y ,1), tf.argmax(self.target_output,1), name = "correct")
        self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32), name = "accuracy")

        variables_to_restore = []
        for var in tf.contrib.framework.get_variables_to_restore():
            if 'Adam' in var.name:
                continue
            print(var)
            variables_to_restore.append(var)
        self.saver = tf.train.Saver(variables_to_restore)#
        # self.saver = tf.train.Saver()

        with tf.control_dependencies(tf.get_collection(tf.GraphKeys.UPDATE_OPS)):
            self.train_step = tf.train.AdamOptimizer(self.learningRate).minimize(self.cross_entropy)

        # self.summary_writer = tf.summary.FileWriter(self.logdir, self.sess.graph)

        self.sess.run(tf.global_variables_initializer())

    def train_batch(self, data, labels):
        #print "input shape@train: ", data.shape
        self.train_step.run(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: .5})

    def test_batch(self, data, labels):
        #print "input shape@test: ", data.shape
        return self.accuracy.eval(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: 1.0})

    def feed_batch(self, data):
        out = self.softmax_out.eval(session = self.sess, feed_dict = {self.x: data, self.keep_prob: 1.0})
        print("FEED OUTPUT: ", out, "_shape: ", out.shape)
        return out

# def Unet():
#     def __init__(self, input, mask, labels=None):
#         self.nClasses = 5
#         self.height, self.width, self.channels = 200, 200, 3
#         self.logdir = 'unetlog'
#
#         self.learningRate = 2e-4
#         self.build(input, mask)
#
#         self.logTrainStats = lambda epoch, scores : self.log_stats(epoch, scores, labels = ['ClassTrainAccuracy', 'ClassTrainCE', 'DiceTrain'])
#         self.logValStats = lambda epoch, scores : self.log_stats(epoch, scores, labels = ['ClassValAccuracy', 'ClassValCE', 'DiceVal'])
#         self.logTestStats = lambda epoch, scores : self.log_stats(epoch, scores, labels = ['ClassTestAccuracy', 'ClassTestCE', 'DiceTest'])
#
#     def save(self, name):
#         self.saver.save(self.sess, self.logdir + '/' + name + '.ckpt')
#
#     def restore(self, name, logdir = None):
#         dir = logdir if logdir is not None else self.logdir
#
#         self.saver.restore(self.sess, dir + '/' + name + '.ckpt')
#
#     def conv2d(self, input, filters):
#         layer = tf.layers.Conv2D(filters = filters, kernel_size = 3, padding = "VALID", use_bias = False)(input)
#         return tf.nn.relu(layer)
#
#     def build(self, input, mask):
#         config = tf.ConfigProto()
#         config.gpu_options.allow_growth = True
#         self.sess = tf.InteractiveSession(config = config)
#
#
#         padding = 97
#         self.input = input#tf.placeholder(tf.float32, shape = [None, self.height, self.width, self.channels])
#         self.training = tf.placeholder(tf.bool)
#         input_padded = tf.pad(self.input, [[0,0], [padding, padding], [padding, padding], [0,0]], "CONSTANT")
#         #self.y = tf.placeholder(tf.float32, shape = [None, self.height, self.width, self.channels])
#         self.targets = mask#tf.placeholder(tf.float32, shape = [None, self.height, self.width, self.nClasses])
#
#         self.y = tf.layers.Conv2D(filters = self.nClasses, kernel_size = 1, padding = 'VALID')(c18)
#         self.y_softmax = tf.nn.softmax(self.y)
#         self.y_onehot = tf.one_hot(tf.argmax(self.y_softmax, axis = 3), depth = 21)
#         self.cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels = self.targets, logits = self.y), name='cross_entropy')
#         self.y_sum = tf.reduce_sum(self.y_softmax)
#         self.tarsum = tf.reduce_sum(self.targets)
#         intersection = tf.reduce_sum(self.y_softmax * self.targets)
#         self.int = intersection
#         union = 0.00001 + tf.reduce_sum(self.y_softmax) + tf.reduce_sum(self.targets)
#         self.union = union
#         self.dice = 2.0 * intersection / union
#
#         hardIntersect = tf.reduce_sum(self.y_onehot * self.targets)
#         hardUnion = 0.00001 + tf.reduce_sum(self.y_onehot) + tf.reduce_sum(self.targets)
#         self.hardDice = tf.math.multiply(hardIntersect / hardUnion, 2.0, name='hardDice')
#
#         negative_intersection = tf.reduce_sum(self.y_softmax * (1.0 - self.targets))
#         negative_union = 0.00001 + tf.reduce_sum(self.y_softmax) + tf.reduce_sum((1.0 - self.targets))
#         self.negdice = 2 * negative_intersection / negative_union
#
#
#         with tf.control_dependencies(tf.get_collection(tf.GraphKeys.UPDATE_OPS)):
#             self.optimizer = tf.train.AdamOptimizer(self.learningRate).minimize(self.cross_entropy - self.dice + self.negdice)# + tf.log(self.negdice))
#         self.accuracy = tf.reduce_mean(tf.cast(tf.equal(tf.argmax(self.y_softmax, 3), tf.argmax(self.targets, 3)), tf.float32), name='accuracy_tf.equal')
#         self.saver = tf.train.Saver()
#
#         self.summary_writer = tf.summary.FileWriter(self.logdir, self.sess.graph)
#         self.sess.run(tf.global_variables_initializer())
#
#     def predict_segmentation(self, inputs):
#         heat = self.sess.run(self.y_softmax, feed_dict = {self.training : False, self.input : inputs})
#         return np.argmax(heat, axis=-1)
#
#     def log_stats(self, epoch, scores, labels):
#         summ = tf.Summary()
#         for idx, score in enumerate(scores):
#             summ.value.add(tag=labels[idx], simple_value=scores[idx])
#
#         try:
#             self.summary_writer.add_summary(summ, epoch)
#         except:
#             print('abstract_network.py: Summary writer possibly undefined.')
#
#     def __del__(self):
#         self.sess.close()
# tf.reset_default_graph()

if __name__ == "__main__":
    with tf.InteractiveSession().as_default() as sess:
        network = Network(sess)
