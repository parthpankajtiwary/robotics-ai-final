import tensorflow as tf
import numpy as np
import cv2
import re


# FLAGS = tf.app.flags.FLAGS
TOWER_NAME = 'tower'

class Network:
  def __init__(self, sess):
	self.inputWidth = 64
	self.inputHeight = 64
	self.inputChannels = 3
	self.nLabels = 5
	self.learningRate = 0.002
	self.sess = sess
	self.build()

	variables_to_restore_full = tf.contrib.framework.get_variables_to_restore()
	variables_to_restore = []

	for var in variables_to_restore_full:
		if 'Adam' in var.name:
			continue
		variables_to_restore.append(var)

	self.saver = tf.train.Saver(variables_to_restore)

  def weight_variable(self, shape):
	initial = tf.truncated_normal(shape, stddev=0.1)
	return tf.Variable(initial)

  def bias_variable(self, shape):
	initial = tf.constant(0.1, shape=shape)
	return tf.Variable(initial)

  def conv2d(self, x, W, padding = 'SAME'):
	return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

  def max_pool_2x2(self, x, padding = 'SAME'):
	return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding=padding)

  def store_checkpoint(self, ckpt):
	  self.saver.save(self.sess, ckpt)

  def load_checkpoint(self, ckpt):
	  self.saver.restore(self.sess, ckpt)

  def build(self):

	#with tf.device('/gpu:0'):
	self.keep_prob = tf.placeholder(tf.float32)
	self.x = tf.placeholder(tf.float32, shape=[None, self.inputHeight, self.inputWidth, self.inputChannels])
	self.target_output = tf.placeholder(tf.float32, shape=[None, self.nLabels])

	image = tf.reshape(self.x, [-1, self.inputWidth, self.inputHeight, self.inputChannels])

	with tf.name_scope('reshape'):
		x_image = tf.reshape(image, [-1, 64, 64, 3])

	# First convolutional layer - maps one grayscale image to 32 feature maps.
	with tf.name_scope('conv1'):
		W_conv1 = self.weight_variable([5, 5, 3, 64])
		b_conv1 = self.bias_variable([64])
		h_conv1 = tf.nn.relu(self.conv2d(x_image, W_conv1) + b_conv1)

	# Pooling layer - downsamples by 2X.
	with tf.name_scope('pool1'):
		h_pool1 = self.max_pool_2x2(h_conv1)

	# Second convolutional layer -- maps 32 feature maps to 64.
	with tf.name_scope('conv2'):
		W_conv2 = self.weight_variable([5, 5, 64, 128])
		b_conv2 = self.bias_variable([128])
		h_conv2 = tf.nn.relu(self.conv2d(h_pool1, W_conv2) + b_conv2)

	# Second pooling layer.
	with tf.name_scope('pool2'):
		h_pool2 = self.max_pool_2x2(h_conv2)

	# Fully connected layer 1 -- after 2 round of downsampling, our 28x28 image
	# is down to 7x7x64 feature maps -- maps this to 1024 features.
	with tf.name_scope('fc1'):
		W_fc1 = self.weight_variable([16 * 16 * 128, 2048])
		b_fc1 = self.bias_variable([2048])

		h_pool2_flat = tf.reshape(h_pool2, [-1, 16 * 16 * 128])
		h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

	# Dropout - controls the complexity of the model, prevents co-adaptation of
	# features.
	with tf.name_scope('dropout'):
		h_fc1_drop = tf.nn.dropout(h_fc1, self.keep_prob)

	# Map the 1024 features to 5 classes, one for each digit
	with tf.name_scope('fc2'):
		W_fc2 = self.weight_variable([2048, 5])
		b_fc2 = self.bias_variable([5])

		y = tf.matmul(h_fc1_drop, W_fc2) + b_fc2
        self.softmax_out = tf.nn.softmax(y)
	self.cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels = self.target_output, logits = y), name="mean_cross_entropy")
	# self.train_step = tf.train.AdamOptimizer(self.learningRate).minimize(self.cross_entropy)
	self.train_step = tf.train.GradientDescentOptimizer(self.learningRate).minimize(self.cross_entropy)


	self.correct_prediction = tf.equal(tf.argmax(y ,1), tf.argmax(self.target_output,1), name = "correct")
	self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32), name = "accuracy")

	self.summary = tf.summary.merge_all()
	self.variables = tf.global_variables_initializer()
	print self.sess.run(self.variables)

	self.training_summary = tf.summary.scalar("training accuracy", self.accuracy)
	self.loss_summary = tf.summary.scalar("loss", self.cross_entropy)
	self.merged = tf.summary.merge_all()
	self.summary_writer = tf.summary.FileWriter("./tensorboard/", self.sess.graph)

  def train_batch(self, data, labels):
	#print "input shape@train: ", data.shape
	self.train_step.run(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: 1.0})

  def test_batch(self, data, labels):
	#print "input shape@test: ", data.shape
	return self.accuracy.eval(feed_dict = {self.x: data, self.target_output: labels, self.keep_prob: 1.0})


  def feed_batch(self, data):
	out = self.softmax_out.eval(session = self.sess, feed_dict = {self.x: data, self.keep_prob: 1.0})
	print("FEED OUTPUT: ", out, "_shape: ", out.shape)
	return out

if __name__ == "__main__":
  network = Network()
