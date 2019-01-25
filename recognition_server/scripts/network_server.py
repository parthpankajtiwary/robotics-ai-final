import rospy
import actionlib
import numpy as np

from alice_msgs.msg import RecogniseObjectsAction, RecogniseObjectsResult
from matplotlib.pyplot import imshow
from cv2 import resize
import cv2

from alice_msgs.msg import ObjectROIAction, ObjectROIResult, ObjectROIGoal
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge


import sys
sys.path.append("/home/student/sudo/ros/catkin_ws/src/object_recognition/scripts")
from network import Network

CHECKPOINT_DIR = "/home/student/sudo/ros/catkin_ws/src/object_recognition/scripts/ckpt/network.ckpt"

class network_server(object):
    def request_rois(self):
        print("request rois")
        roi_goal = ObjectROIGoal()
        roi_goal.action = "can i get some rois pls?"
        self.client_roi.send_goal(roi_goal)

    def __init__(self):
        self.server = actionlib.SimpleActionServer('neuralObjectRecognition', RecogniseObjectsAction, self.recognise)
        self.server.start()

        self.client_roi = actionlib.SimpleActionClient("get_objects", ObjectROIAction)
        self.client_roi.wait_for_server()

        self.bridge = CvBridge()

        print('network server started')
        self.net = Network()
        self.net.load_checkpoint(CHECKPOINT_DIR)
        print("Network Server Initialised")
        self.classes = ['basetech', 'usbhub', 'eraserbox', 'evergreen', 'tomatosoup']

    def recognise(self, req):
        # Get image
        img = rospy.wait_for_message("/front_xtion/rgb/image_raw", Image)
        specs = rospy.wait_for_message("/front_xtion/rgb/camera_info", CameraInfo)

        # Get rois
        rois = None
        print("self.client_roi.get_state()", self.client_roi.get_state())
        if(self.client_roi.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print("client_roi SUCCEEDED")
            rois = self.client_roi.get_result().roi
        elif self.client_roi.get_state() in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.LOST]:
            print('one of the pending or failed states')
            self.request_rois()

        if rois == None:
            print('rois is 0 and you know what')
            result = RecogniseObjectsResult()
            result.classes = []
            self.server.set_succeeded(result)
            return

        # Use ROI parameters to get image regions
        def get_roi(image, roi):
            return image[roi.top:roi.bottom, roi.left:roi.right]

        # reshape img into 2D, and get its regions using ROIs
        height, width = (specs.height, specs.width)

        img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        img = np.reshape(img, (height, width, 3))
        regions = [get_roi(img, roi) for roi in rois]

        for region in regions:
            cv2.imshow('Cropped Image', region)
            cv2.waitKey(0)

        # Gather data and get predictions
        data = np.stack([resize(img_roi, (32, 32)) for img_roi in regions])
        predictions = self.net.feed_batch(data)
        object_count = 1
        for prediction in predictions:
            print('Object ' + str(object_count) + ' belongs to class: ' +  self.classes[prediction])
            object_count += 1

        # Send result to client
        result = RecogniseObjectsResult()
        result.classes = predictions
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("object_recognition")
    server = network_server()
    rospy.spin()
