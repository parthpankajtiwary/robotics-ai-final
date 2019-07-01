import rospy
import actionlib
from alice_msgs.msg import ObjectROIAction, ObjectROIGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class AliceObject(object):
    
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.client = actionlib.SimpleActionClient("get_objects", ObjectROIAction)
        
        print "Waiting for Alice object server..."
        self.client.wait_for_server()
        print "Connected to Alcie object server"
        
        
    def CreateBinaryImage(self, image_depth):
        delta = (image_depth[0][0] - image_depth[27][0]) / 28.0
        
        for y in range(28):
            for x in range(28):
                image_depth[y][x] += delta * y
        
        min_value = 9999
    	max_value = 0
    	mean_array = []
    	
    	for x in range(28):
    	    for y in range(28):
                if not np.isnan(image_depth[x][y]):
    	            if image_depth[x][y] < min_value:
    	                min_value = image_depth[x][y]
    	            if image_depth[x][y] > max_value:
                        max_value = image_depth[x][y]
                    mean_array.append(image_depth[x][y])
                  
        for x in range(28):
            for y in range(28):
                if np.isnan(image_depth[x][y]):
                    image_depth[x][y] = min_value
        
        mean_value = np.mean(mean_array)
        
        for x in range(28):
            for y in range(28):
                if image_depth[x][y] > mean_value:
                    image_depth[x][y] = 1.0
                else:
                    image_depth[x][y] = 0.0
        
        return image_depth      
                    
    def GetObjectInformation(self):
                        
        image_msgs = rospy.wait_for_message("front_xtion/rgb/image_raw", Image)
        try:
            image_depth_msgs = rospy.wait_for_message("/front_xtion/depth/image", Image, timeout=1.0)
        except:
            image_depth_msgs = rospy.wait_for_message("front_xtion/depth/image_raw", Image)
            
        image = self.cv_bridge.imgmsg_to_cv2(image_msgs, desired_encoding="bgr8")
        image_depth = self.cv_bridge.imgmsg_to_cv2(image_depth_msgs)
      
        image_height, image_width, _ = image.shape
        
        goal = ObjectROIGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        all_objects = []
        if (self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            result = self.client.get_result()
            rois = result.roi
            
            
            for roi in rois:
                roi_data = []
                middle_x = (roi.left + roi.right) / 2
                middle_y = (roi.top + roi.bottom) /2
                
                left_right_padding = roi.right - roi.left
                top_down_padding = roi.bottom - roi.top
                
                padding = left_right_padding if left_right_padding > top_down_padding else top_down_padding
                padding /= 2
                
                #for pad in np.arange(0, 20, 1):
                new_padding = padding 
                
                left = middle_x - new_padding if middle_x - new_padding > 0 else 0
                right = middle_x + new_padding if middle_x + new_padding < image_width else image_width
                top = middle_y - new_padding if middle_y - new_padding > 0 else 0
                bottom = middle_y + new_padding if middle_y + new_padding < image_height else image_height
                
                cropped_image = image[top:bottom, left:right]
                resized_image = cv2.resize(cropped_image, (28, 28))
                scaled_image = resized_image / 255.0
                
                cropped_depth_image = image_depth[top:bottom, left:right]
                resized_depth_image = cv2.resize(cropped_depth_image, (28, 28))
                
                roi_data.append((scaled_image, resized_depth_image, roi.x, roi.y, roi.z, roi.z_min, roi.z_max))
            
                all_objects.append(roi_data)
        
        return all_objects       
                    
                    
                    
                    
                    
                    
                    
                    
