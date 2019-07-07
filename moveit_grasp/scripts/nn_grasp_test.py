import rospy
from keras.models import load_model
from alice_object import AliceObject
from moveit import MoveIt
import numpy as np
from math import pi, degrees
import cv2
from sensor_msgs.msg import Image, PointCloud2
from gazebo_msgs.srv import DeleteModel
from alice_msgs.srv import MoveHead
from alice_msgs.srv._MoveHead import MoveHeadRequest
from std_srvs.srv import Empty
import time
import os


def CallbackRGBImage(data):
    pass

def CallbackPointCloud(data):
    pass

def CallbackDepthImage(data):
    pass


# Function for delete the model from Gazebo, e.g. "box1"
def DeleteModelName(name):
    delete_model(name)


rospy.init_node("grasp_testing")

# Run subscribers to that the rgb image exposure doesn't cause problems 
rospy.Subscriber("front_xtion/rgb/image_raw", Image, CallbackRGBImage)
rospy.Subscriber("front_xtion/depth/image_raw", Image, CallbackDepthImage)
rospy.Subscriber("front_xtion/depth/points", PointCloud2, CallbackDepthImage)
time.sleep(2)

move_head = rospy.ServiceProxy("move_head", MoveHead)
clear_octomap = rospy.ServiceProxy("clear_octomap", Empty)

# Set the head into 
move_head_req = MoveHeadRequest()
move_head_req.pitch = 0.7
move_head_req.yaw = 0.0
move_head(move_head_req)

model = load_model(os.environ["HOME"] + "/DATA/model.h5")
model_classification = load_model(os.environ["HOME"] + "/DATA/model_classification.h5")

clear_octomap() # Reseting the Octomap because we moved before
alice_object = AliceObject() # Class for interfacing with alice_object node
moveit = MoveIt() # Class for interfacing with MoveIt (Needs to be implemented!)

# For deleting the model from Gazebo (Pretending we dropped in a bin on Alice)
delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

# you can change the cropping in GetObjectInformation function
all_object_data = alice_object.GetObjectInformation() # Gather RGB and Depth cropping data and x,y,z information

moveit.close_fingers() # Close the fingers before planning!

zero_image = np.zeros((28,28))
one_image = np.ones((28,28))

for objects in all_object_data:
    x = 0
    y = 0
    z = 0
    z_min = 0
    z_max = 0
    
    predictions = []
    classifications = []
    
    for object in objects:
        image_color, image_depth, x, y, z, z_min, z_max = object
        image_depth = np.asarray(image_depth)   
        image_depth = alice_object.CreateBinaryImage(image_depth)        
    	image_depth = image_depth.reshape(1, 28, 28, 1)
    	prediction = model.predict(image_depth)[0][0]
    	prediction *= pi
    
    	print "Deg: ", degrees(prediction)
    
        if np.array_equal(zero_image, image_depth) or np.array_equal(one_image,image_depth):
            print 'zero image'
            # uncomment to show binary depth image
            #cv2.imshow("binary depth image" , image_depth[0])
        else:            
            #uncomment to show binary depth image
            #cv2.imshow("binary depth image", image_depth[0])
            pass
            
	    image_depth = image_depth.reshape(1, 28, 28, 1)
        predictions.append(prediction)
        
        image_color = np.asarray([image_color])
        output_classes = model_classification.predict([image_color])[0]
        class_index = np.argmax(output_classes)
        
        #uncomment to show images
        #cv2.imshow("color image", image_color[0])
        #cv2.waitKey(0)    
            
    print "With orientation:", str(degrees(np.mean(predictions)))
    print "Box class:", class_index
 
    # Call the grasp function 
