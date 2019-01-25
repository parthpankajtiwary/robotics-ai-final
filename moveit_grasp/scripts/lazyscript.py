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
from alice_msgs.msg import RecogniseObjectsAction, RecogniseObjectsGoal
import time
import os
import json
import actionlib
from geometry_msgs.msg import Quaternion

object_sizes = []
with open('objects.json', 'r') as object_file:
    object_sizes = json.load(object_file)

def get_size(obj_class):
    # Return {name, x, y, z}
    props = object_sizes[obj_class]
    return (props['x'], props['y'], props['z'])

def CallbackRGBImage(data):
    pass

def CallbackPointCloud(data):
    pass

def CallbackDepthImage(data):
    pass

# Function for delete the model from Gazebo, e.g. "box1"
def DeleteModelName(name):
    delete_model(name)

def get_my_classes(client):
    """Return predictions [class1, classn] or None"""
    print("Requesting new classes")
    recognise_goal = RecogniseObjectsGoal()
    recognise_goal.anything = "classes pls"
    client.send_goal(recognise_goal)
    client.wait_for_result()

    # Getin prediction result
    recognised = client.get_result()
    if(recognised is not None):
        recognised = recognised.classes
        print("Client saw objects " + " and ".join([str(cls) for cls in recognised]))
    return recognised

rospy.init_node("grasp_testing")

move_head = rospy.ServiceProxy("move_head", MoveHead)
clear_octomap = rospy.ServiceProxy("clear_octomap", Empty)

# Run subscribers to that the rgb image exposure doesn't cause problems
rospy.Subscriber("front_xtion/rgb/image_raw", Image, CallbackRGBImage)
rospy.Subscriber("front_xtion/depth/image_raw", Image, CallbackDepthImage)
rospy.Subscriber("front_xtion/depth/points", PointCloud2, CallbackDepthImage)
time.sleep(2)

# client = actionlib.SimpleActionClient("neuralObjectRecognition", RecogniseObjectsAction)
# client.wait_for_server()
print("RecogniserClient initialised.")

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


moveit.open_fingers()
moveit.remove_object()

exit()

# For deleting the model from Gazebo (Pretending we dropped in a bin on Alice)
delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

# you can change the cropping in GetObjectInformation function
all_object_data = alice_object.GetObjectInformation() # Gather RGB and Depth cropping data and x,y,z information

moveit.close_fingers() # Close the fingers before planning!

zero_image = np.zeros((28,28))
one_image = np.ones((28,28))

# classifications = get_my_classes()
# classifications = [] if None else classifications

# all_object_data : tuple list of (scaled_image, resized_depth_image, roi.x, roi.y, roi.z, roi.z_min, roi.z_max)
print("all_object_data", [obj[2:] for obj in all_object_data])
all_object_data = [obj for obj in all_object_data if obj[3] > -.5]
print("all_object_data", [obj[2:] for obj in all_object_data])

for object in all_object_data:
    x,y,z = (0,0,0)
    z_min, z_max = (0,0)

    image_color, image_depth, x, y, z, z_min, z_max = object
    image_depth = np.asarray(image_depth)
    image_depth = alice_object.CreateBinaryImage(image_depth)
    image_depth = image_depth.reshape(1, 28, 28, 1)
    prediction = model.predict(image_depth)[0][0]
    prediction *= pi

    print "Deg: ", degrees(prediction)

    #uncomment to show binary depth image
    # if np.array_equal(zero_image, image_depth) or np.array_equal(one_image,image_depth):
    #     print 'zero image'
    #     #cv2.imshow("binary depth image" , image_depth[0])
    # else:
    #     #cv2.imshow("binary depth image", image_depth[0])
    #     pass

    image_depth = image_depth.reshape(1, 28, 28, 1)

    image_color = np.asarray([image_color])
    output_classes = model_classification.predict([image_color])[0]
    class_index = np.argmax(output_classes)

    #uncomment to show images
    #cv2.imshow("color image", image_color[0])
    #cv2.waitKey(0)

    rotation = degrees(np.mean(prediction))
    print("With orientation:", str(rotation))

    size = get_size(class_index)
    print "Box class:", class_index, " and size", size

    # Call the grasp function
    moveit.grasp(x,y,z, rotation, size)

    # Move object to base of alice
    home_orient_quat = [0.6897, 0.7219, 0.0286, 0.0493]
    moveit.move_to(0.07762, 0.19728, 0.185099, 0, quaternion=home_orient_quat)

    # Drop object
    moveit.open_fingers()
    moveit.remove_object()
