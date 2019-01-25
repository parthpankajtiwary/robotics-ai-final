from keras.models import load_model
import numpy as np
from math import pi, degrees
from sensor_msgs.msg import Image, PointCloud2
from gazebo_msgs.srv import DeleteModel
from alice_msgs.srv import MoveHead
from alice_msgs.srv._MoveHead import MoveHeadRequest
from std_srvs.srv import Empty
from alice_msgs.msg import RecogniseObjectsAction, RecogniseObjectsGoal
from alice_msgs.msg import GraspObjectsAction, GraspObjectsResult
import time, os, json, actionlib, rospy, cv2
from geometry_msgs.msg import Quaternion
import tensorflow as tf
from tts import TTS
import time
from math import radians
import sys
import os
sys.path.append("/home/student/sudo/ros/catkin_ws/src/moveit_grasp/scripts")
from alice_object import AliceObject
from moveit import MoveIt

sys.path.append("/home/student/sudo/ros/catkin_ws/src/object_recognition/scripts")
from network import Network

CHECKPOINT_DIR = "/home/student/sudo/ros/catkin_ws/src/object_recognition/scripts/checkpoint/network.ckpt"



class GraspObjectsServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer("object_grasping", GraspObjectsAction, self.grabObjects)

        # Grab object sizes
        self.object_sizes = []
        self.objectList = []
        print(os.getcwd())
        with open('objects.json', 'r') as object_file:
            self.object_sizes = json.load(object_file)

        self.move_head      = rospy.ServiceProxy("move_head", MoveHead)
        self.clear_octomap  = rospy.ServiceProxy("clear_octomap", Empty)
        self.delete_model   = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        print("CURRENTLY USING GIVEN RECOGNTION NETWORK")
        # client = actionlib.SimpleActionClient("neuralObjectRecognition", RecogniseObjectsAction)
        # client.wait_for_server()

        move_head_req = MoveHeadRequest()
        move_head_req.pitch = 0.7
        move_head_req.yaw = 0.0
        self.move_head(move_head_req)


        self.model = load_model(os.environ["HOME"] + "/DATA/model.h5")
        self.model._make_predict_function() #https://github.com/keras-team/keras/issues/6462

        self.graph_recog = tf.Graph()
        with self.graph_recog.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth=True
            self.session_recog = tf.InteractiveSession(config = config)

            self.classification_model = Network(self.session_recog)
            self.classification_model.load_checkpoint(CHECKPOINT_DIR)
        # self.classification_model._make_predict_function()

        # self.main_behaviour = self.get_behaviour("final_demo")

        self.clear_octomap() # Reseting the Octomap because we moved before
        self.alice_object = AliceObject() # Class for interfacing with alice_object node
        self.moveit = MoveIt() # Class for interfacing with MoveIt (Needs to be implemented!)
        self.moveit.close_fingers() # Close the fingers before planning!

        objects_to_go = True

        self.server.start()

    # Function for delete the model from Gazebo, e.g. "box1"
    def DeleteModelName(self, name):
        self.delete_model(name)

    def get_size(self, obj_class):
        # Return {name, x, y, z}
        props = self.object_sizes[obj_class]
        return (props['x'], props['y'], props['z'])

    def get_my_classes(self, client):
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

    def recogniseObjects(self):
        all_object_data = self.alice_object.GetObjectInformation() # Gather RGB and Depth cropping data and x,y,z information
        objects_to_go = {}

        objects_seen = []

        for object in all_object_data:
            x,y,z = (0,0,0)
            z_min, z_max = (0,0)

            colored_img, image_depth, x, y, z, z_min, z_max = object
            image_depth = np.asarray(image_depth)
            image_depth = self.alice_object.CreateBinaryImage(image_depth)
            image_depth = image_depth.reshape(1, 28, 28, 1)
            prediction = self.model.predict(image_depth)[0][0]
            prediction *= pi

            colored_img = np.asarray(colored_img)

            output_classes = []
            class_index = 0
            with self.graph_recog.as_default():
                with self.session_recog.as_default():
                    p1 = self.classification_model.feed_batch([colored_img])
                    # output_class = p1[0]
                    class_index = np.argmax(p1, axis=1)[0]
                    print("class_index",class_index)

            class_name = self.object_sizes[class_index]['name']
            objects_seen.append((class_name, x))

            tm = time.gmtime(time.time())
            time_str = "h" + str(tm.tm_hour + 1) + "m" + str(tm.tm_min) + "s" + str(tm.tm_sec)
            roi_out = "./rois/"
            flag = (class_name in self.objectList) and y > -.5
            cv2.imwrite(roi_out + time_str + "_class=" + class_name + "_p=" + str(np.amax(p1)) + "_used=" + str(flag) + ".jpg", colored_img*255)

            rotation = degrees(np.mean(prediction))

            size = self.get_size(class_index)
            print("class:" + str(class_index) + "( " + str(self.object_sizes[class_index]) + ") and orientation " + str(rotation) + ", size: " + str(size))

            if class_name in self.objectList:
                props = (x, y, z, rotation, size, class_name)
                print('position of objects', props)
                if props[1] > -.5:
                    objects_to_go[class_name] = props

        # Please confirm increasing X against the axis direction to "left to right"
        # Current sorting is from low x to high x
        objects_seen.sort(key = lambda tup : tup[1])#reverse=True
        objects_seen = [tup[0] for tup in objects_seen]
        TTS.say_once(" I see " + " and ".join(objects_seen))#, condition=TTS.main_behaviour.local_state)#condition=TTS.main_behaviour.local_state?

        return objects_to_go

    def grabObjects(self, goal):
        self.objectList = goal.objects

        objects_to_go = self.recogniseObjects()
        init_objects = list(objects_to_go.keys())
        print("objects_to_go", objects_to_go)

        attempts_left = len(objects_to_go) + 3
        while objects_to_go:
            print("grabbing.. : ", objects_to_go)
            x,y,z, rotation, size, name = objects_to_go.values()[0]
            TTS.publish("grabbing " + str(name))
            self.moveit.grasp(x,y,z, rotation, size)

            # Move object to base of alice
            # self.moveit.move_to(0.30, 0.04, 0.2064081862876137, radians(90))
            self.moveit.move_to(0.2517, 0.1621, 0.2262, radians(90))

            # Drop object
            self.moveit.open_fingers()
            self.moveit.remove_object()
            self.moveit.close_fingers()

            objects_to_go = self.recogniseObjects()
            attempts_left -= 1
            if attempts_left == 0:
                TTS.publish("Ive been grabbing too long. I am sad now.")
                break

        grabbed_objects = [obj_name for obj_name in init_objects if obj_name not in objects_to_go.keys()]
        print("temporary grasp: grabbed_objects", grabbed_objects)

        result = GraspObjectsResult()
        result.grabbedObjects = grabbed_objects
        self.server.set_succeeded(result)

    # def reset(self):
        # self.already_said = set()

if __name__ == "__main__":
    rospy.init_node("Server for object grasping")
    server = GraspObjectsServer()
    rospy.spin()
