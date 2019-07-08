#!/usr/bin/env python
import rospy 
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import tf
import rospkg
from math import pi
from random import shuffle, uniform


def SpawnObject(object_name, x, y, z, rotation):
    path = rospack.get_path("alice_description")
    with open(path +  "/sdf/" + str(object_name) + ".sdf") as f:
        box_xml = f.read()
        
    orientation = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, rotation)
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    
    pose = Pose(Point(x, y, z), orientation)
    
    spawn_model(object_name, box_xml, "", pose, "world")            
        
if __name__ == "__main__":
    rospy.init_node("set_objects")
    rospack = rospkg.RosPack()
    print 'Waiting for service...'
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print 'Connected to service'
    
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    
    # Need 5 positions
    # per table at least one object
    # table 1 pos = (3.57, 3.0), x - 0.3 
    pos1 = [3.33, 3.15]
    pos2 = [3.33, 2.85]
    
    pos3 = [0.22, 4.00]
    pos4 = [0.22, 3.70]
    
    pos5 = [0.74, 3.85]
    
    objects = ["box0", "box1", "box2", "box3", "box4"]
    
    for object in objects:
        delete_model(object)
    
    shuffle(objects)
    
    
    SpawnObject(objects[0], pos1[0], pos1[1], 0.77, uniform(0, pi))
    SpawnObject(objects[1], pos2[0], pos2[1], 0.77, uniform(0, pi))
    SpawnObject(objects[2], pos3[0], pos3[1], 0.77, uniform(0, pi))
    SpawnObject(objects[3], pos4[0], pos4[1], 0.77, uniform(0, pi))
    SpawnObject(objects[4], pos5[0], pos5[1], 0.77, uniform(0, pi))
    
    
    
    print 'Done'

# randomly set all objects on the table..
# there are 5 objects, 2 on table 1, 2 on 1 side, 1 on the other side

# need locations of table...

# table1 on middle_pos 
#table2 on middle_pos (0.48, 3.85)

