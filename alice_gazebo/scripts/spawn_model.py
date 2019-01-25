import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import tf
import os
import math
import sys

def SpawnTable(x = 0.6, y = 0.0, z = 0.0):
  with open(os.environ["HOME"] + "/repositories/alice/ros/catkin_ws/src/alice_description/sdf/table.sdf") as f:
    table_xml = f.read()

  orientation = Quaternion()
  pose = Pose(Point(x, y, z), orientation)
  delete_model("table")
  spawn_model("table", table_xml, "", pose, "world")
  
def SpawnBox(name, x = 0.4, y = 0.0, z = 0.77, rotation = 0):
  with open(os.environ["HOME"] + "/repositories/alice/ros/catkin_ws/src/alice_description/sdf/" + str(object_name) + ".sdf") as f:
    box_xml = f.read()
    
  orientation = Quaternion()
  quaternion = tf.transformations.quaternion_from_euler(0, 0, rotation)
  orientation.x = quaternion[0]
  orientation.y = quaternion[1]
  orientation.z = quaternion[2]
  orientation.w = quaternion[3]  
  
  pose = Pose(Point(x, y, z), orientation)
  delete_model("box")
  spawn_model("box", box_xml, "", pose, "world")
  
def SpawnCan(x = 0.4, y = 0.0, z = 0.81, rotation = 0):
  with open(os.environ["HOME"] + "/repositories/alice/ros/catkin_ws/src/alice_description/sdf/can.sdf") as f:
    can_xml = f.read()
    
  orientation = Quaternion()
  pose = Pose(Point(x, y, z), orientation)
  quaternion = tf.transformations.quaternion_from_euler(0, math.pi/2, rotation)
  orientation.x = quaternion[0]
  orientation.y = quaternion[1]
  orientation.z = quaternion[2]
  orientation.w = quaternion[3] 
  delete_model("can")
  spawn_model("can", can_xml, "", pose, "world")

if __name__ == "__main__":
  rospy.init_node("spawn_model")
  print "Waiting for service"
  rospy.wait_for_service("/gazebo/spawn_sdf_model")
  print "connected to service"

  spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
  delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

  SpawnTable()

  object_name = "box0"
  if len(sys.argv) == 2:
      object_name = sys.argv[1]
  
  SpawnBox(object_name, rotation = 0)
  
  print 'Done'
   
  
  
  
