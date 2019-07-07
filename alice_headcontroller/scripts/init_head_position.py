#!/usr/bin/env python
import rospy
from alice_msgs.srv import MoveHead

rospy.init_node("init_head_position")
rospy.sleep(4) # allow gazebo to start-up
move_head = rospy.ServiceProxy("move_head", MoveHead)
move_head.wait_for_service()
move_head(0.6, 0) # (pitch, yaw)
