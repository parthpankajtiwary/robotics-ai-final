import rospy
import actionlib
from alice_msgs.msg import *

rospy.init_node("move_forward")
client = actionlib.SimpleActionClient('alicecontroller', alicecontrollerfunctionAction)
client.wait_for_server()

goal = alicecontrollerfunctionGoal()
goal.function = "move"
goal.meter = -0.2

client.send_goal(goal)
client.wait_for_result()

if (client.get_state() == actionlib.GoalStatus.ABORTED):
	print 'aborted!'
elif (client.get_state() == actionlib.GoalStatus.SUCCEEDED):
	print 'Success'

