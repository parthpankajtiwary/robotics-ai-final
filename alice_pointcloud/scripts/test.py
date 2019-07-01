import rospy
from alice_msgs.msg import *
import actionlib

rospy.init_node("testing_node")
client = actionlib.SimpleActionClient("point_cloud_function", PointCloudFunctionAction)

client.wait_for_server()
print 'connected to point cloud function server'

goal = PointCloudFunctionGoal()
goal.function = "find_clusters"
goal.transform_to_link = "base_link"
#goal.return_type = "closest"

#goal.filter_min.x = 0.0
#goal.filter_max.x = 1.5
goal.filter_min.z = 0.2
goal.filter_max.z = 1.5 

client.send_goal(goal)

client.wait_for_result()

if (client.get_state() == actionlib.GoalStatus.SUCCEEDED):
    result = client.get_result()
    
    for i in range(len(result.point)):
        print "Point:\n", result.point[i]
        print ' '
elif (client.get_state() == actionlib.GoalStatus.ABORTED):
    print 'Aborted'
