from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml



class action_client(AbstractBehaviour):

    def init(self):

        print('Action client initialized')

    	self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    	# Connect to MoveBase
    	self.client.wait_for_server()
    	print('Connected to MoveBase')
    	self.goal = MoveBaseGoal()
    	self.goal.target_pose.header.frame_id = "map"
    	self.goal.target_pose.header.stamp = rospy.Time.now()

	self.result = 0
    	# Read yaml file
    	with open('/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/move_base/waypoints.yaml', 'r') as f:
    		self.doc = yaml.load(f)

	self.names = []
	self.index = 0

	for point_name in self.doc:
		self.names.append(point_name)



    def update(self):
	print self.state
	print self.names[self.index]

	#print(doc[point_name]['position'])
	if self.state == State.start:
    	# Move 0.5 meters forward along the x axis of the "map" coordinate frame
	    	self.goal.target_pose.pose.position.x = self.doc[self.names[self.index]]['position']['x']
		self.goal.target_pose.pose.position.y = self.doc[self.names[self.index]]['position']['y']
		self.goal.target_pose.pose.position.z = self.doc[self.names[self.index]]['position']['z']
	   	# No rotation of the mobile base frame w.r.t. map frame
		self.goal.target_pose.pose.orientation.x = self.doc[self.names[self.index]]['orientation']['x']
		self.goal.target_pose.pose.orientation.y = self.doc[self.names[self.index]]['orientation']['y']
		self.goal.target_pose.pose.orientation.z = self.doc[self.names[self.index]]['orientation']['z']
	    	self.goal.target_pose.pose.orientation.w = self.doc[self.names[self.index]]['orientation']['w']

	    	self.client.send_goal(self.goal)
		self.state = State.waiting



   	# Waits for the server to finish performing the action.
    	#self.wait = self.client.wait_for_result()

    	if self.state == State.waiting:

		if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			# Successfully finished
			self.result = self.client.get_result()
			rospy.loginfo("Goal execution done!")
			self.set_state(State.end_current_point)

		elif self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.ACTIVE:
			# Still pending request
			pass
		else:
			self.set_state(State.end_waiting)


	if self.state == State.end_waiting:
		# Failed request, end it all
		print('Request failed')
		self.finish()

	if self.state == State.end_current_point:
		self.index += 1
		self.state = State.start
		if self.index >= len(self.names):
			self.finish()

    def reset(self):
        self.state = State.idle
        self.init()
