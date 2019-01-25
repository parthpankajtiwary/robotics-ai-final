from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
from tts import TTS



class nav2table(AbstractBehaviour):

    def init(self):

        print('Action client initialized')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Connect to MoveBase
        self.client.wait_for_server()
        print('Connected to MoveBase')
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.result = 0

        main = self.get_behaviour("final_demo")
        self.locations = main.current_waypoints
        self.index = 0

    def update(self):
        locnames = [l['name'] for l in self.locations]
        TTS.print_different("navigation: self.state" + str(self.state) + ", moving to " + str(self.locations[self.index]['name']), condition='nav')
        TTS.print_different("nav2table stack: " + str(locnames), condition='navstack')
        if self.state == State.start:
            # Move 0.5 meters forward along the x axis of the "map" coordinate frame
            self.goal.target_pose.pose.position.x = self.locations[self.index]['position']['x']
            self.goal.target_pose.pose.position.y = self.locations[self.index]['position']['y']
            self.goal.target_pose.pose.position.z = self.locations[self.index]['position']['z']

            # No rotation of the mobile base frame w.r.t. map frame
            self.goal.target_pose.pose.orientation.x = self.locations[self.index]['orientation']['x']
            self.goal.target_pose.pose.orientation.y = self.locations[self.index]['orientation']['y']
            self.goal.target_pose.pose.orientation.z = self.locations[self.index]['orientation']['z']
            self.goal.target_pose.pose.orientation.w = self.locations[self.index]['orientation']['w']

            self.client.send_goal(self.goal)
            self.state = State.waiting


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
            if self.index >= len(self.locations):
                self.finish()

    def reset(self):
        self.state = State.idle
        self.init()
